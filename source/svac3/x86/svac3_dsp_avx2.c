/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright (c) 2026 OpenIPC contributors
 *
 * AVX2 kernels, bit-exact with the C reference in svac3_dsp.c. Built with
 * -mavx2 and used only on a CPU that has it. Widths that are not a
 * multiple of 8 go to C.
 */
#include <immintrin.h>
#include <stddef.h>
#include <string.h>

#include "svac3_dsp.h"

/* Two 16-bit values in one 32-bit lane, lo first, for madd */
static inline int pair16(int lo, int hi)
{
    return (int)((uint32_t)(uint16_t)lo | ((uint32_t)(uint16_t)hi << 16));
}

/* 16 int16 lanes to 16 uint8, saturating, in order */
static inline __m128i pack_u8(__m256i v)
{
    return _mm_packus_epi16(_mm256_castsi256_si128(v), _mm256_extracti128_si256(v, 1));
}

/* ------------------------------------------------------------------------ */

static void matmul_avx2(int16_t *out, int ostride, const int16_t *a, int astride,
                        const int16_t *b, int bstride, int rows, int depth, int cols,
                        int shift, int lo, int hi)
{
    /* b's rows interleaved in pairs, (b[k][c], b[k + 1][c]) for madd */
    int16_t bp[33 * 2 * 64];
    const int pairs = (depth + 1) / 2;
    for (int p = 0; p < pairs; p++) {
        const int16_t *b0 = b + 2 * p * bstride;
        const int16_t *b1 = 2 * p + 1 < depth ? b0 + bstride : NULL;
        for (int c = 0; c < cols; c++) {
            bp[(p * cols + c) * 2] = b0[c];
            bp[(p * cols + c) * 2 + 1] = b1 ? b1[c] : 0;
        }
    }
    const __m256i rnd = _mm256_set1_epi32(1 << (shift - 1));
    const __m128i sh = _mm_cvtsi32_si128(shift);
    const __m256i vlo = _mm256_set1_epi32(lo), vhi = _mm256_set1_epi32(hi);

    for (int r = 0; r < rows; r++) {
        const int16_t *ar = a + r * astride;
        int c = 0;
        for (; c + 8 <= cols; c += 8) {
            __m256i acc = _mm256_setzero_si256();
            for (int p = 0; p < pairs; p++) {
                const int a1 = 2 * p + 1 < depth ? ar[2 * p + 1] : 0;
                const __m256i av = _mm256_set1_epi32(pair16(ar[2 * p], a1));
                const __m256i bv = _mm256_loadu_si256((const __m256i *)&bp[(p * cols + c) * 2]);
                acc = _mm256_add_epi32(acc, _mm256_madd_epi16(bv, av));
            }
            acc = _mm256_sra_epi32(_mm256_add_epi32(acc, rnd), sh);
            acc = _mm256_min_epi32(_mm256_max_epi32(acc, vlo), vhi);
            _mm_storeu_si128((__m128i *)&out[r * ostride + c],
                             _mm_packs_epi32(_mm256_castsi256_si128(acc),
                                             _mm256_extracti128_si256(acc, 1)));
        }
        if (c < cols) {                         /* cols == 4 */
            __m128i acc = _mm_setzero_si128();
            for (int p = 0; p < pairs; p++) {
                const int a1 = 2 * p + 1 < depth ? ar[2 * p + 1] : 0;
                const __m128i av = _mm_set1_epi32(pair16(ar[2 * p], a1));
                const __m128i bv = _mm_loadu_si128((const __m128i *)&bp[(p * cols + c) * 2]);
                acc = _mm_add_epi32(acc, _mm_madd_epi16(bv, av));
            }
            acc = _mm_sra_epi32(_mm_add_epi32(acc, _mm256_castsi256_si128(rnd)), sh);
            acc = _mm_min_epi32(_mm_max_epi32(acc, _mm256_castsi256_si128(vlo)),
                                _mm256_castsi256_si128(vhi));
            _mm_storel_epi64((__m128i *)&out[r * ostride + c], _mm_packs_epi32(acc, acc));
        }
    }
}

/* ------------------------------------------------------------------------ */

static void add_block_avx2(uint8_t *dst, int stride, const uint8_t *pred, const int16_t *resid,
                           int w, int h)
{
    if (w < 8 || (w & 7)) {
        svac3_dsp_c.add_block(dst, stride, pred, resid, w, h);
        return;
    }
    for (int y = 0; y < h; y++) {
        const uint8_t *p = pred + y * w;
        const int16_t *r = resid + y * w;
        uint8_t *d = dst + (size_t)y * stride;
        int x = 0;
        for (; x + 16 <= w; x += 16) {
            const __m256i pv = _mm256_cvtepu8_epi16(_mm_loadu_si128((const __m128i *)(p + x)));
            const __m256i rv = _mm256_loadu_si256((const __m256i *)(r + x));
            _mm_storeu_si128((__m128i *)(d + x), pack_u8(_mm256_add_epi16(pv, rv)));
        }
        if (x < w) {                            /* 8 left */
            const __m128i pv = _mm_cvtepu8_epi16(_mm_loadl_epi64((const __m128i *)(p + x)));
            const __m128i rv = _mm_loadu_si128((const __m128i *)(r + x));
            const __m128i s = _mm_add_epi16(pv, rv);
            _mm_storel_epi64((__m128i *)(d + x), _mm_packus_epi16(s, s));
        }
    }
}

/* ------------------------------------------------------------------------ */
/* Interpolation. Sums of 8-bit samples by taps whose magnitudes add to
 * under 128 fit in 16 bits, so the one-dimensional sums and the
 * horizontal pass of the two-dimensional one run in 16-bit lanes. */

/* 16 horizontal sums at s (taps over s[-lead .. ntap - 1 - lead]) */
static inline __m256i hsum16(const uint8_t *s, const int8_t *t, int ntap)
{
    __m256i acc = _mm256_setzero_si256();
    for (int k = 0; k < ntap; k++) {
        const __m256i v = _mm256_cvtepu8_epi16(_mm_loadu_si128((const __m128i *)(s + k)));
        acc = _mm256_add_epi16(acc, _mm256_mullo_epi16(v, _mm256_set1_epi16(t[k])));
    }
    return acc;
}

static inline __m128i hsum8(const uint8_t *s, const int8_t *t, int ntap)
{
    __m128i acc = _mm_setzero_si128();
    for (int k = 0; k < ntap; k++) {
        const __m128i v = _mm_cvtepu8_epi16(_mm_loadl_epi64((const __m128i *)(s + k)));
        acc = _mm_add_epi16(acc, _mm_mullo_epi16(v, _mm_set1_epi16(t[k])));
    }
    return acc;
}

static inline __m256i vsum16(const uint8_t *s, int stride, const int8_t *t, int ntap)
{
    __m256i acc = _mm256_setzero_si256();
    for (int k = 0; k < ntap; k++) {
        const __m256i v =
            _mm256_cvtepu8_epi16(_mm_loadu_si128((const __m128i *)(s + (size_t)k * stride)));
        acc = _mm256_add_epi16(acc, _mm256_mullo_epi16(v, _mm256_set1_epi16(t[k])));
    }
    return acc;
}

static inline __m128i vsum8(const uint8_t *s, int stride, const int8_t *t, int ntap)
{
    __m128i acc = _mm_setzero_si128();
    for (int k = 0; k < ntap; k++) {
        const __m128i v =
            _mm_cvtepu8_epi16(_mm_loadl_epi64((const __m128i *)(s + (size_t)k * stride)));
        acc = _mm_add_epi16(acc, _mm_mullo_epi16(v, _mm_set1_epi16(t[k])));
    }
    return acc;
}

static void mc_h_avx2(uint8_t *dst, const uint8_t *src, int stride, int w, int h,
                      const int8_t *t, int ntap)
{
    if (w < 8 || (w & 7)) {
        svac3_dsp_c.mc_h(dst, src, stride, w, h, t, ntap);
        return;
    }
    const uint8_t *s0 = src - (ntap / 2 - 1);
    const __m256i r32 = _mm256_set1_epi16(32);
    for (int y = 0; y < h; y++) {
        const uint8_t *s = s0 + (size_t)y * stride;
        int x = 0;
        for (; x + 16 <= w; x += 16) {
            const __m256i v = _mm256_srai_epi16(_mm256_add_epi16(hsum16(s + x, t, ntap), r32), 6);
            _mm_storeu_si128((__m128i *)(dst + y * w + x), pack_u8(v));
        }
        if (x < w) {
            const __m128i v = _mm_srai_epi16(_mm_add_epi16(hsum8(s + x, t, ntap),
                                                           _mm256_castsi256_si128(r32)), 6);
            _mm_storel_epi64((__m128i *)(dst + y * w + x), _mm_packus_epi16(v, v));
        }
    }
}

static void mc_v_avx2(uint8_t *dst, const uint8_t *src, int stride, int w, int h,
                      const int8_t *t, int ntap)
{
    if (w < 8 || (w & 7)) {
        svac3_dsp_c.mc_v(dst, src, stride, w, h, t, ntap);
        return;
    }
    const uint8_t *s0 = src - (size_t)(ntap / 2 - 1) * stride;
    const __m256i r32 = _mm256_set1_epi16(32);
    for (int y = 0; y < h; y++) {
        const uint8_t *s = s0 + (size_t)y * stride;
        int x = 0;
        for (; x + 16 <= w; x += 16) {
            const __m256i v = _mm256_srai_epi16(_mm256_add_epi16(vsum16(s + x, stride, t, ntap), r32), 6);
            _mm_storeu_si128((__m128i *)(dst + y * w + x), pack_u8(v));
        }
        if (x < w) {
            const __m128i v = _mm_srai_epi16(_mm_add_epi16(vsum8(s + x, stride, t, ntap),
                                                           _mm256_castsi256_si128(r32)), 6);
            _mm_storel_epi64((__m128i *)(dst + y * w + x), _mm_packus_epi16(v, v));
        }
    }
}

static void mc_hv_avx2(uint8_t *dst, const uint8_t *src, int stride, int w, int h,
                       const int8_t *th, const int8_t *tv, int ntap)
{
    if (w < 8 || (w & 7)) {
        svac3_dsp_c.mc_hv(dst, src, stride, w, h, th, tv, ntap);
        return;
    }
    int16_t tmp[(64 + 7) * 64];
    const int lead = ntap / 2 - 1;
    const uint8_t *s0 = src - (size_t)lead * stride - lead;

    /* horizontal, unrounded */
    for (int y = 0; y < h + ntap - 1; y++) {
        const uint8_t *s = s0 + (size_t)y * stride;
        int x = 0;
        for (; x + 16 <= w; x += 16)
            _mm256_storeu_si256((__m256i *)(tmp + y * w + x), hsum16(s + x, th, ntap));
        if (x < w)
            _mm_storeu_si128((__m128i *)(tmp + y * w + x), hsum8(s + x, th, ntap));
    }

    /* vertical in 32 bits, two rows at a time through madd */
    __m256i tp[4];
    for (int p = 0; p < ntap / 2; p++)
        tp[p] = _mm256_set1_epi32(pair16(tv[2 * p], tv[2 * p + 1]));
    const __m256i r2048 = _mm256_set1_epi32(1 << 11);
    for (int y = 0; y < h; y++) {
        int x = 0;
        for (; x + 16 <= w; x += 16) {
            __m256i lo = _mm256_setzero_si256(), hi = _mm256_setzero_si256();
            for (int p = 0; p < ntap / 2; p++) {
                const __m256i a = _mm256_loadu_si256((const __m256i *)(tmp + (y + 2 * p) * w + x));
                const __m256i b = _mm256_loadu_si256((const __m256i *)(tmp + (y + 2 * p + 1) * w + x));
                lo = _mm256_add_epi32(lo, _mm256_madd_epi16(_mm256_unpacklo_epi16(a, b), tp[p]));
                hi = _mm256_add_epi32(hi, _mm256_madd_epi16(_mm256_unpackhi_epi16(a, b), tp[p]));
            }
            lo = _mm256_srai_epi32(_mm256_add_epi32(lo, r2048), 12);
            hi = _mm256_srai_epi32(_mm256_add_epi32(hi, r2048), 12);
            /* per 128-bit lane: columns 0-3 / 4-7 and 8-11 / 12-15 */
            const __m256i v = _mm256_packs_epi32(lo, hi);
            _mm_storeu_si128((__m128i *)(dst + y * w + x), pack_u8(v));
        }
        if (x < w) {
            __m128i lo = _mm_setzero_si128(), hi = _mm_setzero_si128();
            for (int p = 0; p < ntap / 2; p++) {
                const __m128i a = _mm_loadu_si128((const __m128i *)(tmp + (y + 2 * p) * w + x));
                const __m128i b = _mm_loadu_si128((const __m128i *)(tmp + (y + 2 * p + 1) * w + x));
                const __m128i tq = _mm256_castsi256_si128(tp[p]);
                lo = _mm_add_epi32(lo, _mm_madd_epi16(_mm_unpacklo_epi16(a, b), tq));
                hi = _mm_add_epi32(hi, _mm_madd_epi16(_mm_unpackhi_epi16(a, b), tq));
            }
            lo = _mm_srai_epi32(_mm_add_epi32(lo, _mm256_castsi256_si128(r2048)), 12);
            hi = _mm_srai_epi32(_mm_add_epi32(hi, _mm256_castsi256_si128(r2048)), 12);
            const __m128i v = _mm_packs_epi32(lo, hi);
            _mm_storel_epi64((__m128i *)(dst + y * w + x), _mm_packus_epi16(v, v));
        }
    }
}

/* ------------------------------------------------------------------------ */
/* Deblocking: the 8 lines in 8 lanes of 16 bits, the per-line decisions of
 * 5.4.11.4 as lane masks, each filter computed for every lane and the
 * right one selected. Filter type 1, and runs of fewer than 8 lines, go
 * to C. */

#define SEL(m, a, b) _mm_blendv_epi8((b), (a), (m))      /* m ? a : b */

/* Transpose 8 x 8 bytes held in the low halves of r[0..7] */
static inline void transpose8x8(__m128i r[8])
{
    const __m128i a0 = _mm_unpacklo_epi8(r[0], r[1]), a1 = _mm_unpacklo_epi8(r[2], r[3]);
    const __m128i a2 = _mm_unpacklo_epi8(r[4], r[5]), a3 = _mm_unpacklo_epi8(r[6], r[7]);
    const __m128i b0 = _mm_unpacklo_epi16(a0, a1), b1 = _mm_unpackhi_epi16(a0, a1);
    const __m128i b2 = _mm_unpacklo_epi16(a2, a3), b3 = _mm_unpackhi_epi16(a2, a3);
    const __m128i c0 = _mm_unpacklo_epi32(b0, b2), c1 = _mm_unpackhi_epi32(b0, b2);
    const __m128i c2 = _mm_unpacklo_epi32(b1, b3), c3 = _mm_unpackhi_epi32(b1, b3);
    r[0] = c0;
    r[1] = _mm_srli_si128(c0, 8);
    r[2] = c1;
    r[3] = _mm_srli_si128(c1, 8);
    r[4] = c2;
    r[5] = _mm_srli_si128(c2, 8);
    r[6] = c3;
    r[7] = _mm_srli_si128(c3, 8);
}

static inline __m128i lt16(__m128i a, __m128i b) { return _mm_cmpgt_epi16(b, a); }
static inline __m128i rshr(__m128i v, int rnd, int sh)
{
    return _mm_srai_epi16(_mm_add_epi16(v, _mm_set1_epi16((short)rnd)), sh);
}
#define MUL(v, k) _mm_mullo_epi16((v), _mm_set1_epi16(k))

static void deblock_avx2(uint8_t *s, ptrdiff_t step, ptrdiff_t lstep, int n, int group,
                         const int *alpha, const int *beta, int chroma, int type)
{
    if (type != 0 || n != 8) {
        svac3_dsp_c.deblock(s, step, lstep, n, group, alpha, beta, chroma, type);
        return;
    }
    /* v[k]: sample p3..q3 (k = 0..7) of the 8 lines, one per lane */
    __m128i r[8], v[8];
    if (lstep == 1) {
        for (int k = 0; k < 8; k++)
            r[k] = _mm_loadl_epi64((const __m128i *)(s + (k - 4) * step));
    } else {
        for (int l = 0; l < 8; l++)
            r[l] = _mm_loadl_epi64((const __m128i *)(s + l * lstep - 4));
        transpose8x8(r);
    }
    for (int k = 0; k < 8; k++)
        v[k] = _mm_cvtepu8_epi16(r[k]);
    const __m128i p3 = v[0], p2 = v[1], p1 = v[2], p0 = v[3];
    const __m128i q0 = v[4], q1 = v[5], q2 = v[6], q3 = v[7];

    int16_t al[8], be[8];
    for (int l = 0; l < 8; l++) {
        al[l] = (int16_t)alpha[l / group];
        be[l] = (int16_t)beta[l / group];
    }
    const __m128i A = _mm_loadu_si128((const __m128i *)al), B = _mm_loadu_si128((const __m128i *)be);
    const __m128i on = _mm_cmpgt_epi16(A, _mm_set1_epi16(-1));
    const __m128i one = _mm_set1_epi16(1), two = _mm_set1_epi16(2);

    const __m128i d_p0p1 = _mm_abs_epi16(_mm_sub_epi16(p0, p1));
    const __m128i d_q0q1 = _mm_abs_epi16(_mm_sub_epi16(q0, q1));
    const __m128i fL = _mm_add_epi16(_mm_and_si128(lt16(d_p0p1, B), two),
                                     _mm_and_si128(lt16(_mm_abs_epi16(_mm_sub_epi16(p0, p2)), B), one));
    const __m128i fR = _mm_add_epi16(_mm_and_si128(lt16(d_q0q1, B), two),
                                     _mm_and_si128(lt16(_mm_abs_epi16(_mm_sub_epi16(q0, q2)), B), one));
    const __m128i sum = _mm_add_epi16(fL, fR);

    const __m128i B4 = _mm_srai_epi16(B, 2);
    const __m128i strong = _mm_and_si128(_mm_and_si128(_mm_cmpgt_epi16(_mm_add_epi16(B4, one), d_p0p1),
                                                       _mm_cmpgt_epi16(_mm_add_epi16(B4, one), d_q0q1)),
                                         lt16(_mm_abs_epi16(_mm_sub_epi16(p0, q0)), A));
    const __m128i flat = _mm_and_si128(_mm_cmpeq_epi16(p0, p1), _mm_cmpeq_epi16(q0, q1));
    const __m128i bs6 = SEL(strong, _mm_set1_epi16(4), _mm_set1_epi16(3));
    const __m128i bs5 = SEL(flat, _mm_set1_epi16(3), two);
    const __m128i bs4 = SEL(_mm_cmpeq_epi16(fL, two), two, one);
    const __m128i bs3 = _mm_and_si128(lt16(_mm_abs_epi16(_mm_sub_epi16(p1, q1)), B), one);
    __m128i bs = _mm_setzero_si128();
    bs = SEL(_mm_cmpeq_epi16(sum, _mm_set1_epi16(3)), bs3, bs);
    bs = SEL(_mm_cmpeq_epi16(sum, _mm_set1_epi16(4)), bs4, bs);
    bs = SEL(_mm_cmpeq_epi16(sum, _mm_set1_epi16(5)), bs5, bs);
    bs = SEL(_mm_cmpeq_epi16(sum, _mm_set1_epi16(6)), bs6, bs);
    if (chroma)
        bs = _mm_max_epi16(_mm_sub_epi16(bs, one), _mm_setzero_si128());
    bs = _mm_and_si128(bs, on);

    __m128i np2 = p2, np1 = p1, np0 = p0, nq0 = q0, nq1 = q1, nq2 = q2;
    if (chroma) {
        const __m128i any = _mm_cmpgt_epi16(bs, _mm_setzero_si128());
        const __m128i b3 = _mm_cmpeq_epi16(bs, _mm_set1_epi16(3));
        np0 = SEL(any, rshr(_mm_add_epi16(_mm_add_epi16(MUL(p1, 3), MUL(p0, 10)), MUL(q0, 3)), 8, 4), p0);
        nq0 = SEL(any, rshr(_mm_add_epi16(_mm_add_epi16(MUL(p0, 3), MUL(q0, 10)), MUL(q1, 3)), 8, 4), q0);
        np1 = SEL(b3, rshr(_mm_add_epi16(_mm_add_epi16(MUL(p2, 3), MUL(p1, 8)),
                                         _mm_add_epi16(MUL(p0, 3), MUL(q0, 2))), 8, 4), p1);
        nq1 = SEL(b3, rshr(_mm_add_epi16(_mm_add_epi16(MUL(q2, 3), MUL(q1, 8)),
                                         _mm_add_epi16(MUL(q0, 3), MUL(p0, 2))), 8, 4), q1);
    } else {
        const __m128i b4 = _mm_cmpeq_epi16(bs, _mm_set1_epi16(4));
        const __m128i b3 = _mm_cmpeq_epi16(bs, _mm_set1_epi16(3));
        const __m128i b2 = _mm_cmpeq_epi16(bs, two);
        const __m128i b1 = _mm_cmpeq_epi16(bs, one);
        /* Bs 4 */
        const __m128i f4p0 = rshr(_mm_add_epi16(_mm_add_epi16(_mm_add_epi16(MUL(p2, 3), MUL(p1, 8)), MUL(p0, 10)),
                                                _mm_add_epi16(MUL(q0, 8), MUL(q1, 3))), 16, 5);
        const __m128i f4p1 = rshr(_mm_add_epi16(_mm_add_epi16(MUL(p2, 4), MUL(p1, 5)),
                                                _mm_add_epi16(MUL(p0, 4), MUL(q0, 3))), 8, 4);
        const __m128i f4p2 = rshr(_mm_add_epi16(_mm_add_epi16(_mm_add_epi16(MUL(p3, 2), MUL(p2, 2)), MUL(p1, 2)),
                                                _mm_add_epi16(p0, q0)), 4, 3);
        const __m128i f4q0 = rshr(_mm_add_epi16(_mm_add_epi16(_mm_add_epi16(MUL(p1, 3), MUL(p0, 8)), MUL(q0, 10)),
                                                _mm_add_epi16(MUL(q1, 8), MUL(q2, 3))), 16, 5);
        const __m128i f4q1 = rshr(_mm_add_epi16(_mm_add_epi16(MUL(p0, 3), MUL(q0, 4)),
                                                _mm_add_epi16(MUL(q1, 5), MUL(q2, 4))), 8, 4);
        const __m128i f4q2 = rshr(_mm_add_epi16(_mm_add_epi16(_mm_add_epi16(p0, q0), MUL(q1, 2)),
                                                _mm_add_epi16(MUL(q2, 2), MUL(q3, 2))), 4, 3);
        /* Bs 3 */
        const __m128i f3p0 = rshr(_mm_add_epi16(_mm_add_epi16(_mm_add_epi16(p2, MUL(p1, 4)), MUL(p0, 6)),
                                                _mm_add_epi16(MUL(q0, 4), q1)), 8, 4);
        const __m128i f3p1 = rshr(_mm_add_epi16(_mm_add_epi16(MUL(p2, 3), MUL(p1, 8)),
                                                _mm_add_epi16(MUL(p0, 4), q0)), 8, 4);
        const __m128i f3q0 = rshr(_mm_add_epi16(_mm_add_epi16(_mm_add_epi16(p1, MUL(p0, 4)), MUL(q0, 6)),
                                                _mm_add_epi16(MUL(q1, 4), q2)), 8, 4);
        const __m128i f3q1 = rshr(_mm_add_epi16(_mm_add_epi16(MUL(q2, 3), MUL(q1, 8)),
                                                _mm_add_epi16(MUL(q0, 4), p0)), 8, 4);
        /* Bs 2 and 1 */
        const __m128i f2p0 = rshr(_mm_add_epi16(_mm_add_epi16(MUL(p1, 3), MUL(p0, 10)), MUL(q0, 3)), 8, 4);
        const __m128i f2q0 = rshr(_mm_add_epi16(_mm_add_epi16(MUL(p0, 3), MUL(q0, 10)), MUL(q1, 3)), 8, 4);
        const __m128i f1p0 = rshr(_mm_add_epi16(MUL(p0, 3), q0), 2, 2);
        const __m128i f1q0 = rshr(_mm_add_epi16(MUL(q0, 3), p0), 2, 2);

        np0 = SEL(b4, f4p0, SEL(b3, f3p0, SEL(b2, f2p0, SEL(b1, f1p0, p0))));
        nq0 = SEL(b4, f4q0, SEL(b3, f3q0, SEL(b2, f2q0, SEL(b1, f1q0, q0))));
        np1 = SEL(b4, f4p1, SEL(b3, f3p1, p1));
        nq1 = SEL(b4, f4q1, SEL(b3, f3q1, q1));
        np2 = SEL(b4, f4p2, p2);
        nq2 = SEL(b4, f4q2, q2);
    }

    const __m128i out[6] = { np2, np1, np0, nq0, nq1, nq2 };
    for (int k = 0; k < 6; k++)
        r[k + 1] = _mm_packus_epi16(out[k], out[k]);
    if (lstep == 1) {
        for (int k = 1; k < 7; k++)
            _mm_storel_epi64((__m128i *)(s + (k - 4) * step), r[k]);
    } else {
        transpose8x8(r);                    /* p3 and q3 go back unchanged */
        for (int l = 0; l < 8; l++)
            _mm_storel_epi64((__m128i *)(s + l * lstep - 4), r[l]);
    }
}

/* ------------------------------------------------------------------------ */
/* Angular intra prediction rows: the four taps in 32-bit lanes. */

static inline __m128i ipred4(const int16_t *r, const __m128i t[4], __m128i rnd, __m128i sh)
{
    __m128i acc = rnd;
    for (int k = 0; k < 4; k++)
        acc = _mm_add_epi32(acc, _mm_mullo_epi32(_mm_cvtepi16_epi32(_mm_loadl_epi64((const __m128i *)(r + k))), t[k]));
    return _mm_sra_epi32(acc, sh);
}

static void ipred_row_avx2(uint8_t *dst, const int16_t *ref, int w, const int8_t *t, int rnd, int shift)
{
    if (w & 3) {
        svac3_dsp_c.ipred_row(dst, ref, w, t, rnd, shift);
        return;
    }
    const __m128i sh = _mm_cvtsi32_si128(shift);
    int x = 0;
    if (w >= 8) {
        __m256i tv[4];
        for (int k = 0; k < 4; k++)
            tv[k] = _mm256_set1_epi32(t[k]);
        const __m256i r8 = _mm256_set1_epi32(rnd);
        for (; x + 8 <= w; x += 8) {
            __m256i acc = r8;
            for (int k = 0; k < 4; k++)
                acc = _mm256_add_epi32(acc, _mm256_mullo_epi32(
                    _mm256_cvtepi16_epi32(_mm_loadu_si128((const __m128i *)(ref + x + k))), tv[k]));
            acc = _mm256_sra_epi32(acc, sh);
            const __m128i v = _mm_packs_epi32(_mm256_castsi256_si128(acc), _mm256_extracti128_si256(acc, 1));
            _mm_storel_epi64((__m128i *)(dst + x), _mm_packus_epi16(v, v));
        }
    }
    if (x < w) {                                          /* 4 left */
        __m128i tq[4];
        for (int k = 0; k < 4; k++)
            tq[k] = _mm_set1_epi32(t[k]);
        const __m128i v = ipred4(ref + x, tq, _mm_set1_epi32(rnd), sh);
        const __m128i p = _mm_packs_epi32(v, v);
        const int out = _mm_cvtsi128_si32(_mm_packus_epi16(p, p));
        memcpy(dst + x, &out, 4);
    }
}

/* ------------------------------------------------------------------------ */
/* SAO */

/* -1 / 0 / +1 per byte: the sign of a - b, unsigned bytes */
static inline __m128i sign8(__m128i a, __m128i b)
{
    const __m128i k80 = _mm_set1_epi8((char)0x80);
    const __m128i as = _mm_xor_si128(a, k80), bs = _mm_xor_si128(b, k80);
    return _mm_sub_epi8(_mm_cmpgt_epi8(bs, as), _mm_cmpgt_epi8(as, bs));
}

/* v + o (signed bytes), clipped to 0..255 */
static inline __m128i add_clip(__m128i v, __m128i o)
{
    const __m256i s = _mm256_add_epi16(_mm256_cvtepu8_epi16(v), _mm256_cvtepi8_epi16(o));
    return pack_u8(s);
}

static void sao_edge_avx2(uint8_t *dst, const uint8_t *src, ptrdiff_t stride, int w, int h,
                          ptrdiff_t na, ptrdiff_t nb, const int8_t offset[5])
{
    int8_t tab[16] = { 0 };
    memcpy(tab, offset, 5);
    const __m128i lut = _mm_loadu_si128((const __m128i *)tab), two = _mm_set1_epi8(2);
    for (int y = 0; y < h; y++) {
        const uint8_t *c = src + y * stride;
        uint8_t *d = dst + y * stride;
        int x = 0;
        for (; x + 16 <= w; x += 16) {
            const __m128i v = _mm_loadu_si128((const __m128i *)(c + x));
            const __m128i a = _mm_loadu_si128((const __m128i *)(c + x + na));
            const __m128i b = _mm_loadu_si128((const __m128i *)(c + x + nb));
            const __m128i idx = _mm_add_epi8(_mm_add_epi8(sign8(v, a), sign8(v, b)), two);
            _mm_storeu_si128((__m128i *)(d + x), add_clip(v, _mm_shuffle_epi8(lut, idx)));
        }
        if (x < w)
            svac3_dsp_c.sao_edge(d + x, c + x, stride, w - x, 1, na, nb, offset);
    }
}

static void sao_band_avx2(uint8_t *dst, const uint8_t *src, ptrdiff_t stride, int w, int h,
                          const int8_t band[32])
{
    const __m128i lo = _mm_loadu_si128((const __m128i *)band);
    const __m128i hi = _mm_loadu_si128((const __m128i *)(band + 16));
    const __m128i k15 = _mm_set1_epi8(15), k1f = _mm_set1_epi8(0x1f);
    for (int y = 0; y < h; y++) {
        const uint8_t *c = src + y * stride;
        uint8_t *d = dst + y * stride;
        int x = 0;
        for (; x + 16 <= w; x += 16) {
            const __m128i v = _mm_loadu_si128((const __m128i *)(c + x));
            const __m128i idx = _mm_and_si128(_mm_srli_epi16(v, 3), k1f);       /* v >> 3 */
            const __m128i i4 = _mm_and_si128(idx, k15);
            const __m128i o = _mm_blendv_epi8(_mm_shuffle_epi8(lo, i4), _mm_shuffle_epi8(hi, i4),
                                              _mm_cmpgt_epi8(idx, k15));
            _mm_storeu_si128((__m128i *)(d + x), add_clip(v, o));
        }
        if (x < w)
            svac3_dsp_c.sao_band(d + x, c + x, stride, w - x, 1, band);
    }
}

const svac3_dsp_t svac3_dsp_avx2 = {
    .name = "avx2",
    .matmul = matmul_avx2,
    .add_block = add_block_avx2,
    .mc_h = mc_h_avx2,
    .mc_v = mc_v_avx2,
    .mc_hv = mc_hv_avx2,
    .deblock = deblock_avx2,
    .ipred_row = ipred_row_avx2,
    .sao_edge = sao_edge_avx2,
    .sao_band = sao_band_avx2,
};
