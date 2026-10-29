/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright (c) 2026 OpenIPC contributors
 *
 * NEON kernels for AArch64, bit-exact with the C reference in svac3_dsp.c.
 * Widths that are not a multiple of 8 go to C.
 */
#include <arm_neon.h>
#include <stddef.h>
#include <string.h>

#include "svac3_dsp.h"

static void matmul_neon(int16_t *out, int ostride, const int16_t *a, int astride,
                        const int16_t *b, int bstride, int rows, int depth, int cols,
                        int shift, int lo, int hi)
{
    const int32x4_t sh = vdupq_n_s32(-shift);        /* rounding right shift */
    const int32x4_t vlo = vdupq_n_s32(lo), vhi = vdupq_n_s32(hi);
    for (int r = 0; r < rows; r++) {
        const int16_t *ar = a + r * astride;
        int c = 0;
        for (; c + 8 <= cols; c += 8) {
            int32x4_t l = vdupq_n_s32(0), h = vdupq_n_s32(0);
            for (int k = 0; k < depth; k++) {
                const int16x8_t bv = vld1q_s16(b + k * bstride + c);
                l = vmlal_n_s16(l, vget_low_s16(bv), ar[k]);
                h = vmlal_n_s16(h, vget_high_s16(bv), ar[k]);
            }
            l = vminq_s32(vmaxq_s32(vrshlq_s32(l, sh), vlo), vhi);
            h = vminq_s32(vmaxq_s32(vrshlq_s32(h, sh), vlo), vhi);
            vst1q_s16(out + r * ostride + c, vcombine_s16(vmovn_s32(l), vmovn_s32(h)));
        }
        if (c < cols) {                                  /* cols == 4 */
            int32x4_t l = vdupq_n_s32(0);
            for (int k = 0; k < depth; k++)
                l = vmlal_n_s16(l, vld1_s16(b + k * bstride + c), ar[k]);
            l = vminq_s32(vmaxq_s32(vrshlq_s32(l, sh), vlo), vhi);
            vst1_s16(out + r * ostride + c, vmovn_s32(l));
        }
    }
}

static void add_block_neon(uint8_t *dst, int stride, const uint8_t *pred, const int16_t *resid,
                           int w, int h)
{
    if (w & 7) {
        svac3_dsp_c.add_block(dst, stride, pred, resid, w, h);
        return;
    }
    for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x += 8) {
            const int16x8_t p = vreinterpretq_s16_u16(vmovl_u8(vld1_u8(pred + y * w + x)));
            const int16x8_t s = vaddq_s16(p, vld1q_s16(resid + y * w + x));
            vst1_u8(dst + (size_t)y * stride + x, vqmovun_s16(s));
        }
}

/* Sums of 8-bit samples by taps whose magnitudes add to under 128 fit in
 * 16 bits: the one-dimensional sums and the horizontal pass of the
 * two-dimensional one run in 16-bit lanes. */
static inline int16x8_t sum8(const uint8_t *s, ptrdiff_t step, const int8_t *t, int ntap)
{
    int16x8_t acc = vdupq_n_s16(0);
    for (int k = 0; k < ntap; k++)
        acc = vmlaq_n_s16(acc, vreinterpretq_s16_u16(vmovl_u8(vld1_u8(s + k * step))), t[k]);
    return acc;
}

static void mc_h_neon(uint8_t *dst, const uint8_t *src, int stride, int w, int h,
                      const int8_t *t, int ntap)
{
    if (w & 7) {
        svac3_dsp_c.mc_h(dst, src, stride, w, h, t, ntap);
        return;
    }
    const uint8_t *s0 = src - (ntap / 2 - 1);
    for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x += 8)    /* (sum + 32) >> 6, saturated to 0..255 */
            vst1_u8(dst + y * w + x, vqrshrun_n_s16(sum8(s0 + (size_t)y * stride + x, 1, t, ntap), 6));
}

static void mc_v_neon(uint8_t *dst, const uint8_t *src, int stride, int w, int h,
                      const int8_t *t, int ntap)
{
    if (w & 7) {
        svac3_dsp_c.mc_v(dst, src, stride, w, h, t, ntap);
        return;
    }
    const uint8_t *s0 = src - (size_t)(ntap / 2 - 1) * stride;
    for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x += 8)
            vst1_u8(dst + y * w + x,
                    vqrshrun_n_s16(sum8(s0 + (size_t)y * stride + x, stride, t, ntap), 6));
}

static void mc_hv_neon(uint8_t *dst, const uint8_t *src, int stride, int w, int h,
                       const int8_t *th, const int8_t *tv, int ntap)
{
    if (w & 7) {
        svac3_dsp_c.mc_hv(dst, src, stride, w, h, th, tv, ntap);
        return;
    }
    int16_t tmp[(64 + 7) * 64];
    const int lead = ntap / 2 - 1;
    const uint8_t *s0 = src - (size_t)lead * stride - lead;
    for (int y = 0; y < h + ntap - 1; y++)
        for (int x = 0; x < w; x += 8)
            vst1q_s16(tmp + y * w + x, sum8(s0 + (size_t)y * stride + x, 1, th, ntap));
    for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x += 8) {
            int32x4_t l = vdupq_n_s32(0), hh = vdupq_n_s32(0);
            for (int k = 0; k < ntap; k++) {
                const int16x8_t v = vld1q_s16(tmp + (y + k) * w + x);
                l = vmlal_n_s16(l, vget_low_s16(v), tv[k]);
                hh = vmlal_n_s16(hh, vget_high_s16(v), tv[k]);
            }
            /* (sum + 2048) >> 12, then saturated to 0..255 */
            const int16x8_t s = vcombine_s16(vrshrn_n_s32(l, 12), vrshrn_n_s32(hh, 12));
            vst1_u8(dst + y * w + x, vqmovun_s16(s));
        }
}

/* Deblocking: the 8 lines in 8 lanes of 16 bits, the per-line decisions of
 * 5.4.11.4 as lane masks, each filter computed for every lane and the
 * right one selected. Filter type 1, and runs of fewer than 8 lines, go
 * to C. */

#define SEL(m, a, b) vbslq_s16((m), (a), (b))             /* m ? a : b */
#define K(v) vdupq_n_s16(v)
#define MUL(v, k) vmulq_n_s16((v), (k))
#define ADD vaddq_s16
#define ABD(a, b) vabdq_s16((a), (b))

/* Transpose 8 x 8 bytes */
static inline void transpose8x8(uint8x8_t r[8])
{
    const uint8x8x2_t t01 = vtrn_u8(r[0], r[1]), t23 = vtrn_u8(r[2], r[3]);
    const uint8x8x2_t t45 = vtrn_u8(r[4], r[5]), t67 = vtrn_u8(r[6], r[7]);
    const uint16x4x2_t u02 = vtrn_u16(vreinterpret_u16_u8(t01.val[0]), vreinterpret_u16_u8(t23.val[0]));
    const uint16x4x2_t u13 = vtrn_u16(vreinterpret_u16_u8(t01.val[1]), vreinterpret_u16_u8(t23.val[1]));
    const uint16x4x2_t u46 = vtrn_u16(vreinterpret_u16_u8(t45.val[0]), vreinterpret_u16_u8(t67.val[0]));
    const uint16x4x2_t u57 = vtrn_u16(vreinterpret_u16_u8(t45.val[1]), vreinterpret_u16_u8(t67.val[1]));
    const uint32x2x2_t w04 = vtrn_u32(vreinterpret_u32_u16(u02.val[0]), vreinterpret_u32_u16(u46.val[0]));
    const uint32x2x2_t w15 = vtrn_u32(vreinterpret_u32_u16(u13.val[0]), vreinterpret_u32_u16(u57.val[0]));
    const uint32x2x2_t w26 = vtrn_u32(vreinterpret_u32_u16(u02.val[1]), vreinterpret_u32_u16(u46.val[1]));
    const uint32x2x2_t w37 = vtrn_u32(vreinterpret_u32_u16(u13.val[1]), vreinterpret_u32_u16(u57.val[1]));
    r[0] = vreinterpret_u8_u32(w04.val[0]);
    r[1] = vreinterpret_u8_u32(w15.val[0]);
    r[2] = vreinterpret_u8_u32(w26.val[0]);
    r[3] = vreinterpret_u8_u32(w37.val[0]);
    r[4] = vreinterpret_u8_u32(w04.val[1]);
    r[5] = vreinterpret_u8_u32(w15.val[1]);
    r[6] = vreinterpret_u8_u32(w26.val[1]);
    r[7] = vreinterpret_u8_u32(w37.val[1]);
}

static inline int16x8_t rshr(int16x8_t v, int rnd, int sh)
{
    return vshlq_s16(vaddq_s16(v, vdupq_n_s16((int16_t)rnd)), vdupq_n_s16((int16_t)-sh));
}

static void deblock_neon(uint8_t *s, ptrdiff_t step, ptrdiff_t lstep, int n, int group,
                         const int *alpha, const int *beta, int chroma, int type)
{
    if (type != 0 || n != 8) {
        svac3_dsp_c.deblock(s, step, lstep, n, group, alpha, beta, chroma, type);
        return;
    }
    uint8x8_t r[8];                                      /* r[k]: p3..q3 of the 8 lines */
    if (lstep == 1) {
        for (int k = 0; k < 8; k++)
            r[k] = vld1_u8(s + (k - 4) * step);
    } else {
        for (int l = 0; l < 8; l++)
            r[l] = vld1_u8(s + l * lstep - 4);
        transpose8x8(r);
    }
    int16x8_t v[8];
    for (int k = 0; k < 8; k++)
        v[k] = vreinterpretq_s16_u16(vmovl_u8(r[k]));
    const int16x8_t p3 = v[0], p2 = v[1], p1 = v[2], p0 = v[3];
    const int16x8_t q0 = v[4], q1 = v[5], q2 = v[6], q3 = v[7];

    int16_t al[8], be[8];
    for (int l = 0; l < 8; l++) {
        al[l] = (int16_t)alpha[l / group];
        be[l] = (int16_t)beta[l / group];
    }
    const int16x8_t A = vld1q_s16(al), B = vld1q_s16(be);
    const uint16x8_t on = vcgeq_s16(A, K(0));

    const int16x8_t d_p0p1 = ABD(p0, p1), d_q0q1 = ABD(q0, q1);
    const int16x8_t fL = ADD(vandq_s16(vreinterpretq_s16_u16(vcltq_s16(d_p0p1, B)), K(2)),
                             vandq_s16(vreinterpretq_s16_u16(vcltq_s16(ABD(p0, p2), B)), K(1)));
    const int16x8_t fR = ADD(vandq_s16(vreinterpretq_s16_u16(vcltq_s16(d_q0q1, B)), K(2)),
                             vandq_s16(vreinterpretq_s16_u16(vcltq_s16(ABD(q0, q2), B)), K(1)));
    const int16x8_t sum = ADD(fL, fR);

    const int16x8_t B4 = vshrq_n_s16(B, 2);
    const uint16x8_t strong = vandq_u16(vandq_u16(vcleq_s16(d_p0p1, B4), vcleq_s16(d_q0q1, B4)),
                                        vcltq_s16(ABD(p0, q0), A));
    const uint16x8_t flat = vandq_u16(vceqq_s16(p0, p1), vceqq_s16(q0, q1));
    int16x8_t bs = K(0);
    bs = SEL(vceqq_s16(sum, K(3)), vandq_s16(vreinterpretq_s16_u16(vcltq_s16(ABD(p1, q1), B)), K(1)), bs);
    bs = SEL(vceqq_s16(sum, K(4)), SEL(vceqq_s16(fL, K(2)), K(2), K(1)), bs);
    bs = SEL(vceqq_s16(sum, K(5)), SEL(flat, K(3), K(2)), bs);
    bs = SEL(vceqq_s16(sum, K(6)), SEL(strong, K(4), K(3)), bs);
    if (chroma)
        bs = vmaxq_s16(vsubq_s16(bs, K(1)), K(0));
    bs = vandq_s16(bs, vreinterpretq_s16_u16(on));

    int16x8_t np2 = p2, np1 = p1, np0 = p0, nq0 = q0, nq1 = q1, nq2 = q2;
    if (chroma) {
        const uint16x8_t any = vcgtq_s16(bs, K(0)), b3 = vceqq_s16(bs, K(3));
        np0 = SEL(any, rshr(ADD(ADD(MUL(p1, 3), MUL(p0, 10)), MUL(q0, 3)), 8, 4), p0);
        nq0 = SEL(any, rshr(ADD(ADD(MUL(p0, 3), MUL(q0, 10)), MUL(q1, 3)), 8, 4), q0);
        np1 = SEL(b3, rshr(ADD(ADD(MUL(p2, 3), MUL(p1, 8)), ADD(MUL(p0, 3), MUL(q0, 2))), 8, 4), p1);
        nq1 = SEL(b3, rshr(ADD(ADD(MUL(q2, 3), MUL(q1, 8)), ADD(MUL(q0, 3), MUL(p0, 2))), 8, 4), q1);
    } else {
        const uint16x8_t b4 = vceqq_s16(bs, K(4)), b3 = vceqq_s16(bs, K(3));
        const uint16x8_t b2 = vceqq_s16(bs, K(2)), b1 = vceqq_s16(bs, K(1));
        const int16x8_t f4p0 = rshr(ADD(ADD(ADD(MUL(p2, 3), MUL(p1, 8)), MUL(p0, 10)), ADD(MUL(q0, 8), MUL(q1, 3))), 16, 5);
        const int16x8_t f4p1 = rshr(ADD(ADD(MUL(p2, 4), MUL(p1, 5)), ADD(MUL(p0, 4), MUL(q0, 3))), 8, 4);
        const int16x8_t f4p2 = rshr(ADD(ADD(ADD(MUL(p3, 2), MUL(p2, 2)), MUL(p1, 2)), ADD(p0, q0)), 4, 3);
        const int16x8_t f4q0 = rshr(ADD(ADD(ADD(MUL(p1, 3), MUL(p0, 8)), MUL(q0, 10)), ADD(MUL(q1, 8), MUL(q2, 3))), 16, 5);
        const int16x8_t f4q1 = rshr(ADD(ADD(MUL(p0, 3), MUL(q0, 4)), ADD(MUL(q1, 5), MUL(q2, 4))), 8, 4);
        const int16x8_t f4q2 = rshr(ADD(ADD(ADD(p0, q0), MUL(q1, 2)), ADD(MUL(q2, 2), MUL(q3, 2))), 4, 3);
        const int16x8_t f3p0 = rshr(ADD(ADD(ADD(p2, MUL(p1, 4)), MUL(p0, 6)), ADD(MUL(q0, 4), q1)), 8, 4);
        const int16x8_t f3p1 = rshr(ADD(ADD(MUL(p2, 3), MUL(p1, 8)), ADD(MUL(p0, 4), q0)), 8, 4);
        const int16x8_t f3q0 = rshr(ADD(ADD(ADD(p1, MUL(p0, 4)), MUL(q0, 6)), ADD(MUL(q1, 4), q2)), 8, 4);
        const int16x8_t f3q1 = rshr(ADD(ADD(MUL(q2, 3), MUL(q1, 8)), ADD(MUL(q0, 4), p0)), 8, 4);
        const int16x8_t f2p0 = rshr(ADD(ADD(MUL(p1, 3), MUL(p0, 10)), MUL(q0, 3)), 8, 4);
        const int16x8_t f2q0 = rshr(ADD(ADD(MUL(p0, 3), MUL(q0, 10)), MUL(q1, 3)), 8, 4);
        const int16x8_t f1p0 = rshr(ADD(MUL(p0, 3), q0), 2, 2);
        const int16x8_t f1q0 = rshr(ADD(MUL(q0, 3), p0), 2, 2);
        np0 = SEL(b4, f4p0, SEL(b3, f3p0, SEL(b2, f2p0, SEL(b1, f1p0, p0))));
        nq0 = SEL(b4, f4q0, SEL(b3, f3q0, SEL(b2, f2q0, SEL(b1, f1q0, q0))));
        np1 = SEL(b4, f4p1, SEL(b3, f3p1, p1));
        nq1 = SEL(b4, f4q1, SEL(b3, f3q1, q1));
        np2 = SEL(b4, f4p2, p2);
        nq2 = SEL(b4, f4q2, q2);
    }

    const int16x8_t out[6] = { np2, np1, np0, nq0, nq1, nq2 };
    for (int k = 0; k < 6; k++)
        r[k + 1] = vqmovun_s16(out[k]);
    if (lstep == 1) {
        for (int k = 1; k < 7; k++)
            vst1_u8(s + (k - 4) * step, r[k]);
    } else {
        transpose8x8(r);                                 /* p3 and q3 go back unchanged */
        for (int l = 0; l < 8; l++)
            vst1_u8(s + l * lstep - 4, r[l]);
    }
}

const svac3_dsp_t svac3_dsp_neon = {
    .name = "neon",
    .matmul = matmul_neon,
    .add_block = add_block_neon,
    .mc_h = mc_h_neon,
    .mc_v = mc_v_neon,
    .mc_hv = mc_hv_neon,
    .deblock = deblock_neon,
};
