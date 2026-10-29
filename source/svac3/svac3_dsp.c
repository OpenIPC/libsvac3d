/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright (c) 2026 OpenIPC contributors
 *
 * C reference kernels and the choice of SIMD set.
 */
#include "svac3_dsp.h"

#include <stddef.h>

static inline int clip3(int lo, int hi, int v) { return v < lo ? lo : v > hi ? hi : v; }
static inline int clip1(int v) { return clip3(0, 255, v); }
static inline int iabs(int v) { return v < 0 ? -v : v; }

static void matmul_c(int16_t *out, int ostride, const int16_t *a, int astride,
                     const int16_t *b, int bstride, int rows, int depth, int cols,
                     int shift, int lo, int hi)
{
    const int32_t rnd = 1 << (shift - 1);
    for (int r = 0; r < rows; r++)
        for (int c = 0; c < cols; c++) {
            int32_t s = 0;
            for (int k = 0; k < depth; k++)
                s += (int32_t)a[r * astride + k] * b[k * bstride + c];
            out[r * ostride + c] = (int16_t)clip3(lo, hi, (s + rnd) >> shift);
        }
}

static void add_block_c(uint8_t *dst, int stride, const uint8_t *pred, const int16_t *resid,
                        int w, int h)
{
    for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++)
            dst[y * stride + x] = (uint8_t)clip1(pred[y * w + x] + resid[y * w + x]);
}

static void mc_h_c(uint8_t *dst, const uint8_t *src, int stride, int w, int h,
                   const int8_t *t, int ntap)
{
    const uint8_t *s0 = src - (ntap / 2 - 1);
    for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++) {
            int s = 0;
            for (int k = 0; k < ntap; k++)
                s += t[k] * s0[y * stride + x + k];
            dst[y * w + x] = (uint8_t)clip1((s + 32) >> 6);
        }
}

static void mc_v_c(uint8_t *dst, const uint8_t *src, int stride, int w, int h,
                   const int8_t *t, int ntap)
{
    const uint8_t *s0 = src - (ntap / 2 - 1) * stride;
    for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++) {
            int s = 0;
            for (int k = 0; k < ntap; k++)
                s += t[k] * s0[(y + k) * stride + x];
            dst[y * w + x] = (uint8_t)clip1((s + 32) >> 6);
        }
}

static void mc_hv_c(uint8_t *dst, const uint8_t *src, int stride, int w, int h,
                    const int8_t *th, const int8_t *tv, int ntap)
{
    int16_t tmp[(64 + 7) * 64];
    const int lead = ntap / 2 - 1;
    const uint8_t *s0 = src - lead * stride - lead;
    for (int y = 0; y < h + ntap - 1; y++)
        for (int x = 0; x < w; x++) {
            int s = 0;
            for (int k = 0; k < ntap; k++)
                s += th[k] * s0[y * stride + x + k];
            tmp[y * w + x] = (int16_t)s;     /* within +-2^15: |taps| sum to under 128 */
        }
    for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++) {
            int s = 0;
            for (int k = 0; k < ntap; k++)
                s += tv[k] * tmp[(y + k) * w + x];
            dst[y * w + x] = (uint8_t)clip1((s + (1 << 11)) >> 12);
        }
}

/* Deblocking of one line, 5.4.11.4 (Bs from the samples) and the filters
 * of 5.4.11.5-5.4.11.10: s points at q0, step is the distance from p0 to
 * q0. */
static void deblock_line(uint8_t *s, ptrdiff_t step, int alpha, int beta, int chroma, int type)
{
    const int p0 = s[-step], p1 = s[-2 * step], p2 = s[-3 * step];
    const int q0 = s[0], q1 = s[step], q2 = s[2 * step];
    int fL = 0, fR = 0, bs;

    if (type == 1 && iabs(p0 - q0) >= 4)
        return;
    if (iabs(p0 - p1) < beta)
        fL += 2;
    if (iabs(p0 - p2) < beta)
        fL++;
    if (iabs(q0 - q1) < beta)
        fR += 2;
    if (iabs(q0 - q2) < beta)
        fR++;
    switch (fL + fR) {
    case 6:
        bs = (iabs(p0 - p1) <= beta / 4 && iabs(q0 - q1) <= beta / 4 && iabs(p0 - q0) < alpha) ? 4 : 3;
        if (type == 1 && bs == 4 &&
            !(iabs(p0 - s[-4 * step]) <= beta / 2 && iabs(q0 - s[3 * step]) <= beta / 2))
            bs = 3;
        break;
    case 5:
        bs = (p0 == p1 && q0 == q1) ? 3 : 2;
        if (type == 1 && bs == 3 && !(iabs(p2 - q2) < alpha))
            bs = 2;
        break;
    case 4:
        bs = fL == 2 ? 2 : 1;
        break;
    case 3:
        bs = iabs(p1 - q1) < beta ? 1 : 0;
        break;
    default:
        bs = 0;
        break;
    }
    if (chroma && bs)
        bs--;
    if (!bs)
        return;

    if (chroma) {
        s[-step] = (uint8_t)((p1 * 3 + p0 * 10 + q0 * 3 + 8) >> 4);
        s[0] = (uint8_t)((p0 * 3 + q0 * 10 + q1 * 3 + 8) >> 4);
        if (bs == 3) {
            s[-2 * step] = (uint8_t)((p2 * 3 + p1 * 8 + p0 * 3 + q0 * 2 + 8) >> 4);
            s[step] = (uint8_t)((q2 * 3 + q1 * 8 + q0 * 3 + p0 * 2 + 8) >> 4);
        }
        return;
    }
    const int p3 = s[-4 * step], q3 = s[3 * step];
    switch (bs) {
    case 4:
        s[-step]     = (uint8_t)((p2 * 3 + p1 * 8 + p0 * 10 + q0 * 8 + q1 * 3 + 16) >> 5);
        s[-2 * step] = (uint8_t)((p2 * 4 + p1 * 5 + p0 * 4 + q0 * 3 + 8) >> 4);
        s[-3 * step] = (uint8_t)((p3 * 2 + p2 * 2 + p1 * 2 + p0 + q0 + 4) >> 3);
        s[0]         = (uint8_t)((p1 * 3 + p0 * 8 + q0 * 10 + q1 * 8 + q2 * 3 + 16) >> 5);
        s[step]      = (uint8_t)((p0 * 3 + q0 * 4 + q1 * 5 + q2 * 4 + 8) >> 4);
        s[2 * step]  = (uint8_t)((p0 + q0 + q1 * 2 + q2 * 2 + q3 * 2 + 4) >> 3);
        break;
    case 3:
        s[-step]     = (uint8_t)((p2 + (p1 << 2) + (p0 << 2) + (p0 << 1) + (q0 << 2) + q1 + 8) >> 4);
        s[-2 * step] = (uint8_t)(((p2 << 1) + p2 + (p1 << 3) + (p0 << 2) + q0 + 8) >> 4);
        s[0]         = (uint8_t)((p1 + (p0 << 2) + (q0 << 2) + (q0 << 1) + (q1 << 2) + q2 + 8) >> 4);
        s[step]      = (uint8_t)(((q2 << 1) + q2 + (q1 << 3) + (q0 << 2) + p0 + 8) >> 4);
        break;
    case 2:
        s[-step] = (uint8_t)(((p1 << 1) + p1 + (p0 << 3) + (p0 << 1) + (q0 << 1) + q0 + 8) >> 4);
        s[0]     = (uint8_t)(((p0 << 1) + p0 + (q0 << 3) + (q0 << 1) + (q1 << 1) + q1 + 8) >> 4);
        break;
    case 1:
        s[-step] = (uint8_t)(((p0 << 1) + p0 + q0 + 2) >> 2);
        s[0]     = (uint8_t)(((q0 << 1) + q0 + p0 + 2) >> 2);
        break;
    }
}

static void deblock_c(uint8_t *s, ptrdiff_t step, ptrdiff_t lstep, int n, int group,
                      const int *alpha, const int *beta, int chroma, int type)
{
    for (int l = 0; l < n; l++)
        if (alpha[l / group] >= 0)
            deblock_line(s + l * lstep, step, alpha[l / group], beta[l / group], chroma, type);
}

const svac3_dsp_t svac3_dsp_c = {
    .name = "c",
    .matmul = matmul_c,
    .add_block = add_block_c,
    .mc_h = mc_h_c,
    .mc_v = mc_v_c,
    .mc_hv = mc_hv_c,
    .deblock = deblock_c,
};

#if defined(SVAC3_HAVE_AVX2)
extern const svac3_dsp_t svac3_dsp_avx2;
const svac3_dsp_t *svac3_dsp_simd(void)
{
    return __builtin_cpu_supports("avx2") ? &svac3_dsp_avx2 : NULL;
}
#elif defined(SVAC3_HAVE_NEON)
extern const svac3_dsp_t svac3_dsp_neon;
const svac3_dsp_t *svac3_dsp_simd(void)
{
    return &svac3_dsp_neon;               /* every AArch64 CPU has it */
}
#else
const svac3_dsp_t *svac3_dsp_simd(void)
{
    return NULL;
}
#endif

const svac3_dsp_t *svac3_dsp(void)
{
    const svac3_dsp_t *d = svac3_dsp_simd();
    return d ? d : &svac3_dsp_c;
}
