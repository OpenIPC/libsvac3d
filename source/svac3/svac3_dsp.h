/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright (c) 2026 OpenIPC contributors
 *
 * The decoder's sample-processing kernels, as a table with a C reference
 * and SIMD versions (AVX2 on x86-64, chosen when the CPU has it; NEON on
 * AArch64). Every version gives the same output bits as the C one;
 * tests/test_dsp.c checks that on random inputs.
 */
#ifndef SVAC3_DSP_H
#define SVAC3_DSP_H

#include <stddef.h>
#include <stdint.h>

typedef struct {
    const char *name;

    /* out[r][c] = Clip3(lo, hi, (sum over k < depth of a[r][k] * b[k][c]
     *              + (1 << (shift - 1))) >> shift), for r < rows, c < cols.
     * The sums must fit in 32 bits; cols is 4 or a multiple of 8. */
    void (*matmul)(int16_t *out, int ostride, const int16_t *a, int astride,
                   const int16_t *b, int bstride, int rows, int depth, int cols,
                   int shift, int lo, int hi);

    /* dst = Clip1(pred + resid) over a w x h block, pred and resid packed */
    void (*add_block)(uint8_t *dst, int stride, const uint8_t *pred, const int16_t *resid,
                      int w, int h);

    /* Fractional-sample interpolation with ntap (8 or 4) taps, every source
     * sample inside the plane: src is the block's integer position, and
     * the taps reach ntap / 2 - 1 samples before it and ntap / 2 after.
     * One-dimensional positions round (sum + 32) >> 6; mc_hv keeps the
     * horizontal sums unrounded and rounds (sum + 2048) >> 12. Output is
     * packed w x h. */
    void (*mc_h)(uint8_t *dst, const uint8_t *src, int stride, int w, int h,
                 const int8_t *taps, int ntap);
    void (*mc_v)(uint8_t *dst, const uint8_t *src, int stride, int w, int h,
                 const int8_t *taps, int ntap);
    void (*mc_hv)(uint8_t *dst, const uint8_t *src, int stride, int w, int h,
                  const int8_t *th, const int8_t *tv, int ntap);

    /* Deblock n lines across one edge (5.4.11.4-10): s points at q0 of the
     * first line, step is the distance from p0 to q0 and lstep from one
     * line to the next. Lines come in groups of group lines sharing alpha
     * and beta (alpha[l / group]); a group with alpha < 0 is left alone.
     * chroma selects the chroma filters, type is DeblockingFilterType. */
    void (*deblock)(uint8_t *s, ptrdiff_t step, ptrdiff_t lstep, int n, int group,
                    const int *alpha, const int *beta, int chroma, int type);

    /* One row of angular intra prediction: dst[x] = Clip1((ref[x] * t[0] +
     * ref[x + 1] * t[1] + ref[x + 2] * t[2] + ref[x + 3] * t[3] + rnd) >>
     * shift) for x < w, ref holding 8-bit samples (5.4.8.1.4.2) */
    void (*ipred_row)(uint8_t *dst, const int16_t *ref, int w, const int8_t *t, int rnd, int shift);

    /* SAO on a w x h region whose neighbours all lie in the picture, src the
     * deblocked samples (same stride as dst). Edge mode: the class of each
     * sample from its neighbours at src + na and src + nb, offset[class + 2]
     * for class -2..2 (offset[2] = 0). Band mode: band[sample >> 3]. */
    void (*sao_edge)(uint8_t *dst, const uint8_t *src, ptrdiff_t stride, int w, int h,
                     ptrdiff_t na, ptrdiff_t nb, const int8_t offset[5]);
    void (*sao_band)(uint8_t *dst, const uint8_t *src, ptrdiff_t stride, int w, int h,
                     const int8_t band[32]);
} svac3_dsp_t;

/* The kernels to use on this CPU */
const svac3_dsp_t *svac3_dsp(void);

/* The C reference, and the SIMD set this build has (NULL if none, or if
 * the CPU lacks it) */
extern const svac3_dsp_t svac3_dsp_c;
const svac3_dsp_t *svac3_dsp_simd(void);

#endif /* SVAC3_DSP_H */
