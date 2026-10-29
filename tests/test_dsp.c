/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright (c) 2026 OpenIPC contributors
 *
 * The SIMD kernels against the C reference on random inputs: every output
 * bit must agree. With no SIMD set on this build or CPU, passes trivially.
 * usage: test_dsp [iterations]
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "svac3_dsp.h"

static int failures;
static uint32_t seed = 12345;

static uint32_t rnd(void)
{
    seed = seed * 1664525u + 1013904223u;
    return seed >> 8;
}

static int range(int lo, int hi)
{
    return lo + (int)(rnd() % (uint32_t)(hi - lo + 1));
}

#define FAIL(...)                                                         \
    do {                                                                  \
        if (failures++ < 10)                                              \
            fprintf(stderr, __VA_ARGS__);                                 \
    } while (0)

/* the taps the decoder uses: luma 1/4, 1/2, 3/4 and chroma eighths */
static const int8_t luma_tap[3][8] = {
    { -1, 4, -10, 57, 19, -7, 3, -1 },
    { -1, 4, -11, 40, 40, -11, 4, -1 },
    { -1, 3, -7, 19, 57, -10, 4, -1 },
};
static const int8_t chroma_tap[7][4] = {
    { -4, 62, 6, 0 },  { -6, 56, 15, -1 }, { -5, 47, 25, -3 }, { -4, 36, 36, -4 },
    { -3, 25, 47, -5 }, { -1, 15, 56, -6 }, { 0, 6, 62, -4 },
};

static void test_matmul(const svac3_dsp_t *s)
{
    static int16_t a[64 * 64], b[64 * 64], o1[64 * 64], o2[64 * 64];
    static const int sizes[] = { 4, 8, 16, 32, 64 };
    const int rows = sizes[range(0, 4)], depth = range(1, 64);
    const int cols = range(0, 4) == 0 ? 4 : 8 * range(1, 8);
    /* one side transform-sized (|v| <= 127), the other up to 16 bits, as
     * the inverse transform's two passes have them */
    const int big_a = (int)(rnd() & 1);
    for (int i = 0; i < rows * depth; i++)
        a[i] = (int16_t)(big_a ? range(-32768, 32767) : range(-127, 127));
    for (int i = 0; i < depth * cols; i++)
        b[i] = (int16_t)(big_a ? range(-127, 127) : range(-32768, 32767));
    const int shift = range(0, 1) ? 5 : 12;
    const int lo = range(0, 1) ? -32768 : -256, hi = lo == -32768 ? 32767 : 255;
    memset(o1, 0x55, sizeof(o1));
    memset(o2, 0x55, sizeof(o2));
    svac3_dsp_c.matmul(o1, 64, a, depth, b, cols, rows, depth, cols, shift, lo, hi);
    s->matmul(o2, 64, a, depth, b, cols, rows, depth, cols, shift, lo, hi);
    if (memcmp(o1, o2, sizeof(o1)))
        FAIL("matmul %dx%dx%d shift %d differs\n", rows, depth, cols, shift);
}

static void test_add_block(const svac3_dsp_t *s)
{
    static uint8_t pred[64 * 64], d1[64 * 80], d2[64 * 80];
    static int16_t res[64 * 64];
    static const int widths[] = { 2, 4, 8, 12, 16, 24, 32, 48, 64 };
    const int w = widths[range(0, 8)], h = range(1, 64), stride = 80;
    for (int i = 0; i < w * h; i++) {
        pred[i] = (uint8_t)rnd();
        res[i] = (int16_t)range(-256, 255);
    }
    for (int i = 0; i < (int)sizeof(d1); i++)
        d1[i] = d2[i] = (uint8_t)rnd();
    svac3_dsp_c.add_block(d1, stride, pred, res, w, h);
    s->add_block(d2, stride, pred, res, w, h);
    if (memcmp(d1, d2, sizeof(d1)))
        FAIL("add_block %dx%d differs\n", w, h);
}

static void test_mc(const svac3_dsp_t *s)
{
    /* a plane with room for the taps on every side */
    enum { STRIDE = 96, ROWS = 80 };
    static uint8_t plane[STRIDE * ROWS], p1[64 * 64 + 16], p2[64 * 64 + 16];
    static const int widths[] = { 2, 4, 8, 12, 16, 24, 32, 64 };
    for (int i = 0; i < STRIDE * ROWS; i++)
        plane[i] = (uint8_t)(range(0, 3) ? rnd() : (rnd() & 1) * 255);   /* extremes too */
    const int luma = (int)(rnd() & 1), ntap = luma ? 8 : 4;
    const int w = widths[range(0, 7)], h = range(1, 64);
    const int8_t *th = luma ? luma_tap[range(0, 2)] : chroma_tap[range(0, 6)];
    const int8_t *tv = luma ? luma_tap[range(0, 2)] : chroma_tap[range(0, 6)];
    const uint8_t *src = plane + 8 * STRIDE + 8;
    const int which = range(0, 2);
    memset(p1, 0x55, sizeof(p1));
    memset(p2, 0x55, sizeof(p2));
    if (which == 0) {
        svac3_dsp_c.mc_h(p1, src, STRIDE, w, h, th, ntap);
        s->mc_h(p2, src, STRIDE, w, h, th, ntap);
    } else if (which == 1) {
        svac3_dsp_c.mc_v(p1, src, STRIDE, w, h, tv, ntap);
        s->mc_v(p2, src, STRIDE, w, h, tv, ntap);
    } else {
        svac3_dsp_c.mc_hv(p1, src, STRIDE, w, h, th, tv, ntap);
        s->mc_hv(p2, src, STRIDE, w, h, th, tv, ntap);
    }
    if (memcmp(p1, p2, sizeof(p1)))
        FAIL("mc_%s %d-tap %dx%d differs\n", which == 0 ? "h" : which == 1 ? "v" : "hv",
             ntap, w, h);
}

static void test_deblock(const svac3_dsp_t *s)
{
    /* 8 lines across one edge in a 16 x 16 window, edge at the middle */
    enum { S = 16 };
    static uint8_t w1[S * S], w2[S * S];
    const int base = range(0, 255), spread = range(0, 3) ? range(0, 8) : 255;
    for (int i = 0; i < S * S; i++) {
        const int v = base + range(-spread, spread);
        w1[i] = (uint8_t)(v < 0 ? 0 : v > 255 ? 255 : v);
    }
    if (rnd() & 1)                     /* a step across the edge, as at a block boundary */
        for (int y = 0; y < S; y++)
            for (int x = S / 2; x < S; x++)
                w1[y * S + x] = (uint8_t)((w1[y * S + x] + range(0, 40)) & 255);
    memcpy(w2, w1, sizeof(w1));
    const int ver = (int)(rnd() & 1), chroma = (int)(rnd() & 1);
    const int group = chroma ? 2 : 4, n = range(0, 4) ? 8 : 4, type = range(0, 5) ? 0 : 1;
    int alpha[4], beta[4];
    for (int g = 0; g < 4; g++) {
        alpha[g] = range(0, 4) ? range(0, 59) : -1;
        beta[g] = range(0, 27);
    }
    const ptrdiff_t step = ver ? 1 : S, lstep = ver ? S : 1;
    uint8_t *q0a = w1 + (ver ? 4 * S + S / 2 : (S / 2) * S + 4);
    uint8_t *q0b = w2 + (ver ? 4 * S + S / 2 : (S / 2) * S + 4);
    svac3_dsp_c.deblock(q0a, step, lstep, n, group, alpha, beta, chroma, type);
    s->deblock(q0b, step, lstep, n, group, alpha, beta, chroma, type);
    if (memcmp(w1, w2, sizeof(w1)))
        FAIL("deblock %s %s n=%d type %d differs\n", ver ? "vertical" : "horizontal",
             chroma ? "chroma" : "luma", n, type);
}

int main(int argc, char **argv)
{
    const int iters = argc > 1 ? atoi(argv[1]) : 20000;
    const svac3_dsp_t *s = svac3_dsp_simd();
    if (!s) {
        printf("no SIMD kernels on this build or CPU: nothing to compare\n");
        return 0;
    }
    for (int i = 0; i < iters; i++) {
        test_matmul(s);
        test_add_block(s);
        test_mc(s);
        test_deblock(s);
    }
    if (failures)
        fprintf(stderr, "%s: %d mismatch(es)\n", s->name, failures);
    else
        printf("%s: %d x 4 kernel groups agree with C\n", s->name, iters);
    return failures ? 1 : 0;
}
