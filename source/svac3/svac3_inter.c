/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright (c) 2026 OpenIPC contributors
 *
 * SVAC3 luma and chroma interpolation, 8-bit, written from 5.4.9.2.6.2 and
 * 5.4.9.2.7.2.
 */
#include "svac3_inter.h"

static inline int clip3(int lo, int hi, int v) { return v < lo ? lo : v > hi ? hi : v; }
static inline int clip1(int v) { return clip3(0, 255, v); }

/* 5.4.9.2.6.2: taps on A[-3] .. A[4] for the 1/4, 1/2 and 3/4 positions */
static const int8_t luma_tap[4][8] = {
    { 0, 0, 0, 64, 0, 0, 0, 0 },
    { -1, 4, -10, 57, 19, -7, 3, -1 },
    { -1, 4, -11, 40, 40, -11, 4, -1 },
    { -1, 3, -7, 19, 57, -10, 4, -1 },
};

/* 表130: taps on A[-1] .. A[2] for the eighth-sample positions */
static const int8_t chroma_tap[8][4] = {
    { 0, 64, 0, 0 },
    { -4, 62, 6, 0 },
    { -6, 56, 15, -1 },
    { -5, 47, 25, -3 },
    { -4, 36, 36, -4 },
    { -3, 25, 47, -5 },
    { -1, 15, 56, -6 },
    { 0, 6, 62, -4 },
};

static inline int px(const svac3_frame_t *f, int c, int x, int y)
{
    x = clip3(0, f->width[c] - 1, x);
    y = clip3(0, f->height[c] - 1, y);
    return f->plane[c][y * f->stride[c] + x];
}

/* Separable interpolation with ntap taps and the rounding of the draft:
 * one-dimensional positions are (sum + 32) >> 6; two-dimensional ones keep
 * the horizontal sums unrounded (>> (BitDepth - 8) = 0) and round the
 * vertical pass with (1 << 11) >> 12. w and h are at most 64. */
static void interp(const svac3_frame_t *ref, int c, int ix, int iy, int fx, int fy,
                   int w, int h, const int8_t *th, const int8_t *tv, int ntap, int lead,
                   uint8_t *pred)
{
    if (!fx && !fy) {
        for (int y = 0; y < h; y++)
            for (int x = 0; x < w; x++)
                pred[y * w + x] = (uint8_t)px(ref, c, ix + x, iy + y);
    } else if (!fy) {
        for (int y = 0; y < h; y++)
            for (int x = 0; x < w; x++) {
                int s = 0;
                for (int k = 0; k < ntap; k++)
                    s += th[k] * px(ref, c, ix + x + k - lead, iy + y);
                pred[y * w + x] = (uint8_t)clip1((s + 32) >> 6);
            }
    } else if (!fx) {
        for (int y = 0; y < h; y++)
            for (int x = 0; x < w; x++) {
                int s = 0;
                for (int k = 0; k < ntap; k++)
                    s += tv[k] * px(ref, c, ix + x, iy + y + k - lead);
                pred[y * w + x] = (uint8_t)clip1((s + 32) >> 6);
            }
    } else {
        /* horizontal sums of the h + ntap - 1 rows the vertical taps reach */
        int32_t tmp[(64 + 7) * 64];
        for (int y = 0; y < h + ntap - 1; y++)
            for (int x = 0; x < w; x++) {
                int s = 0;
                for (int k = 0; k < ntap; k++)
                    s += th[k] * px(ref, c, ix + x + k - lead, iy + y - lead);
                tmp[y * w + x] = s;
            }
        for (int y = 0; y < h; y++)
            for (int x = 0; x < w; x++) {
                int s = 0;
                for (int k = 0; k < ntap; k++)
                    s += tv[k] * tmp[(y + k) * w + x];
                pred[y * w + x] = (uint8_t)clip1((s + (1 << 11)) >> 12);
            }
    }
}

void svac3_mc_luma(const svac3_frame_t *ref, int x0, int y0, int w, int h,
                   int mvx, int mvy, uint8_t *pred)
{
    const int xp = (x0 << 2) + mvx, yp = (y0 << 2) + mvy;
    /* arithmetic shift of a possibly negative position: floor division */
    const int ix = xp >> 2, iy = yp >> 2, fx = xp & 3, fy = yp & 3;
    interp(ref, 0, ix, iy, fx, fy, w, h, luma_tap[fx], luma_tap[fy], 8, 3, pred);
}

void svac3_mc_chroma(const svac3_frame_t *ref, int c, int cx0, int cy0, int w, int h,
                     int mvx, int mvy, uint8_t *pred)
{
    /* (xE + 2x) << 2 + mv in luma quarter samples is chroma eighth samples */
    const int xp = ((cx0 * 2) << 2) + mvx, yp = ((cy0 * 2) << 2) + mvy;
    const int ix = xp >> 3, iy = yp >> 3, fx = xp & 7, fy = yp & 7;
    interp(ref, c, ix, iy, fx, fy, w, h, chroma_tap[fx], chroma_tap[fy], 4, 1, pred);
}
