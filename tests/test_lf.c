/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright (c) 2026 OpenIPC contributors
 *
 * Deblocking a picture in consecutive row ranges, as the wavefront does,
 * against deblocking it whole: random frames and random unit information,
 * ranges of every LCU height (and the odd multiples of 8 in between).
 * usage: test_lf [iterations]
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "svac3_loopfilter.h"

static uint32_t seed = 777;
static uint32_t rnd(void)
{
    seed = seed * 1664525u + 1013904223u;
    return seed >> 8;
}

enum { W = 128, H = 96, UW = W / 4, UH = H / 4 };

static void make_frame(svac3_frame_t *f, uint8_t *buf)
{
    f->plane[0] = buf;
    f->plane[1] = buf + W * H;
    f->plane[2] = f->plane[1] + W * H / 4;
    f->stride[0] = f->width[0] = W;
    f->stride[1] = f->stride[2] = f->width[1] = f->width[2] = W / 2;
    f->height[0] = H;
    f->height[1] = f->height[2] = H / 2;
}

int main(int argc, char **argv)
{
    const int iters = argc > 1 ? atoi(argv[1]) : 300;
    static uint8_t a[W * H * 3 / 2], b[W * H * 3 / 2];
    static uint32_t id[UW * UH];
    static uint8_t qp[UW * UH], cbf[UW * UH];
    static int8_t ref[UW * UH];
    static int16_t mv[UW * UH * 2];
    static const int steps[] = { 8, 16, 24, 32, 64 };
    int failures = 0;

    for (int it = 0; it < iters; it++) {
        /* smooth content with steps at block edges, so filters fire */
        const int base = (int)(rnd() % 200);
        for (int i = 0; i < W * H * 3 / 2; i++)
            a[i] = (uint8_t)(base + (int)(rnd() % 6) + ((i / 8) % 2) * (int)(rnd() % 12));
        /* coding units of 4..32 samples, as random rectangles of ids */
        for (int y = 0; y < UH; y++)
            for (int x = 0; x < UW; x++) {
                const int bw = 1 << (rnd() % 3), bh = 1 << (rnd() % 3);
                id[y * UW + x] = (uint32_t)((y / bh) * 1000 + x / bw + it * 100000);
                qp[y * UW + x] = (uint8_t)(20 + rnd() % 40);
                cbf[y * UW + x] = (uint8_t)(rnd() & 1);
                ref[y * UW + x] = (int8_t)((int)(rnd() % 3) - 1);
                mv[2 * (y * UW + x)] = (int16_t)((int)(rnd() % 9) - 4);
                mv[2 * (y * UW + x) + 1] = (int16_t)((int)(rnd() % 9) - 4);
            }
        svac3_dbk_info_t d = {
            .pic_w = W, .pic_h = H, .stride = UW, .cu_id = id, .qp = qp,
            .inter = (int)(rnd() & 1), .cbf = cbf, .ref = ref, .mv = mv,
            .alpha_offset = (int)(rnd() % 9) - 4, .beta_offset = (int)(rnd() % 9) - 4,
            .filter_type = (int)(rnd() % 4 == 0), .cb_delta = (int)(rnd() % 5) - 2,
            .cr_delta = (int)(rnd() % 5) - 2,
        };
        svac3_frame_t fa, fb;
        memcpy(b, a, sizeof(a));
        make_frame(&fa, a);
        make_frame(&fb, b);
        svac3_deblock(&fa, &d, NULL);
        const int step = steps[rnd() % 5];
        for (int y = 0; y < H; y += step)
            svac3_deblock_rows(&fb, &d, y, y + step, NULL);
        if (memcmp(a, b, sizeof(a))) {
            if (failures++ < 5)
                fprintf(stderr, "iteration %d: rows of %d differ from the whole picture\n", it, step);
        }
    }
    if (failures)
        fprintf(stderr, "%d mismatch(es)\n", failures);
    else
        printf("%d pictures deblocked in row ranges agree with whole-picture deblocking\n", iters);
    return failures ? 1 : 0;
}
