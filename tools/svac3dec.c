/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright (c) 2026 OpenIPC contributors
 *
 * svac3dec: decode an SVAC3 elementary stream to raw I420 (one frame per
 * picture, cropped to the display size). Only I pictures are reconstructed
 * so far; other pictures are written grey.
 * usage: svac3dec stream.svac3 out.yuv
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "svac3_hls.h"
#include "svac3_patch.h"

int main(int argc, char **argv)
{
    if (argc != 3) {
        fprintf(stderr, "usage: %s stream.svac3 out.yuv\n", argv[0]);
        return 2;
    }
    FILE *f = fopen(argv[1], "rb"), *o = fopen(argv[2], "wb");
    if (!f || !o) {
        perror(!f ? argv[1] : argv[2]);
        return 1;
    }
    fseek(f, 0, SEEK_END);
    long len = ftell(f);
    fseek(f, 0, SEEK_SET);
    uint8_t *buf = malloc(len > 0 ? (size_t)len : 1), *rbsp = malloc(len > 0 ? (size_t)len : 1);
    svac3_hls_t *hls = calloc(1, sizeof(*hls));
    svac3_frame_t fr;
    uint8_t *planes = NULL;
    size_t planes_size = 0;
    int rc = 1, pics = 0;

    memset(&fr, 0, sizeof(fr));
    if (!buf || !rbsp || !hls || fread(buf, 1, (size_t)len, f) != (size_t)len)
        goto out;

    size_t pos = 0, nal_len;
    const uint8_t *nal;
    while (svac3_next_nal(buf, (size_t)len, &pos, &nal, &nal_len)) {
        if (nal_len < 2)
            continue;
        if (svac3_parse_nal(hls, nal, nal_len, 0, NULL))
            goto out;
        if (!svac3_nal_is_patch(nal[1] & 31u))
            continue;

        const int W = hls->sps.pic_width_in_luma, H = hls->sps.pic_height_in_luma;
        const size_t need = (size_t)W * H * 3 / 2;
        if (need != planes_size) {
            free(planes);
            planes = malloc(need);
            if (!planes)
                goto out;
            planes_size = need;
        }
        memset(planes, 128, need);
        fr.plane[0] = planes;
        fr.plane[1] = planes + (size_t)W * H;
        fr.plane[2] = fr.plane[1] + (size_t)W * H / 4;
        fr.stride[0] = fr.width[0] = W;
        fr.stride[1] = fr.stride[2] = fr.width[1] = fr.width[2] = W / 2;
        fr.height[0] = H;
        fr.height[1] = fr.height[2] = H / 2;

        svac3_patch_stats_t st;
        size_t rl = svac3_unescape(nal + 2, nal_len - 2, rbsp);
        int ret = svac3_patch_decode(hls, rbsp, rl, &fr, &st);
        if (ret) {
            fprintf(stderr, "picture %d: decode error %d\n", pics, ret);
            goto out;
        }
        /* one patch per picture is all that is supported: write it out */
        const int dw = hls->sps.horizontal_size, dh = hls->sps.vertical_size;
        for (int c = 0; c < 3; c++) {
            const int cw = c ? dw / 2 : dw, ch = c ? dh / 2 : dh;
            for (int y = 0; y < ch; y++)
                fwrite(fr.plane[c] + (size_t)y * fr.stride[c], 1, (size_t)cw, o);
        }
        pics++;
    }
    fprintf(stderr, "%d pictures\n", pics);
    rc = 0;
out:
    fclose(f);
    fclose(o);
    free(planes);
    free(hls);
    free(rbsp);
    free(buf);
    return rc;
}
