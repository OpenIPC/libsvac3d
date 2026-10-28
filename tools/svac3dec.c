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

#include "read_file.h"
#include "svac3_hls.h"
#include "svac3_patch.h"

int main(int argc, char **argv)
{
    if (argc != 3) {
        fprintf(stderr, "usage: %s stream.svac3 out.yuv\n", argv[0]);
        return 2;
    }
    size_t len;
    uint8_t *buf = read_file(argv[1], &len);
    if (!buf)
        return 1;
    FILE *o = fopen(argv[2], "wb");
    if (!o) {
        perror(argv[2]);
        free(buf);
        return 1;
    }
    uint8_t *rbsp = malloc(len ? len : 1);
    svac3_hls_t *hls = calloc(1, sizeof(*hls));
    svac3_frame_t fr;
    uint8_t *planes = NULL;
    size_t planes_size = 0;
    int rc = 1, pics = 0;

    memset(&fr, 0, sizeof(fr));
    if (!rbsp || !hls)
        goto out;

    size_t pos = 0, nal_len;
    const uint8_t *nal;
    unsigned idx = 0;
    while (svac3_next_nal(buf, len, &pos, &nal, &nal_len)) {
        const unsigned this_idx = idx++;
        if (nal_len < 2)
            continue;
        int ret = svac3_parse_nal(hls, nal, nal_len, this_idx, NULL);
        if (ret) {
            fprintf(stderr, "NAL unit %u: header error %d\n", this_idx, ret);
            goto out;
        }
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
        ret = svac3_patch_decode(hls, rbsp, rl, &fr, &st);
        if (ret == SVAC3_ERR_CORRUPT) {
            fprintf(stderr, "picture %d: patch data out of sync from LCU %u, not written\n",
                    pics, st.first_bad_lcu);
            goto out;
        }
        if (ret) {
            fprintf(stderr, "picture %d: decode error %d\n", pics, ret);
            goto out;
        }
        /* one patch per picture is all that is supported: write it out. The
         * SPS parser guarantees the display size lies within the coded size. */
        const int dw = hls->sps.horizontal_size, dh = hls->sps.vertical_size;
        for (int c = 0; c < 3; c++) {
            const int cw = c ? (dw + 1) / 2 : dw, ch = c ? (dh + 1) / 2 : dh;
            for (int y = 0; y < ch; y++)
                if (fwrite(fr.plane[c] + (size_t)y * fr.stride[c], 1, (size_t)cw, o) != (size_t)cw) {
                    perror(argv[2]);
                    goto out;
                }
        }
        pics++;
    }
    fprintf(stderr, "%d pictures\n", pics);
    rc = 0;
out:
    if (fclose(o) != 0)
        rc = 1;
    free(planes);
    free(hls);
    free(rbsp);
    free(buf);
    return rc;
}
