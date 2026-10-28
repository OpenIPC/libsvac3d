/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright (c) 2026 OpenIPC contributors
 *
 * svac3dec: decode an SVAC3 elementary stream to raw I420 (one frame per
 * picture, cropped to the display size).
 * usage: svac3dec stream.svac3 out.yuv
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "read_file.h"
#include "svac3_dec.h"

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
    svac3_dec_t *dec = svac3_dec_create();
    int rc = 1, pics = 0;

    if (!dec)
        goto out;

    size_t pos = 0, nal_len;
    const uint8_t *nal;
    unsigned idx = 0;
    while (svac3_next_nal(buf, len, &pos, &nal, &nal_len)) {
        const unsigned this_idx = idx++;
        if (nal_len < 2)
            continue;
        const svac3_frame_t *fr;
        int ret = svac3_dec_nal(dec, nal, nal_len, &fr);
        if (ret == SVAC3_ERR_CORRUPT && svac3_nal_is_patch(nal[1] & 31u)) {
            fprintf(stderr, "picture %d: patch data out of sync from LCU %u, or reference missing\n",
                    pics, svac3_dec_stats(dec)->first_bad_lcu);
            goto out;
        }
        if (ret) {
            fprintf(stderr, "NAL unit %u: error %d\n", this_idx, ret);
            goto out;
        }
        if (!fr)
            continue;
        /* one patch per picture is all that is supported: write it out. The
         * SPS parser guarantees the display size lies within the coded size. */
        const svac3_sps_t *sps = &svac3_dec_hls(dec)->sps;
        const int dw = sps->horizontal_size, dh = sps->vertical_size;
        for (int c = 0; c < 3; c++) {
            const int cw = c ? (dw + 1) / 2 : dw, ch = c ? (dh + 1) / 2 : dh;
            for (int y = 0; y < ch; y++)
                if (fwrite(fr->plane[c] + (size_t)y * fr->stride[c], 1, (size_t)cw, o) != (size_t)cw) {
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
    svac3_dec_destroy(dec);
    free(buf);
    return rc;
}
