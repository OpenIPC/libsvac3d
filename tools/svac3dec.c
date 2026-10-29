/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright (c) 2026 OpenIPC contributors
 *
 * svac3dec: decode an SVAC3 elementary stream to raw I420 (one frame per
 * picture, cropped to the display size).
 * usage: svac3dec [-t threads] stream.svac3 out.yuv
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "read_file.h"
#include "svac3_dec.h"

typedef struct {
    FILE *o;
    const char *name;
    int pics, err;
} sink_t;

/* one picture, cropped to its display size; the SPS parser guarantees
 * that lies within the coded size */
static int write_pic(sink_t *s, const svac3_frame_t *fr, int dw, int dh)
{
    for (int c = 0; c < 3; c++) {
        const int cw = c ? (dw + 1) / 2 : dw, ch = c ? (dh + 1) / 2 : dh;
        for (int y = 0; y < ch; y++)
            if (fwrite(fr->plane[c] + (size_t)y * fr->stride[c], 1, (size_t)cw, s->o) != (size_t)cw) {
                perror(s->name);
                return -1;
            }
    }
    s->pics++;
    return 0;
}

/* from picture threads: stop at the first picture in error */
static void on_picture(void *ctx, const svac3_dec_pic_t *p)
{
    sink_t *s = ctx;
    if (s->err)
        return;
    if (p->err) {
        fprintf(stderr, "picture %d: error %d\n", s->pics, p->err);
        s->err = 1;
    } else if (write_pic(s, p->frame, p->width, p->height)) {
        s->err = 1;
    }
}

int main(int argc, char **argv)
{
    int threads = 1;
    if (argc == 5 && !strcmp(argv[1], "-t")) {
        threads = atoi(argv[2]);
        argv += 2;
        argc -= 2;
    }
    if (argc != 3 || threads < 1) {
        fprintf(stderr, "usage: %s [-t threads] stream.svac3 out.yuv\n", argv[0]);
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
    sink_t sink = { o, argv[2], 0, 0 };
    int rc = 1;

    if (!dec)
        goto out;
    if (threads > 1)
        svac3_dec_set_threads(dec, threads);
    svac3_dec_set_output(dec, on_picture, &sink);

    size_t pos = 0, nal_len;
    const uint8_t *nal;
    unsigned idx = 0;
    while (!sink.err && svac3_next_nal(buf, len, &pos, &nal, &nal_len)) {
        const unsigned this_idx = idx++;
        if (nal_len < 2)
            continue;
        const svac3_frame_t *fr;
        int ret = svac3_dec_nal(dec, nal, nal_len, &fr);
        if (ret == SVAC3_ERR_CORRUPT && svac3_nal_is_patch(nal[1] & 31u)) {
            fprintf(stderr, "picture %d: patch data out of sync from LCU %u, or reference missing\n",
                    sink.pics, svac3_dec_stats(dec)->first_bad_lcu);
            goto out;
        }
        if (ret) {
            fprintf(stderr, "NAL unit %u: error %d\n", this_idx, ret);
            goto out;
        }
        if (!fr)
            continue;
        const svac3_sps_t *sps = &svac3_dec_hls(dec)->sps;
        if (write_pic(&sink, fr, (int)sps->horizontal_size, (int)sps->vertical_size))
            goto out;
    }
    svac3_dec_flush(dec);
    if (sink.err)
        goto out;
    fprintf(stderr, "%d pictures\n", sink.pics);
    rc = 0;
out:
    if (fclose(o) != 0)
        rc = 1;
    svac3_dec_destroy(dec);
    free(buf);
    return rc;
}
