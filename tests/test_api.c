/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright (c) 2026 OpenIPC contributors
 *
 * Tests of the installed libsvac3d interface: the pictures svac3d_decode()
 * gives are those of the internal decoder, however the stream is cut into
 * buffers, and the stream information is reported.
 * usage: test_api <stream.svac3>
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "read_file.h"
#include "svac3_dec.h"
#include "svac3d.h"

static int failures;

#define CHECK(cond)                                                       \
    do {                                                                  \
        if (!(cond)) {                                                    \
            fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__,        \
                    __LINE__, #cond);                                     \
            failures++;                                                   \
        }                                                                 \
    } while (0)

/* FNV-1a over the cropped planes of every picture, in order */
typedef struct {
    uint64_t hash;
    int pics, last_type;
    int64_t last_poi;
} sink_t;

static void hash_plane(uint64_t *h, const uint8_t *p, int stride, int w, int rows)
{
    for (int y = 0; y < rows; y++)
        for (int x = 0; x < w; x++)
            *h = (*h ^ p[(size_t)y * stride + x]) * 0x100000001b3ULL;
}

static void out(void *opaque, const svac3d_picture_t *pic)
{
    sink_t *s = opaque;
    for (int c = 0; c < 3; c++)
        hash_plane(&s->hash, pic->data[c], pic->linesize[c],
                   c ? (pic->width + 1) / 2 : pic->width, c ? (pic->height + 1) / 2 : pic->height);
    s->pics++;
    s->last_type = pic->pict_type;
    s->last_poi = pic->poi;
}

int main(int argc, char **argv)
{
    if (argc != 2) {
        fprintf(stderr, "usage: %s stream.svac3\n", argv[0]);
        return 2;
    }
    size_t len;
    uint8_t *buf = read_file(argv[1], &len);
    if (!buf)
        return 1;

    /* reference: the internal decoder, NAL unit by NAL unit */
    sink_t ref = { 0xcbf29ce484222325ULL, 0, 0, 0 };
    {
        svac3_dec_t *d = svac3_dec_create();
        size_t pos = 0, nal_len;
        const uint8_t *nal;
        while (svac3_next_nal(buf, len, &pos, &nal, &nal_len)) {
            const svac3_frame_t *f;
            if (nal_len < 2 || svac3_dec_nal(d, nal, nal_len, &f) || !f)
                continue;
            const svac3_sps_t *s = &svac3_dec_hls(d)->sps;
            svac3d_picture_t pic = { .width = s->horizontal_size, .height = s->vertical_size };
            for (int c = 0; c < 3; c++) {
                pic.data[c] = f->plane[c];
                pic.linesize[c] = f->stride[c];
            }
            out(&ref, &pic);
        }
        svac3_dec_destroy(d);
    }
    CHECK(ref.pics > 0);

    /* the whole stream in one call */
    svac3d_t *d = svac3d_open();
    svac3d_info_t info;
    CHECK(d != NULL);
    CHECK(svac3d_get_info(d, &info) == SVAC3D_ERR_INVALID);
    sink_t a = { 0xcbf29ce484222325ULL, 0, 0, 0 };
    CHECK(svac3d_decode(d, buf, len, out, &a) == 0);
    CHECK(a.pics == ref.pics && a.hash == ref.hash);
    CHECK(svac3d_get_info(d, &info) == 0 && info.profile_id == 0x30 && info.width > 0 &&
          info.coded_width >= info.width && info.frame_rate_num > 0);
    CHECK(a.last_type == 2 && a.last_poi > 0);
    svac3d_close(d);

    /* one access unit per call, as a demuxer with a parser hands them over:
     * cut before every SPS or picture header that follows a patch */
    d = svac3d_open();
    sink_t b = { 0xcbf29ce484222325ULL, 0, 0, 0 };
    size_t pos = 0, start = 0, nal_len;
    const uint8_t *nal;
    int patch = 0;
    while (svac3_next_nal(buf, len, &pos, &nal, &nal_len)) {
        const unsigned t = nal_len >= 2 ? nal[1] & 31u : 0;
        if (patch && (t == SVAC3_NAL_SPS || t == SVAC3_NAL_PIC_HEAD)) {
            /* this unit's start code begins at most 4 bytes before it */
            size_t cut = (size_t)(nal - buf) - 3;
            if (cut > start && buf[cut - 1] == 0)
                cut--;
            CHECK(svac3d_decode(d, buf + start, cut - start, out, &b) == 0);
            start = cut;
            patch = 0;
        }
        patch |= svac3_nal_is_patch(t);
    }
    CHECK(svac3d_decode(d, buf + start, len - start, out, &b) == 0);
    CHECK(b.pics == ref.pics && b.hash == ref.hash);

    /* after a reset, P pictures are refused until the next I picture */
    svac3d_reset(d);
    sink_t c = { 0xcbf29ce484222325ULL, 0, 0, 0 };
    CHECK(svac3d_decode(d, buf + start, len - start, out, &c) == SVAC3D_ERR_CORRUPT && c.pics == 0);
    svac3d_close(d);

    CHECK(svac3d_decode(NULL, buf, len, out, NULL) == SVAC3D_ERR_ARG);
    CHECK(strcmp(svac3d_version(), "0.1.0") >= 0);

    free(buf);
    if (failures)
        fprintf(stderr, "%d check(s) failed\n", failures);
    else
        printf("all checks passed (%d pictures)\n", ref.pics);
    return failures ? 1 : 0;
}
