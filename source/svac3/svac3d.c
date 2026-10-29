/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright (c) 2026 OpenIPC contributors
 *
 * The installed libsvac3d interface over the internal picture decoder.
 */
#include "svac3d.h"
#include "svac3_dec.h"

#include <stdlib.h>

#define STR_(x) #x
#define STR(x) STR_(x)

struct svac3d {
    svac3_dec_t *dec;
    /* for the call under way */
    svac3d_output_fn out;
    void *opaque;
    int first;               /* its first error */
};

/* 表61 */
static const int frame_rate[16][2] = {
    { 0, 0 }, { 24000, 1001 }, { 24, 1 }, { 25, 1 }, { 30000, 1001 }, { 30, 1 }, { 50, 1 },
    { 60000, 1001 }, { 60, 1 }, { 100, 1 }, { 120, 1 }, { 200, 1 }, { 240, 1 }, { 300, 1 },
    { 120000, 1001 }, { 0, 0 },
};

const char *svac3d_version(void)
{
    return STR(SVAC3D_VERSION_MAJOR) "." STR(SVAC3D_VERSION_MINOR) "." STR(SVAC3D_VERSION_PATCH);
}

svac3d_t *svac3d_open(void)
{
    svac3d_t *d = calloc(1, sizeof(*d));
    if (d && !(d->dec = svac3_dec_create())) {
        free(d);
        d = NULL;
    }
    return d;
}

void svac3d_close(svac3d_t *d)
{
    if (!d)
        return;
    svac3_dec_destroy(d->dec);
    free(d);
}

int svac3d_set_threads(svac3d_t *d, int threads)
{
    if (!d || threads < 1)
        return SVAC3D_ERR_ARG;
    return svac3_dec_set_threads(d->dec, threads);
}

void svac3d_reset(svac3d_t *d)
{
    if (d)
        svac3_dec_reset(d->dec);
}

static void emit(svac3d_t *d, const svac3_frame_t *f, int width, int height, int type,
                 int64_t poi, int64_t tag)
{
    if (!d->out)
        return;
    svac3d_picture_t pic = {
        .width = width, .height = height, .pict_type = type, .poi = poi, .tag = tag,
    };
    for (int c = 0; c < 3; c++) {
        pic.data[c] = f->plane[c];
        pic.linesize[c] = f->stride[c];
    }
    d->out(d->opaque, &pic);
}

/* A picture from a picture thread */
static void on_picture(void *ctx, const svac3_dec_pic_t *p)
{
    svac3d_t *d = ctx;
    if (p->err && !d->first)
        d->first = p->err;
    if (p->frame)
        emit(d, p->frame, p->width, p->height, p->pict_type, p->poi, p->tag);
}

static void begin_call(svac3d_t *d, svac3d_output_fn out, void *opaque)
{
    d->out = out;
    d->opaque = opaque;
    d->first = 0;
    svac3_dec_set_output(d->dec, on_picture, d);
}

static int end_call(svac3d_t *d)
{
    svac3_dec_set_output(d->dec, NULL, NULL);
    d->out = NULL;
    d->opaque = NULL;
    return d->first;
}

int svac3d_decode(svac3d_t *d, const uint8_t *buf, size_t len, svac3d_output_fn out,
                  void *opaque)
{
    return svac3d_decode_tag(d, buf, len, 0, out, opaque);
}

int svac3d_decode_tag(svac3d_t *d, const uint8_t *buf, size_t len, int64_t tag,
                      svac3d_output_fn out, void *opaque)
{
    if (!d || (!buf && len))
        return SVAC3D_ERR_ARG;
    size_t pos = 0, nal_len;
    const uint8_t *nal;
    begin_call(d, out, opaque);
    svac3_dec_set_tag(d->dec, tag);
    while (svac3_next_nal(buf, len, &pos, &nal, &nal_len)) {
        if (!nal_len)
            continue;                        /* two start codes in a row */
        const svac3_frame_t *f;
        const int ret = svac3_dec_nal(d->dec, nal, nal_len, &f);
        if (ret && !d->first)
            d->first = ret;
        /* a unit in error may still complete a picture: one that reveals
         * an earlier picture's lost patch */
        if (!f)
            continue;
        const svac3_hls_t *hls = svac3_dec_hls(d->dec);
        emit(d, f, (int)hls->sps.horizontal_size, (int)hls->sps.vertical_size,
             hls->ph.picture_coding_type, svac3_dec_poi(d->dec), svac3_dec_tag(d->dec));
    }
    return end_call(d);
}

int svac3d_flush(svac3d_t *d, svac3d_output_fn out, void *opaque)
{
    if (!d)
        return SVAC3D_ERR_ARG;
    begin_call(d, out, opaque);
    svac3_dec_flush(d->dec);
    return end_call(d);
}

int svac3d_get_info(const svac3d_t *d, svac3d_info_t *info)
{
    if (!d || !info)
        return SVAC3D_ERR_ARG;
    const svac3_hls_t *hls = svac3_dec_hls(d->dec);
    if (!hls->have_sps)
        return SVAC3D_ERR_INVALID;
    const svac3_sps_t *s = &hls->sps;
    info->profile_id = s->profile_id;
    info->level_id = s->level_id;
    info->width = s->horizontal_size;
    info->height = s->vertical_size;
    info->coded_width = s->pic_width_in_luma;
    info->coded_height = s->pic_height_in_luma;
    info->frame_rate_num = frame_rate[s->frame_rate_code & 15][0];
    info->frame_rate_den = frame_rate[s->frame_rate_code & 15][1];
    return 0;
}
