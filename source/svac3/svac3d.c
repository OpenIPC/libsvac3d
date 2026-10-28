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

void svac3d_reset(svac3d_t *d)
{
    if (d)
        svac3_dec_reset(d->dec);
}

int svac3d_decode(svac3d_t *d, const uint8_t *buf, size_t len, svac3d_output_fn out,
                  void *opaque)
{
    if (!d || (!buf && len))
        return SVAC3D_ERR_ARG;
    size_t pos = 0, nal_len;
    const uint8_t *nal;
    while (svac3_next_nal(buf, len, &pos, &nal, &nal_len)) {
        if (!nal_len)
            continue;                        /* two start codes in a row */
        const svac3_frame_t *f;
        const int ret = svac3_dec_nal(d->dec, nal, nal_len, &f);
        if (ret)
            return ret;
        if (!f || !out)
            continue;
        const svac3_hls_t *hls = svac3_dec_hls(d->dec);
        svac3d_picture_t pic = {
            .width = hls->sps.horizontal_size,
            .height = hls->sps.vertical_size,
            .pict_type = hls->ph.picture_coding_type,
            .poi = svac3_dec_poi(d->dec),
        };
        for (int c = 0; c < 3; c++) {
            pic.data[c] = f->plane[c];
            pic.linesize[c] = f->stride[c];
        }
        out(opaque, &pic);
    }
    return 0;
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
