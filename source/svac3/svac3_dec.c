/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright (c) 2026 OpenIPC contributors
 *
 * SVAC3 picture decoder around the patch decoder: 5.4.3.2 (DOI and POI),
 * 5.4.3.3 (reference picture lists) and the decoded picture buffer.
 */
#include "svac3_dec.h"

#include <stdlib.h>
#include <string.h>

/* every reference a list may name, plus the picture being decoded */
#define DPB_SLOTS (2 * SVAC3_MAX_REFS + 1)

typedef struct {
    uint8_t *buf;
    svac3_frame_t f;
    svac3_tmv_t *tmv;
    int64_t doi;             /* DOI + 256 * DOICycleCnt */
    int64_t poi;
    int used;
} pic_t;

struct svac3_dec {
    svac3_hls_t hls;
    svac3_patch_stats_t st;
    uint8_t *rbsp;
    size_t rbsp_cap;
    pic_t pic[DPB_SLOTS];
    int w, h;
    int64_t cycle;
    int doi_prev, have_prev;
};

svac3_dec_t *svac3_dec_create(void)
{
    return calloc(1, sizeof(svac3_dec_t));
}

static void free_pics(svac3_dec_t *d)
{
    for (int i = 0; i < DPB_SLOTS; i++) {
        free(d->pic[i].buf);
        free(d->pic[i].tmv);
        memset(&d->pic[i], 0, sizeof(d->pic[i]));
    }
}

void svac3_dec_destroy(svac3_dec_t *d)
{
    if (!d)
        return;
    free_pics(d);
    free(d->rbsp);
    free(d);
}

void svac3_dec_reset(svac3_dec_t *d)
{
    for (int i = 0; i < DPB_SLOTS; i++)
        d->pic[i].used = 0;
    d->have_prev = 0;
}

const svac3_hls_t *svac3_dec_hls(const svac3_dec_t *d)
{
    return &d->hls;
}

const svac3_patch_stats_t *svac3_dec_stats(const svac3_dec_t *d)
{
    return &d->st;
}

static int alloc_pic(pic_t *p, int w, int h)
{
    const size_t ysz = (size_t)w * h;
    p->buf = malloc(ysz * 3 / 2);
    p->tmv = malloc(svac3_tmv_count(w, h) * sizeof(*p->tmv));
    if (!p->buf || !p->tmv) {
        free(p->buf);
        free(p->tmv);
        p->buf = NULL;
        p->tmv = NULL;
        return SVAC3_ERR_NOMEM;
    }
    p->f.plane[0] = p->buf;
    p->f.plane[1] = p->buf + ysz;
    p->f.plane[2] = p->f.plane[1] + ysz / 4;
    p->f.stride[0] = p->f.width[0] = w;
    p->f.stride[1] = p->f.stride[2] = p->f.width[1] = p->f.width[2] = w / 2;
    p->f.height[0] = h;
    p->f.height[1] = p->f.height[2] = h / 2;
    return SVAC3_OK;
}

static pic_t *find_doi(svac3_dec_t *d, int64_t doi)
{
    for (int i = 0; i < DPB_SLOTS; i++)
        if (d->pic[i].used && d->pic[i].doi == doi)
            return &d->pic[i];
    return NULL;
}

/* The reference picture list l of the current picture, or NULL */
static const svac3_rpl_t *rpl_of(const svac3_hls_t *hls, int l)
{
    const svac3_sps_t *s = &hls->sps;
    const svac3_ph_t *ph = &hls->ph;
    if (!ph->ref_pic_list_set_flag[l])
        return &ph->rpl[l];
    const uint32_t idx = (l == 1 && !s->rpl1_index_exist_flag) ? ph->ref_pic_list_set_index[0]
                                                                 : ph->ref_pic_list_set_index[l];
    if (idx >= s->num_ref_pic_list_set[l] || idx >= SVAC3_MAX_RPLS)
        return NULL;
    return &s->rpls[l][idx];
}

/* 5.4.3.3: RefPicDoiList for list l; returns the number of entries, or -1
 * for a list naming a library picture */
static int ref_dois(const svac3_rpl_t *r, int64_t doi, int64_t out[SVAC3_MAX_REFS])
{
    int64_t base = doi;
    const int n = r->num_of_ref_pic < SVAC3_MAX_REFS ? r->num_of_ref_pic : SVAC3_MAX_REFS;
    for (int j = 0; j < n; j++) {
        if (r->library_index_flag[j])
            return -1;
        base -= r->delta_doi[j];
        out[j] = base;
    }
    return n;
}

static int decode_picture(svac3_dec_t *d, const uint8_t *rbsp, size_t rl,
                          const svac3_frame_t **out)
{
    svac3_hls_t *hls = &d->hls;
    int ret = svac3_patch_supported(hls);
    if (ret)
        return ret;

    const int w = hls->sps.pic_width_in_luma, h = hls->sps.pic_height_in_luma;
    if (w != d->w || h != d->h) {
        free_pics(d);
        d->w = w;
        d->h = h;
    }

    /* 5.4.3.2 */
    const int doi7 = hls->ph.decode_order_index;
    if (d->have_prev && doi7 < d->doi_prev)
        d->cycle++;
    d->doi_prev = doi7;
    d->have_prev = 1;
    const int64_t doi = doi7 + 256 * d->cycle;
    const int64_t poi = doi + hls->ph.picture_output_delay;

    /* reference lists, and the pictures either of them keeps */
    int64_t list[2][SVAC3_MAX_REFS];
    int nlist[2];
    for (int l = 0; l < 2; l++) {
        const svac3_rpl_t *r = rpl_of(hls, l);
        if (!r)
            return SVAC3_ERR_CORRUPT;
        nlist[l] = ref_dois(r, doi, list[l]);
        if (nlist[l] < 0)
            return SVAC3_ERR_UNSUPPORTED;       /* library pictures */
    }
    /* which pictures the lists keep; the buffer is pruned only once the
     * picture has decoded, so a rejected one costs no reference */
    int keep[DPB_SLOTS];
    for (int i = 0; i < DPB_SLOTS; i++) {
        const pic_t *p = &d->pic[i];
        keep[i] = 0;
        for (int l = 0; l < 2; l++)
            for (int j = 0; j < nlist[l]; j++)
                keep[i] |= p->used && p->doi == list[l][j];
    }

    svac3_refs_t refs;
    memset(&refs, 0, sizeof(refs));
    if (hls->ph.picture_coding_type == 2) {
        refs.num = nlist[0];
        for (int j = 0; j < nlist[0]; j++) {
            const pic_t *r = find_doi(d, list[0][j]);
            if (!r)
                return SVAC3_ERR_CORRUPT;       /* reference not decoded */
            refs.ref[j] = &r->f;
            /* bounded for a conforming stream (POIs within 128); clamp a corrupt one */
            const int64_t dist = 2 * (poi - r->poi);
            refs.dist[j] = dist < -32768 ? -32768 : dist > 32767 ? 32767 : (int)dist;
            if (j == 0)
                refs.col = r->tmv;
        }
    }

    /* a free slot, or failing that one the lists let go: at most
     * 2 * SVAC3_MAX_REFS are kept, so one always exists */
    pic_t *cur = NULL;
    for (int i = 0; i < DPB_SLOTS && !cur; i++)
        if (!d->pic[i].used)
            cur = &d->pic[i];
    for (int i = 0; i < DPB_SLOTS && !cur; i++)
        if (!keep[i])
            cur = &d->pic[i];
    if (!cur)
        return SVAC3_ERR_CORRUPT;
    cur->used = 0;                              /* its picture is about to be overwritten */
    if (!cur->buf && (ret = alloc_pic(cur, w, h)))
        return ret;

    ret = svac3_patch_decode(hls, rbsp, rl, &cur->f, &refs, cur->tmv, &d->st);
    if (ret)
        return ret;
    for (int i = 0; i < DPB_SLOTS; i++)
        d->pic[i].used = keep[i];
    cur->used = 1;
    cur->doi = doi;
    cur->poi = poi;
    *out = &cur->f;
    return SVAC3_OK;
}

int svac3_dec_nal(svac3_dec_t *d, const uint8_t *nal, size_t len, const svac3_frame_t **out)
{
    *out = NULL;
    if (len < 2)
        return SVAC3_ERR_INVALID;
    int ret = svac3_parse_nal(&d->hls, nal, len, 0, NULL);
    if (ret)
        return ret;
    if (!svac3_nal_is_patch(nal[1] & 31u))
        return SVAC3_OK;
    if (len - 2 > d->rbsp_cap) {
        uint8_t *b = realloc(d->rbsp, len - 2);
        if (!b)
            return SVAC3_ERR_NOMEM;
        d->rbsp = b;
        d->rbsp_cap = len - 2;
    }
    const size_t rl = svac3_unescape(nal + 2, len - 2, d->rbsp);
    return decode_picture(d, d->rbsp, rl, out);
}
