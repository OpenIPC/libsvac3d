/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright (c) 2026 OpenIPC contributors
 *
 * SVAC3 picture decoder around the patch decoder: 5.4.3.2 (DOI and POI),
 * 5.4.3.3 (reference picture lists), the decoded picture buffer, and the
 * library pictures that display pictures may reference.
 */
#include "svac3_dec.h"

#include <stdlib.h>
#include <string.h>

/* every reference a list may name, plus the picture being decoded */
#define DPB_SLOTS (2 * SVAC3_MAX_REFS + 1)
/* the library picture kept (the Main profile allows one in the buffer,
 * F.2; a new one replaces it, 5.4.3.4), and one being decoded */
#define LIB_SLOTS 2

typedef struct {
    uint8_t *buf;
    svac3_frame_t f;
    svac3_tmv_t *tmv;
    int64_t doi;             /* DOI + 256 * DOICycleCnt */
    int64_t poi;
    int used;
    uint32_t lib_index;      /* library pictures: library_picture_index */
} pic_t;

struct svac3_dec {
    svac3_hls_t hls;
    svac3_patch_stats_t st;
    uint8_t *rbsp;
    size_t rbsp_cap;
    pic_t pic[DPB_SLOTS];
    pic_t lib[LIB_SLOTS];
    int w, h;
    int64_t cycle;
    int doi_prev, have_prev;
    int64_t last_poi;

    /* the picture whose patches are being decoded */
    svac3_picture_t *open;
    pic_t *open_slot;
    int open_lib;
    int open_keep[DPB_SLOTS];
    svac3_refs_t open_refs;
    svac3_pool_t *pool;
    svac3_picture_cache_t cache;
    int64_t open_doi, open_poi;
    uint32_t open_index;
};

static void abandon(svac3_dec_t *d);

svac3_dec_t *svac3_dec_create(void)
{
    return calloc(1, sizeof(svac3_dec_t));
}

static void free_pic(pic_t *p)
{
    free(p->buf);
    free(p->tmv);
    memset(p, 0, sizeof(*p));
}

static void free_pics(svac3_dec_t *d)
{
    for (int i = 0; i < DPB_SLOTS; i++)
        free_pic(&d->pic[i]);
    for (int i = 0; i < LIB_SLOTS; i++)
        free_pic(&d->lib[i]);
}

void svac3_dec_destroy(svac3_dec_t *d)
{
    if (!d)
        return;
    abandon(d);
    free_pics(d);
    svac3_picture_cache_free(&d->cache);
    svac3_pool_destroy(d->pool);
    free(d->rbsp);
    free(d);
}

int svac3_dec_set_threads(svac3_dec_t *d, int threads)
{
    abandon(d);                       /* a picture keeps the pool it began with */
    svac3_pool_destroy(d->pool);
    d->pool = svac3_pool_create(threads);
    return svac3_pool_threads(d->pool);
}

void svac3_dec_reset(svac3_dec_t *d)
{
    abandon(d);
    for (int i = 0; i < DPB_SLOTS; i++)
        d->pic[i].used = 0;
    for (int i = 0; i < LIB_SLOTS; i++)
        d->lib[i].used = 0;
    d->have_prev = 0;
}

int64_t svac3_dec_poi(const svac3_dec_t *d)
{
    return d->last_poi;
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

typedef struct {
    int lib;                 /* a library picture, by index */
    int64_t key;             /* its DOI, or its library_picture_index */
} ref_t;

/* 5.4.3.3: RefPicDoiList for list l; returns the number of entries */
static int ref_list(const svac3_rpl_t *r, int64_t doi, ref_t out[SVAC3_MAX_REFS])
{
    int64_t base = doi;
    const int n = r->num_of_ref_pic < SVAC3_MAX_REFS ? r->num_of_ref_pic : SVAC3_MAX_REFS;
    for (int j = 0; j < n; j++) {
        out[j].lib = r->library_index_flag[j];
        if (out[j].lib) {
            out[j].key = r->delta_doi[j];   /* ReferencedLibraryPictureIndex */
        } else {
            base -= r->delta_doi[j];
            out[j].key = base;
        }
    }
    return n;
}

static pic_t *find_lib(svac3_dec_t *d, int64_t index)
{
    for (int i = 0; i < LIB_SLOTS; i++)
        if (d->lib[i].used && d->lib[i].lib_index == index)
            return &d->lib[i];
    return NULL;
}

static int64_t clamp_dist(int64_t dist)
{
    /* bounded for a conforming stream (POIs within 128); clamp a corrupt one */
    return dist < -32768 ? -32768 : dist > 32767 ? 32767 : dist;
}

static void abandon(svac3_dec_t *d)
{
    svac3_picture_release(d->open, &d->cache);
    d->open = NULL;
}

/* A library picture (library_stream_flag 1): an I picture kept, with its
 * library_picture_index, for display pictures to reference, and output
 * since only display library patches (type 17) are accepted. A display
 * picture naming another index finds no reference. It is decoded into
 * the slot not holding the current library picture, which gives way only
 * once the new one has decoded. */
static int start_library(svac3_dec_t *d)
{
    svac3_hls_t *hls = &d->hls;
    int ret;

    if (hls->ph.picture_coding_type != 1)
        return SVAC3_ERR_UNSUPPORTED;           /* inter-coded library pictures */
    pic_t *cur = d->lib[0].used ? &d->lib[1] : &d->lib[0];
    cur->used = 0;
    if (!cur->buf && (ret = alloc_pic(cur, d->w, d->h)))
        return ret;
    if ((ret = svac3_picture_begin_cached(hls, &cur->f, NULL, cur->tmv, &d->cache, &d->open)))
        return ret;
    svac3_picture_set_pool(d->open, d->pool);
    d->open_slot = cur;
    d->open_lib = 1;
    d->open_index = hls->ph.library_picture_index;
    return SVAC3_OK;
}

/* Begin a picture at its first patch: DOI and POI, reference lists, a slot */
static int start_picture(svac3_dec_t *d)
{
    svac3_hls_t *hls = &d->hls;
    const svac3_sps_t *sps = &hls->sps;
    int ret;

    const int w = sps->pic_width_in_luma, h = sps->pic_height_in_luma;
    if (w != d->w || h != d->h) {
        free_pics(d);
        d->w = w;
        d->h = h;
    }

    /* 5.4.3.2: a library stream's DOIs count only in library picture mode 1 */
    const int doi7 = hls->ph.decode_order_index;
    if (!sps->library_stream_flag || sps->library_picture_mode_index == 1) {
        if (d->have_prev && doi7 < d->doi_prev)
            d->cycle++;
        d->doi_prev = doi7;
        d->have_prev = 1;
    }
    const int64_t doi = doi7 + 256 * d->cycle;
    const int64_t poi = doi + hls->ph.picture_output_delay;
    d->open_doi = doi;
    d->open_poi = poi;
    if (sps->library_stream_flag)
        return start_library(d);

    /* reference lists, and the pictures either of them keeps */
    ref_t list[2][SVAC3_MAX_REFS];
    int nlist[2];
    for (int l = 0; l < 2; l++) {
        const svac3_rpl_t *r = rpl_of(hls, l);
        if (!r)
            return SVAC3_ERR_CORRUPT;
        nlist[l] = ref_list(r, doi, list[l]);
    }
    /* which pictures the lists keep; the buffer is pruned only once the
     * picture has decoded, so a rejected one costs no reference */
    int *keep = d->open_keep;
    for (int i = 0; i < DPB_SLOTS; i++) {
        const pic_t *p = &d->pic[i];
        keep[i] = 0;
        for (int l = 0; l < 2; l++)
            for (int j = 0; j < nlist[l]; j++)
                keep[i] |= p->used && !list[l][j].lib && p->doi == list[l][j].key;
    }

    svac3_refs_t *refs = &d->open_refs;
    memset(refs, 0, sizeof(*refs));
    if (hls->ph.picture_coding_type == 2) {
        refs->num = nlist[0];
        for (int j = 0; j < nlist[0]; j++) {
            const ref_t *e = &list[0][j];
            const pic_t *r = e->lib ? find_lib(d, e->key) : find_doi(d, e->key);
            if (!r)
                return SVAC3_ERR_CORRUPT;       /* reference not decoded */
            refs->ref[j] = &r->f;
            /* a library reference's DistanceIndex is 2 * (POI - 1) (5.4.3.3) */
            refs->dist[j] = (int)clamp_dist(e->lib ? 2 : 2 * (poi - r->poi));
            if (j == 0)
                refs->col = r->tmv;
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
    if ((ret = svac3_picture_begin_cached(hls, &cur->f, refs, cur->tmv, &d->cache, &d->open)))
        return ret;
    svac3_picture_set_pool(d->open, d->pool);
    d->open_slot = cur;
    d->open_lib = 0;
    return SVAC3_OK;
}

/* The picture's last patch has decoded: the buffer takes it */
static void commit(svac3_dec_t *d, const svac3_frame_t **out)
{
    pic_t *cur = d->open_slot;
    if (d->open_lib) {
        for (int i = 0; i < LIB_SLOTS; i++)
            d->lib[i].used = 0;
        cur->lib_index = d->open_index;
    } else {
        for (int i = 0; i < DPB_SLOTS; i++)
            d->pic[i].used = d->open_keep[i];
    }
    cur->used = 1;
    cur->doi = d->open_doi;
    cur->poi = d->open_poi;
    d->last_poi = d->open_poi;
    *out = &cur->f;
}

/* One patch NAL unit. A picture that loses patches is dropped, and the
 * unit that shows it (the next picture's first patch, or a patch out of
 * order) returns SVAC3_ERR_CORRUPT. */
static int decode_patch_nal(svac3_dec_t *d, const uint8_t *rbsp, size_t rl,
                            const svac3_frame_t **out)
{
    svac3_hls_t *hls = &d->hls;
    int ret = svac3_patch_supported(hls), lost = 0;
    if (ret)
        return ret;

    if (hls->patch.patch_index == 0) {
        if (d->open) {
            abandon(d);
            lost = 1;
        }
        if ((ret = start_picture(d)))
            return ret;
    } else if (!d->open) {
        return SVAC3_ERR_CORRUPT;               /* its picture's first patch was lost */
    }

    ret = svac3_picture_patch(d->open, hls, rbsp, rl, &d->st);
    if (ret < 0) {
        abandon(d);
        return ret;
    }
    if (ret == 1) {
        svac3_picture_t *pic = d->open;
        d->open = NULL;
        if ((ret = svac3_picture_end_cached(pic, hls, &d->cache)))
            return ret;
        commit(d, out);
    }
    return lost ? SVAC3_ERR_CORRUPT : SVAC3_OK;
}

int svac3_dec_nal(svac3_dec_t *d, const uint8_t *nal, size_t len, const svac3_frame_t **out)
{
    *out = NULL;
    if (len < 2)
        return SVAC3_ERR_INVALID;
    /* a picture still missing patches when a header that replaces its
     * parameters arrives is lost */
    const unsigned t = nal[1] & 31u;
    int lost = 0;
    if (d->open && (t == SVAC3_NAL_SPS || t == SVAC3_NAL_PPS || t == SVAC3_NAL_PIC_HEAD)) {
        abandon(d);
        lost = 1;
    }
    int ret = svac3_parse_nal(&d->hls, nal, len, 0, NULL);
    if (ret)
        return ret;
    if (!svac3_nal_is_patch(t))
        return lost ? SVAC3_ERR_CORRUPT : SVAC3_OK;
    if (len - 2 > d->rbsp_cap) {
        uint8_t *b = realloc(d->rbsp, len - 2);
        if (!b)
            return SVAC3_ERR_NOMEM;
        d->rbsp = b;
        d->rbsp_cap = len - 2;
    }
    const size_t rl = svac3_unescape(nal + 2, len - 2, d->rbsp);
    return decode_patch_nal(d, d->rbsp, rl, out);
}
