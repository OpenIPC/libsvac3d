/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright (c) 2026 OpenIPC contributors
 *
 * SVAC3 picture decoder around the patch decoder: 5.4.3.2 (DOI and POI),
 * 5.4.3.3 (reference picture lists), the decoded picture buffer, and the
 * library pictures that display pictures may reference.
 *
 * With one thread a picture is decoded as its patches arrive. With more,
 * pictures are decoded side by side, one per picture thread: the caller's
 * thread keeps the buffer and the reference lists, in decoding order, and
 * hands each picture, once its patches are in, to a picture thread; a
 * picture waits for the rows of its references that its motion reaches,
 * as their threads report them final. Each picture thread also has the
 * reconstruction and the filters of its picture follow its parser on
 * threads of their own: that is what makes an I picture, which nothing
 * else can overlap much, quicker.
 */
#include "svac3_dec.h"

#include <pthread.h>
#include <stdlib.h>
#include <string.h>

/* every reference a list may name, the picture being decoded, and one for
 * each picture another thread may still be decoding or about to output */
#define DPB_SLOTS (2 * SVAC3_MAX_REFS + 2 + SVAC3_MAX_THREADS)
/* the library picture kept (the Main profile allows one in the buffer,
 * F.2; a new one replaces it, 5.4.3.4), one being decoded, and the same
 * for pictures in flight */
#define LIB_SLOTS (2 + SVAC3_MAX_THREADS)

typedef struct {
    uint8_t *buf;
    svac3_frame_t f;
    svac3_tmv_t *tmv;
    int w, h;
    int64_t doi;             /* DOI + 256 * DOICycleCnt */
    int64_t poi;
    int used;
    uint32_t lib_index;      /* library pictures: library_picture_index */

    /* pictures in flight */
    int holds;               /* pictures in flight using it (the caller's thread's) */
    svac3_dec_t *dec;
    int rows, tmv_rows;      /* final luma rows, temporal motion rows; atomic */
    int failed;              /* written before the last rows */
} pic_t;

/* What the reference lists make of a picture at its first patch */
typedef struct {
    pic_t *slot;
    int lib;
    uint32_t index;          /* library pictures: library_picture_index */
    int64_t doi, poi;
    int64_t tag;
    int keep[DPB_SLOTS];
    svac3_refs_t refs;
    pic_t *ref[SVAC3_MAX_REFS];
} plan_t;

typedef struct {
    svac3_patch_header_t hdr;
    uint8_t *data;
    size_t len, cap;
} patch_buf_t;

/* A picture for a picture thread */
typedef struct {
    svac3_hls_t hls;                     /* as at its first patch */
    patch_buf_t *patch;
    int npatch, num_patches, patch_cap;
    plan_t plan;
    int ret, done;
    svac3_patch_stats_t st;
} task_t;

typedef struct {
    pthread_t th;
    svac3_dec_t *d;
    svac3_picture_cache_t cache;
    svac3_pool_t *pool;
} worker_t;

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
    int64_t tag, last_tag;

    /* the picture whose patches are being decoded */
    svac3_picture_t *open;
    plan_t open_plan;
    svac3_picture_cache_t cache;

    /* picture threads: tasks in decoding order, ring[head] the oldest;
     * the first taken of them are with a thread or done */
    int nworkers;
    worker_t *workers;
    pthread_mutex_t lock;
    pthread_cond_t work, cond;           /* a task queued; progress or a task done */
    int stop;
    task_t *ring[SVAC3_MAX_THREADS];
    int head, n, taken;
    task_t *collect;                     /* the picture whose patches are arriving */
    task_t *spare[SVAC3_MAX_THREADS + 1];
    int nspare;
    svac3_dec_out_fn out_fn;
    void *out_ctx;
};

static void abandon(svac3_dec_t *d);
static void stop_workers(svac3_dec_t *d);

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

static void free_task(task_t *t)
{
    if (!t)
        return;
    for (int i = 0; i < t->patch_cap; i++)
        free(t->patch[i].data);
    free(t->patch);
    free(t);
}

void svac3_dec_destroy(svac3_dec_t *d)
{
    if (!d)
        return;
    abandon(d);
    stop_workers(d);
    for (int i = 0; i < d->nspare; i++)
        free_task(d->spare[i]);
    for (int i = 0; i < DPB_SLOTS; i++)
        free_pic(&d->pic[i]);
    for (int i = 0; i < LIB_SLOTS; i++)
        free_pic(&d->lib[i]);
    svac3_picture_cache_free(&d->cache);
    free(d->rbsp);
    free(d);
}

int64_t svac3_dec_poi(const svac3_dec_t *d)
{
    return d->last_poi;
}

void svac3_dec_set_tag(svac3_dec_t *d, int64_t tag)
{
    d->tag = tag;
}

int64_t svac3_dec_tag(const svac3_dec_t *d)
{
    return d->last_tag;
}

const svac3_hls_t *svac3_dec_hls(const svac3_dec_t *d)
{
    return &d->hls;
}

const svac3_patch_stats_t *svac3_dec_stats(const svac3_dec_t *d)
{
    return &d->st;
}

void svac3_dec_set_output(svac3_dec_t *d, svac3_dec_out_fn fn, void *ctx)
{
    d->out_fn = fn;
    d->out_ctx = ctx;
}

/* A slot holding a w x h picture: a picture of another size gives way */
static int alloc_pic(pic_t *p, int w, int h)
{
    if (p->buf && (p->w != w || p->h != h))
        free_pic(p);
    if (p->buf)
        return SVAC3_OK;
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
    p->w = w;
    p->h = h;
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
 * a slot not holding the current library picture, which gives way only
 * once the new one has decoded. */
static int plan_library(svac3_dec_t *d, plan_t *pl)
{
    svac3_hls_t *hls = &d->hls;

    if (hls->ph.picture_coding_type != 1)
        return SVAC3_ERR_UNSUPPORTED;           /* inter-coded library pictures */
    pic_t *cur = NULL;
    for (int i = 0; i < LIB_SLOTS && !cur; i++)
        if (!d->lib[i].used && !d->lib[i].holds)
            cur = &d->lib[i];
    if (!cur)
        return SVAC3_ERR_CORRUPT;
    const int ret = alloc_pic(cur, d->w, d->h);
    if (ret)
        return ret;
    pl->slot = cur;
    pl->lib = 1;
    pl->index = hls->ph.library_picture_index;
    return SVAC3_OK;
}

/* A picture at its first patch: DOI and POI, reference lists, a slot */
static int plan_picture(svac3_dec_t *d, plan_t *pl)
{
    svac3_hls_t *hls = &d->hls;
    const svac3_sps_t *sps = &hls->sps;
    int ret;

    memset(pl, 0, sizeof(*pl));
    const int w = sps->pic_width_in_luma, h = sps->pic_height_in_luma;
    if (w != d->w || h != d->h) {
        /* a new size: no picture of the old one is referenced again (the
         * slots are reallocated as they are taken) */
        for (int i = 0; i < DPB_SLOTS; i++)
            d->pic[i].used = 0;
        for (int i = 0; i < LIB_SLOTS; i++)
            d->lib[i].used = 0;
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
    pl->doi = doi;
    pl->poi = poi;
    pl->tag = d->tag;
    if (sps->library_stream_flag)
        return plan_library(d, pl);

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
    int *keep = pl->keep;
    for (int i = 0; i < DPB_SLOTS; i++) {
        const pic_t *p = &d->pic[i];
        keep[i] = 0;
        for (int l = 0; l < 2; l++)
            for (int j = 0; j < nlist[l]; j++)
                keep[i] |= p->used && !list[l][j].lib && p->doi == list[l][j].key;
    }

    svac3_refs_t *refs = &pl->refs;
    if (hls->ph.picture_coding_type == 2) {
        refs->num = nlist[0];
        for (int j = 0; j < nlist[0]; j++) {
            const ref_t *e = &list[0][j];
            pic_t *r = e->lib ? find_lib(d, e->key) : find_doi(d, e->key);
            if (!r)
                return SVAC3_ERR_CORRUPT;       /* reference not decoded */
            pl->ref[j] = r;
            refs->ref[j] = &r->f;
            /* a library reference's DistanceIndex is 2 * (POI - 1) (5.4.3.3) */
            refs->dist[j] = (int)clamp_dist(e->lib ? 2 : 2 * (poi - r->poi));
            if (j == 0)
                refs->col = r->tmv;
        }
    }

    /* a free slot, or failing that one the lists let go: at most
     * 2 * SVAC3_MAX_REFS are kept, so one always exists. A slot a picture
     * in flight uses is not free. */
    pic_t *cur = NULL;
    for (int i = 0; i < DPB_SLOTS && !cur; i++)
        if (!d->pic[i].used && !d->pic[i].holds)
            cur = &d->pic[i];
    for (int i = 0; i < DPB_SLOTS && !cur; i++)
        if (!keep[i] && !d->pic[i].holds)
            cur = &d->pic[i];
    if (!cur)
        return SVAC3_ERR_CORRUPT;
    cur->used = 0;                              /* its picture is about to be overwritten */
    if ((ret = alloc_pic(cur, w, h)))
        return ret;
    pl->slot = cur;
    pl->lib = 0;
    return SVAC3_OK;
}

static int start_picture(svac3_dec_t *d)
{
    plan_t *pl = &d->open_plan;
    int ret = plan_picture(d, pl);
    if (ret)
        return ret;
    pic_t *cur = pl->slot;
    return svac3_picture_begin_cached(&d->hls, &cur->f, pl->lib ? NULL : &pl->refs, cur->tmv,
                                      &d->cache, &d->open);
}

/* The buffer takes the picture */
static void commit(svac3_dec_t *d, const plan_t *pl)
{
    pic_t *cur = pl->slot;
    if (pl->lib) {
        for (int i = 0; i < LIB_SLOTS; i++)
            d->lib[i].used = 0;
        cur->lib_index = pl->index;
    } else {
        for (int i = 0; i < DPB_SLOTS; i++)
            d->pic[i].used = pl->keep[i];
    }
    cur->used = 1;
    cur->doi = pl->doi;
    cur->poi = pl->poi;
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
        commit(d, &d->open_plan);
        d->last_poi = d->open_plan.poi;
        d->last_tag = d->open_plan.tag;
        *out = &d->open_plan.slot->f;
    }
    return lost ? SVAC3_ERR_CORRUPT : SVAC3_OK;
}

/* ------------------------------------------------------------------------ */
/* Pictures on picture threads */

static int load(const int *v)
{
    return __atomic_load_n(v, __ATOMIC_ACQUIRE);
}

/* The rows of slot s that are final, and of its temporal motion; a
 * negative number, or one below what is already known, changes nothing */
static void set_progress(pic_t *s, int rows, int tmv_rows)
{
    svac3_dec_t *d = s->dec;
    pthread_mutex_lock(&d->lock);
    if (rows > s->rows)
        __atomic_store_n(&s->rows, rows, __ATOMIC_RELEASE);
    if (tmv_rows > s->tmv_rows)
        __atomic_store_n(&s->tmv_rows, tmv_rows, __ATOMIC_RELEASE);
    pthread_cond_broadcast(&d->cond);
    pthread_mutex_unlock(&d->lock);
}

/* From the patch decoder's threads. The picture's last report waits for
 * its picture thread to know whether it failed: a picture referencing it
 * reads that first. */
static void on_progress(void *ctx, int rows, int tmv_rows)
{
    pic_t *s = ctx;
    if (rows >= s->h)
        return;
    if (rows > load(&s->rows) || tmv_rows > load(&s->tmv_rows))
        set_progress(s, rows, tmv_rows);
}

static void wait_on(pic_t *s, const int *v, int rows)
{
    if (load(v) >= rows)
        return;
    svac3_dec_t *d = s->dec;
    pthread_mutex_lock(&d->lock);
    while (load(v) < rows)
        pthread_cond_wait(&d->cond, &d->lock);
    pthread_mutex_unlock(&d->lock);
}

static void wait_rows(void *ctx, int rows)
{
    pic_t *s = ctx;
    wait_on(s, &s->rows, rows);
}

static void wait_tmv(void *ctx, int rows)
{
    pic_t *s = ctx;
    wait_on(s, &s->tmv_rows, rows);
}

static void run_task(task_t *t, svac3_picture_cache_t *cache, svac3_pool_t *pool)
{
    plan_t *pl = &t->plan;
    pic_t *s = pl->slot;
    const int inter = !pl->lib && t->hls.ph.picture_coding_type == 2;
    svac3_picture_t *pic;
    memset(&t->st, 0, sizeof(t->st));
    int ret = svac3_picture_begin_cached(&t->hls, &s->f, inter ? &pl->refs : NULL, s->tmv,
                                         cache, &pic);
    if (!ret) {
        svac3_picture_set_progress(pic, on_progress, s);
        if (pool)
            svac3_picture_set_pool(pic, pool);
        for (int i = 0; i < t->npatch && !ret; i++) {
            t->hls.patch = t->patch[i].hdr;
            const int r = svac3_picture_patch(pic, &t->hls, t->patch[i].data, t->patch[i].len,
                                              &t->st);
            if (r < 0)
                ret = r;
        }
        if (ret)
            svac3_picture_release(pic, cache);
        else
            ret = svac3_picture_end_cached(pic, &t->hls, cache);
    }
    /* a picture made from a reference that failed failed too */
    for (int j = 0; inter && j < pl->refs.num; j++) {
        wait_rows(pl->ref[j], pl->ref[j]->h);
        if (!ret && load(&pl->ref[j]->failed))
            ret = SVAC3_ERR_CORRUPT;
    }
    t->ret = ret;
    if (ret)
        __atomic_store_n(&s->failed, 1, __ATOMIC_RELEASE);
    set_progress(s, s->h, (s->h + 15) >> 4);
}

static void *worker_main(void *arg)
{
    worker_t *w = arg;
    svac3_dec_t *d = w->d;
    pthread_mutex_lock(&d->lock);
    for (;;) {
        while (!d->stop && d->taken == d->n)
            pthread_cond_wait(&d->work, &d->lock);
        if (d->taken == d->n)
            break;                              /* stopping, nothing queued */
        task_t *t = d->ring[(d->head + d->taken++) % SVAC3_MAX_THREADS];
        pthread_mutex_unlock(&d->lock);
        run_task(t, &w->cache, w->pool);
        pthread_mutex_lock(&d->lock);
        t->done = 1;
        pthread_cond_broadcast(&d->cond);
    }
    pthread_mutex_unlock(&d->lock);
    return NULL;
}

/* The slots a planned picture uses are held while it is in flight */
static void hold(plan_t *pl, int by)
{
    pl->slot->holds += by;
    for (int j = 0; j < SVAC3_MAX_REFS; j++)
        if (pl->ref[j])
            pl->ref[j]->holds += by;
}

static void spare_task(svac3_dec_t *d, task_t *t)
{
    hold(&t->plan, -1);
    d->spare[d->nspare++] = t;
}

static void drop_collect(svac3_dec_t *d)
{
    if (d->collect)
        spare_task(d, d->collect);
    d->collect = NULL;
}

/* The oldest picture in flight leaves, once done: to the output function
 * when emit is set. A picture that failed leaves the buffer, as one that
 * fails on the caller's thread never enters it. */
static int retire(svac3_dec_t *d, int wait, int emit)
{
    if (!d->n)
        return 0;
    task_t *t = d->ring[d->head];
    pthread_mutex_lock(&d->lock);
    while (wait && !t->done)
        pthread_cond_wait(&d->cond, &d->lock);
    const int done = t->done;
    if (done) {
        d->head = (d->head + 1) % SVAC3_MAX_THREADS;
        d->n--;
        d->taken--;
    }
    pthread_mutex_unlock(&d->lock);
    if (!done)
        return 0;
    pic_t *s = t->plan.slot;
    if (t->ret)
        s->used = 0;
    d->st = t->st;
    d->last_poi = t->plan.poi;
    d->last_tag = t->plan.tag;
    if (emit && d->out_fn) {
        const svac3_dec_pic_t o = {
            .frame = t->ret ? NULL : &s->f,
            .width = (int)t->hls.sps.horizontal_size,
            .height = (int)t->hls.sps.vertical_size,
            .pict_type = t->hls.ph.picture_coding_type,
            .poi = t->plan.poi,
            .tag = t->plan.tag,
            .err = t->ret,
        };
        d->out_fn(d->out_ctx, &o);
    }
    spare_task(d, t);
    return 1;
}

/* Every picture in flight, done, out (with emit) */
static void drain(svac3_dec_t *d, int emit)
{
    drop_collect(d);
    while (retire(d, 1, emit))
        ;
}

/* The picture's patches are in: the buffer takes it, a thread decodes it */
static void dispatch(svac3_dec_t *d, task_t *t)
{
    plan_t *pl = &t->plan;
    commit(d, pl);
    pic_t *s = pl->slot;
    s->dec = d;
    s->rows = s->tmv_rows = s->failed = 0;      /* no thread uses the slot */
    pl->refs.wait = wait_rows;
    pl->refs.wait_tmv = wait_tmv;
    for (int j = 0; j < SVAC3_MAX_REFS; j++)
        pl->refs.ctx[j] = pl->ref[j];
    pl->refs.col_ctx = pl->ref[0];
    t->done = 0;
    t->ret = 0;
    while (retire(d, 0, 1))                     /* what is done, out first */
        ;
    while (d->n >= d->nworkers)                 /* as many in flight as threads */
        retire(d, 1, 1);
    pthread_mutex_lock(&d->lock);
    d->ring[(d->head + d->n++) % SVAC3_MAX_THREADS] = t;
    pthread_cond_signal(&d->work);
    pthread_mutex_unlock(&d->lock);
}

static task_t *take_task(svac3_dec_t *d)
{
    if (d->nspare)
        return d->spare[--d->nspare];
    return calloc(1, sizeof(task_t));
}

/* A patch NAL unit for a picture thread: kept until its picture's patches
 * are all in, with the same checks of their order as the caller's thread
 * makes; what is in the patches is checked by the picture's thread, and
 * its errors come with the picture. */
static int queue_patch(svac3_dec_t *d, const uint8_t *nal, size_t len)
{
    svac3_hls_t *hls = &d->hls;
    int ret = svac3_patch_supported(hls), lost = 0;
    if (ret)
        return ret;

    if (hls->patch.patch_index == 0) {
        if (d->collect) {
            drop_collect(d);
            lost = 1;
        }
        const int np = svac3_picture_patches(hls);
        if (np < 1)
            return SVAC3_ERR_INVALID;
        task_t *t = take_task(d);
        if (!t)
            return SVAC3_ERR_NOMEM;
        if ((ret = plan_picture(d, &t->plan))) {
            d->spare[d->nspare++] = t;
            return ret;
        }
        hold(&t->plan, 1);
        t->hls = *hls;
        t->npatch = 0;
        t->num_patches = np;
        d->collect = t;
    } else if (!d->collect) {
        return SVAC3_ERR_CORRUPT;               /* its picture's first patch was lost */
    }

    task_t *t = d->collect;
    if (hls->patch.patch_index != t->npatch) {
        drop_collect(d);
        return SVAC3_ERR_CORRUPT;               /* a patch lost or out of order */
    }
    if (t->npatch == t->patch_cap) {
        const int cap = t->patch_cap ? 2 * t->patch_cap : 1;
        patch_buf_t *p = realloc(t->patch, (size_t)cap * sizeof(*p));
        if (!p) {
            drop_collect(d);
            return SVAC3_ERR_NOMEM;
        }
        memset(p + t->patch_cap, 0, (size_t)(cap - t->patch_cap) * sizeof(*p));
        t->patch = p;
        t->patch_cap = cap;
    }
    patch_buf_t *pb = &t->patch[t->npatch];
    if (len - 2 > pb->cap) {
        uint8_t *b = realloc(pb->data, len - 2);
        if (!b) {
            drop_collect(d);
            return SVAC3_ERR_NOMEM;
        }
        pb->data = b;
        pb->cap = len - 2;
    }
    pb->len = svac3_unescape(nal + 2, len - 2, pb->data);
    pb->hdr = hls->patch;
    if (++t->npatch == t->num_patches) {
        d->collect = NULL;
        dispatch(d, t);
    }
    return lost ? SVAC3_ERR_CORRUPT : SVAC3_OK;
}

static void stop_workers(svac3_dec_t *d)
{
    if (!d->nworkers)
        return;
    drain(d, 0);
    pthread_mutex_lock(&d->lock);
    d->stop = 1;
    pthread_cond_broadcast(&d->work);
    pthread_mutex_unlock(&d->lock);
    for (int i = 0; i < d->nworkers; i++) {
        pthread_join(d->workers[i].th, NULL);
        svac3_picture_cache_free(&d->workers[i].cache);
        svac3_pool_destroy(d->workers[i].pool);
    }
    free(d->workers);
    d->workers = NULL;
    d->nworkers = 0;
    d->stop = 0;
    pthread_cond_destroy(&d->work);
    pthread_cond_destroy(&d->cond);
    pthread_mutex_destroy(&d->lock);
}

/* threads picture threads; fewer when they cannot all be started, and
 * none (0) unless at least two are */
static int start_workers(svac3_dec_t *d, int threads)
{
    if (pthread_mutex_init(&d->lock, NULL))
        return 0;
    if (pthread_cond_init(&d->work, NULL)) {
        pthread_mutex_destroy(&d->lock);
        return 0;
    }
    if (pthread_cond_init(&d->cond, NULL)) {
        pthread_cond_destroy(&d->work);
        pthread_mutex_destroy(&d->lock);
        return 0;
    }
    d->workers = calloc((size_t)threads, sizeof(*d->workers));
    int n = 0;
    while (d->workers && n < threads) {
        d->workers[n].d = d;
        /* a picture's own threads: reconstruction and the filters beside
         * its parser, as on one thread with a pool of 3; they wait while
         * the picture waits for its references */
        d->workers[n].pool = svac3_pool_create(3);
        if (pthread_create(&d->workers[n].th, NULL, worker_main, &d->workers[n])) {
            svac3_pool_destroy(d->workers[n].pool);
            break;
        }
        n++;
    }
    d->nworkers = n;
    if (n < 2) {
        stop_workers(d);
        if (!n) {                               /* stop_workers() left them be */
            free(d->workers);
            d->workers = NULL;
            pthread_cond_destroy(&d->work);
            pthread_cond_destroy(&d->cond);
            pthread_mutex_destroy(&d->lock);
        }
        return 0;
    }
    return n;
}

int svac3_dec_set_threads(svac3_dec_t *d, int threads)
{
    abandon(d);
    if (d->nworkers)
        drain(d, 1);
    stop_workers(d);
    if (threads > SVAC3_MAX_THREADS)
        threads = SVAC3_MAX_THREADS;
    if (threads >= 2) {
        const int n = start_workers(d, threads);
        if (n)
            return n;
    }
    return 1;
}

void svac3_dec_flush(svac3_dec_t *d)
{
    if (d->nworkers)
        drain(d, 1);
}

void svac3_dec_reset(svac3_dec_t *d)
{
    abandon(d);
    if (d->nworkers)
        drain(d, 0);
    for (int i = 0; i < DPB_SLOTS; i++)
        d->pic[i].used = 0;
    for (int i = 0; i < LIB_SLOTS; i++)
        d->lib[i].used = 0;
    d->have_prev = 0;
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
    if (t == SVAC3_NAL_SPS || t == SVAC3_NAL_PPS || t == SVAC3_NAL_PIC_HEAD) {
        if (d->open) {
            abandon(d);
            lost = 1;
        }
        if (d->collect) {
            drop_collect(d);
            lost = 1;
        }
    }
    int ret = svac3_parse_nal(&d->hls, nal, len, 0, NULL);
    if (ret)
        return ret;
    if (!svac3_nal_is_patch(t))
        return lost ? SVAC3_ERR_CORRUPT : SVAC3_OK;
    if (d->nworkers)
        return queue_patch(d, nal, len);
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
