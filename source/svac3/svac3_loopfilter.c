/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright (c) 2026 OpenIPC contributors
 *
 * SVAC3 deblocking (5.4.11) and basic SAO (5.4.12.1), 8-bit 4:2:0. The
 * picture is a single patch, so patch boundaries never stop a filter.
 */
#include "svac3_loopfilter.h"
#include "svac3_dsp.h"

#include <stdlib.h>
#include <string.h>

static inline int clip3(int lo, int hi, int v) { return v < lo ? lo : v > hi ? hi : v; }
static inline int clip1(int v) { return clip3(0, 255, v); }
static inline int iabs(int v) { return v < 0 ? -v : v; }

/* 表137 */
static const uint8_t alpha_tab[64] = {
    0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 4,
    4, 4, 5, 5, 6, 6, 7, 7, 8, 9, 10, 10, 11, 12, 13, 15, 16, 17, 19, 21, 23, 25, 27, 29, 32, 35,
    38, 41, 45, 49, 54, 59,
};
static const uint8_t beta_tab[64] = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3,
    4, 4, 5, 5, 5, 6, 6, 7, 8, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 23,
    24, 24, 25, 25, 26, 27,
};

/* ------------------------------------------------------------------------ */
/* Deblocking */

static inline int unit(const svac3_dbk_info_t *d, int x, int y)
{
    return (y >> 2) * d->stride + (x >> 2);
}

/* 5.4.11.4, first paragraph: an edge between two inter coding units with
 * no coefficients, the same reference and motion vectors less than one
 * sample apart has Bs 0 (P pictures: list 1 is unused on both sides). */
static int inter_bs0(const svac3_dbk_info_t *d, int up, int uq)
{
    if (!d->inter || d->cbf[up] || d->cbf[uq] || d->ref[up] < 0 || d->ref[uq] < 0)
        return 0;
    return d->ref[up] == d->ref[uq] && iabs(d->mv[2 * up] - d->mv[2 * uq]) < 4 &&
           iabs(d->mv[2 * up + 1] - d->mv[2 * uq + 1]) < 4;
}

/* Deblocking parameters of the 4x4 unit pair across an edge (P on the left
 * or above): alpha < 0 when the edge is not filtered */
static void edge_params(const svac3_dbk_info_t *d, int xp, int yp, int xq, int yq,
                        int chroma_delta, int chroma, int *alpha, int *beta)
{
    const int up = unit(d, xp, yp), uq = unit(d, xq, yq);
    *alpha = *beta = -1;
    if (d->cu_id[up] == d->cu_id[uq] || inter_bs0(d, up, uq))
        return;
    int qpav;
    if (chroma)
        qpav = (svac3_chroma_qp(d->qp[up], chroma_delta) + svac3_chroma_qp(d->qp[uq], chroma_delta) + 1) >> 1;
    else
        qpav = (d->qp[up] + d->qp[uq] + 1) >> 1;
    *alpha = alpha_tab[clip3(0, 63, qpav + d->alpha_offset)];
    *beta = beta_tab[clip3(0, 63, qpav + d->beta_offset)];
}

/* All vertical, then all horizontal edges of one plane: edges 8 samples
 * apart touch disjoint samples, so each run of 8 lines across one edge is
 * filtered at once, with the parameters of each 4x4 luma unit pair (4 luma
 * or 2 chroma lines). */
static void plane_edges(svac3_frame_t *f, const svac3_dbk_info_t *d, int c, int ver,
                        const svac3_dsp_t *dsp, int b0, int b1)
{
    uint8_t *P = f->plane[c];
    const ptrdiff_t s = f->stride[c];
    const int W = f->width[c], H = f->height[c], chroma = c != 0;
    const int group = chroma ? 2 : 4, sc = chroma ? 2 : 1;
    const int delta = c == 1 ? d->cb_delta : c == 2 ? d->cr_delta : 0;
    int alpha[4], beta[4];

    /* rows b0..b1 of the plane: the lines of vertical edges, or the
     * positions of horizontal ones */
    if (b1 > H)
        b1 = H;
    const int e0 = ver ? 8 : (b0 > 8 ? b0 : 8), e1 = ver ? W : b1;
    const int a0 = ver ? b0 : 0, a1 = ver ? b1 : W;
    for (int e = e0; e < e1; e += 8)                     /* the edge's position */
        for (int a = a0; a < a1; a += 8) {               /* along it */
            const int n = a1 - a < 8 ? a1 - a : 8;
            int any = 0;
            for (int g = 0; g * group < n; g++) {
                const int along = (a + g * group) * sc, across = e * sc;
                if (ver)
                    edge_params(d, across - 1, along, across, along, delta, chroma, &alpha[g], &beta[g]);
                else
                    edge_params(d, along, across - 1, along, across, delta, chroma, &alpha[g], &beta[g]);
                any |= alpha[g] >= 0;
            }
            if (!any)
                continue;
            uint8_t *q0 = ver ? P + (size_t)a * s + e : P + (size_t)e * s + a;
            dsp->deblock(q0, ver ? 1 : s, ver ? s : 1, n, group, alpha, beta, chroma,
                         d->filter_type);
        }
}

/* Bands of 64 luma (32 chroma) rows: within one direction, the edges of
 * different bands touch different samples, so bands run in parallel */
#define BAND 64

typedef struct {
    svac3_frame_t *f;
    const svac3_dbk_info_t *d;
    const svac3_dsp_t *dsp;
    int ver, bands[3], y0[3], y1[3];
} dbk_job_t;

static void dbk_band(void *arg, int i)
{
    const dbk_job_t *j = arg;
    int c = 0;
    while (i >= j->bands[c]) {
        i -= j->bands[c];
        c++;
    }
    const int h = c ? BAND / 2 : BAND, b0 = j->y0[c] + i * h;
    plane_edges(j->f, j->d, c, j->ver, j->dsp, b0, b0 + h < j->y1[c] ? b0 + h : j->y1[c]);
}

void svac3_deblock_rows(svac3_frame_t *f, const svac3_dbk_info_t *d, int y0, int y1,
                        svac3_pool_t *pool)
{
    dbk_job_t j = { .f = f, .d = d, .dsp = svac3_dsp() };
    if (y1 > f->height[0])
        y1 = f->height[0];
    for (int c = 0; c < 3; c++) {
        const int h = c ? BAND / 2 : BAND;
        j.y0[c] = c ? y0 / 2 : y0;
        j.y1[c] = c ? (y1 == f->height[0] ? f->height[c] : y1 / 2) : y1;
        j.bands[c] = j.y1[c] > j.y0[c] ? (j.y1[c] - j.y0[c] + h - 1) / h : 0;
    }
    const int n = j.bands[0] + j.bands[1] + j.bands[2];
    /* the vertical edges of the rows, then the horizontal ones */
    j.ver = 1;
    svac3_pool_run(pool, dbk_band, &j, n);
    j.ver = 0;
    svac3_pool_run(pool, dbk_band, &j, n);
}

void svac3_deblock(svac3_frame_t *f, const svac3_dbk_info_t *d, svac3_pool_t *pool)
{
    svac3_deblock_rows(f, d, 0, f->height[0], pool);
}

/* ------------------------------------------------------------------------ */
/* SAO, 5.4.12.1 */

typedef struct {
    uint8_t *dst;
    const uint8_t *src;
    const svac3_sao_t (*sao)[3];
    const svac3_dsp_t *dsp;
    int c, W, H, s, L, lcu_w, lcu_h;
    int row0;                    /* sao_row_at(): the first LCU row */
} sao_job_t;

/* One row of LCUs of one component: SAO reads the deblocked copy, so rows
 * run in parallel */
static void sao_row(void *arg, int ly)
{
    const sao_job_t *j = arg;
    uint8_t *dst = j->dst;
    const uint8_t *src = j->src;
    const svac3_sao_t (*sao)[3] = j->sao;
    const svac3_dsp_t *dsp = j->dsp;
    const int c = j->c, W = j->W, H = j->H, s = j->s, L = j->L, lcu_w = j->lcu_w, lcu_h = j->lcu_h;
    for (int lx = 0; lx < lcu_w; lx++) {
        const svac3_sao_t *p = &sao[ly * lcu_w + lx][c];
        if (p->mode == 0)
            continue;
        /* 5.4.12.1.2: the LCU moved up and left by 4, clipped to the
         * picture, and extended to the right/bottom picture edge */
        int x0 = lx * L - 4, y0 = ly * L - 4, x1 = (lx + 1) * L - 4, y1 = (ly + 1) * L - 4;
        if (x0 < 0)
            x0 = 0;
        if (y0 < 0)
            y0 = 0;
        if (lx == lcu_w - 1 || x1 > W)
            x1 = W;
        if (ly == lcu_h - 1 || y1 > H)
            y1 = H;

        const ptrdiff_t off = (ptrdiff_t)y0 * s + x0;
        if (p->mode == 1) {
            int8_t band[32] = { 0 };
            band[p->start_pos] = p->offset[0];
            band[(p->start_pos + 1) % 32] = p->offset[1];
            band[(p->start_pos + p->delta_pos_minus2 + 2) % 32] = p->offset[2];
            band[(p->start_pos + p->delta_pos_minus2 + 3) % 32] = p->offset[3];
            dsp->sao_band(dst + off, src + off, s, x1 - x0, y1 - y0, band);
        } else {
            /* neighbours a and b of each sample; a sample with one
             * outside the picture is left as it is, so the region
             * shrinks to those with both inside */
            static const int dx[4][2] = { { -1, 1 }, { 0, 0 }, { -1, 1 }, { 1, -1 } };
            static const int dy[4][2] = { { 0, 0 }, { -1, 1 }, { -1, 1 }, { -1, 1 } };
            const int t = p->edge_type;
            const int ex = dx[t][0] != 0, ey = dy[t][0] != 0;
            const int xs = x0 > ex ? x0 : ex, xe = x1 < W - ex ? x1 : W - ex;
            const int ys = y0 > ey ? y0 : ey, ye = y1 < H - ey ? y1 : H - ey;
            if (xs >= xe || ys >= ye)
                continue;
            const int8_t o[5] = { (int8_t)p->offset[0], (int8_t)p->offset[1], 0,
                                  (int8_t)p->offset[2], (int8_t)p->offset[3] };
            const ptrdiff_t at = (ptrdiff_t)ys * s + xs;
            dsp->sao_edge(dst + at, src + at, s, xe - xs, ye - ys,
                          dy[t][0] * (ptrdiff_t)s + dx[t][0], dy[t][1] * (ptrdiff_t)s + dx[t][1], o);
        }
    }
}

static void sao_row_at(void *arg, int i)
{
    const sao_job_t *j = arg;
    sao_row(arg, j->row0 + i);
}

void svac3_sao_rows(svac3_frame_t *f, uint8_t *const src[3], const svac3_sao_t (*sao)[3],
                    int lcu_size, int lcu_w, int lcu_h, int r0, int r1, svac3_pool_t *pool)
{
    const svac3_dsp_t *dsp = svac3_dsp();
    for (int c = 0; c < 3; c++) {
        const int L = c ? lcu_size / 2 : lcu_size;
        sao_job_t j = { .dst = f->plane[c], .src = src[c], .sao = sao, .dsp = dsp, .c = c,
                        .W = f->width[c], .H = f->height[c], .s = f->stride[c], .L = L,
                        .lcu_w = lcu_w, .lcu_h = lcu_h };
        /* sao_row() reads the rows it needs from src, at the frame's offsets */
        for (int r = r0; r < r1; r += 64) {
            const int n = r1 - r < 64 ? r1 - r : 64;
            sao_job_t jj = j;
            jj.row0 = r;
            svac3_pool_run(pool, sao_row_at, &jj, n);
        }
    }
}

int svac3_sao(svac3_frame_t *f, const svac3_sao_t (*sao)[3], int lcu_size, int lcu_w, int lcu_h,
              svac3_pool_t *pool)
{
    uint8_t *src[3];
    for (int c = 0; c < 3; c++) {
        const size_t n = (size_t)f->stride[c] * f->height[c];
        src[c] = malloc(n);                     /* SAO reads deblocked samples */
        if (!src[c]) {
            while (c--)
                free(src[c]);
            return -1;
        }
        memcpy(src[c], f->plane[c], n);
    }
    svac3_sao_rows(f, src, sao, lcu_size, lcu_w, lcu_h, 0, lcu_h, pool);
    for (int c = 0; c < 3; c++)
        free(src[c]);
    return 0;
}
