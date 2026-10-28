/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright (c) 2026 OpenIPC contributors
 *
 * SVAC3 deblocking (5.4.11) and basic SAO (5.4.12.1), 8-bit 4:2:0. The
 * picture is a single patch, so patch boundaries never stop a filter.
 */
#include "svac3_loopfilter.h"

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

/* A luma sample pair across an edge lies on a coding-unit boundary. */
static inline int cu_edge(const svac3_dbk_info_t *d, int xp, int yp, int xq, int yq)
{
    return d->cu_id[unit(d, xp, yp)] != d->cu_id[unit(d, xq, yq)];
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

/* 5.4.11.4 (intra in both coding units: Bs from the samples) and the
 * filters of 5.4.11.5-5.4.11.10. s points at q0; step is the distance
 * between p0 and q0 (1 for a vertical edge, stride for a horizontal one). */
static void filter_line(uint8_t *s, int step, int qpav, const svac3_dbk_info_t *d, int chroma)
{
    const int p0 = s[-step], p1 = s[-2 * step], p2 = s[-3 * step];
    const int q0 = s[0], q1 = s[step], q2 = s[2 * step];
    const int iA = clip3(0, 63, qpav + d->alpha_offset), iB = clip3(0, 63, qpav + d->beta_offset);
    const int alpha = alpha_tab[iA], beta = beta_tab[iB];
    int fL = 0, fR = 0, bs;

    if (d->filter_type == 1 && iabs(p0 - q0) >= 4)
        return;
    if (iabs(p0 - p1) < beta)
        fL += 2;
    if (iabs(p0 - p2) < beta)
        fL++;
    if (iabs(q0 - q1) < beta)
        fR += 2;
    if (iabs(q0 - q2) < beta)
        fR++;
    switch (fL + fR) {
    case 6:
        bs = (iabs(p0 - p1) <= beta / 4 && iabs(q0 - q1) <= beta / 4 && iabs(p0 - q0) < alpha) ? 4 : 3;
        if (d->filter_type == 1 && bs == 4 &&
            !(iabs(p0 - s[-4 * step]) <= beta / 2 && iabs(q0 - s[3 * step]) <= beta / 2))
            bs = 3;
        break;
    case 5:
        bs = (p0 == p1 && q0 == q1) ? 3 : 2;
        if (d->filter_type == 1 && bs == 3 && !(iabs(p2 - q2) < alpha))
            bs = 2;
        break;
    case 4:
        bs = fL == 2 ? 2 : 1;
        break;
    case 3:
        bs = iabs(p1 - q1) < beta ? 1 : 0;
        break;
    default:
        bs = 0;
        break;
    }
    if (chroma && bs)
        bs--;
    if (!bs)
        return;

    if (chroma) {
        s[-step] = (uint8_t)((p1 * 3 + p0 * 10 + q0 * 3 + 8) >> 4);
        s[0] = (uint8_t)((p0 * 3 + q0 * 10 + q1 * 3 + 8) >> 4);
        if (bs == 3) {
            s[-2 * step] = (uint8_t)((p2 * 3 + p1 * 8 + p0 * 3 + q0 * 2 + 8) >> 4);
            s[step] = (uint8_t)((q2 * 3 + q1 * 8 + q0 * 3 + p0 * 2 + 8) >> 4);
        }
        return;
    }
    const int p3 = s[-4 * step], q3 = s[3 * step];
    switch (bs) {
    case 4:
        s[-step]     = (uint8_t)((p2 * 3 + p1 * 8 + p0 * 10 + q0 * 8 + q1 * 3 + 16) >> 5);
        s[-2 * step] = (uint8_t)((p2 * 4 + p1 * 5 + p0 * 4 + q0 * 3 + 8) >> 4);
        s[-3 * step] = (uint8_t)((p3 * 2 + p2 * 2 + p1 * 2 + p0 + q0 + 4) >> 3);
        s[0]         = (uint8_t)((p1 * 3 + p0 * 8 + q0 * 10 + q1 * 8 + q2 * 3 + 16) >> 5);
        s[step]      = (uint8_t)((p0 * 3 + q0 * 4 + q1 * 5 + q2 * 4 + 8) >> 4);
        s[2 * step]  = (uint8_t)((p0 + q0 + q1 * 2 + q2 * 2 + q3 * 2 + 4) >> 3);
        break;
    case 3:
        s[-step]     = (uint8_t)((p2 + (p1 << 2) + (p0 << 2) + (p0 << 1) + (q0 << 2) + q1 + 8) >> 4);
        s[-2 * step] = (uint8_t)(((p2 << 1) + p2 + (p1 << 3) + (p0 << 2) + q0 + 8) >> 4);
        s[0]         = (uint8_t)((p1 + (p0 << 2) + (q0 << 2) + (q0 << 1) + (q1 << 2) + q2 + 8) >> 4);
        s[step]      = (uint8_t)(((q2 << 1) + q2 + (q1 << 3) + (q0 << 2) + p0 + 8) >> 4);
        break;
    case 2:
        s[-step] = (uint8_t)(((p1 << 1) + p1 + (p0 << 3) + (p0 << 1) + (q0 << 1) + q0 + 8) >> 4);
        s[0]     = (uint8_t)(((p0 << 1) + p0 + (q0 << 3) + (q0 << 1) + (q1 << 1) + q1 + 8) >> 4);
        break;
    case 1:
        s[-step] = (uint8_t)(((p0 << 1) + p0 + q0 + 2) >> 2);
        s[0]     = (uint8_t)(((q0 << 1) + q0 + p0 + 2) >> 2);
        break;
    }
}

/* luma edges of the 8x8 filter block at (bx, by): ver = vertical edge */
static void luma_edge(svac3_frame_t *f, const svac3_dbk_info_t *d, int bx, int by, int ver)
{
    uint8_t *Y = f->plane[0];
    const int s = f->stride[0];
    if (ver ? bx == 0 : by == 0)
        return;                              /* picture boundary */
    for (int i = 0; i < 8; i++) {
        const int x = ver ? bx : bx + i, y = ver ? by + i : by;
        if (x >= d->pic_w || y >= d->pic_h)
            break;
        const int xp = ver ? x - 1 : x, yp = ver ? y : y - 1;
        if (!cu_edge(d, xp, yp, x, y) || inter_bs0(d, unit(d, xp, yp), unit(d, x, y)))
            continue;
        const int qpav = (d->qp[unit(d, xp, yp)] + d->qp[unit(d, x, y)] + 1) >> 1;
        filter_line(Y + y * s + x, ver ? 1 : s, qpav, d, 0);
    }
}

static void chroma_edge(svac3_frame_t *f, const svac3_dbk_info_t *d, int bx, int by, int ver)
{
    if (ver ? bx == 0 : by == 0)
        return;
    for (int c = 1; c <= 2; c++) {
        uint8_t *P = f->plane[c];
        const int s = f->stride[c], delta = c == 1 ? d->cb_delta : d->cr_delta;
        for (int i = 0; i < 8; i++) {
            const int x = ver ? bx : bx + i, y = ver ? by + i : by;
            if (x >= f->width[c] || y >= f->height[c])
                break;
            const int lx = 2 * x, ly = 2 * y;
            const int lxp = ver ? lx - 1 : lx, lyp = ver ? ly : ly - 1;
            if (!cu_edge(d, lxp, lyp, lx, ly) || inter_bs0(d, unit(d, lxp, lyp), unit(d, lx, ly)))
                continue;
            const int qpp = svac3_chroma_qp(d->qp[unit(d, lxp, lyp)], delta);
            const int qpq = svac3_chroma_qp(d->qp[unit(d, lx, ly)], delta);
            filter_line(P + y * s + x, ver ? 1 : s, (qpp + qpq + 1) >> 1, d, 1);
        }
    }
}

void svac3_deblock(svac3_frame_t *f, const svac3_dbk_info_t *d)
{
    const int lw = d->pic_w, lh = d->pic_h, cw = f->width[1], ch = f->height[1];

    for (int by = 0; by < lh; by += 8)
        for (int bx = 0; bx < lw; bx += 8)
            luma_edge(f, d, bx, by, 1);
    for (int by = 0; by < lh; by += 8)
        for (int bx = 0; bx < lw; bx += 8)
            luma_edge(f, d, bx, by, 0);
    for (int by = 0; by < ch; by += 8)
        for (int bx = 0; bx < cw; bx += 8)
            chroma_edge(f, d, bx, by, 1);
    for (int by = 0; by < ch; by += 8)
        for (int bx = 0; bx < cw; bx += 8)
            chroma_edge(f, d, bx, by, 0);
}

/* ------------------------------------------------------------------------ */
/* SAO, 5.4.12.1 */

int svac3_sao(svac3_frame_t *f, const svac3_sao_t (*sao)[3], int lcu_size, int lcu_w, int lcu_h)
{
    for (int c = 0; c < 3; c++) {
        const int W = f->width[c], H = f->height[c], s = f->stride[c];
        const int L = c ? lcu_size / 2 : lcu_size;
        uint8_t *src = malloc((size_t)s * H);   /* SAO reads deblocked samples */
        if (!src)
            return -1;
        memcpy(src, f->plane[c], (size_t)s * H);
        uint8_t *dst = f->plane[c];

        for (int ly = 0; ly < lcu_h; ly++)
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

                if (p->mode == 1) {
                    int band[32] = { 0 };
                    band[p->start_pos] = p->offset[0];
                    band[(p->start_pos + 1) % 32] = p->offset[1];
                    band[(p->start_pos + p->delta_pos_minus2 + 2) % 32] = p->offset[2];
                    band[(p->start_pos + p->delta_pos_minus2 + 3) % 32] = p->offset[3];
                    for (int y = y0; y < y1; y++)
                        for (int x = x0; x < x1; x++)
                            dst[y * s + x] = (uint8_t)clip1(src[y * s + x] + band[src[y * s + x] >> 3]);
                } else {
                    static const int dx[4][2] = { { -1, 1 }, { 0, 0 }, { -1, 1 }, { 1, -1 } };
                    static const int dy[4][2] = { { 0, 0 }, { -1, 1 }, { -1, 1 }, { -1, 1 } };
                    const int t = p->edge_type;
                    for (int y = y0; y < y1; y++)
                        for (int x = x0; x < x1; x++) {
                            const int ax = x + dx[t][0], ay = y + dy[t][0];
                            const int bx = x + dx[t][1], by = y + dy[t][1];
                            if (ax < 0 || ay < 0 || ax >= W || ay >= H ||
                                bx < 0 || by < 0 || bx >= W || by >= H)
                                continue;
                            const int xc = src[y * s + x], xa = src[ay * s + ax], xb = src[by * s + bx];
                            int o = 0;
                            if (xc < xa && xc < xb)
                                o = p->offset[0];
                            else if ((xc < xa && xc == xb) || (xc == xa && xc < xb))
                                o = p->offset[1];
                            else if ((xc > xa && xc == xb) || (xc == xa && xc > xb))
                                o = p->offset[2];
                            else if (xc > xa && xc > xb)
                                o = p->offset[3];
                            dst[y * s + x] = (uint8_t)clip1(xc + o);
                        }
                }
            }
        free(src);
    }
    return 0;
}
