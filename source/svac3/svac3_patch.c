/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright (c) 2026 OpenIPC contributors
 *
 * SVAC3 patch data parser. Each function names the table or section of the
 * SVAC 3.0 draft it implements; context indices are the ctxIndexStart
 * values of 表77.
 */
#include "svac3_patch.h"
#include "svac3_inter.h"
#include "svac3_recon.h"
#include "svac3_loopfilter.h"

#include <stdlib.h>
#include <string.h>

/* ctxIndexStart, 表77 (IsPrivacyFlag 0) */
enum {
    CTX_LCU_QP_DELTA     = 0,
    CTX_SAO_MERGE        = 4,
    CTX_SAO_MODE         = 7,
    CTX_SAO_OFFSET       = 8,
    CTX_QT_SPLIT         = 10,
    CTX_BET_SPLIT        = 14,
    CTX_BET_SPLIT_TYPE   = 23,
    CTX_BET_SPLIT_DIR    = 26,
    CTX_ROOT_CU          = 31,
    CTX_INTRA_CHROMA     = 33,
    CTX_CTP_U            = 39,
    CTX_CTP_V            = 40,
    CTX_SKIP             = 43,
    CTX_DIRECT           = 50,
    CTX_INTRA_CU         = 57,
    CTX_CU_SUBTYPE       = 94,
    CTX_REF_IDX          = 127,
    CTX_MVD_X            = 130,
    CTX_MVD_Y            = 133,
    CTX_INTRA_LUMA       = 153,
    CTX_LUMA_EIPM        = 160,
    CTX_CTP_ZERO         = 163,
    CTX_CTP_Y            = 166,
    CTX_CU_QP_DELTA      = 167,
    CTX_SCAN_REGION_X    = 178,
    CTX_SCAN_REGION_Y    = 206,
    CTX_SIG              = 234,
    CTX_GT               = 294,
};

/* Annex L */
static const int prefix_ctx[8]    = { 0, 0, 0, 3, 6, 10, 15, 21 };
static const int min_in_group[10] = { 0, 1, 2, 3, 4, 6, 8, 12, 16, 24 };
static const int group_index[32]  = { 0, 1, 2, 3, 4, 4, 5, 5, 6, 6, 6, 6, 7, 7, 7, 7,
                                      8, 8, 8, 8, 8, 8, 8, 8, 9, 9, 9, 9, 9, 9, 9, 9 };

enum { PRED_NO_CONSTRAINT, PRED_INTRA_ONLY, PRED_INTER_ONLY };
enum { COMP_LUMACHROMA, COMP_LUMA, COMP_CHROMA };
enum { NO_SPLIT, SPLIT_QT, SPLIT_BT_VER, SPLIT_BT_HOR, SPLIT_EQT_VER, SPLIT_EQT_HOR };

/* What later coding units need to know about a decoded one, per 4x4. */
typedef struct {
    uint8_t coded;
    uint8_t w, h;            /* CU size in luma samples */
    uint8_t skip, intra;
    uint8_t ipm;             /* IntraLumaPredMode, when intra */
    uint8_t qp;              /* QPY of the coding unit */
    uint8_t cbf;             /* the coding unit has coefficients */
    uint8_t patch;           /* patch_index of the patch it was decoded in */
    int8_t ref;              /* L0 reference index, -1 when intra */
    int16_t mv[2];           /* L0 motion vector */
    uint32_t cu_id;          /* which coding unit, for the deblocking filter */
} scu_t;

typedef struct {
    svac3_aec_t aec;
    svac3_ctx_t ctx[SVAC3_NUM_CTX];
    const svac3_sps_t *sps;
    const svac3_pps_t *pps;
    const svac3_ph_t *ph;
    svac3_patch_stats_t *st;

    int pct;                 /* picture_coding_type */
    int pic_w, pic_h, lcu_size, lcu_w, lcu_h;
    int min_qt, max_bt, min_bt, max_eqt, min_eqt, max_part_ratio, max_split;
    int fixed_qp, cu_delta_qp_flag, qp_group_area;
    int num_ref_active0;
    int num_delta_qp;
    int component, luma_w, luma_h;

    int scu_stride;
    scu_t *scu;

    /* the patch being decoded, in LCUs: [plx0, plx1) x [ply0, ply1) */
    int cur_patch, plx0, plx1, ply0, ply1;
    int num_patches, next_patch;   /* patches the picture has, and the one due */
    svac3_tmv_t *tmv_out;

    /* scratch for one transform block */
    int16_t order[64 * 64];
    uint8_t sig[64 * 64];
    uint16_t absv[64 * 64];

    /* reconstruction */
    svac3_frame_t *frame;
    const svac3_refs_t *refs;
    int tmvp;                /* PictureTmvpEnableFlag */
    int recon;
    int mipf;
    int patch_qp, prev_cu_qp, lcu_qp;
    int qp_group_x, qp_group_y;
    int cb_delta, cr_delta;
    uint32_t next_cu_id;
    svac3_sao_t (*sao)[3];   /* per LCU */
    int16_t coef[3][64 * 64];
    int16_t resid[64 * 64];
    uint8_t pred[64 * 64];
} parser_t;

/* ------------------------------------------------------------------------ */

static inline int bin(parser_t *p, int ctx)
{
    return svac3_aec_decision(&p->aec, &p->ctx[ctx]);
}

static inline int byp(parser_t *p)
{
    return svac3_aec_bypass(&p->aec);
}

static int ilog2(int v)
{
    int l = 0;
    while ((1 << l) < v)
        l++;
    return l;
}

/* order-0 Exp-Golomb in bypass bins (表75/表99) */
static uint32_t byp_eg0(parser_t *p)
{
    int zeros = 0;
    uint32_t v = 0;
    while (!byp(p)) {
        if (++zeros > 24)
            return 0;            /* corrupt; the stuffing check will notice */
    }
    for (int i = 0; i < zeros; i++)
        v = (v << 1) | (uint32_t)byp(p);
    return ((1u << zeros) - 1) + v;
}

/* truncated unary in bypass bins: count zeros, at most max (表80) */
static int byp_tu(parser_t *p, int max)
{
    int v = 0;
    while (v < max && !byp(p))
        v++;
    return v;
}

/* The decoded unit at (x, y) of any patch */
static const scu_t *scu_any(const parser_t *p, int x, int y)
{
    if (x < 0 || y < 0 || x >= p->pic_w || y >= p->pic_h)
        return NULL;
    const scu_t *s = &p->scu[(y >> 2) * p->scu_stride + (x >> 2)];
    return s->coded ? s : NULL;
}

/* The unit at (x, y) if it is available: decoded, and in the current
 * patch, since a neighbour in another patch is not available */
static const scu_t *scu_at(const parser_t *p, int x, int y)
{
    const scu_t *s = scu_any(p, x, y);
    return s && s->patch == p->cur_patch ? s : NULL;
}

static void scu_mark(parser_t *p, int x0, int y0, int w, int h, int skip, int intra)
{
    for (int y = y0; y < y0 + h && y < p->pic_h; y += 4)
        for (int x = x0; x < x0 + w && x < p->pic_w; x += 4) {
            scu_t *s = &p->scu[(y >> 2) * p->scu_stride + (x >> 2)];
            s->coded = 1;
            s->w = (uint8_t)(w > 255 ? 255 : w);
            s->h = (uint8_t)(h > 255 ? 255 : h);
            s->skip = (uint8_t)skip;
            s->intra = (uint8_t)intra;
            s->cu_id = p->next_cu_id;
            s->patch = (uint8_t)p->cur_patch;
        }
    p->next_cu_id++;
}

/* ------------------------------------------------------------------------ */
/* SAO parameters, 表32; binarisation 5.3.3.4.6-10 (as in AVS3). */

static void parse_sao(parser_t *p, int lcu_x, int lcu_y, const uint8_t sao_en[3],
                      svac3_sao_t out[3])
{
    /* merge candidates are LCUs of the same patch */
    const int left = lcu_x > p->plx0, up = lcu_y > p->ply0;
    int merge = 0;

    memset(out, 0, sizeof(svac3_sao_t) * 3);
    if (left + up > 0) {
        int sym;
        if (left + up == 1) {
            sym = bin(p, CTX_SAO_MERGE + 0);
        } else {
            sym = bin(p, CTX_SAO_MERGE + 1);
            if (sym != 1)
                sym += bin(p, CTX_SAO_MERGE + 2) << 1;
        }
        /* 表138: with both neighbours, 1 = left and 2 = up */
        if (left && !up)
            merge = sym ? 1 : 0;
        else if (up && !left)
            merge = sym ? 2 : 0;
        else
            merge = sym;
    }
    if (merge) {
        if (p->sao) {
            const int l = lcu_y * p->lcu_w + lcu_x;
            memcpy(out, p->sao[merge == 1 ? l - 1 : l - p->lcu_w], sizeof(svac3_sao_t) * 3);
        }
        return;
    }

    for (int c = 0; c < 3; c++) {
        if (!sao_en[c])
            continue;
        /* sao_mode: truncated unary, maxVal 2; bin 0 coded, bin 1 bypass */
        int mode;
        if (bin(p, CTX_SAO_MODE))
            mode = 0;
        else
            mode = byp(p) ? 1 : 2;           /* 1 = interval, 2 = edge */
        out[c].mode = (uint8_t)mode;
        if (mode == 0)
            continue;
        if (mode == 1) {
            for (int j = 0; j < 4; j++) {
                /* sao_interval_offset_abs: TU maxVal 7, bin 0 coded */
                int a = !bin(p, CTX_SAO_OFFSET);
                if (a) {
                    a += byp_tu(p, 6);
                    if (byp(p))              /* sao_interval_offset_sign */
                        a = -a;
                }
                out[c].offset[j] = (int8_t)a;
            }
            int start = 0;                   /* sao_interval_start_pos, LSB first */
            for (int k = 0; k < 5; k++)
                start |= byp(p) << k;
            /* sao_interval_delta_pos_minus2 (表85) */
            const int order = 1 + byp_tu(p, 3);
            int d = (1 << order) - 2, v = 0;
            for (int k = order % 4 - 1; k >= 0; k--)
                v |= byp(p) << k;
            out[c].start_pos = (uint8_t)start;
            out[c].delta_pos_minus2 = (uint8_t)(d + v);
        } else {
            /* sao_edge_offset (表86/表87), then sao_edge_type */
            static const int inv0[8] = { 1, 0, 2, -1, 3, 4, 5, 6 };
            int cnt;
            cnt = byp_tu(p, 7);
            out[c].offset[0] = (int8_t)inv0[cnt];
            cnt = byp_tu(p, 1);
            out[c].offset[1] = (int8_t)cnt;
            cnt = byp_tu(p, 1);
            out[c].offset[2] = (int8_t)-cnt;
            cnt = byp_tu(p, 7);
            out[c].offset[3] = (int8_t)-inv0[cnt];
            int t = byp(p);
            t |= byp(p) << 1;
            out[c].edge_type = (uint8_t)t;
        }
    }
}

/* ------------------------------------------------------------------------ */
/* Transform block with scan-region-based coefficient coding, 表36. */

static int parse_scan_region(parser_t *p, int ctx_start, int M, int is_chroma)
{
    const int logM = ilog2(M);
    int offset, shift;

    if (!is_chroma) {
        offset = prefix_ctx[logM];
        shift = (logM + 1) >> 2;
    } else {
        offset = 25;
        shift = M >> 3;
        if (shift > 2)
            shift = 2;
    }
    const int maxp = group_index[(M < 32 ? M : 32) - 1];
    int count = 0;
    for (int b = 0; b < maxp; b++) {
        if (!bin(p, ctx_start + offset + (b >> shift)))
            break;
        count++;
    }
    if (count <= 3)
        return count;
    int v = 0;
    for (int i = ((count - 2) >> 1) - 1; i >= 0; i--)
        v += byp(p) << i;
    return min_in_group[count] + v;
}

static int pos_class(int px, int py, int srx, int sry, int intra)
{
    if (intra) {
        int i = (px <= srx / 2) ? (px <= srx / 4 ? 0 : 1) : (px <= srx * 3 / 4 ? 2 : 3);
        int j = (py <= sry / 2) ? (py <= sry / 4 ? 0 : 1) : (py <= sry * 3 / 4 ? 2 : 3);
        int s = i + j;
        return s <= 1 ? 4 : s > 2 ? 8 : 12;
    }
    return (px <= srx / 2 && py <= sry / 2) ? 4 : 8;
}

static void parse_srcc_block(parser_t *p, int w, int h, int is_chroma, int intra, int16_t *out)
{
    int16_t *order = p->order;
    uint8_t *sig = p->sig;
    uint16_t *absv = p->absv;
    const int log2w = ilog2(w);

    memset(out, 0, sizeof(*out) * (size_t)w * h);
    const int srx = parse_scan_region(p, CTX_SCAN_REGION_X, w, is_chroma);
    const int sry = parse_scan_region(p, CTX_SCAN_REGION_Y, h, is_chroma);
    if (srx >= w || sry >= h) {
        p->aec.overrun += 1;                 /* impossible region: mark corrupt */
        return;
    }

    /* initScanOrder (Annex K text) */
    {
        const int xs = srx + 1, ys = sry + 1;
        int pos = 0;
        order[pos++] = 0;
        for (int l = 1; l < xs + ys - 1; l++) {
            if (l % 2) {
                int x = l < xs - 1 ? l : xs - 1;
                int y = l - (xs - 1) > 0 ? l - (xs - 1) : 0;
                while (x >= 0 && y < ys) {
                    order[pos++] = (int16_t)(y * w + x);
                    x--;
                    y++;
                }
            } else {
                int y = l < ys - 1 ? l : ys - 1;
                int x = l - (ys - 1) > 0 ? l - (ys - 1) : 0;
                while (y >= 0 && x < xs) {
                    order[pos++] = (int16_t)(y * w + x);
                    x++;
                    y--;
                }
            }
        }
    }

    const int last_pos = (srx + 1) * (sry + 1) - 1;
    const int last_set = last_pos >> 4;
    const int area = (srx + 1) * (sry + 1);
    int is_last_x = 0, is_last_y = 0, first_done = 0;
    int pre_gt0[5] = { 0 }, pre_gtx[5] = { 0 };

    for (int s = last_set; s >= 0; s--) {
        const int set_pos = s << 4;
        const int top = (s == last_set) ? last_pos - set_pos : 15;
        int nz = 0;

        for (int n = top; n >= 0; n--) {
            const int bp = order[set_pos + n];
            const int px = bp & (w - 1), py = bp >> log2w;
            int f;
            if ((px == 0 && py == sry && !is_last_y) || (py == 0 && px == srx && !is_last_x)) {
                f = 1;
            } else {
                int inc = is_chroma ? 51 : (area <= 4 ? 0 : (17 << (area <= 16 ? 0 : 1)));
                if (px != srx || py != sry) {
                    if (px || py)
                        inc += is_chroma ? 4 : pos_class(px, py, srx, sry, intra);
                    int num = 0;
                    for (int j = 0; j < 5; j++)
                        num += pre_gt0[j] != 0;
                    inc += (num < 3 ? num : 3) + 1;
                }
                f = bin(p, CTX_SIG + inc);
            }
            sig[bp] = (uint8_t)f;
            if (f) {
                if (px == srx)
                    is_last_x = 1;
                if (py == sry)
                    is_last_y = 1;
                nz++;
            }
            for (int j = 4; j > 0; j--)
                pre_gt0[j] = pre_gt0[j - 1];
            pre_gt0[0] = f;
        }

        if (!nz)
            continue;
        int escape = 0;
        for (int n = top; n >= 0; n--) {
            const int bp = order[set_pos + n];
            if (!sig[bp])
                continue;
            const int px = bp & (w - 1), py = bp >> log2w;
            int base = is_chroma ? 17 : 0, cls = 0;
            if (first_done && !is_chroma && (px || py))
                cls = pos_class(px, py, srx, sry, intra);
            int a;
            for (int k = 1;; k = 2) {
                int inc = base;
                if (first_done) {
                    int num = 0;
                    for (int j = 0; j < 5; j++)
                        num += pre_gtx[j] > k;
                    inc += cls + (num < 3 ? num : 3) + 1;
                }
                int g = bin(p, CTX_GT + inc);
                if (k == 1) {
                    if (!g) {
                        a = 1;
                        break;
                    }
                } else {
                    a = g ? 3 : 2;
                    if (g)
                        escape = 1;
                    break;
                }
            }
            absv[bp] = (uint16_t)a;
            first_done = 1;
            for (int j = 4; j > 0; j--)
                pre_gtx[j] = pre_gtx[j - 1];
            pre_gtx[0] = a;
        }
        if (escape) {
            for (int n = top; n >= 0; n--) {
                const int bp = order[set_pos + n];
                if (sig[bp] && absv[bp] == 3)
                    absv[bp] = (uint16_t)(absv[bp] + byp_eg0(p));
            }
        }
        for (int n = top; n >= 0; n--) {
            const int bp = order[set_pos + n];
            if (sig[bp]) {
                const int neg = byp(p);      /* coeff_sign */
                const int px = bp & (w - 1), py = bp >> log2w;
                int a = absv[bp] > 32767 ? 32767 : absv[bp];
                out[py * w + px] = (int16_t)(neg ? -a : a);
                p->st->coeffs++;
            }
        }
    }
    p->st->coded_blocks++;
}

/* ------------------------------------------------------------------------ */
/* Coding unit, 表35, restricted to the supported tool set. */

static int parse_mvd_abs(parser_t *p, int ctx)
{
    /* 表96: three coded bins, then parity and order-0 Exp-Golomb in bypass */
    if (!bin(p, ctx + 0))
        return 0;
    if (!bin(p, ctx + 1))
        return 1;
    if (!bin(p, ctx + 2))
        return 2;
    int even = byp(p);
    return 3 + 2 * (int)byp_eg0(p) + even;
}

static int avail_cb(void *opaque, int x, int y)
{
    return scu_at((const parser_t *)opaque, x, y) != NULL;
}

/* 5.4.6.6.3.2: IntraLumaPredMode from the parsed index and the neighbours */
static int derive_luma_mode(const parser_t *p, int x0, int y0, int idx, int eipm)
{
    const scu_t *A = scu_at(p, x0 - 1, y0), *B = scu_at(p, x0, y0 - 1);
    const int a = (A && A->intra) ? A->ipm : 0, b = (B && B->intra) ? B->ipm : 0;
    int pred0, pred1;

    if (a != b) {
        pred0 = a < b ? a : b;
        pred1 = a < b ? b : a;
    } else if (a == 0) {
        pred0 = 0;
        pred1 = 2;
    } else {
        pred0 = 0;
        pred1 = a;
    }
    if (idx == 0)
        return pred0;
    if (idx == 1)
        return pred1;
    int m = idx + eipm * 32;
    if (m - 2 < pred0)
        m -= 2;
    else if (m - 1 > pred0 && m - 1 < pred1)
        m -= 1;
    return m;
}

/* 5.4.6.6.3.4 */
static int derive_chroma_mode(int idx, int luma_mode)
{
    if (!(luma_mode == 0 || luma_mode == 2 || luma_mode == 12 || luma_mode == 24))
        return idx;
    const int pred = luma_mode == 0 ? 1 : luma_mode == 2 ? 4 : luma_mode == 12 ? 3 : 2;
    if (idx == 0)
        return 0;
    return idx < pred ? idx : idx + 1;
}

/* 5.4.6.2, first step */
static int derive_qp(parser_t *p, int x0, int y0, int ctp, int delta)
{
    if (p->fixed_qp || !p->cu_delta_qp_flag)
        return p->lcu_qp;
    if (x0 == p->qp_group_x && y0 == p->qp_group_y) {
        const scu_t *A = scu_at(p, x0 - 1, y0);
        p->prev_cu_qp = A ? A->qp : p->patch_qp;
    }
    /* CuDeltaQp is -32..31 for 8-bit video; a value outside it is not a
     * conforming stream, but must still give a QP in 0..63 */
    const int qp = ctp ? (((p->prev_cu_qp + delta) % 64) + 64) % 64 : p->prev_cu_qp;
    p->prev_cu_qp = qp;
    return qp;
}

static void recon_chroma(parser_t *p, int x0, int y0, int w, int h, int cmode, int lmode,
                         int qp, int ctp)
{
    const int K = w / 2, L = h / 2;
    for (int c = 1; c <= 2; c++) {
        svac3_intra_pred(p->frame, c, x0 / 2, y0 / 2, K, L, cmode, lmode, p->mipf,
                         avail_cb, p, p->pred);
        const int16_t *res = NULL;
        if (ctp & (1 << c)) {
            const int cqp = svac3_chroma_qp(qp, c == 1 ? p->cb_delta : p->cr_delta);
            svac3_dequant_itrans(p->coef[c], K, L, cqp, p->resid);
            res = p->resid;
        }
        svac3_put_block(p->frame, c, x0 / 2, y0 / 2, K, L, p->pred, res);
    }
}

/* ------------------------------------------------------------------------ */
/* Motion information, 5.4.6.7, for P pictures: one reference list, no
 * affine, AMVR, UMVE, HMVP or MVAP. */

static inline int clip3(int lo, int hi, int v) { return v < lo ? lo : v > hi ? hi : v; }

/* Scale mv from block distance bdx to bde (the formula of 5.4.6.7.6.2) */
static int scale_mv(int mv, int bde, int bdx)
{
    if (!bdx)
        bdx = 1;                             /* not a conforming stream */
    const int64_t m = (int64_t)mv * bde;
    const int64_t a = m * (16384 / bdx);
    const int64_t v = ((a < 0 ? -a : a) + 8192) >> 14;
    const int sign = (m == 0) ? 0 : ((m < 0) != (bdx < 0)) ? -1 : 1;
    const int64_t r = sign * v;
    return r < -32768 ? -32768 : r > 32767 ? 32767 : (int)r;
}

static int ref_dist(const parser_t *p, int ref)
{
    return p->refs->dist[ref];
}

/* L0 motion of the inter coding unit covering (x, y), or ref -1 */
static int nb_motion(const parser_t *p, int x, int y, int mv[2])
{
    const scu_t *s = scu_at(p, x, y);
    mv[0] = mv[1] = 0;
    if (!s || s->intra || s->ref < 0)
        return -1;
    mv[0] = s->mv[0];
    mv[1] = s->mv[1];
    return s->ref;
}

static int median3(int a, int b, int c)
{
    if ((a < 0 && b > 0 && c > 0) || (a > 0 && b < 0 && c < 0))
        return (b + c) / 2;
    if ((b < 0 && a > 0 && c > 0) || (b > 0 && a < 0 && c < 0))
        return (a + c) / 2;
    if ((c < 0 && a > 0 && b > 0) || (c > 0 && a < 0 && b < 0))
        return (a + b) / 2;
    const int ab = abs(a - b), bc = abs(b - c), ca = abs(c - a);
    if (ab <= bc && ab <= ca)
        return (a + b) / 2;
    if (bc <= ab && bc <= ca)
        return (b + c) / 2;
    return (a + c) / 2;
}

/* 5.4.6.7.6.2 and 5.4.6.7.7.2: the motion vector of a 'P_2M_2N' unit */
static void derive_mvp(const parser_t *p, int x0, int y0, int w, int ref, const int mvd[2],
                       int mv[2])
{
    const int x1 = x0 + w - 1;
    const int pos[4][2] = { { x0 - 1, y0 }, { x0, y0 - 1 }, { x1 + 1, y0 - 1 }, { x0 - 1, y0 - 1 } };
    int nmv[4][2], nref[4];

    /* A neighbour outside the picture is unavailable. 5.4.6.7.6.2 moves it
     * back inside the LCU and the picture first; the reference decoder does
     * not (docs/errata.md). */
    for (int i = 0; i < 4; i++)
        nref[i] = nb_motion(p, pos[i][0], pos[i][1], nmv[i]);
    if (nref[2] < 0) {                       /* C falls back to D */
        nref[2] = nref[3];
        nmv[2][0] = nmv[3][0];
        nmv[2][1] = nmv[3][1];
    }
    const int bde = ref_dist(p, ref);
    int MV[3][2] = { { 0 } }, n = 0, last = 0;
    for (int i = 0; i < 3; i++) {
        if (nref[i] < 0)
            continue;
        const int bdx = ref_dist(p, nref[i]);
        MV[i][0] = scale_mv(nmv[i][0], bde, bdx);
        MV[i][1] = scale_mv(nmv[i][1], bde, bdx);
        n++;
        last = i;
    }
    for (int c = 0; c < 2; c++) {
        const int pred = n == 1 ? MV[last][c] : median3(MV[0][c], MV[1][c], MV[2][c]);
        mv[c] = clip3(-32768, 32767, pred + mvd[c]);
    }
}

/* 5.4.6.7.8: 'P_Skip'/'P_Direct' with CuSubTypeIndex 0 (temporal, 5.4.6.7.8.4.2)
 * or 1 (spatial list 0, 5.4.6.7.8.3.2) */
static void derive_skip_motion(const parser_t *p, int x0, int y0, int w, int h, int subtype,
                               int *ref, int mv[2])
{
    *ref = 0;
    mv[0] = mv[1] = 0;
    if (subtype == 0) {
        if (!p->tmvp || !p->refs->col)
            return;
        const svac3_tmv_t *t = &p->refs->col[(y0 >> 4) * ((p->pic_w + 15) >> 4) + (x0 >> 4)];
        if (t->ref < 0)
            return;
        mv[0] = scale_mv(t->mv[0], ref_dist(p, 0), t->dist);
        mv[1] = scale_mv(t->mv[1], ref_dist(p, 0), t->dist);
        return;
    }
    const int x1 = x0 + w - 1, y1 = y0 + h - 1;
    const int pos[6][2] = { { x0 - 1, y1 }, { x1, y0 - 1 }, { x1 + 1, y0 - 1 },
                            { x0 - 1, y0 }, { x0, y0 - 1 }, { x0 - 1, y0 - 1 } };   /* F G C A B D */
    for (int i = 0; i < 6; i++) {
        const int r = nb_motion(p, pos[i][0], pos[i][1], mv);
        if (r >= 0) {
            *ref = r;
            return;
        }
    }
}

static void recon_inter(parser_t *p, int x0, int y0, int w, int h, int ref, const int mv[2],
                        int qp, int ctp, int chroma)
{
    const svac3_frame_t *rf = p->refs->ref[ref];
    svac3_mc_luma(rf, x0, y0, w, h, mv[0], mv[1], p->pred);
    const int16_t *res = NULL;
    if (ctp & 1) {
        svac3_dequant_itrans(p->coef[0], w, h, qp, p->resid);
        res = p->resid;
    }
    svac3_put_block(p->frame, 0, x0, y0, w, h, p->pred, res);
    if (!chroma)
        return;
    for (int c = 1; c <= 2; c++) {
        svac3_mc_chroma(rf, c, x0 / 2, y0 / 2, w / 2, h / 2, mv[0], mv[1], p->pred);
        res = NULL;
        if (ctp & (1 << c)) {
            const int cqp = svac3_chroma_qp(qp, c == 1 ? p->cb_delta : p->cr_delta);
            svac3_dequant_itrans(p->coef[c], w / 2, h / 2, cqp, p->resid);
            res = p->resid;
        }
        svac3_put_block(p->frame, c, x0 / 2, y0 / 2, w / 2, h / 2, p->pred, res);
    }
}

/* The chroma of a luma region coded as several inter coding units: each
 * 2x2 chroma unit is predicted with the motion of its 4x4 luma unit. No
 * stream in the corpus has one (the encoders do not split an inter-only
 * node below 8x8), so this path is not checked against the reference. */
static void recon_chroma_inter(parser_t *p, int x0, int y0, int w, int h, int qp, int ctp)
{
    const int K = w / 2, L = h / 2;
    for (int c = 1; c <= 2; c++) {
        for (int y = 0; y < L; y += 2)
            for (int x = 0; x < K; x += 2) {
                const scu_t *s = scu_at(p, x0 + 2 * x, y0 + 2 * y);
                uint8_t blk[4] = { 128, 128, 128, 128 };
                if (s && s->ref >= 0)
                    svac3_mc_chroma(p->refs->ref[s->ref], c, x0 / 2 + x, y0 / 2 + y, 2, 2,
                                    s->mv[0], s->mv[1], blk);
                for (int j = 0; j < 2; j++)
                    for (int i = 0; i < 2; i++)
                        p->pred[(y + j) * K + x + i] = blk[j * 2 + i];
            }
        const int16_t *res = NULL;
        if (ctp & (1 << c)) {
            const int cqp = svac3_chroma_qp(qp, c == 1 ? p->cb_delta : p->cr_delta);
            svac3_dequant_itrans(p->coef[c], K, L, cqp, p->resid);
            res = p->resid;
        }
        svac3_put_block(p->frame, c, x0 / 2, y0 / 2, K, L, p->pred, res);
    }
}

static void coding_unit_chroma(parser_t *p, int x0, int y0, int w, int h)
{
    const scu_t *luma = scu_at(p, x0 + w - 1, y0 + h - 1);
    const int prior_intra = luma && luma->intra;
    int ctp, cidx = 0;

    if (prior_intra)
        for (; cidx < 4 && !bin(p, CTX_INTRA_CHROMA + (cidx ? 1 : 0)); cidx++)
            ;
    ctp = (bin(p, CTX_CTP_U) << 1);
    ctp |= (bin(p, CTX_CTP_V) << 2);
    for (int i = 1; i < 3; i++)
        if (ctp & (1 << i))
            parse_srcc_block(p, w / 2, h / 2, 1, prior_intra, p->coef[i]);
    p->st->chroma_cus++;

    if (p->recon && luma && prior_intra)
        recon_chroma(p, x0, y0, w, h, derive_chroma_mode(cidx, luma->ipm), luma->ipm,
                     luma->qp, ctp);
    else if (p->recon && luma)
        recon_chroma_inter(p, x0, y0, w, h, luma->qp, ctp);
}

static void coding_unit(parser_t *p, int x0, int y0, int w, int h, int mode, int comp)
{
    int skip = 0, direct = 0, intra = p->pct == 1;
    int lidx = 0, eipm = 0, cidx = 0, dqp = 0;
    const scu_t *A = scu_at(p, x0 - 1, y0), *B = scu_at(p, x0, y0 - 1);

    if (p->pct != 1) {
        if (mode != PRED_INTRA_ONLY) {
            int inc = (w * h < 64) ? 3 : (A && A->skip) + (B && B->skip);
            skip = bin(p, CTX_SKIP + inc);
        }
        if (!skip) {
            if (mode != PRED_INTRA_ONLY)
                direct = bin(p, CTX_DIRECT + ((w * h < 64 || w > 64 || h > 64) ? 1 : 0));
            if (!direct && mode == PRED_NO_CONSTRAINT) {
                int inc;
                if (w > 64 || h > 64) {
                    inc = 5;
                } else {
                    inc = (A && A->intra) + (B && B->intra);
                    if (inc == 0)
                        inc = w * h > 256 ? 0 : w * h > 64 ? 3 : 4;
                }
                intra = bin(p, CTX_INTRA_CU + inc);
            } else {
                intra = !direct && mode == PRED_INTRA_ONLY;
            }
        }
        if (skip || direct)
            intra = 0;
    }

    int subtype = 0, ref = 0, mvd[2] = { 0, 0 };
    if (skip || direct) {
        /* cu_subtype_index: TU, maxVal 1 for P (no MVAP or HMVP candidates) */
        const int maxv = (p->pct == 2 ? 1 : 3);
        while (subtype < maxv && !bin(p, CTX_CU_SUBTYPE + subtype))
            subtype++;
    }

    if (!skip && !direct) {
        if (!intra) {
            if (p->num_ref_active0 > 1)
                while (ref < p->num_ref_active0 - 1 && !bin(p, CTX_REF_IDX + (ref < 2 ? ref : 2)))
                    ref++;
            for (int i = 0; i < 2; i++) {
                mvd[i] = parse_mvd_abs(p, i ? CTX_MVD_Y : CTX_MVD_X);
                if (mvd[i] && byp(p))        /* mv_diff_sign */
                    mvd[i] = -mvd[i];
            }
        } else {
            /* intra_luma_pred_mode_index (表78/表98), then luma_eipm_index */
            if (bin(p, CTX_INTRA_LUMA + 0)) {
                lidx = bin(p, CTX_INTRA_LUMA + 6);
            } else {
                lidx = 0;
                for (int b = 1; b <= 5; b++)
                    lidx = (lidx << 1) | bin(p, CTX_INTRA_LUMA + b);
                lidx += 2;
            }
            if (lidx > 1)
                eipm = bin(p, CTX_LUMA_EIPM);
            if (comp == COMP_LUMACHROMA)
                for (; cidx < 4 && !bin(p, CTX_INTRA_CHROMA + (cidx ? 1 : 0)); cidx++)
                    ;
        }
    }

    int ctp = 0, u = 0, v = 0, y = 0;
    if (!intra && !skip) {
        int zero = 0;
        if (!direct && comp == COMP_LUMACHROMA)
            zero = bin(p, CTX_CTP_ZERO + ((w > 64 || h > 64) ? 1 : 0));
        if (!zero) {
            if (comp == COMP_LUMACHROMA) {
                u = bin(p, CTX_CTP_U);
                v = bin(p, CTX_CTP_V);
            }
            y = (u || v || comp != COMP_LUMACHROMA) ? bin(p, CTX_CTP_Y) : 1;
            ctp = y | (u << 1) | (v << 2);
        }
    } else if (!skip) {
        y = bin(p, CTX_CTP_Y);
        if (comp == COMP_LUMACHROMA) {
            u = bin(p, CTX_CTP_U);
            v = bin(p, CTX_CTP_V);
        }
        ctp = y | (u << 1) | (v << 2);
    }

    if (!p->fixed_qp && p->cu_delta_qp_flag && ctp) {
        /* cu_qp_delta_abs: unary; bin 0 context from NumDeltaQp */
        int a = 0;
        int inc = p->num_delta_qp < 2 ? p->num_delta_qp : 2;
        while (!bin(p, CTX_CU_QP_DELTA + inc)) {
            a++;
            inc = 3;
            if (a > 128)
                break;
        }
        dqp = a;
        if (a && byp(p))                     /* cu_qp_delta_sign */
            dqp = -a;
        p->num_delta_qp++;
    }

    /* Chroma blocks are half the CU size (4:2:0). 表36's SRCC branch does not
     * halve them, but only the halved reading parses (docs/errata.md). */
    if (ctp & 1)
        parse_srcc_block(p, w, h, 0, intra, p->coef[0]);
    if (ctp & 2)
        parse_srcc_block(p, w / 2, h / 2, 1, intra, p->coef[1]);
    if (ctp & 4)
        parse_srcc_block(p, w / 2, h / 2, 1, intra, p->coef[2]);

    const int qp = derive_qp(p, x0, y0, ctp, dqp);
    const int lmode = intra ? derive_luma_mode(p, x0, y0, lidx, eipm) : 0;

    int mv[2] = { 0, 0 };
    if (p->recon && intra) {
        svac3_intra_pred(p->frame, 0, x0, y0, w, h, lmode, 0, p->mipf, avail_cb, p, p->pred);
        const int16_t *res = NULL;
        if (ctp & 1) {
            svac3_dequant_itrans(p->coef[0], w, h, qp, p->resid);
            res = p->resid;
        }
        svac3_put_block(p->frame, 0, x0, y0, w, h, p->pred, res);
    } else if (p->recon) {
        if (skip || direct)
            derive_skip_motion(p, x0, y0, w, h, subtype, &ref, mv);
        else
            derive_mvp(p, x0, y0, w, ref, mvd, mv);
        recon_inter(p, x0, y0, w, h, ref, mv, qp, ctp, comp == COMP_LUMACHROMA);
    }

    scu_mark(p, x0, y0, w, h, skip, intra);
    /* the mark above makes this CU's own samples "available"; record its mode,
     * QP and motion for later neighbours and for a deferred chroma CU */
    for (int yy = y0; yy < y0 + h && yy < p->pic_h; yy += 4)
        for (int xx = x0; xx < x0 + w && xx < p->pic_w; xx += 4) {
            scu_t *s = &p->scu[(yy >> 2) * p->scu_stride + (xx >> 2)];
            s->ipm = (uint8_t)lmode;
            s->qp = (uint8_t)qp;
            s->cbf = ctp != 0;
            s->ref = (int8_t)(intra ? -1 : ref);
            s->mv[0] = (int16_t)mv[0];
            s->mv[1] = (int16_t)mv[1];
        }

    if (p->recon && intra && comp == COMP_LUMACHROMA)
        recon_chroma(p, x0, y0, w, h, derive_chroma_mode(cidx, lmode), lmode, qp, ctp);

    p->st->cus++;
    p->st->intra_cus += intra;
    p->st->skip_cus += skip;
    p->st->direct_cus += direct;
}

/* ------------------------------------------------------------------------ */
/* Coding tree, 表33. */

static int split_ctx_ab(const parser_t *p, int x0, int y0, int w, int h)
{
    const scu_t *A = scu_at(p, x0 - 1, y0), *B = scu_at(p, x0, y0 - 1);
    return (A && A->h < h) + (B && B->w < w);
}

static void coding_unit_tree(parser_t *p, int x0, int y0, int split, int w, int h,
                             int qt, int mode)
{
    const int pw = p->pic_w, ph = p->pic_h;
    const int is_boundary = (x0 + w > pw) || (y0 + h > ph);
    const int right = (x0 + w > pw) && (y0 + h <= ph);
    const int bottom = (x0 + w <= pw) && (y0 + h > ph);
    int no_split = 0, s_qt = 0, bt_ver = 0, bt_hor = 0, eqt_ver = 0, eqt_hor = 0;
    const int I = p->pct == 1;

    if (is_boundary) {
        if (I && w > 64 && h > 64) {
            s_qt = 1;
            no_split = 1;
        } else if (((w == 64 && h > 64) || (h == 64 && w > 64)) && w <= p->max_bt && h <= p->max_bt) {
            bt_hor = bt_ver = 1;
        } else if ((!right && !bottom) || w > p->max_bt || h > p->max_bt) {
            s_qt = 1;
        } else if (right) {
            bt_ver = 1;
        } else if (bottom) {
            bt_hor = 1;
        }
    } else {
        if ((w == 64 && h > 64) || (h == 64 && w > 64)) {
            if (w <= p->max_bt && h <= p->max_bt)
                bt_hor = bt_ver = 1;
            no_split = 1;
        } else if (split >= p->max_split) {
            no_split = 1;
        } else if (I && w == 128 && h == 128) {
            s_qt = 1;
            no_split = 1;
        } else {
            const int mpr = p->max_part_ratio;
            if (w <= h * mpr && h <= w * mpr)
                no_split = 1;
            if (w > p->min_qt && qt)
                s_qt = 1;
            if (w <= p->max_bt && h <= p->max_bt && w > p->min_bt && h < mpr * w)
                bt_ver = 1;
            if (w <= p->max_bt && h <= p->max_bt && h > p->min_bt && w < mpr * h)
                bt_hor = 1;
            if (w <= p->max_eqt && h <= p->max_eqt && h >= p->min_eqt * 2 &&
                w >= p->min_eqt * 4 && h * 4 <= mpr * w)
                eqt_ver = 1;
            if (w <= p->max_eqt && h <= p->max_eqt && w >= p->min_eqt * 2 &&
                h >= p->min_eqt * 4 && w * 4 <= mpr * h)
                eqt_hor = 1;
        }
    }
    const int s_bt = bt_ver || bt_hor, s_eqt = eqt_ver || eqt_hor;

    int qt_flag = s_qt, bet = 0, bet_type = 0, bet_dir = 0;
    if (s_qt && (no_split || s_bt || s_eqt)) {
        int inc = (I && w == 128) ? 3 : split_ctx_ab(p, x0, y0, w, h);
        qt_flag = bin(p, CTX_QT_SPLIT + inc);
    }
    if (!qt_flag) {
        bet = !no_split;
        if (no_split && (s_bt || s_eqt)) {
            int inc = split_ctx_ab(p, x0, y0, w, h);
            inc += w * h > 1024 ? 0 : w * h > 256 ? 3 : 6;
            bet = bin(p, CTX_BET_SPLIT + inc);
        }
        if (bet) {
            if (s_bt && s_eqt)
                bet_type = bin(p, CTX_BET_SPLIT_TYPE + split_ctx_ab(p, x0, y0, w, h));
            else
                bet_type = !(bt_ver || bt_hor);
            if ((!bet_type && bt_hor && bt_ver) || (bet_type && eqt_hor && eqt_ver)) {
                int inc = (w == 128 && h == 64) ? 4 : (w == 64 && h == 128) ? 3 :
                          h > w ? 2 : w > h ? 1 : 0;
                bet_dir = bin(p, CTX_BET_SPLIT_DIR + inc);
            } else {
                bet_dir = !bet_type ? bt_ver : eqt_ver;
            }
        }
    }
    int smode = qt_flag ? SPLIT_QT : !bet ? NO_SPLIT :
                !bet_type ? (bet_dir ? SPLIT_BT_VER : SPLIT_BT_HOR)
                          : (bet_dir ? SPLIT_EQT_VER : SPLIT_EQT_HOR);

    int mode_child = mode;
    if (mode == PRED_NO_CONSTRAINT && !I &&
        ((((bet && !bet_type) || qt_flag) && w * h == 64) || (bet_type && w * h == 128))) {
        mode_child = bin(p, CTX_ROOT_CU) ? PRED_INTRA_ONLY : PRED_INTER_ONLY;
    }

    const int child4 =
        (smode == SPLIT_QT && (w == 8 || h == 8)) ||
        (smode == SPLIT_BT_VER && w == 8) ||
        (smode == SPLIT_BT_HOR && h == 8) ||
        (smode == SPLIT_EQT_VER && (w == 16 || h == 8)) ||
        (smode == SPLIT_EQT_HOR && (h == 16 || w == 8));
    if (child4 && p->component == 0) {
        p->luma_w = w;
        p->luma_h = h;
        p->component = 1;
    }
    if (!p->fixed_qp && p->cu_delta_qp_flag && w * h >= p->qp_group_area) {
        p->num_delta_qp = 0;
        p->qp_group_x = x0;
        p->qp_group_y = y0;
    }

    switch (smode) {
    case SPLIT_QT: {
        const int hw = w / 2, hh = h / 2, x1 = x0 + hw, y1 = y0 + hh;
        coding_unit_tree(p, x0, y0, split + 1, hw, hh, 1, mode_child);
        if (x1 < pw)
            coding_unit_tree(p, x1, y0, split + 1, hw, hh, 1, mode_child);
        if (y1 < ph)
            coding_unit_tree(p, x0, y1, split + 1, hw, hh, 1, mode_child);
        if (x1 < pw && y1 < ph)
            coding_unit_tree(p, x1, y1, split + 1, hw, hh, 1, mode_child);
        break;
    }
    case SPLIT_BT_VER:
        coding_unit_tree(p, x0, y0, split + 1, w / 2, h, 0, mode_child);
        if (x0 + w / 2 < pw)
            coding_unit_tree(p, x0 + w / 2, y0, split + 1, w / 2, h, 0, mode_child);
        break;
    case SPLIT_BT_HOR:
        coding_unit_tree(p, x0, y0, split + 1, w, h / 2, 0, mode_child);
        if (y0 + h / 2 < ph)
            coding_unit_tree(p, x0, y0 + h / 2, split + 1, w, h / 2, 0, mode_child);
        break;
    case SPLIT_EQT_VER:
        coding_unit_tree(p, x0, y0, split + 1, w / 4, h, 0, mode_child);
        coding_unit_tree(p, x0 + w / 4, y0, split + 1, w / 2, h / 2, 0, mode_child);
        coding_unit_tree(p, x0 + w / 4, y0 + h / 2, split + 1, w / 2, h / 2, 0, mode_child);
        coding_unit_tree(p, x0 + 3 * w / 4, y0, split + 1, w / 4, h, 0, mode_child);
        break;
    case SPLIT_EQT_HOR:
        coding_unit_tree(p, x0, y0, split + 1, w, h / 4, 0, mode_child);
        coding_unit_tree(p, x0, y0 + h / 4, split + 1, w / 2, h / 2, 0, mode_child);
        coding_unit_tree(p, x0 + w / 2, y0 + h / 4, split + 1, w / 2, h / 2, 0, mode_child);
        coding_unit_tree(p, x0, y0 + 3 * h / 4, split + 1, w, h / 4, 0, mode_child);
        break;
    default:
        coding_unit(p, x0, y0, w, h, mode, p->component == 0 ? COMP_LUMACHROMA : COMP_LUMA);
        break;
    }
    if (smode != NO_SPLIT && p->luma_w == w && p->luma_h == h && child4) {
        coding_unit_chroma(p, x0, y0, w, h);
        p->component = 0;
    }
}

/* ------------------------------------------------------------------------ */

static int loop_filters(parser_t *p, const svac3_hls_t *hls)
{
    const size_t n = (size_t)p->scu_stride * ((p->pic_h + 3) / 4);
    uint32_t *id = malloc(n * sizeof(*id));
    uint8_t *qp = malloc(n), *cbf = malloc(n);
    int8_t *ref = malloc(n);
    int16_t *mv = malloc(n * 2 * sizeof(*mv));

    if (!id || !qp || !cbf || !ref || !mv) {
        free(id);
        free(qp);
        free(cbf);
        free(ref);
        free(mv);
        return SVAC3_ERR_NOMEM;
    }
    if (!hls->pps.deblocking_filter_disable_flag) {
        for (size_t i = 0; i < n; i++) {
            id[i] = p->scu[i].cu_id;
            qp[i] = p->scu[i].qp;
            cbf[i] = p->scu[i].cbf;
            ref[i] = p->scu[i].intra ? -1 : p->scu[i].ref;
            mv[2 * i] = p->scu[i].mv[0];
            mv[2 * i + 1] = p->scu[i].mv[1];
        }
        svac3_dbk_info_t d = {
            .pic_w = p->pic_w, .pic_h = p->pic_h, .stride = p->scu_stride,
            .cu_id = id, .qp = qp,
            .inter = p->pct != 1, .cbf = cbf, .ref = ref, .mv = mv,
            .alpha_offset = hls->ph.alpha_c_offset, .beta_offset = hls->ph.beta_offset,
            .filter_type = hls->pps.deblocking_filter_type,
            .cb_delta = p->cb_delta, .cr_delta = p->cr_delta,
        };
        svac3_deblock(p->frame, &d);
    }
    free(id);
    free(qp);
    free(cbf);
    free(ref);
    free(mv);
    if (p->sps->sao_enable_flag)
        return svac3_sao(p->frame, (const svac3_sao_t (*)[3])p->sao, p->lcu_size, p->lcu_w, p->lcu_h)
               ? SVAC3_ERR_NOMEM : SVAC3_OK;
    return SVAC3_OK;
}

/* A frame for reconstruction must hold the coded picture, 4:2:0. */
static int frame_ok(const svac3_frame_t *f, int w, int h)
{
    for (int c = 0; c < 3; c++) {
        const int pw = c ? w / 2 : w, ph = c ? h / 2 : h;
        if (!f->plane[c] || f->width[c] != pw || f->height[c] != ph || f->stride[c] < pw)
            return 0;
    }
    return 1;
}

size_t svac3_tmv_count(int pic_w, int pic_h)
{
    return (size_t)((pic_w + 15) >> 4) * (size_t)((pic_h + 15) >> 4);
}

/* 5.4.6.7.10.3 with EtmvpMvapEnableFlag, SbTmvpEnableFlag,
 * MvpRefLayerEnableFlag and AwpEnableFlag 0: the motion at the centre of
 * each 16x16 unit */
static void store_tmv(const parser_t *p, svac3_tmv_t *t)
{
    const int tw = (p->pic_w + 15) >> 4, th = (p->pic_h + 15) >> 4;
    for (int ty = 0; ty < th; ty++)
        for (int tx = 0; tx < tw; tx++) {
            const int x0 = tx << 4, y0 = ty << 4;
            const int nw = x0 + 16 >= p->pic_w ? p->pic_w - x0 : 16;
            const int nh = y0 + 16 >= p->pic_h ? p->pic_h - y0 : 16;
            const scu_t *s = scu_any(p, x0 + nw / 2, y0 + nh / 2);
            svac3_tmv_t *o = &t[ty * tw + tx];
            memset(o, 0, sizeof(*o));
            o->ref = -1;
            if (s && !s->intra && s->ref >= 0 && p->refs) {
                o->ref = s->ref;
                o->mv[0] = s->mv[0];
                o->mv[1] = s->mv[1];
                o->dist = p->refs->dist[s->ref];
            }
        }
}

int svac3_patch_supported(const svac3_hls_t *hls)
{
    const svac3_sps_t *s = &hls->sps;
    const svac3_pps_t *p = &hls->pps;

    if (!hls->have_sps || !hls->have_pps || !hls->have_ph || !hls->have_patch)
        return SVAC3_ERR_INVALID;
    /* display pictures, RL pictures (which reference library pictures) and
     * displayed library pictures; not the non-displayed library patches
     * (12, 18), for want of a stream to check them against, nor privacy
     * patches */
    const unsigned t = hls->patch.nal_unit_type;
    if (t == SVAC3_NAL_CRR_DP_PATCH) {
        if (!s->library_stream_flag)
            return SVAC3_ERR_INVALID;       /* a library patch in a display stream */
    } else if (t == SVAC3_NAL_NRAP_PATCH || t == SVAC3_NAL_RAPI_PATCH ||
               t == SVAC3_NAL_IDR_PATCH || t == SVAC3_NAL_RL_PATCH) {
        if (s->library_stream_flag)
            return SVAC3_ERR_INVALID;       /* a display patch in a library stream */
        if (t == SVAC3_NAL_RL_PATCH && hls->ph.picture_coding_type == 1)
            return SVAC3_ERR_INVALID;       /* an RL picture references a library picture */
    } else {
        return SVAC3_ERR_UNSUPPORTED;
    }
    if (s->profile_id != 0x30 || s->chroma_format != 1 || s->sample_precision != 1)
        return SVAC3_ERR_UNSUPPORTED;
    if (s->weight_quant_enable_flag ||
        s->st_enable_flag || s->alf_enable_flag || s->affine_enable_flag ||
        s->smvd_enable_flag || s->ipcm_enable_flag || s->amvr_enable_flag ||
        s->num_of_hmvp_cand || s->umve_enable_flag || s->intra_pf_enable_flag ||
        s->tscpm_enable_flag || s->dt_enable_flag || s->pbt_enable_flag ||
        s->pmc_enable_flag || s->iip_enable_flag || s->sawp_enable_flag ||
        s->awp_enable_flag || s->sbtmvp_enable_flag || s->etmvp_mvap_enable_flag ||
        s->dmvr_enable_flag || s->bio_enable_flag || s->bgc_enable_flag ||
        s->inter_pf_enable_flag || s->inter_pc_enable_flag || s->obmc_enable_flag ||
        s->sbt_enable_flag || s->ist_enable_flag || s->dbr_enable_flag ||
        s->esao_enable_flag || s->ccsao_enable_flag)
        return SVAC3_ERR_UNSUPPORTED;
    if (!s->srcc_enable_flag)
        return SVAC3_ERR_UNSUPPORTED;       /* run-level coefficients not written yet */
    if (!s->stable_patch_flag || !s->uniform_patch_flag)
        return SVAC3_ERR_UNSUPPORTED;
    /* loop filters stopping at patch boundaries are not written: several
     * patches need cross_patch_loop_filter_enable_flag */
    if (!s->cross_patch_loop_filter_enable_flag && svac3_picture_patches(hls) != 1)
        return SVAC3_ERR_UNSUPPORTED;
    /* picture_qp and patch_qp are 0..63 for 8-bit video (7-bit fields) */
    if (hls->ph.picture_qp > 63 || (!hls->pps.fixed_picture_qp_flag && hls->patch.patch_qp > 63))
        return SVAC3_ERR_INVALID;
    if ((s->pic_width_in_luma & 7) || (s->pic_height_in_luma & 7) ||
        !s->pic_width_in_luma || !s->pic_height_in_luma)
        return SVAC3_ERR_INVALID;           /* coded size is a multiple of MiniSize (8) */
    if (s->log2_max_part_ratio_minus2 > 1)
        return SVAC3_ERR_INVALID;           /* MaxPartRatio is 8 at most in the Main profile (F.2) */
    if (s->log2_lcu_size_minus2 > 4)
        return SVAC3_ERR_UNSUPPORTED;       /* 128x128 LCUs (transform zero-out not written) */
    (void)p;
    if (hls->ph.picture_coding_type == 3)
        return SVAC3_ERR_UNSUPPORTED;       /* B pictures */
    if (hls->ph.picture_coding_type != 1 && hls->ph.picture_coding_type != 2)
        return SVAC3_ERR_INVALID;           /* 0 is reserved */
    return SVAC3_OK;
}

/* References for a P picture: enough of them, each the size of frame, with
 * temporal motion when TMVP needs it */
static int refs_ok(const svac3_refs_t *r, int need, const svac3_frame_t *frame, int tmvp)
{
    if (!r || r->num < need || r->num > SVAC3_MAX_REFS)
        return 0;
    for (int i = 0; i < r->num; i++) {
        const svac3_frame_t *f = r->ref[i];
        if (!f)
            return 0;
        for (int c = 0; c < 3; c++)
            if (!f->plane[c] || f->width[c] != frame->width[c] ||
                f->height[c] != frame->height[c] || f->stride[c] < f->width[c])
                return 0;
    }
    return !tmvp || r->col;
}

struct svac3_picture {
    parser_t p;
};

int svac3_picture_patches(const svac3_hls_t *hls)
{
    const svac3_sps_t *s = &hls->sps;
    const int lcu = 1 << (s->log2_lcu_size_minus2 + 2);
    const uint64_t lw = ((uint64_t)s->pic_width_in_luma + lcu - 1) / lcu;
    const uint64_t lh = ((uint64_t)s->pic_height_in_luma + lcu - 1) / lcu;
    const uint64_t pw = (uint64_t)s->patch_width_minus1 + 1, ph = (uint64_t)s->patch_height_minus1 + 1;
    const uint64_t n = ((lw + pw - 1) / pw) * ((lh + ph - 1) / ph);
    return n > 256 ? -1 : (int)n;           /* patch_index is 8 bits */
}

void svac3_picture_free(svac3_picture_t *pic)
{
    if (!pic)
        return;
    free(pic->p.sao);
    free(pic->p.scu);
    free(pic);
}

int svac3_picture_begin(const svac3_hls_t *hls, svac3_frame_t *frame, const svac3_refs_t *refs,
                        svac3_tmv_t *tmv, svac3_picture_t **out)
{
    int ret = svac3_patch_supported(hls);

    *out = NULL;
    if (ret)
        return ret;
    svac3_picture_t *pic = calloc(1, sizeof(*pic));
    if (!pic)
        return SVAC3_ERR_NOMEM;
    parser_t *p = &pic->p;
    const svac3_sps_t *s = p->sps = &hls->sps;
    p->pps = &hls->pps;
    p->ph = &hls->ph;
    p->pct = hls->ph.picture_coding_type;
    p->pic_w = s->pic_width_in_luma;
    p->pic_h = s->pic_height_in_luma;
    p->lcu_size = 1 << (s->log2_lcu_size_minus2 + 2);
    p->lcu_w = (p->pic_w + p->lcu_size - 1) / p->lcu_size;
    p->lcu_h = (p->pic_h + p->lcu_size - 1) / p->lcu_size;
    p->min_qt = 1 << (s->log2_min_qt_size_minus2 + 2);
    p->max_bt = 1 << (s->log2_max_bt_size_minus2 + 2);
    p->min_bt = p->min_eqt = 1 << (s->log2_min_cu_size_minus2 + 2);
    p->max_eqt = 1 << (s->log2_max_eqt_size_minus3 + 3);
    p->max_part_ratio = 1 << (s->log2_max_part_ratio_minus2 + 2);
    p->max_split = s->max_split_times_minus6 + 6;
    p->cu_delta_qp_flag = hls->pps.cu_delta_qp_flag;
    p->qp_group_area = 1 << ((hls->pps.cu_qp_group_size_log2_minus3 + 3) * 2);
    p->num_patches = svac3_picture_patches(hls);
    const uint32_t ref_minus1 = hls->ph.num_ref_active_override_flag
                                ? hls->ph.num_ref_active_minus1[0]
                                : s->num_ref_default_active_minus1[0];
    ret = SVAC3_OK;
    if (p->num_patches < 1)
        ret = SVAC3_ERR_INVALID;
    else if (ref_minus1 >= SVAC3_MAX_REFS)
        ret = SVAC3_ERR_INVALID;
    else if (frame && !frame_ok(frame, s->pic_width_in_luma, s->pic_height_in_luma))
        ret = SVAC3_ERR_INVALID;
    p->num_ref_active0 = ret ? 1 : (int)ref_minus1 + 1;   /* in range once ret is 0 */
    p->tmvp = hls->ph.picture_tmvp_enable_flag;
    if (!ret && frame && p->pct == 2 && !refs_ok(refs, p->num_ref_active0, frame, p->tmvp))
        ret = SVAC3_ERR_INVALID;
    if (ret) {
        free(pic);
        return ret;
    }
    p->frame = frame;
    p->refs = p->pct == 2 ? refs : NULL;
    p->recon = frame != NULL;
    p->tmv_out = tmv;
    p->mipf = s->mipf_enable_flag;
    p->cb_delta = hls->ph.chroma_quant_param_disable_flag ? 0 : hls->ph.chroma_quant_param_delta_cb;
    p->cr_delta = hls->ph.chroma_quant_param_disable_flag ? 0 : hls->ph.chroma_quant_param_delta_cr;
    p->scu_stride = (p->pic_w + 3) / 4;
    p->scu = calloc((size_t)p->scu_stride * ((p->pic_h + 3) / 4), sizeof(scu_t));
    if (p->scu && p->recon)
        p->sao = calloc((size_t)p->lcu_w * p->lcu_h, sizeof(*p->sao));
    if (!p->scu || (p->recon && !p->sao)) {
        svac3_picture_free(pic);
        return SVAC3_ERR_NOMEM;
    }
    *out = pic;
    return SVAC3_OK;
}

/* Decode the patch whose header is hls->patch into the picture. */
static int decode_patch(parser_t *p, const svac3_hls_t *hls, const uint8_t *data, size_t len,
                        svac3_patch_stats_t *st)
{
    const svac3_sps_t *s = &hls->sps;
    const int k = hls->patch.patch_index;

    memset(st, 0, sizeof(*st));
    if (hls->patch.aec_data_offset > len || k >= p->num_patches)
        return SVAC3_ERR_INVALID;
    p->st = st;

    /* uniform patches in raster order (7.2.4) */
    /* a patch at least as wide (tall) as the picture is one column (row) */
    const uint64_t pw64 = (uint64_t)s->patch_width_minus1 + 1, ph64 = (uint64_t)s->patch_height_minus1 + 1;
    const int pw = pw64 > (uint64_t)p->lcu_w ? p->lcu_w : (int)pw64;
    const int ph = ph64 > (uint64_t)p->lcu_h ? p->lcu_h : (int)ph64;
    const int cols = (p->lcu_w + pw - 1) / pw;
    p->cur_patch = k;
    p->plx0 = (k % cols) * pw;
    p->ply0 = (k / cols) * ph;
    p->plx1 = p->plx0 + pw < p->lcu_w ? p->plx0 + pw : p->lcu_w;
    p->ply1 = p->ply0 + ph < p->lcu_h ? p->ply0 + ph : p->lcu_h;

    p->fixed_qp = hls->pps.fixed_picture_qp_flag ? 1 : hls->patch.fixed_patch_qp_flag;
    p->patch_qp = hls->pps.fixed_picture_qp_flag ? hls->ph.picture_qp : hls->patch.patch_qp;
    p->prev_cu_qp = p->lcu_qp = p->patch_qp;
    /* a QP group does not continue into the next patch */
    p->num_delta_qp = 0;
    p->qp_group_x = p->plx0 * p->lcu_size;
    p->qp_group_y = p->ply0 * p->lcu_size;
    svac3_ctx_init(p->ctx, SVAC3_NUM_CTX);
    svac3_aec_init(&p->aec, data + hls->patch.aec_data_offset,
                   len - hls->patch.aec_data_offset, s->maec_enable_flag, p->pct);
    st->bytes_avail = len - hls->patch.aec_data_offset;
    st->stuffing_ok = 1;

    const int pcols = p->plx1 - p->plx0;
    const int n_lcu = pcols * (p->ply1 - p->ply0);
    int prev_dqp = 0;
    for (int i = 0; i < n_lcu; i++) {
        const int lx = p->plx0 + i % pcols, ly = p->ply0 + i / pcols;
        const int l = ly * p->lcu_w + lx;
        if (!p->fixed_qp && !p->cu_delta_qp_flag) {
            /* lcu_qp_delta: unary, 5.3.3.3.2.2 / 5.3.3.4.5 */
            int b = 0, sym = 0;
            while (!bin(p, CTX_LCU_QP_DELTA + (b == 0 ? (prev_dqp != 0) : b == 1 ? 2 : 3))) {
                sym++;
                b++;
                if (sym > 128)
                    break;
            }
            prev_dqp = (sym & 1) ? (sym + 1) / 2 : -(sym / 2);
            p->lcu_qp = (((p->lcu_qp + prev_dqp) % 64) + 64) % 64;
        }
        if (s->sao_enable_flag && (hls->patch.patch_sao_enable_flag[0] ||
                                   hls->patch.patch_sao_enable_flag[1] ||
                                   hls->patch.patch_sao_enable_flag[2]))
            parse_sao(p, lx, ly, hls->patch.patch_sao_enable_flag,
                      p->sao ? p->sao[l] : (svac3_sao_t[3]){ { 0 } });
        p->component = 0;
        coding_unit_tree(p, lx * p->lcu_size, ly * p->lcu_size, 0, p->lcu_size, p->lcu_size,
                         1, PRED_NO_CONSTRAINT);
        const int stuff = svac3_aec_stuffing_bit(&p->aec);
        st->lcus++;
        if (stuff != (i == n_lcu - 1)) {
            if (st->stuffing_ok)
                st->first_bad_lcu = (unsigned)l;
            st->stuffing_ok = 0;
        }
        if (p->aec.overrun > 64) {
            /* ran out of data: out of sync or truncated, stop here */
            if (st->stuffing_ok)
                st->first_bad_lcu = (unsigned)l;
            st->stuffing_ok = 0;
            break;
        }
    }
    st->bytes_used = svac3_aec_bytes_read(&p->aec);
    st->overrun_bits = p->aec.overrun;
    return SVAC3_OK;
}

int svac3_picture_patch(svac3_picture_t *pic, const svac3_hls_t *hls, const uint8_t *data,
                        size_t len, svac3_patch_stats_t *st)
{
    parser_t *p = &pic->p;
    int ret = svac3_patch_supported(hls);

    memset(st, 0, sizeof(*st));
    if (ret)
        return ret;
    if (hls->patch.patch_index != p->next_patch)
        return SVAC3_ERR_CORRUPT;               /* a patch lost or out of order */
    ret = decode_patch(p, hls, data, len, st);
    if (ret)
        return ret;
    /* a patch that did not parse to its exact end is not a patch */
    if (!st->stuffing_ok || st->overrun_bits)
        return SVAC3_ERR_CORRUPT;
    p->next_patch++;
    return p->next_patch == p->num_patches ? 1 : 0;
}

int svac3_picture_end(svac3_picture_t *pic, const svac3_hls_t *hls)
{
    parser_t *p = &pic->p;
    int ret = SVAC3_OK;

    if (p->next_patch != p->num_patches)
        ret = SVAC3_ERR_CORRUPT;                /* patches missing */
    else if (p->recon)
        ret = loop_filters(p, hls);
    if (ret == SVAC3_OK && p->tmv_out)
        store_tmv(p, p->tmv_out);
    svac3_picture_free(pic);
    return ret;
}

int svac3_patch_parse(const svac3_hls_t *hls, const uint8_t *data, size_t len,
                      svac3_patch_stats_t *st)
{
    svac3_picture_t *pic;
    int ret;

    memset(st, 0, sizeof(*st));
    if ((ret = svac3_picture_begin(hls, NULL, NULL, NULL, &pic)))
        return ret;
    ret = decode_patch(&pic->p, hls, data, len, st);
    svac3_picture_free(pic);
    return ret;
}

int svac3_patch_decode(const svac3_hls_t *hls, const uint8_t *data, size_t len,
                       svac3_frame_t *frame, const svac3_refs_t *refs, svac3_tmv_t *tmv,
                       svac3_patch_stats_t *st)
{
    svac3_picture_t *pic;
    int ret;

    memset(st, 0, sizeof(*st));
    if (svac3_patch_supported(hls) == SVAC3_OK && svac3_picture_patches(hls) != 1)
        return SVAC3_ERR_UNSUPPORTED;           /* use svac3_picture_*() */
    if ((ret = svac3_picture_begin(hls, frame, refs, tmv, &pic)))
        return ret;
    ret = svac3_picture_patch(pic, hls, data, len, st);
    if (ret < 0) {
        svac3_picture_free(pic);
        return ret;
    }
    return svac3_picture_end(pic, hls);
}
