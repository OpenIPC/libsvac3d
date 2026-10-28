/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright (c) 2026 OpenIPC contributors
 *
 * SVAC3 patch data parser. Each function names the table or section of the
 * SVAC 3.0 draft it implements; context indices are the ctxIndexStart
 * values of 表77.
 */
#include "svac3_patch.h"

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
    uint8_t w, h;            /* CU size in luma samples (w, h <= 128: stored /1) */
    uint8_t skip, intra;
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

    /* scratch for one transform block */
    int16_t order[64 * 64];
    uint8_t sig[64 * 64];
    uint16_t absv[64 * 64];
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

static const scu_t *scu_at(const parser_t *p, int x, int y)
{
    if (x < 0 || y < 0 || x >= p->pic_w || y >= p->pic_h)
        return NULL;
    const scu_t *s = &p->scu[(y >> 2) * p->scu_stride + (x >> 2)];
    return s->coded ? s : NULL;
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
        }
}

/* ------------------------------------------------------------------------ */
/* SAO parameters, 表32; binarisation 5.3.3.4.6-10 (as in AVS3). */

static void parse_sao(parser_t *p, int lcu_x, int lcu_y, const uint8_t sao_en[3])
{
    const int left = lcu_x > 0, up = lcu_y > 0;
    int merge = 0;

    if (left + up > 0) {
        int sym;
        if (left + up == 1) {
            sym = bin(p, CTX_SAO_MERGE + 0);
        } else {
            sym = bin(p, CTX_SAO_MERGE + 1);
            if (sym != 1)
                sym += bin(p, CTX_SAO_MERGE + 2) << 1;
        }
        merge = sym;
    }
    if (merge)
        return;

    for (int c = 0; c < 3; c++) {
        if (!sao_en[c])
            continue;
        /* sao_mode: truncated unary, maxVal 2; bin 0 coded, bin 1 bypass */
        int mode;
        if (bin(p, CTX_SAO_MODE))
            mode = 0;
        else
            mode = byp(p) ? 1 : 2;           /* 1 = interval (BO), 2 = edge */
        if (mode == 0)
            continue;
        if (mode == 1) {
            for (int j = 0; j < 4; j++) {
                /* sao_interval_offset_abs: TU maxVal 7, bin 0 coded */
                int a = !bin(p, CTX_SAO_OFFSET);
                if (a) {
                    a += byp_tu(p, 6);
                    byp(p);                  /* sao_interval_offset_sign */
                }
            }
            for (int k = 0; k < 5; k++)
                byp(p);                      /* sao_interval_start_pos */
            /* sao_interval_delta_pos_minus2 (表85) */
            int order = 1 + byp_tu(p, 3);
            for (int k = 0; k < order % 4; k++)
                byp(p);
        } else {
            /* sao_edge_offset[j]: TU in bypass; maxima as in AVS3 */
            static const int maxv[4] = { 7, 1, 1, 7 };
            for (int j = 0; j < 4; j++)
                byp_tu(p, maxv[j] == 7 ? 6 + 1 : 1);
            byp(p);                          /* sao_edge_type, 2 bins */
            byp(p);
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

static void parse_srcc_block(parser_t *p, int w, int h, int is_chroma, int intra)
{
    int16_t *order = p->order;
    uint8_t *sig = p->sig;
    uint16_t *absv = p->absv;
    const int log2w = ilog2(w);

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
        for (int n = top; n >= 0; n--)
            if (sig[order[set_pos + n]]) {
                byp(p);                      /* coeff_sign */
                p->st->coeffs++;
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

static void coding_unit_chroma(parser_t *p, int x0, int y0, int w, int h)
{
    const scu_t *luma = scu_at(p, x0 + w - 1, y0 + h - 1);
    const int prior_intra = luma && luma->intra;
    int ctp;

    if (prior_intra)
        for (int b = 0; b < 4 && !bin(p, CTX_INTRA_CHROMA + (b ? 1 : 0)); b++)
            ;
    ctp = (bin(p, CTX_CTP_U) << 1);
    ctp |= (bin(p, CTX_CTP_V) << 2);
    for (int i = 1; i < 3; i++)
        if (ctp & (1 << i))
            parse_srcc_block(p, w / 2, h / 2, 1, prior_intra);
    p->st->chroma_cus++;
}

static void coding_unit(parser_t *p, int x0, int y0, int w, int h, int mode, int comp)
{
    int skip = 0, direct = 0, intra = p->pct == 1;
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

    if (skip || direct) {
        /* cu_subtype_index: TU, maxVal 1 for P (no MVAP or HMVP candidates) */
        const int maxv = (p->pct == 2 ? 1 : 3);
        for (int b = 0; b < maxv && !bin(p, CTX_CU_SUBTYPE + b); b++)
            ;
    }

    if (!skip && !direct) {
        if (!intra) {
            if (p->num_ref_active0 > 1)
                for (int b = 0; b < p->num_ref_active0 - 1 &&
                     !bin(p, CTX_REF_IDX + (b < 2 ? b : 2)); b++)
                    ;
            if (parse_mvd_abs(p, CTX_MVD_X))
                byp(p);
            if (parse_mvd_abs(p, CTX_MVD_Y))
                byp(p);
        } else {
            /* intra_luma_pred_mode_index (表78/表98), then luma_eipm_index */
            int idx;
            if (bin(p, CTX_INTRA_LUMA + 0)) {
                idx = bin(p, CTX_INTRA_LUMA + 6);
            } else {
                idx = 0;
                for (int b = 1; b <= 5; b++)
                    idx = (idx << 1) | bin(p, CTX_INTRA_LUMA + b);
                idx += 2;
            }
            if (idx > 1)
                bin(p, CTX_LUMA_EIPM);
            if (comp == COMP_LUMACHROMA)
                for (int b = 0; b < 4 && !bin(p, CTX_INTRA_CHROMA + (b ? 1 : 0)); b++)
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
        if (a)
            byp(p);                          /* cu_qp_delta_sign */
        p->num_delta_qp++;
    }

    /* Chroma blocks are half the CU size (4:2:0). 表36's SRCC branch does not
     * halve them, but only the halved reading parses (docs/errata.md). */
    if (ctp & 1)
        parse_srcc_block(p, w, h, 0, intra);
    if (ctp & 2)
        parse_srcc_block(p, w / 2, h / 2, 1, intra);
    if (ctp & 4)
        parse_srcc_block(p, w / 2, h / 2, 1, intra);

    scu_mark(p, x0, y0, w, h, skip, intra);
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
    if (!p->fixed_qp && p->cu_delta_qp_flag && w * h >= p->qp_group_area)
        p->num_delta_qp = 0;

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

int svac3_patch_supported(const svac3_hls_t *hls)
{
    const svac3_sps_t *s = &hls->sps;
    const svac3_pps_t *p = &hls->pps;

    if (!hls->have_sps || !hls->have_pps || !hls->have_ph)
        return SVAC3_ERR_INVALID;
    if (s->profile_id != 0x30 || s->chroma_format != 1 || s->sample_precision != 1)
        return SVAC3_ERR_UNSUPPORTED;
    if (s->library_picture_enable_flag || s->weight_quant_enable_flag ||
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
    if (s->log2_lcu_size_minus2 > 4)
        return SVAC3_ERR_UNSUPPORTED;       /* 128x128 LCUs (transform zero-out not written) */
    (void)p;
    if (hls->ph.picture_coding_type == 3)
        return SVAC3_ERR_UNSUPPORTED;       /* B pictures */
    return SVAC3_OK;
}

int svac3_patch_parse(const svac3_hls_t *hls, const uint8_t *data, size_t len,
                      svac3_patch_stats_t *st)
{
    parser_t *p;
    int ret = svac3_patch_supported(hls);

    memset(st, 0, sizeof(*st));
    if (ret)
        return ret;
    if (hls->patch.aec_data_offset > len)
        return SVAC3_ERR_INVALID;

    p = calloc(1, sizeof(*p));
    if (!p)
        return SVAC3_ERR_INVALID;
    const svac3_sps_t *s = p->sps = &hls->sps;
    p->pps = &hls->pps;
    p->ph = &hls->ph;
    p->st = st;
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
    p->fixed_qp = hls->pps.fixed_picture_qp_flag ? 1 : hls->patch.fixed_patch_qp_flag;
    p->cu_delta_qp_flag = hls->pps.cu_delta_qp_flag;
    p->qp_group_area = 1 << ((hls->pps.cu_qp_group_size_log2_minus3 + 3) * 2);
    p->num_ref_active0 = hls->ph.num_ref_active_override_flag
                         ? (int)hls->ph.num_ref_active_minus1[0] + 1
                         : (int)s->num_ref_default_active_minus1[0] + 1;
    p->scu_stride = (p->pic_w + 3) / 4;
    p->scu = calloc((size_t)p->scu_stride * ((p->pic_h + 3) / 4), sizeof(scu_t));
    if (!p->scu) {
        free(p);
        return SVAC3_ERR_INVALID;
    }

    /* a single patch covering the picture */
    const int patch_w = (int)s->patch_width_minus1 + 1, patch_h = (int)s->patch_height_minus1 + 1;
    if (patch_w < p->lcu_w || patch_h < p->lcu_h || hls->patch.patch_index != 0) {
        free(p->scu);
        free(p);
        return SVAC3_ERR_UNSUPPORTED;       /* several patches per picture */
    }

    svac3_ctx_init(p->ctx, SVAC3_NUM_CTX);
    svac3_aec_init(&p->aec, data + hls->patch.aec_data_offset,
                   len - hls->patch.aec_data_offset, s->maec_enable_flag, p->pct);
    st->bytes_avail = len - hls->patch.aec_data_offset;
    st->stuffing_ok = 1;

    const int n_lcu = p->lcu_w * p->lcu_h;
    int prev_dqp = 0;
    for (int l = 0; l < n_lcu; l++) {
        const int lx = l % p->lcu_w, ly = l / p->lcu_w;
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
        }
        if (s->sao_enable_flag && (hls->patch.patch_sao_enable_flag[0] ||
                                   hls->patch.patch_sao_enable_flag[1] ||
                                   hls->patch.patch_sao_enable_flag[2]))
            parse_sao(p, lx, ly, hls->patch.patch_sao_enable_flag);
        p->component = 0;
        coding_unit_tree(p, lx * p->lcu_size, ly * p->lcu_size, 0, p->lcu_size, p->lcu_size,
                         1, PRED_NO_CONSTRAINT);
        const int stuff = svac3_aec_stuffing_bit(&p->aec);
        st->lcus++;
        if (stuff != (l == n_lcu - 1)) {
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
    free(p->scu);
    free(p);
    return SVAC3_OK;
}
