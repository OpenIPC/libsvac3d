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

#include <pthread.h>
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
    svac3_pool_t *pool;          /* threads for the loop filters, or NULL */
    struct svac3_picture *pic;   /* parsing: where coding-unit records go */
    struct svac3_picture *owner; /* reconstruction: the picture, for the loop filters */

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
        svac3_aec_mark_corrupt(&p->aec);     /* impossible region */
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

    /* pos_class() of every position in the scan region, from the zone of
     * its column and of its row (fixed for the block) */
    uint8_t zx[64 + 16], zy[64 + 16];          /* room for a vectorised store */
    for (int x = 0; x <= srx && x < 64; x++)
        zx[x] = (uint8_t)(intra ? (x <= srx / 2 ? (x <= srx / 4 ? 0 : 1) : (x <= srx * 3 / 4 ? 2 : 3))
                                : x > srx / 2);
    for (int y = 0; y <= sry && y < 64; y++)
        zy[y] = (uint8_t)(intra ? (y <= sry / 2 ? (y <= sry / 4 ? 0 : 1) : (y <= sry * 3 / 4 ? 2 : 3))
                                : y > sry / 2);
    static const uint8_t intra_cls[7] = { 4, 4, 12, 8, 8, 8, 8 };   /* by zone sum */
#define POS_CLASS(px, py) (intra ? intra_cls[zx[px] + zy[py]] : (zx[px] | zy[py]) ? 8 : 4)

    const int last_pos = (srx + 1) * (sry + 1) - 1;
    const int last_set = last_pos >> 4;
    const int area = (srx + 1) * (sry + 1);
    int is_last_x = 0, is_last_y = 0, first_done = 0;
    /* the last five coefficients scanned, as 5-bit masks (bit 0 the most
     * recent): significant, level > 1, level > 2 */
    unsigned m_sig = 0, m_gt1 = 0, m_gt2 = 0;
    static const uint8_t ones[32] = { 0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4,
                                      1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5 };

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
                        inc += is_chroma ? 4 : POS_CLASS(px, py);
                    const int num = ones[m_sig];
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
            m_sig = ((m_sig << 1) | (unsigned)f) & 31;
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
                cls = POS_CLASS(px, py);
            int a;
            for (int k = 1;; k = 2) {
                int inc = base;
                if (first_done) {
                    const int num = ones[k == 1 ? m_gt1 : m_gt2];
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
            m_gt1 = ((m_gt1 << 1) | (unsigned)(a > 1)) & 31;
            m_gt2 = ((m_gt2 << 1) | (unsigned)(a > 2)) & 31;
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
#undef POS_CLASS
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
        if (p->refs->wait_tmv)
            p->refs->wait_tmv(p->refs->col_ctx, (y0 >> 4) + 1);
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

/* A reference decoded at the same time: wait for the rows that the
 * interpolation of an h-row block at y0 with vertical motion mvy reads,
 * the 8-tap luma one and the 4-tap chroma one */
static void ref_wait(const parser_t *p, int ref, int y0, int h, int mvy)
{
    if (!p->refs->wait)
        return;
    const int luma = (((y0 << 2) + mvy) >> 2) + h + 4;
    const int chroma = 2 * (((((y0 >> 1) * 2 << 2) + mvy) >> 3) + h / 2 + 2);
    int need = luma > chroma ? luma : chroma;
    /* reads above the picture are of its first row */
    need = need < 1 ? 1 : need > p->pic_h ? p->pic_h : need;
    p->refs->wait(p->refs->ctx[ref], need);
}

static void recon_inter(parser_t *p, int x0, int y0, int w, int h, int ref, const int mv[2],
                        int qp, int ctp, int chroma)
{
    const svac3_frame_t *rf = p->refs->ref[ref];
    ref_wait(p, ref, y0, h, mv[1]);
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
                if (s && s->ref >= 0) {
                    ref_wait(p, s->ref, y0 + 2 * y, 4, s->mv[1]);
                    svac3_mc_chroma(p->refs->ref[s->ref], c, x0 / 2 + x, y0 / 2 + y, 2, 2,
                                    s->mv[0], s->mv[1], blk);
                }
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

/* ------------------------------------------------------------------------ */
/* Parsing and reconstruction meet through coding-unit records: the parser
 * (one parser_t) emits one per coding unit, the reconstruction (another,
 * with its own map of decoded units, since availability for prediction
 * means reconstructed) consumes them in order, on the same thread or on
 * one of its own. */

enum { REC_CU, REC_CHROMA, REC_PATCH, REC_ROW };

typedef struct {
    uint8_t kind;
    uint8_t comp, skip, direct, intra, subtype, ref, lidx, eipm, cidx, ctp;
    uint8_t group_start;         /* the CU opens a QP group */
    uint8_t lcu_qp;
    uint8_t patch, patch_qp, fixed_qp;   /* REC_PATCH */
    uint16_t row;                /* REC_ROW: this LCU row is done */
    int16_t dqp;
    int32_t mvd[2];              /* as parsed: derive_mvp() clips the sum */
    uint16_t x0, y0, w, h;
    const int16_t *coef[3];      /* by component, when its ctp bit is set */
} cu_rec_t;

static void emit(parser_t *p, cu_rec_t *rec);
static void filter_rows_done(struct svac3_picture *pic, int rows);

static void cu_recon_chroma(parser_t *r, const cu_rec_t *rec)
{
    const int x0 = rec->x0, y0 = rec->y0, w = rec->w, h = rec->h;
    const scu_t *luma = scu_at(r, x0 + w - 1, y0 + h - 1);
    for (int i = 1; i < 3; i++)
        if (rec->ctp & (1 << i))
            memcpy(r->coef[i], rec->coef[i], sizeof(int16_t) * (size_t)(w / 2) * (h / 2));
    if (luma && luma->intra)
        recon_chroma(r, x0, y0, w, h, derive_chroma_mode(rec->cidx, luma->ipm), luma->ipm,
                     luma->qp, rec->ctp);
    else if (luma)
        recon_chroma_inter(r, x0, y0, w, h, luma->qp, rec->ctp);
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

    cu_rec_t rec = { .kind = REC_CHROMA, .cidx = (uint8_t)cidx, .ctp = (uint8_t)ctp,
                     .x0 = (uint16_t)x0, .y0 = (uint16_t)y0, .w = (uint16_t)w, .h = (uint16_t)h,
                     .coef = { NULL, p->coef[1], p->coef[2] } };
    emit(p, &rec);
}

static void cu_recon(parser_t *r, const cu_rec_t *rec)
{
    const int x0 = rec->x0, y0 = rec->y0, w = rec->w, h = rec->h;
    const int intra = rec->intra, skip = rec->skip, direct = rec->direct, ctp = rec->ctp;
    int ref = rec->ref;

    r->lcu_qp = rec->lcu_qp;
    r->qp_group_x = rec->group_start ? x0 : -1;
    r->qp_group_y = rec->group_start ? y0 : -1;
    for (int c = 0; c < 3; c++)
        if (ctp & (1 << c))
            memcpy(r->coef[c], rec->coef[c],
                   sizeof(int16_t) * (size_t)(c ? (w / 2) * (h / 2) : w * h));

    const int qp = derive_qp(r, x0, y0, ctp, rec->dqp);
    const int lmode = intra ? derive_luma_mode(r, x0, y0, rec->lidx, rec->eipm) : 0;

    int mv[2] = { 0, 0 };
    if (intra) {
        svac3_intra_pred(r->frame, 0, x0, y0, w, h, lmode, 0, r->mipf, avail_cb, r, r->pred);
        const int16_t *res = NULL;
        if (ctp & 1) {
            svac3_dequant_itrans(r->coef[0], w, h, qp, r->resid);
            res = r->resid;
        }
        svac3_put_block(r->frame, 0, x0, y0, w, h, r->pred, res);
    } else {
        const int mvd[2] = { rec->mvd[0], rec->mvd[1] };
        if (skip || direct)
            derive_skip_motion(r, x0, y0, w, h, rec->subtype, &ref, mv);
        else
            derive_mvp(r, x0, y0, w, ref, mvd, mv);
        recon_inter(r, x0, y0, w, h, ref, mv, qp, ctp, rec->comp == COMP_LUMACHROMA);
    }

    scu_mark(r, x0, y0, w, h, skip, intra);
    /* the mark above makes this CU's own samples "available"; record its mode,
     * QP and motion for later neighbours and for a deferred chroma CU */
    for (int yy = y0; yy < y0 + h && yy < r->pic_h; yy += 4)
        for (int xx = x0; xx < x0 + w && xx < r->pic_w; xx += 4) {
            scu_t *s = &r->scu[(yy >> 2) * r->scu_stride + (xx >> 2)];
            s->ipm = (uint8_t)lmode;
            s->qp = (uint8_t)qp;
            s->cbf = ctp != 0;
            s->ref = (int8_t)(intra ? -1 : ref);
            s->mv[0] = (int16_t)mv[0];
            s->mv[1] = (int16_t)mv[1];
        }

    if (intra && rec->comp == COMP_LUMACHROMA)
        recon_chroma(r, x0, y0, w, h, derive_chroma_mode(rec->cidx, lmode), lmode, qp, ctp);
}

/* One record, on the reconstruction's parser_t */
static void recon_rec(parser_t *r, const cu_rec_t *rec)
{
    switch (rec->kind) {
    case REC_PATCH:
        r->cur_patch = rec->patch;
        r->patch_qp = rec->patch_qp;
        r->prev_cu_qp = r->lcu_qp = rec->patch_qp;
        r->fixed_qp = rec->fixed_qp;
        break;
    case REC_CU:
        cu_recon(r, rec);
        break;
    case REC_CHROMA:
        cu_recon_chroma(r, rec);
        break;
    case REC_ROW:
        filter_rows_done(r->owner, rec->row + 1);
        break;
    }
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

    cu_rec_t rec = {
        .kind = REC_CU, .comp = (uint8_t)comp, .skip = (uint8_t)skip, .direct = (uint8_t)direct,
        .intra = (uint8_t)intra, .subtype = (uint8_t)subtype, .ref = (uint8_t)ref,
        .lidx = (uint8_t)lidx, .eipm = (uint8_t)eipm, .cidx = (uint8_t)cidx, .ctp = (uint8_t)ctp,
        .group_start = x0 == p->qp_group_x && y0 == p->qp_group_y, .lcu_qp = (uint8_t)p->lcu_qp,
        .dqp = (int16_t)dqp, .mvd = { mvd[0], mvd[1] },
        .x0 = (uint16_t)x0, .y0 = (uint16_t)y0, .w = (uint16_t)w, .h = (uint16_t)h,
        .coef = { p->coef[0], p->coef[1], p->coef[2] },
    };
    emit(p, &rec);

    /* the parser's own map: what later syntax depends on */
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
static void store_tmv_rows(const parser_t *p, svac3_tmv_t *t, int ty0, int ty1)
{
    const int tw = (p->pic_w + 15) >> 4;
    for (int ty = ty0; ty < ty1; ty++)
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

/* Records, and the coefficients they point at, in chunks that never move */
#define REC_CHUNK  2048
#define COEF_CHUNK (1 << 18)

typedef struct rec_chunk {
    struct rec_chunk *next;
    cu_rec_t rec[REC_CHUNK];
} rec_chunk_t;

typedef struct coef_chunk {
    struct coef_chunk *next;
    size_t used;
    int16_t data[COEF_CHUNK];
} coef_chunk_t;

struct svac3_picture {
    parser_t p;                  /* parsing */
    parser_t r;                  /* reconstruction */

    /* with a pool: records go to a reconstruction thread */
    int threaded, failed;        /* failed: out of memory for records */
    pthread_t worker;
    pthread_mutex_t lock;
    pthread_cond_t ready;
    rec_chunk_t *rec_head, *rec_tail;
    int tail_n;                  /* records in rec_tail */
    long produced, published;
    int ending;                  /* every record is published: the worker stops */

    /* loop filters: deblocking in LCU-row bands one row behind the
     * reconstruction (a band's bottom samples are the next row's intra
     * references), SAO one band behind that, from a copy of the deblocked
     * rows it reads; with patches narrower than the picture, at the end */
    int wavefront;
    int dbk_off, sao_on;
    svac3_progress_fn progress;
    void *progress_ctx;
    int tmv_rows;                /* rows of temporal motion stored */
    /* with 3 threads or more, the filters have a thread of their own */
    int filter_threaded, filter_ending, rows_ready;
    pthread_t filter;
    pthread_mutex_t flock;
    pthread_cond_t fready;
    svac3_dbk_info_t dbk;
    int dbk_done, sao_done, copied[3];
    uint8_t *sao_src[3];         /* deblocked samples SAO reads, frame layout */
    uint8_t *sao_buf;
    size_t sao_buf_n, unit_n;
    uint32_t *u_id;
    uint8_t *u_qp, *u_cbf;
    int8_t *u_ref;
    int16_t *u_mv;
    coef_chunk_t *coef_head, *coef_tail;

    /* kept for the next picture of a cache: spare chunks, buffer sizes */
    rec_chunk_t *rec_spare;
    coef_chunk_t *coef_spare;
    size_t scu_n, sao_n;
};

/* Tell the progress callback which rows are final: those no filter step
 * still to come can change. SAO of LCU row k leaves luma rows above y1 - 4
 * final (y1 the next band's first row) and chroma rows above y1 / 2 - 4;
 * deblocking alone, y1 - 3 and y1 / 2 - 3; in luma rows that is y1 - 8
 * and y1 - 6 for every plane. */
static void report(svac3_picture_t *pic)
{
    if (!pic->progress)
        return;
    const parser_t *r = &pic->r;
    const int L = r->lcu_size;
    int rows;
    if (pic->sao_on)
        rows = pic->sao_done >= r->lcu_h ? r->pic_h : pic->sao_done * L - 8;
    else if (!pic->dbk_off)
        rows = pic->dbk_done >= r->lcu_h ? r->pic_h : pic->dbk_done * L - 6;
    else
        rows = pic->dbk_done >= r->lcu_h ? r->pic_h : pic->dbk_done * L;
    pic->progress(pic->progress_ctx, rows < 0 ? 0 : rows, -1);
}

/* The deblocking parameters of LCU row k's units */
static void fill_units(svac3_picture_t *pic, int k)
{
    const parser_t *r = &pic->r;
    const int urows = (r->pic_h + 3) / 4, u0 = k * r->lcu_size / 4;
    int u1 = (k + 1) * r->lcu_size / 4;
    if (u1 > urows)
        u1 = urows;
    for (size_t i = (size_t)u0 * r->scu_stride; i < (size_t)u1 * r->scu_stride; i++) {
        const scu_t *u = &r->scu[i];
        pic->u_id[i] = u->cu_id;
        pic->u_qp[i] = u->qp;
        pic->u_cbf[i] = u->cbf;
        pic->u_ref[i] = u->intra ? -1 : u->ref;
        pic->u_mv[2 * i] = u->mv[0];
        pic->u_mv[2 * i + 1] = u->mv[1];
    }
}

/* Filter what the reconstruction of rows LCU rows allows (all of it when
 * final): deblocking of band k once row k + 1 is reconstructed, SAO of LCU
 * row k once band k is deblocked. Called on increasing rows, and the same
 * result as filtering the whole picture at once. */
static void filter_upto(svac3_picture_t *pic, int rows, int final)
{
    parser_t *r = &pic->r;
    const int L = r->lcu_size, lh = r->lcu_h;
    const int dbk_to = final ? lh : rows - 1;
    /* every ready band in one call: the vertical edges of all their rows,
     * then the horizontal ones, which is what band after band gives */
    if (pic->dbk_done < dbk_to) {
        if (!pic->dbk_off) {
            for (int k = pic->dbk_done; k < dbk_to; k++)
                fill_units(pic, k);
            svac3_deblock_rows(r->frame, &pic->dbk, pic->dbk_done * L, dbk_to * L, r->pool);
        }
        pic->dbk_done = dbk_to;
    }
    if (!pic->sao_on || pic->sao_done >= pic->dbk_done) {
        report(pic);
        return;
    }
    /* the deblocked rows the SAO of LCU rows sao_done .. dbk_done - 1 reads
     * that are final now: up to 4 above the next band (whose top edge
     * changes the 3 above it) */
    const int k = pic->dbk_done - 1;
    for (int c = 0; c < 3; c++) {
        const int Lc = c ? L / 2 : L, Hc = r->frame->height[c], s = r->frame->stride[c];
        int upto = k == lh - 1 ? Hc : (k + 1) * Lc - 3;
        if (upto > Hc)
            upto = Hc;
        if (upto > pic->copied[c]) {
            memcpy(pic->sao_src[c] + (size_t)pic->copied[c] * s,
                   r->frame->plane[c] + (size_t)pic->copied[c] * s,
                   (size_t)(upto - pic->copied[c]) * s);
            pic->copied[c] = upto;
        }
    }
    svac3_sao_rows(r->frame, pic->sao_src, (const svac3_sao_t (*)[3])r->sao, L, r->lcu_w, lh,
                   pic->sao_done, pic->dbk_done, r->pool);
    pic->sao_done = pic->dbk_done;
    report(pic);
}

/* The reconstruction has finished LCU rows 0 .. rows - 1 */
static void filter_rows_done(svac3_picture_t *pic, int rows)
{
    if (!pic || !pic->wavefront)
        return;
    /* the temporal motion of the rows: a later picture may be waiting */
    if (pic->progress && pic->p.tmv_out) {
        const parser_t *r = &pic->r;
        int ty1 = rows * r->lcu_size / 16;
        if (ty1 > (r->pic_h + 15) >> 4)
            ty1 = (r->pic_h + 15) >> 4;
        if (ty1 > pic->tmv_rows) {
            store_tmv_rows(r, pic->p.tmv_out, pic->tmv_rows, ty1);
            pic->tmv_rows = ty1;
            pic->progress(pic->progress_ctx, -1, ty1);
        }
    }
    if (!pic->filter_threaded) {
        filter_upto(pic, rows, 0);
        return;
    }
    pthread_mutex_lock(&pic->flock);
    pic->rows_ready = rows;
    pthread_cond_signal(&pic->fready);
    pthread_mutex_unlock(&pic->flock);
}

static void *filter_worker(void *arg)
{
    svac3_picture_t *pic = arg;
    int seen = 0;
    for (;;) {
        pthread_mutex_lock(&pic->flock);
        while (pic->rows_ready == seen && !pic->filter_ending)
            pthread_cond_wait(&pic->fready, &pic->flock);
        const int rows = pic->rows_ready, last = pic->filter_ending;
        pthread_mutex_unlock(&pic->flock);
        if (rows > seen)
            filter_upto(pic, rows, 0);
        seen = rows;
        if (last)
            return NULL;
    }
}

/* Start the filter thread; on failure the filters run on the
 * reconstruction thread */
static void start_filter_thread(svac3_picture_t *pic)
{
    if (pthread_mutex_init(&pic->flock, NULL))
        return;
    if (pthread_cond_init(&pic->fready, NULL)) {
        pthread_mutex_destroy(&pic->flock);
        return;
    }
    pic->filter_threaded = 1;
    /* one LCU row has little to share out: the thread works alone */
    pic->r.pool = NULL;
    if (pthread_create(&pic->filter, NULL, filter_worker, pic)) {
        pthread_mutex_destroy(&pic->flock);
        pthread_cond_destroy(&pic->fready);
        pic->filter_threaded = 0;
        pic->r.pool = pic->p.pool;
    }
}

/* Stop the filter thread once it has filtered what it was given */
static void finish_filter_thread(svac3_picture_t *pic)
{
    if (!pic->filter_threaded)
        return;
    pthread_mutex_lock(&pic->flock);
    pic->filter_ending = 1;
    pthread_cond_signal(&pic->fready);
    pthread_mutex_unlock(&pic->flock);
    pthread_join(pic->filter, NULL);
    pthread_mutex_destroy(&pic->flock);
    pthread_cond_destroy(&pic->fready);
    pic->filter_threaded = 0;
    pic->r.pool = pic->p.pool;
}

/* Buffers for the loop filters of a picture of r's size, reused from the
 * cache's picture when they fit */
static int filter_setup(svac3_picture_t *pic, const svac3_hls_t *hls)
{
    parser_t *r = &pic->r;
    const size_t n = (size_t)r->scu_stride * ((r->pic_h + 3) / 4);
    pic->dbk_off = hls->pps.deblocking_filter_disable_flag;
    pic->sao_on = r->sps->sao_enable_flag;
    if (pic->unit_n != n) {
        free(pic->u_id);
        free(pic->u_qp);
        free(pic->u_cbf);
        free(pic->u_ref);
        free(pic->u_mv);
        pic->u_id = malloc(n * sizeof(*pic->u_id));
        pic->u_qp = malloc(n);
        pic->u_cbf = malloc(n);
        pic->u_ref = malloc(n);
        pic->u_mv = malloc(n * 2 * sizeof(*pic->u_mv));
        pic->unit_n = (pic->u_id && pic->u_qp && pic->u_cbf && pic->u_ref && pic->u_mv) ? n : 0;
        if (!pic->unit_n)
            return SVAC3_ERR_NOMEM;
    }
    const svac3_frame_t *f = r->frame;
    const size_t fs = (size_t)f->stride[0] * f->height[0] + (size_t)f->stride[1] * f->height[1] +
                      (size_t)f->stride[2] * f->height[2];
    if (pic->sao_on && pic->sao_buf_n < fs) {
        free(pic->sao_buf);
        pic->sao_buf = malloc(fs);
        pic->sao_buf_n = pic->sao_buf ? fs : 0;
        if (!pic->sao_buf)
            return SVAC3_ERR_NOMEM;
    }
    if (pic->sao_on) {
        pic->sao_src[0] = pic->sao_buf;
        pic->sao_src[1] = pic->sao_src[0] + (size_t)f->stride[0] * f->height[0];
        pic->sao_src[2] = pic->sao_src[1] + (size_t)f->stride[1] * f->height[1];
    }
    svac3_dbk_info_t d = {
        .pic_w = r->pic_w, .pic_h = r->pic_h, .stride = r->scu_stride,
        .cu_id = pic->u_id, .qp = pic->u_qp,
        .inter = r->pct != 1, .cbf = pic->u_cbf, .ref = pic->u_ref, .mv = pic->u_mv,
        .alpha_offset = hls->ph.alpha_c_offset, .beta_offset = hls->ph.beta_offset,
        .filter_type = hls->pps.deblocking_filter_type,
        .cb_delta = r->cb_delta, .cr_delta = r->cr_delta,
    };
    pic->dbk = d;
    /* rows complete in order only when patches span the picture's width;
     * and LCUs under 16 rows would put band boundaries between chroma
     * edges, which nothing in the corpus exercises: those filter at the end */
    pic->wavefront = (uint64_t)r->sps->patch_width_minus1 + 1 >= (uint64_t)r->lcu_w &&
                     r->lcu_size >= 16;
    return SVAC3_OK;
}

/* A copy of n coefficients in the picture's chunks, or NULL */
static const int16_t *keep_coefs(svac3_picture_t *pic, const int16_t *c, size_t n)
{
    coef_chunk_t *t = pic->coef_tail;
    if (!t || t->used + n > COEF_CHUNK) {
        if ((t = pic->coef_spare))
            pic->coef_spare = t->next;
        else if (!(t = malloc(sizeof(*t))))
            return NULL;
        t->next = NULL;
        t->used = 0;
        if (pic->coef_tail)
            pic->coef_tail->next = t;
        else
            pic->coef_head = t;
        pic->coef_tail = t;
    }
    int16_t *d = t->data + t->used;
    memcpy(d, c, n * sizeof(*c));
    t->used += n;
    return d;
}

static void publish(svac3_picture_t *pic)
{
    pthread_mutex_lock(&pic->lock);
    pic->published = pic->produced;
    pthread_cond_signal(&pic->ready);
    pthread_mutex_unlock(&pic->lock);
}

static void emit(parser_t *p, cu_rec_t *rec)
{
    svac3_picture_t *pic = p->pic;
    if (!p->recon || !pic)
        return;
    if (!pic->threaded) {
        recon_rec(&pic->r, rec);                 /* in step with the parser */
        return;
    }
    if (pic->failed)
        return;
    /* the parser reuses its coefficient buffers: keep a copy */
    for (int c = 0; c < 3; c++)
        if ((rec->kind == REC_CU || rec->kind == REC_CHROMA) && (rec->ctp & (1 << c))) {
            const size_t n = c ? (size_t)(rec->w / 2) * (rec->h / 2) : (size_t)rec->w * rec->h;
            if (!(rec->coef[c] = keep_coefs(pic, rec->coef[c], n))) {
                pic->failed = 1;
                return;
            }
        }
    if (!pic->rec_tail || pic->tail_n == REC_CHUNK) {
        rec_chunk_t *ch = pic->rec_spare;
        if (ch)
            pic->rec_spare = ch->next;
        else if (!(ch = malloc(sizeof(*ch)))) {
            pic->failed = 1;
            return;
        }
        ch->next = NULL;
        if (pic->rec_tail)
            pic->rec_tail->next = ch;
        else
            pic->rec_head = ch;
        pic->rec_tail = ch;
        pic->tail_n = 0;
    }
    pic->rec_tail->rec[pic->tail_n++] = *rec;
    pic->produced++;
}

/* The reconstruction thread: records in order, until they end */
static void *recon_worker(void *arg)
{
    svac3_picture_t *pic = arg;
    rec_chunk_t *ch = NULL;
    int idx = 0;
    long done = 0;
    for (;;) {
        pthread_mutex_lock(&pic->lock);
        while (pic->published == done && !pic->ending)
            pthread_cond_wait(&pic->ready, &pic->lock);
        const long avail = pic->published;
        const int last = pic->ending;
        if (!ch)
            ch = pic->rec_head;
        pthread_mutex_unlock(&pic->lock);
        for (; done < avail; done++) {
            if (idx == REC_CHUNK) {
                ch = ch->next;
                idx = 0;
            }
            recon_rec(&pic->r, &ch->rec[idx++]);
        }
        if (last && done == avail)
            return NULL;
    }
}

/* Stop the reconstruction thread once it has done every record */
static void finish_worker(svac3_picture_t *pic)
{
    if (!pic->threaded)
        return;
    pthread_mutex_lock(&pic->lock);
    pic->published = pic->produced;
    pic->ending = 1;
    pthread_cond_signal(&pic->ready);
    pthread_mutex_unlock(&pic->lock);
    pthread_join(pic->worker, NULL);
    pthread_mutex_destroy(&pic->lock);
    pthread_cond_destroy(&pic->ready);
    pic->threaded = 0;
    finish_filter_thread(pic);
}

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

/* The picture's chunks go back to its spares */
static void spare_chunks(svac3_picture_t *pic)
{
    while (pic->rec_head) {
        rec_chunk_t *n = pic->rec_head->next;
        pic->rec_head->next = pic->rec_spare;
        pic->rec_spare = pic->rec_head;
        pic->rec_head = n;
    }
    while (pic->coef_head) {
        coef_chunk_t *n = pic->coef_head->next;
        pic->coef_head->next = pic->coef_spare;
        pic->coef_spare = pic->coef_head;
        pic->coef_head = n;
    }
    pic->rec_tail = NULL;
    pic->coef_tail = NULL;
}

void svac3_picture_free(svac3_picture_t *pic)
{
    if (!pic)
        return;
    finish_worker(pic);
    spare_chunks(pic);
    while (pic->rec_spare) {
        rec_chunk_t *n = pic->rec_spare->next;
        free(pic->rec_spare);
        pic->rec_spare = n;
    }
    while (pic->coef_spare) {
        coef_chunk_t *n = pic->coef_spare->next;
        free(pic->coef_spare);
        pic->coef_spare = n;
    }
    free(pic->p.sao);
    free(pic->p.scu);
    free(pic->r.scu);
    free(pic->sao_buf);
    free(pic->u_id);
    free(pic->u_qp);
    free(pic->u_cbf);
    free(pic->u_ref);
    free(pic->u_mv);
    free(pic);
}

void svac3_picture_release(svac3_picture_t *pic, svac3_picture_cache_t *cache)
{
    if (!pic)
        return;
    if (!cache || cache->pic) {
        svac3_picture_free(pic);
        return;
    }
    finish_worker(pic);
    spare_chunks(pic);
    cache->pic = pic;
}

void svac3_picture_cache_free(svac3_picture_cache_t *cache)
{
    if (cache) {
        svac3_picture_free(cache->pic);
        cache->pic = NULL;
    }
}

int svac3_picture_begin(const svac3_hls_t *hls, svac3_frame_t *frame, const svac3_refs_t *refs,
                        svac3_tmv_t *tmv, svac3_picture_t **out)
{
    return svac3_picture_begin_cached(hls, frame, refs, tmv, NULL, out);
}

/* A picture from the cache, its buffers kept and everything else cleared,
 * or a new one */
static svac3_picture_t *take_picture(svac3_picture_cache_t *cache)
{
    svac3_picture_t *pic = cache ? cache->pic : NULL;
    if (!pic)
        return calloc(1, sizeof(*pic));
    cache->pic = NULL;
    scu_t *ps = pic->p.scu, *rs = pic->r.scu;
    svac3_sao_t (*sao)[3] = pic->p.sao;
    rec_chunk_t *rsp = pic->rec_spare;
    coef_chunk_t *csp = pic->coef_spare;
    const size_t scu_n = pic->scu_n, sao_n = pic->sao_n;
    uint8_t *sbuf = pic->sao_buf;
    const size_t sbuf_n = pic->sao_buf_n, unit_n = pic->unit_n;
    uint32_t *uid = pic->u_id;
    uint8_t *uqp = pic->u_qp, *ucbf = pic->u_cbf;
    int8_t *uref = pic->u_ref;
    int16_t *umv = pic->u_mv;
    memset(pic, 0, sizeof(*pic));
    pic->sao_buf = sbuf;
    pic->sao_buf_n = sbuf_n;
    pic->unit_n = unit_n;
    pic->u_id = uid;
    pic->u_qp = uqp;
    pic->u_cbf = ucbf;
    pic->u_ref = uref;
    pic->u_mv = umv;
    pic->p.scu = ps;
    pic->r.scu = rs;
    pic->p.sao = sao;
    pic->rec_spare = rsp;
    pic->coef_spare = csp;
    pic->scu_n = scu_n;
    pic->sao_n = sao_n;
    return pic;
}

/* buf, holding *have elements of size sz, cleared and n long */
static void *zeroed(void *buf, size_t *have, size_t n, size_t sz)
{
    if (buf && *have == n) {
        memset(buf, 0, n * sz);
        return buf;
    }
    free(buf);
    buf = calloc(n, sz);
    *have = buf ? n : 0;
    return buf;
}

int svac3_picture_begin_cached(const svac3_hls_t *hls, svac3_frame_t *frame,
                               const svac3_refs_t *refs, svac3_tmv_t *tmv,
                               svac3_picture_cache_t *cache, svac3_picture_t **out)
{
    int ret = svac3_patch_supported(hls);

    *out = NULL;
    if (ret)
        return ret;
    svac3_picture_t *pic = take_picture(cache);
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
        svac3_picture_free(pic);       /* with what the cache had kept */
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
    const size_t scu_n = (size_t)p->scu_stride * ((p->pic_h + 3) / 4);
    size_t have = pic->scu_n, rhave = pic->scu_n;
    p->scu = zeroed(p->scu, &have, scu_n, sizeof(scu_t));
    if (p->recon) {
        p->sao = zeroed(p->sao, &pic->sao_n, (size_t)p->lcu_w * p->lcu_h, sizeof(*p->sao));
        pic->r.scu = zeroed(pic->r.scu, &rhave, scu_n, sizeof(scu_t));
    }
    pic->scu_n = have;
    if (!p->scu || (p->recon && (!p->sao || !pic->r.scu || rhave != scu_n))) {
        svac3_picture_free(pic);
        return SVAC3_ERR_NOMEM;
    }
    p->pic = pic;
    if (p->recon) {
        /* the reconstruction: the same picture, its own map of decoded units */
        parser_t *r = &pic->r;
        scu_t *rs = r->scu;                     /* cleared above */
        memcpy(r, p, sizeof(*r));
        r->pic = NULL;
        r->owner = pic;
        r->scu = rs;
        if (filter_setup(pic, hls)) {
            svac3_picture_free(pic);
            return SVAC3_ERR_NOMEM;
        }
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
    cu_rec_t start = { .kind = REC_PATCH, .patch = (uint8_t)k, .patch_qp = (uint8_t)p->patch_qp,
                       .fixed_qp = (uint8_t)p->fixed_qp };
    emit(p, &start);
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
        if (p->pic && p->pic->wavefront && i % pcols == pcols - 1) {
            cu_rec_t row = { .kind = REC_ROW, .row = (uint16_t)ly };
            emit(p, &row);
        }
        if (p->pic && p->pic->threaded)
            publish(p->pic);                    /* an LCU's records at a time */
        if (stuff != (i == n_lcu - 1)) {
            if (st->stuffing_ok)
                st->first_bad_lcu = (unsigned)l;
            st->stuffing_ok = 0;
        }
        if (svac3_aec_overrun(&p->aec) > 64) {
            /* ran out of data: out of sync or truncated, stop here */
            if (st->stuffing_ok)
                st->first_bad_lcu = (unsigned)l;
            st->stuffing_ok = 0;
            break;
        }
    }
    st->bytes_used = svac3_aec_bytes_read(&p->aec);
    st->overrun_bits = svac3_aec_overrun(&p->aec);
    return SVAC3_OK;
}

void svac3_picture_set_progress(svac3_picture_t *pic, svac3_progress_fn fn, void *ctx)
{
    pic->progress = fn;
    pic->progress_ctx = ctx;
}

void svac3_picture_set_pool(svac3_picture_t *pic, svac3_pool_t *pool)
{
    pic->p.pool = pic->r.pool = pool;
    /* with threads to spare, reconstruction runs beside the parser */
    if (pool && pic->p.recon && !pic->threaded && !pic->p.next_patch) {
        if (pthread_mutex_init(&pic->lock, NULL))
            return;
        if (pthread_cond_init(&pic->ready, NULL)) {
            pthread_mutex_destroy(&pic->lock);
            return;
        }
        pic->threaded = 1;
        /* 3 threads or more: the filters follow the reconstruction on a
         * thread of their own. With 2, filtering the whole picture on both
         * at the end does better than adding it to the reconstruction's. */
        if (pic->wavefront && svac3_pool_threads(pool) >= 3)
            start_filter_thread(pic);
        else
            pic->wavefront = 0;
        if (pthread_create(&pic->worker, NULL, recon_worker, pic)) {
            pthread_mutex_destroy(&pic->lock);
            pthread_cond_destroy(&pic->ready);
            pic->threaded = 0;
            finish_filter_thread(pic);
        }
    }
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
    return svac3_picture_end_cached(pic, hls, NULL);
}

int svac3_picture_end_cached(svac3_picture_t *pic, const svac3_hls_t *hls,
                             svac3_picture_cache_t *cache)
{
    parser_t *p = &pic->p;
    int ret = SVAC3_OK;

    finish_worker(pic);                         /* every record reconstructed */
    if (p->next_patch != p->num_patches)
        ret = SVAC3_ERR_CORRUPT;                /* patches missing */
    else if (pic->failed)
        ret = SVAC3_ERR_NOMEM;
    else if (p->recon)
        filter_upto(pic, 0, 1);                 /* what the rows left to filter */
    if (ret == SVAC3_OK && p->tmv_out)          /* the rows not stored yet */
        store_tmv_rows(p->recon ? &pic->r : p, p->tmv_out, pic->tmv_rows, (p->pic_h + 15) >> 4);
    if (pic->progress)                          /* all of it, even in error */
        pic->progress(pic->progress_ctx, p->pic_h, (p->pic_h + 15) >> 4);
    svac3_picture_release(pic, cache);
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
