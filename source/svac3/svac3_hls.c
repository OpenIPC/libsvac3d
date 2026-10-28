/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright (c) 2026 OpenIPC contributors
 *
 * SVAC3 high-level syntax parser. Mirrors tools/svac3_parse.py field for
 * field; the dump format of the two must stay identical (tests/hls_dump.sh).
 */
#include "svac3_hls.h"

#include <stdlib.h>
#include <string.h>

/* ------------------------------------------------------------------------ */
/* Bit reader. Unlike the AVS3 reader in decoder/bitstream.c it never makes
 * up a value: reading past the end sets err and every later read returns 0. */

typedef struct {
    const uint8_t *buf;
    size_t bits, pos;
    int err;
} bits_t;

static void bits_init(bits_t *b, const uint8_t *buf, size_t len)
{
    b->buf = buf;
    b->bits = len * 8;
    b->pos = 0;
    b->err = 0;
}

static uint32_t rd_u(bits_t *b, int n)
{
    uint32_t v = 0;
    if (b->err || b->bits - b->pos < (size_t)n) {
        b->err = SVAC3_ERR_EOF;
        b->pos = b->bits;
        return 0;
    }
    for (int i = 0; i < n; i++, b->pos++)
        v = (v << 1) | ((b->buf[b->pos >> 3] >> (7 - (b->pos & 7))) & 1);
    return v;
}

static uint32_t rd_ue(bits_t *b)
{
    int zeros = 0;
    while (!b->err && rd_u(b, 1) == 0) {
        if (++zeros > 31) {
            b->err = SVAC3_ERR_INVALID;
            return 0;
        }
    }
    if (b->err)
        return 0;
    return (uint32_t)(((uint64_t)1 << zeros) - 1 + rd_u(b, zeros));
}

static int32_t rd_se(bits_t *b)
{
    uint32_t k = rd_ue(b);
    return (k & 1) ? (int32_t)((k + 1) / 2) : -(int32_t)(k / 2);
}

static int trailing_ok(bits_t *b)
{
    if (b->err || b->pos >= b->bits)
        return 0;
    if (rd_u(b, 1) != 1)
        return 0;
    while (b->pos < b->bits)
        if (rd_u(b, 1) != 0)
            return 0;
    return 1;
}

/* ------------------------------------------------------------------------ */
/* Dump helpers. */

#define E(fmt, ...)                                             \
    do {                                                        \
        if (dump)                                               \
            fprintf(dump, fmt "\n", __VA_ARGS__);               \
    } while (0)

/* Read n bits into dst and dump them under name. */
#define F(dst, name, n)                                         \
    do {                                                        \
        (dst) = rd_u(b, (n));                                   \
        E(name "=%u", (unsigned)(dst));                         \
    } while (0)

static void dump_list(FILE *dump, const char *key, const int32_t *v, int n,
                      const uint8_t *lib)
{
    if (!dump)
        return;
    fprintf(dump, "%s=", key);
    for (int i = 0; i < n; i++) {
        if (i)
            fputc(',', dump);
        if (lib && lib[i])
            fprintf(dump, "L%d", v[i]);
        else
            fprintf(dump, "%d", v[i]);
    }
    fputc('\n', dump);
}

/* ------------------------------------------------------------------------ */
/* Annex-B and the NAL header. */

int svac3_next_nal(const uint8_t *buf, size_t len, size_t *pos,
                   const uint8_t **nal, size_t *nal_len)
{
    size_t i = *pos, s, e;

    while (i + 3 <= len && !(buf[i] == 0 && buf[i + 1] == 0 && buf[i + 2] == 1))
        i++;
    if (i + 3 > len)
        return 0;
    s = i + 3;
    for (i = s; i + 3 <= len; i++)
        if (buf[i] == 0 && buf[i + 1] == 0 && buf[i + 2] == 1)
            break;
    e = (i + 3 <= len) ? i : len;
    *pos = e;
    /* trailing_zero_8bits and the zero_byte of a 4-byte start code */
    while (e > s && buf[e - 1] == 0)
        e--;
    *nal = buf + s;
    *nal_len = e - s;
    return 1;
}

size_t svac3_unescape(const uint8_t *src, size_t src_len, uint8_t *dst)
{
    size_t n = 0;
    int zeros = 0;

    for (size_t i = 0; i < src_len; i++) {
        if (zeros >= 2 && src[i] == 3) {
            zeros = 0;
            continue;
        }
        dst[n++] = src[i];
        zeros = src[i] == 0 ? zeros + 1 : 0;
    }
    return n;
}

int svac3_parse_nal_header(const uint8_t *nal, size_t len, svac3_nal_header_t *h)
{
    if (len < 2)
        return SVAC3_ERR_EOF;
    unsigned v = (nal[0] << 8) | nal[1];
    h->forbidden_zero_bit     = v >> 15;
    h->encryption_idc         = (v >> 13) & 3;
    h->authentication_idc     = (v >> 11) & 3;
    h->authentication_data_id = (v >> 10) & 1;
    h->layer_id               = (v >> 8) & 3;
    h->temporal_id            = (v >> 5) & 7;
    h->nal_unit_type          = v & 31;
    return SVAC3_OK;
}

/* ------------------------------------------------------------------------ */
/* 表14, 表15 */

static int parse_rpl(bits_t *b, const svac3_sps_t *s, svac3_rpl_t *r,
                     const char *prefix, FILE *dump)
{
    char key[64];

    memset(r, 0, sizeof(*r));
    if (s->library_picture_enable_flag && !s->library_stream_flag)
        r->reference_to_library_enable_flag = rd_u(b, 1);
    uint32_t n = rd_ue(b);
    /* num_of_ref_pic shall be less than MaxDpbSize */
    if (b->err || n > s->max_dpb_size_minus1 || n > SVAC3_MAX_REFS)
        return b->err ? b->err : SVAC3_ERR_INVALID;
    r->num_of_ref_pic = n;
    for (uint32_t i = 0; i < n; i++) {
        if (r->reference_to_library_enable_flag)
            r->library_index_flag[i] = rd_u(b, 1);
        if (r->library_index_flag[i]) {
            uint32_t li = rd_ue(b);
            if (li > 511)        /* referenced_library_picture_index: 0..511 */
                return b->err ? b->err : SVAC3_ERR_INVALID;
            r->delta_doi[i] = (int32_t)li;
        } else {
            uint32_t a = rd_ue(b);
            if (a > 255)         /* abs_delta_doi: 0..2^8-1 */
                return b->err ? b->err : SVAC3_ERR_INVALID;
            int sign = a > 0 ? rd_u(b, 1) : 0;
            r->delta_doi[i] = sign ? -(int32_t)a : (int32_t)a;
        }
    }
    E("%s.num_of_ref_pic=%u", prefix, n);
    snprintf(key, sizeof(key), "%s.delta_doi", prefix);
    dump_list(dump, key, r->delta_doi, n, r->library_index_flag);
    return b->err;
}

static int parse_wq_matrix(bits_t *b, uint8_t m4[16], uint8_t m8[64],
                           const char *prefix, FILE *dump)
{
    int32_t v[80];

    for (int i = 0; i < 80; i++) {
        uint32_t c = rd_ue(b);
        if (b->err)
            return b->err;
        if (c < 1 || c > 255)    /* weight_quant_coeff: 1..255 */
            return SVAC3_ERR_INVALID;
        v[i] = (int32_t)c;
    }
    for (int i = 0; i < 16; i++)
        m4[i] = (uint8_t)v[i];
    for (int i = 0; i < 64; i++)
        m8[i] = (uint8_t)v[16 + i];
    if (dump) {
        char key[64];
        snprintf(key, sizeof(key), "%s.weight_quant_matrix", prefix);
        dump_list(dump, key, v, 80, NULL);
    }
    return SVAC3_OK;
}

/* ------------------------------------------------------------------------ */
/* 表13 */

static int parse_sps(bits_t *b, svac3_sps_t *s, FILE *dump)
{
    int high, ret;

    memset(s, 0, sizeof(*s));
    F(s->profile_id, "profile_id", 8);
    F(s->level_id, "level_id", 8);
    F(s->temporal_nesting_flag, "temporal_nesting_flag", 1);
    F(s->library_picture_enable_flag, "library_picture_enable_flag", 1);
    F(s->library_stream_flag, "library_stream_flag", 1);
    if (s->library_picture_enable_flag) {
        F(s->duplicate_sequence_header_flag, "duplicate_sequence_header_flag", 1);
        F(s->library_picture_mode_index, "library_picture_mode_index", 2);
    }
    F(s->rapi_nalu_type_enable_flag, "rapi_nalu_type_enable_flag", 1);
    high = s->profile_id == 0x40 || s->profile_id == 0x42;
    if (high) {
        F(s->num_of_layers_minus1, "num_of_layers_minus1", 2);
        F(s->ssvc_enable_flag, "ssvc_enable_flag", 1);
        F(s->privacy_enable_flag, "privacy_enable_flag", 1);
        if (s->privacy_enable_flag)
            F(s->privacy_mode_index, "privacy_mode_index", 2);
        if (!s->ssvc_enable_flag)
            F(s->reference_picture_resampling_enable_flag,
              "reference_picture_resampling_enable_flag", 1);
        F(s->rcm_enable_flag, "rcm_enable_flag", 1);
        if (s->rcm_enable_flag) {
            F(s->rcm_mvp_enable_flag, "rcm_mvp_enable_flag", 1);
            F(s->rcm_mv_precision_type, "rcm_mv_precision_type", 1);
        }
    }
    if (s->num_of_layers_minus1 > 0)
        return SVAC3_ERR_UNSUPPORTED;            /* SSVC/TSVC layers */
    F(s->horizontal_size, "horizontal_size", 14);
    F(s->vertical_size, "vertical_size", 14);
    F(s->pic_width_in_luma, "pic_width_in_luma", 14);
    F(s->pic_height_in_luma, "pic_height_in_luma", 14);
    /* the display size lies within the coded size (5.2.2 semantics) */
    if (!s->horizontal_size || !s->vertical_size ||
        s->horizontal_size > s->pic_width_in_luma || s->vertical_size > s->pic_height_in_luma)
        return b->err ? b->err : SVAC3_ERR_INVALID;
    if (s->reference_picture_resampling_enable_flag) {
        F(s->rpr_pic_width_in_luma, "rpr_pic_width_in_luma", 14);
        F(s->rpr_pic_height_in_luma, "rpr_pic_height_in_luma", 14);
    }
    F(s->chroma_format, "chroma_format", 2);
    F(s->sample_precision, "sample_precision", 3);
    if (s->profile_id == 0x42)
        F(s->encoding_precision, "encoding_precision", 3);
    F(s->aspect_ratio, "aspect_ratio", 4);
    F(s->frame_rate_code, "frame_rate_code", 4);
    F(s->bit_rate_lower, "bit_rate_lower", 18);
    F(s->bit_rate_upper, "bit_rate_upper", 12);
    if (high) {
        F(s->low_delay, "low_delay", 1);
    } else {
        s->low_delay = 1;                        /* inferred when absent */
        E("low_delay=%s", "1(inferred)");
    }
    F(s->temporal_id_enable_flag, "temporal_id_enable_flag", 1);
    F(s->bbv_buffer_size, "bbv_buffer_size", 18);
    F(s->max_dpb_size_minus1, "max_dpb_size_minus1", 4);
    F(s->rpl1_index_exist_flag, "rpl1_index_exist_flag", 1);
    F(s->rpl1_same_as_rpl0_flag, "rpl1_same_as_rpl0_flag", 1);
    for (int l = 0; l < 2; l++) {
        char prefix[32];
        if (l == 1 && s->rpl1_same_as_rpl0_flag) {
            s->num_ref_pic_list_set[1] = s->num_ref_pic_list_set[0];
            memcpy(s->rpls[1], s->rpls[0], sizeof(s->rpls[0]));
            break;
        }
        s->num_ref_pic_list_set[l] = rd_ue(b);
        E("num_ref_pic_list_set[%d]=%u", l, s->num_ref_pic_list_set[l]);
        if (s->num_ref_pic_list_set[l] > SVAC3_MAX_RPLS)
            return SVAC3_ERR_INVALID;
        for (uint32_t j = 0; j < s->num_ref_pic_list_set[l]; j++) {
            snprintf(prefix, sizeof(prefix), "rpls[%d][%u]", l, j);
            if ((ret = parse_rpl(b, s, &s->rpls[l][j], prefix, dump)))
                return ret;
        }
    }
    s->num_ref_default_active_minus1[0] = rd_ue(b);
    s->num_ref_default_active_minus1[1] = rd_ue(b);
    E("num_ref_default_active_minus1=%u,%u",
      s->num_ref_default_active_minus1[0], s->num_ref_default_active_minus1[1]);
    F(s->log2_lcu_size_minus2, "log2_lcu_size_minus2", 3);
    F(s->log2_min_cu_size_minus2, "log2_min_cu_size_minus2", 2);
    F(s->log2_max_part_ratio_minus2, "log2_max_part_ratio_minus2", 2);
    F(s->max_split_times_minus6, "max_split_times_minus6", 3);
    F(s->log2_min_qt_size_minus2, "log2_min_qt_size_minus2", 3);
    F(s->log2_max_bt_size_minus2, "log2_max_bt_size_minus2", 3);
    F(s->log2_max_eqt_size_minus3, "log2_max_eqt_size_minus3", 2);
    F(s->weight_quant_enable_flag, "weight_quant_enable_flag", 1);
    if (s->weight_quant_enable_flag) {
        F(s->load_seq_weight_quant_data_flag, "load_seq_weight_quant_data_flag", 1);
        if (s->load_seq_weight_quant_data_flag &&
            (ret = parse_wq_matrix(b, s->wq_matrix4x4, s->wq_matrix8x8, "sps", dump)))
            return ret;
    }
    F(s->st_enable_flag, "st_enable_flag", 1);
    F(s->sao_enable_flag, "sao_enable_flag", 1);
    F(s->alf_enable_flag, "alf_enable_flag", 1);
    F(s->affine_enable_flag, "affine_enable_flag", 1);
    F(s->smvd_enable_flag, "smvd_enable_flag", 1);
    F(s->ipcm_enable_flag, "ipcm_enable_flag", 1);
    F(s->amvr_enable_flag, "amvr_enable_flag", 1);
    F(s->num_of_hmvp_cand, "num_of_hmvp_cand", 4);
    F(s->umve_enable_flag, "umve_enable_flag", 1);
    if (s->num_of_hmvp_cand != 0 && s->amvr_enable_flag)
        F(s->emvr_enable_flag, "emvr_enable_flag", 1);
    F(s->intra_pf_enable_flag, "intra_pf_enable_flag", 1);
    F(s->tscpm_enable_flag, "tscpm_enable_flag", 1);
    F(s->dt_enable_flag, "dt_enable_flag", 1);
    if (s->dt_enable_flag)
        F(s->log2_max_dt_size_minus4, "log2_max_dt_size_minus4", 2);
    F(s->pbt_enable_flag, "pbt_enable_flag", 1);
    s->eipm_enable_flag = 1;                     /* not transmitted, always on */
    E("eipm_enable_flag=%s", "1(implicit)");
    F(s->pmc_enable_flag, "pmc_enable_flag", 1);
    F(s->iip_enable_flag, "iip_enable_flag", 1);
    F(s->sawp_enable_flag, "sawp_enable_flag", 1);
    F(s->mipf_enable_flag, "mipf_enable_flag", 1);
    if (s->affine_enable_flag)
        F(s->asr_enable_flag, "asr_enable_flag", 1);
    F(s->awp_enable_flag, "awp_enable_flag", 1);
    F(s->tmvp_enable_flag, "tmvp_enable_flag", 1);
    if (s->tmvp_enable_flag) {
        F(s->sbtmvp_enable_flag, "sbtmvp_enable_flag", 1);
        F(s->etmvp_mvap_enable_flag, "etmvp_mvap_enable_flag", 1);
    }
    F(s->dmvr_enable_flag, "dmvr_enable_flag", 1);
    F(s->bio_enable_flag, "bio_enable_flag", 1);
    F(s->bgc_enable_flag, "bgc_enable_flag", 1);
    F(s->inter_pf_enable_flag, "inter_pf_enable_flag", 1);
    F(s->inter_pc_enable_flag, "inter_pc_enable_flag", 1);
    F(s->obmc_enable_flag, "obmc_enable_flag", 1);
    F(s->sbt_enable_flag, "sbt_enable_flag", 1);
    F(s->ist_enable_flag, "ist_enable_flag", 1);
    if (s->ist_enable_flag)
        F(s->ts_enable_flag, "ts_enable_flag", 1);
    F(s->cu_dqp_enable_flag, "cu_dqp_enable_flag", 1);
    F(s->srcc_enable_flag, "srcc_enable_flag", 1);
    F(s->maec_enable_flag, "maec_enable_flag", 1);
    F(s->dbr_enable_flag, "dbr_enable_flag", 1);
    F(s->esao_enable_flag, "esao_enable_flag", 1);
    F(s->ccsao_enable_flag, "ccsao_enable_flag", 1);
    if (s->esao_enable_flag) {
        s->sao_enable_flag = 0;
        E("sao_enable_flag=%s", "0(forced by esao)");
    }
    if (s->alf_enable_flag)
        F(s->ealf_enable_flag, "ealf_enable_flag", 1);
    if (high) {
        F(s->ibc_enable_flag, "ibc_enable_flag", 1);
        F(s->isc_enable_flag, "isc_enable_flag", 1);
        if (s->ibc_enable_flag || s->isc_enable_flag)
            F(s->num_of_intra_hmvp_cand, "num_of_intra_hmvp_cand", 4);
        F(s->fimc_enable_flag, "fimc_enable_flag", 1);
        F(s->nn_tools_set_hook, "nn_tools_set_hook", 8);
        if (s->nn_tools_set_hook & 1) {
            s->num_of_nn_filter_minus1 = rd_ue(b);
            E("num_of_nn_filter_minus1=%u", s->num_of_nn_filter_minus1);
        }
    }
    if (s->low_delay == 0)
        F(s->output_reorder_delay, "output_reorder_delay", 5);
    F(s->cross_patch_loop_filter_enable_flag, "cross_patch_loop_filter_enable_flag", 1);
    F(s->ref_colocated_patch_flag, "ref_colocated_patch_flag", 1);
    F(s->stable_patch_flag, "stable_patch_flag", 1);
    if (s->stable_patch_flag) {
        F(s->uniform_patch_flag, "uniform_patch_flag", 1);
        if (s->uniform_patch_flag) {
            s->patch_width_minus1 = rd_ue(b);
            s->patch_height_minus1 = rd_ue(b);
            E("patch_width_minus1=%u", s->patch_width_minus1);
            E("patch_height_minus1=%u", s->patch_height_minus1);
        }
    }
    unsigned reserved;
    F(reserved, "reserved_bits", 2);
    (void)reserved;
    return b->err;
}

/* ------------------------------------------------------------------------ */
/* 表30 */

static int parse_pps(bits_t *b, const svac3_sps_t *s, svac3_pps_t *p, FILE *dump)
{
    memset(p, 0, sizeof(*p));
    F(p->fixed_picture_qp_flag, "fixed_picture_qp_flag", 1);
    if (!p->fixed_picture_qp_flag && s->cu_dqp_enable_flag) {
        F(p->cu_delta_qp_flag, "cu_delta_qp_flag", 1);
        if (p->cu_delta_qp_flag)
            F(p->cu_qp_group_size_log2_minus3, "cu_qp_group_size_log2_minus3", 2);
    }
    F(p->deblocking_filter_disable_flag, "deblocking_filter_disable_flag", 1);
    if (!p->deblocking_filter_disable_flag) {
        F(p->deblocking_filter_type, "deblocking_filter_type", 1);
        F(p->deblocking_filter_parameter_flag, "deblocking_filter_parameter_flag", 1);
    }
    if (s->weight_quant_enable_flag) {
        F(p->picture_weight_quant_enable_flag, "picture_weight_quant_enable_flag", 1);
        if (p->picture_weight_quant_enable_flag) {
            F(p->picture_weight_quant_data_index, "picture_weight_quant_data_index", 2);
            if (p->picture_weight_quant_data_index == 1) {
                unsigned reserved;
                F(reserved, "reserved_bits", 1);
                (void)reserved;
                F(p->weight_quant_param_index, "weight_quant_param_index", 2);
                F(p->weight_quant_model, "weight_quant_model", 2);
                if (p->weight_quant_param_index == 1 || p->weight_quant_param_index == 2) {
                    char key[40];
                    for (int i = 0; i < 6; i++)
                        p->weight_quant_param_delta[i] = rd_se(b);
                    snprintf(key, sizeof(key), "weight_quant_param_delta%u",
                             p->weight_quant_param_index);
                    dump_list(dump, key, p->weight_quant_param_delta, 6, NULL);
                }
            } else if (p->picture_weight_quant_data_index == 2) {
                int ret = parse_wq_matrix(b, p->wq_matrix4x4, p->wq_matrix8x8, "pps", dump);
                if (ret)
                    return ret;
            }
        }
    }
    if (s->ibc_enable_flag)
        F(p->picture_ibc_enable_flag, "picture_ibc_enable_flag", 1);
    if (s->isc_enable_flag) {
        F(p->picture_isc_os_enable_flag, "picture_isc_os_enable_flag", 1);
        F(p->picture_isc_evs_ubvs_enable_flag, "picture_isc_evs_ubvs_enable_flag", 1);
    }
    if (s->fimc_enable_flag)
        F(p->picture_fimc_enable_flag, "picture_fimc_enable_flag", 1);
    if (s->affine_enable_flag)
        F(p->affine_subblock_size_flag, "affine_subblock_size_flag", 1);
    if (s->awp_enable_flag || s->sawp_enable_flag)
        F(p->picture_awp_refine_index, "picture_awp_refine_index", 1);
    return b->err;
}

/* ------------------------------------------------------------------------ */
/* 表31 */

static int parse_ph(bits_t *b, const svac3_sps_t *s, const svac3_pps_t *p,
                    svac3_ph_t *h, FILE *dump)
{
    int ret;

    memset(h, 0, sizeof(*h));
    F(h->decode_order_index, "decode_order_index", 8);
    if (s->library_stream_flag) {
        h->library_picture_index = rd_ue(b);
        E("library_picture_index=%u", h->library_picture_index);
        F(h->library_patch_num_minus1, "library_patch_num_minus1", 8);
    }
    F(h->picture_coding_type, "picture_coding_type", 2);
    const int t = h->picture_coding_type;
    if (s->reference_picture_resampling_enable_flag)
        F(h->picture_rpr_flag, "picture_rpr_flag", 1);
    if (s->profile_id != 0x30 && s->tmvp_enable_flag)
        F(h->picture_tmvp_enable_flag, "picture_tmvp_enable_flag", 1);
    else
        h->picture_tmvp_enable_flag = s->tmvp_enable_flag;   /* inferred */
    if (t != 1)
        F(h->random_access_decodable_flag, "random_access_decodable_flag", 1);
    F(h->bbv_delay, "bbv_delay", 32);
    if (t == 1) {
        F(h->time_code_flag, "time_code_flag", 1);
        if (h->time_code_flag)
            F(h->time_code, "time_code", 24);
    }
    if (s->low_delay == 0) {
        h->picture_output_delay = rd_ue(b);
        E("picture_output_delay=%u", h->picture_output_delay);
    } else {
        h->bbv_check_times = rd_ue(b);
        E("bbv_check_times=%u", h->bbv_check_times);
    }
    for (int l = 0; l < 2; l++) {
        if (l == 0) {
            F(h->ref_pic_list_set_flag[0], "ref_pic_list_set_flag[0]", 1);
        } else if (s->rpl1_index_exist_flag) {
            F(h->ref_pic_list_set_flag[1], "ref_pic_list_set_flag[1]", 1);
        } else {
            h->ref_pic_list_set_flag[1] = h->ref_pic_list_set_flag[0];
            E("ref_pic_list_set_flag[1]=%u(inferred)", h->ref_pic_list_set_flag[1]);
        }
        if (h->ref_pic_list_set_flag[l]) {
            if ((l == 0 || s->rpl1_index_exist_flag) && s->num_ref_pic_list_set[l] > 1) {
                h->ref_pic_list_set_index[l] = rd_ue(b);
                E("ref_pic_list_set_index[%d]=%u", l, h->ref_pic_list_set_index[l]);
            }
        } else {
            char prefix[16];
            snprintf(prefix, sizeof(prefix), "ph.rpl[%d]", l);
            if ((ret = parse_rpl(b, s, &h->rpl[l], prefix, dump)))
                return ret;
        }
    }
    if (t != 1) {
        F(h->num_ref_active_override_flag, "num_ref_active_override_flag", 1);
        if (h->num_ref_active_override_flag) {
            h->num_ref_active_minus1[0] = rd_ue(b);
            E("num_ref_active_minus1[0]=%u", h->num_ref_active_minus1[0]);
            if (t == 3) {
                h->num_ref_active_minus1[1] = rd_ue(b);
                E("num_ref_active_minus1[1]=%u", h->num_ref_active_minus1[1]);
            }
        }
    }
    F(h->picture_qp, "picture_qp", 7);
    if (s->privacy_mode_index == 2 || s->privacy_mode_index == 3)
        /* 表31 gives no descriptor; u(1) assumed, see docs/errata.md */
        F(h->picture_privacy_enable_flag, "picture_privacy_enable_flag", 1);
    if (t == 2) {
        unsigned reserved;
        F(reserved, "reserved_bits", 1);
        (void)reserved;
    }
    if (!p->deblocking_filter_disable_flag) {
        if (p->deblocking_filter_parameter_flag) {
            h->alpha_c_offset = rd_se(b);
            h->beta_offset = rd_se(b);
            E("alpha_c_offset=%d", h->alpha_c_offset);
            E("beta_offset=%d", h->beta_offset);
        }
        if (s->dbr_enable_flag) {
            /* DBR parameters are parsed for the dump but not kept yet: no
             * stream in the corpus enables DBR. */
            const char dir[2] = { 'v', 'h' };
            for (int d = 0; d < 2; d++) {
                unsigned en, alt, v;
                en = rd_u(b, 1);
                E("picture_dbr_%c_enable_flag=%u", dir[d], en);
                if (en) {
                    v = rd_u(b, 2); E("dbr_%c_offset0_minus1=%u", dir[d], v);
                    v = rd_u(b, 2); E("dbr_%c_offset1_minus1=%u", dir[d], v);
                }
                alt = rd_u(b, 1);
                E("picture_alt_dbr_%c_enable_flag=%u", dir[d], alt);
                if (alt) {
                    v = rd_u(b, 2); E("dbr_%c_alt_offset0_minus1=%u", dir[d], v);
                    v = rd_u(b, 2); E("dbr_%c_alt_offset1_minus1=%u", dir[d], v);
                }
                if (en || alt) {
                    v = rd_u(b, 1); E("dbr_%c_threshold_minus1=%u", dir[d], v);
                }
            }
        }
    }
    F(h->chroma_quant_param_disable_flag, "chroma_quant_param_disable_flag", 1);
    if (!h->chroma_quant_param_disable_flag) {
        h->chroma_quant_param_delta_cb = rd_se(b);
        h->chroma_quant_param_delta_cr = rd_se(b);
        E("chroma_quant_param_delta_cb=%d", h->chroma_quant_param_delta_cb);
        E("chroma_quant_param_delta_cr=%d", h->chroma_quant_param_delta_cr);
    }
    if (s->esao_enable_flag || s->ccsao_enable_flag)
        return SVAC3_ERR_UNSUPPORTED;            /* ESAO/CCSAO picture parameter sets */
    if (s->alf_enable_flag) {
        int any = 0;
        for (int c = 0; c < 3; c++) {
            h->picture_alf_enable_flag[c] = rd_u(b, 1);
            E("picture_alf_enable_flag[%d]=%u", c, h->picture_alf_enable_flag[c]);
            any |= h->picture_alf_enable_flag[c];
        }
        if (any)
            return SVAC3_ERR_UNSUPPORTED;        /* alf_parameter_set */
    }
    if (s->pmc_enable_flag)
        F(h->picture_pmc_param_index, "picture_pmc_param_index", 1);
    if (s->umve_enable_flag && t != 1)
        F(h->picture_umve_step_set_index, "picture_umve_step_set_index", 1);
    if (s->obmc_enable_flag && t == 3)
        F(h->picture_obmc_blending_set_index, "picture_obmc_blending_set_index", 1);
    if (t != 1 && s->inter_pc_enable_flag)
        F(h->picture_inter_pc_skip_flag, "picture_inter_pc_skip_flag", 1);
    if (s->ts_enable_flag)
        F(h->picture_ts_enable_flag, "picture_ts_enable_flag", 1);
    if (s->nn_tools_set_hook & 1)
        return SVAC3_ERR_UNSUPPORTED;            /* picture NN filter flags */
    if (s->rcm_enable_flag && t != 1)
        return SVAC3_ERR_UNSUPPORTED;            /* RCM picture parameters */
    return b->err;
}

/* ------------------------------------------------------------------------ */
/* 表32, header part */

static int parse_patch_header(bits_t *b, const svac3_sps_t *s, const svac3_pps_t *p,
                              svac3_patch_header_t *ph, FILE *dump)
{
    int align_ok = 1;

    memset(ph, 0, sizeof(*ph));
    F(ph->patch_index, "patch_index", 8);
    if (!p->fixed_picture_qp_flag) {
        F(ph->fixed_patch_qp_flag, "fixed_patch_qp_flag", 1);
        F(ph->patch_qp, "patch_qp", 7);
    }
    if (s->sao_enable_flag) {
        for (int c = 0; c < 3; c++)
            ph->patch_sao_enable_flag[c] = rd_u(b, 1);
        E("patch_sao_enable_flag=%u,%u,%u", ph->patch_sao_enable_flag[0],
          ph->patch_sao_enable_flag[1], ph->patch_sao_enable_flag[2]);
    }
    while (!b->err && (b->pos & 7))
        if (rd_u(b, 1) != 1)
            align_ok = 0;
    E("aec_byte_alignment=%s", align_ok ? "ok" : "BAD");
    ph->aec_data_offset = b->pos / 8;
    E("aec_data_offset=%zu", ph->aec_data_offset);
    return b->err ? b->err : (align_ok ? SVAC3_OK : SVAC3_ERR_INVALID);
}

/* ------------------------------------------------------------------------ */

static const char *nal_name(unsigned t)
{
    static const char *names[] = {
        "FORBIDDEN", "NRAP_PATCH", "RAPI_PATCH", "PIC_HEAD", "IDR_PATCH",
        "EXTENSION", "SEI", "SPS", "PPS", "SEC_PS", "AUTH_DATA", "EOS",
        "CRR_NDP_PATCH", NULL, "RL_PATCH", "PIC_DL", "EOCVS", "CRR_DP_PATCH",
        "CRR_DL_PATCH", "CRR_PRIVACY",
    };
    return t < sizeof(names) / sizeof(names[0]) && names[t] ? names[t] : "RESERVED";
}

int svac3_nal_is_patch(unsigned t)
{
    return t == 1 || t == 2 || t == 4 || t == 12 || t == 14 || t == 17 || t == 18 || t == 19;
}

int svac3_parse_nal(svac3_hls_t *ctx, const uint8_t *nal, size_t len,
                    unsigned idx, FILE *dump)
{
    svac3_nal_header_t nh;
    bits_t bits, *b = &bits;
    uint8_t *rbsp;
    size_t rbsp_len;
    int ret = SVAC3_OK;

    if ((ret = svac3_parse_nal_header(nal, len, &nh)))
        return ret;
    E("[nal %u %s]", idx, nal_name(nh.nal_unit_type));
    E("forbidden_zero_bit=%u", nh.forbidden_zero_bit);
    E("encryption_idc=%u", nh.encryption_idc);
    E("authentication_idc=%u", nh.authentication_idc);
    E("authentication_data_id=%u", nh.authentication_data_id);
    E("layer_id=%u", nh.layer_id);
    E("temporal_id=%u", nh.temporal_id);
    E("nal_unit_type=%u", nh.nal_unit_type);
    /* A parameter set or picture header invalidates everything that depends
     * on its predecessor before it is parsed, so a failure, a malformed
     * trailer or an encrypted payload never leaves stale state behind. */
    switch (nh.nal_unit_type) {
    case SVAC3_NAL_SPS:
        ctx->have_sps = 0;
        /* fall through */
    case SVAC3_NAL_PPS:
        ctx->have_pps = 0;
        /* fall through */
    case SVAC3_NAL_PIC_HEAD:
        ctx->have_ph = 0;
        ctx->have_patch = 0;
        break;
    default:
        if (svac3_nal_is_patch(nh.nal_unit_type))
            ctx->have_patch = 0;
        break;
    }
    if (nh.encryption_idc) {
        E("payload=%s", "encrypted (not parsed)");
        return SVAC3_OK;
    }

    rbsp = malloc(len - 2 ? len - 2 : 1);
    if (!rbsp)
        return SVAC3_ERR_INVALID;
    rbsp_len = svac3_unescape(nal + 2, len - 2, rbsp);
    bits_init(b, rbsp, rbsp_len);

    int *valid = NULL;
    switch (nh.nal_unit_type) {
    case SVAC3_NAL_SPS:
        ret = parse_sps(b, &ctx->sps, dump);
        valid = &ctx->have_sps;
        break;
    case SVAC3_NAL_PPS:
        if (!ctx->have_sps) {
            ret = SVAC3_ERR_INVALID;
            break;
        }
        ret = parse_pps(b, &ctx->sps, &ctx->pps, dump);
        valid = &ctx->have_pps;
        break;
    case SVAC3_NAL_PIC_HEAD:
        if (!ctx->have_sps || !ctx->have_pps) {
            ret = SVAC3_ERR_INVALID;
            break;
        }
        ret = parse_ph(b, &ctx->sps, &ctx->pps, &ctx->ph, dump);
        valid = &ctx->have_ph;
        break;
    default:
        if (svac3_nal_is_patch(nh.nal_unit_type)) {
            if (!ctx->have_ph) {
                ret = SVAC3_ERR_INVALID;
                break;
            }
            ret = parse_patch_header(b, &ctx->sps, &ctx->pps, &ctx->patch, dump);
            ctx->patch.nal_unit_type = nh.nal_unit_type;
            ctx->have_patch = ret == SVAC3_OK;
        } else {
            E("payload_bytes=%zu", rbsp_len);
        }
        free(rbsp);
        return ret;  /* no RBSP trailing check: AEC data or unparsed payload */
    }

    if (ret == SVAC3_OK) {
        int ok = trailing_ok(b);
        E("rbsp_trailing_bits=%s", ok ? "ok" : "BAD");
        if (!ok)
            ret = SVAC3_ERR_INVALID;
    }
    if (valid)
        *valid = ret == SVAC3_OK;
    free(rbsp);
    return ret;
}
