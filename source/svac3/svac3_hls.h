/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright (c) 2026 OpenIPC contributors
 *
 * SVAC3 high-level syntax: NAL units, sequence/picture parameter sets,
 * picture header and patch header.
 *
 * Section and table numbers refer to the SVAC 3.0 public-comment draft,
 * national standard plan 20243294-T-312 (revision of GB/T 25724-2017).
 */
#ifndef SVAC3_HLS_H
#define SVAC3_HLS_H

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

enum {
    SVAC3_OK              =  0,
    SVAC3_ERR_EOF         = -1, /* read past the end of an RBSP */
    SVAC3_ERR_UNSUPPORTED = -2, /* valid syntax this decoder does not handle yet */
    SVAC3_ERR_INVALID     = -3, /* value out of range, or a parameter set is missing */
    SVAC3_ERR_CORRUPT     = -4, /* patch data out of sync or truncated */
    SVAC3_ERR_NOMEM       = -5, /* allocation failed */
};

/* nal_unit_type, 表55 */
enum {
    SVAC3_NAL_NRAP_PATCH    = 1,
    SVAC3_NAL_RAPI_PATCH    = 2,
    SVAC3_NAL_PIC_HEAD      = 3,
    SVAC3_NAL_IDR_PATCH     = 4,
    SVAC3_NAL_EXTENSION     = 5,
    SVAC3_NAL_SEI           = 6,
    SVAC3_NAL_SPS           = 7,
    SVAC3_NAL_PPS           = 8,
    SVAC3_NAL_SEC_PS        = 9,
    SVAC3_NAL_AUTH_DATA     = 10,
    SVAC3_NAL_EOS           = 11,
    SVAC3_NAL_CRR_NDP_PATCH = 12,
    SVAC3_NAL_RL_PATCH      = 14,
    SVAC3_NAL_PIC_DL        = 15,
    SVAC3_NAL_EOCVS         = 16,
    SVAC3_NAL_CRR_DP_PATCH  = 17,
    SVAC3_NAL_CRR_DL_PATCH  = 18,
    SVAC3_NAL_CRR_PRIVACY   = 19,
};

#define SVAC3_MAX_RPLS     64
#define SVAC3_MAX_REFS     17

/* 表12 */
typedef struct {
    uint8_t forbidden_zero_bit;
    uint8_t encryption_idc;
    uint8_t authentication_idc;
    uint8_t authentication_data_id;
    uint8_t layer_id;
    uint8_t temporal_id;
    uint8_t nal_unit_type;
} svac3_nal_header_t;

/* 表14 */
typedef struct {
    uint8_t reference_to_library_enable_flag;
    uint8_t num_of_ref_pic;
    uint8_t library_index_flag[SVAC3_MAX_REFS];
    int32_t delta_doi[SVAC3_MAX_REFS];      /* or the library picture index */
} svac3_rpl_t;

/* 表13 */
typedef struct {
    uint8_t  profile_id, level_id;
    uint8_t  temporal_nesting_flag;
    uint8_t  library_picture_enable_flag, library_stream_flag;
    uint8_t  duplicate_sequence_header_flag, library_picture_mode_index;
    uint8_t  rapi_nalu_type_enable_flag;
    uint8_t  num_of_layers_minus1, ssvc_enable_flag;
    uint8_t  privacy_enable_flag, privacy_mode_index;
    uint8_t  reference_picture_resampling_enable_flag;
    uint8_t  rcm_enable_flag, rcm_mvp_enable_flag, rcm_mv_precision_type;
    uint16_t horizontal_size, vertical_size;
    uint16_t pic_width_in_luma, pic_height_in_luma;
    uint16_t rpr_pic_width_in_luma, rpr_pic_height_in_luma;
    uint8_t  chroma_format, sample_precision, encoding_precision;
    uint8_t  aspect_ratio, frame_rate_code;
    uint32_t bit_rate_lower, bit_rate_upper;
    uint8_t  low_delay, temporal_id_enable_flag;
    uint32_t bbv_buffer_size;
    uint8_t  max_dpb_size_minus1;
    uint8_t  rpl1_index_exist_flag, rpl1_same_as_rpl0_flag;
    uint32_t num_ref_pic_list_set[2];
    svac3_rpl_t rpls[2][SVAC3_MAX_RPLS];
    uint32_t num_ref_default_active_minus1[2];
    uint8_t  log2_lcu_size_minus2, log2_min_cu_size_minus2;
    uint8_t  log2_max_part_ratio_minus2, max_split_times_minus6;
    uint8_t  log2_min_qt_size_minus2, log2_max_bt_size_minus2, log2_max_eqt_size_minus3;
    uint8_t  weight_quant_enable_flag, load_seq_weight_quant_data_flag;
    uint8_t  wq_matrix4x4[16], wq_matrix8x8[64];
    uint8_t  st_enable_flag, sao_enable_flag, alf_enable_flag;
    uint8_t  affine_enable_flag, smvd_enable_flag, ipcm_enable_flag, amvr_enable_flag;
    uint8_t  num_of_hmvp_cand, umve_enable_flag, emvr_enable_flag;
    uint8_t  intra_pf_enable_flag, tscpm_enable_flag, dt_enable_flag, log2_max_dt_size_minus4;
    uint8_t  pbt_enable_flag, eipm_enable_flag, pmc_enable_flag;
    uint8_t  iip_enable_flag, sawp_enable_flag, mipf_enable_flag;
    uint8_t  asr_enable_flag, awp_enable_flag, tmvp_enable_flag;
    uint8_t  sbtmvp_enable_flag, etmvp_mvap_enable_flag;
    uint8_t  dmvr_enable_flag, bio_enable_flag, bgc_enable_flag;
    uint8_t  inter_pf_enable_flag, inter_pc_enable_flag, obmc_enable_flag;
    uint8_t  sbt_enable_flag, ist_enable_flag, ts_enable_flag;
    uint8_t  cu_dqp_enable_flag, srcc_enable_flag, maec_enable_flag;
    uint8_t  dbr_enable_flag, esao_enable_flag, ccsao_enable_flag, ealf_enable_flag;
    uint8_t  ibc_enable_flag, isc_enable_flag, num_of_intra_hmvp_cand;
    uint8_t  fimc_enable_flag, nn_tools_set_hook;
    uint32_t num_of_nn_filter_minus1;
    uint8_t  output_reorder_delay;
    uint8_t  cross_patch_loop_filter_enable_flag, ref_colocated_patch_flag;
    uint8_t  stable_patch_flag, uniform_patch_flag;
    uint32_t patch_width_minus1, patch_height_minus1;
} svac3_sps_t;

/* 表30 */
typedef struct {
    uint8_t fixed_picture_qp_flag;
    uint8_t cu_delta_qp_flag, cu_qp_group_size_log2_minus3;
    uint8_t deblocking_filter_disable_flag;
    uint8_t deblocking_filter_type, deblocking_filter_parameter_flag;
    uint8_t picture_weight_quant_enable_flag, picture_weight_quant_data_index;
    uint8_t weight_quant_param_index, weight_quant_model;
    int32_t weight_quant_param_delta[6];
    uint8_t wq_matrix4x4[16], wq_matrix8x8[64];
    uint8_t picture_ibc_enable_flag;
    uint8_t picture_isc_os_enable_flag, picture_isc_evs_ubvs_enable_flag;
    uint8_t picture_fimc_enable_flag;
    uint8_t affine_subblock_size_flag;
    uint8_t picture_awp_refine_index;
} svac3_pps_t;

/* 表31 */
typedef struct {
    uint8_t  decode_order_index;
    uint32_t library_picture_index;
    uint8_t  library_patch_num_minus1;
    uint8_t  picture_coding_type;            /* 1 = I, 2 = P, 3 = B */
    uint8_t  picture_rpr_flag, picture_tmvp_enable_flag;
    uint8_t  random_access_decodable_flag;
    uint32_t bbv_delay;
    uint8_t  time_code_flag;
    uint32_t time_code;
    uint32_t picture_output_delay, bbv_check_times;
    uint8_t  ref_pic_list_set_flag[2];
    uint32_t ref_pic_list_set_index[2];
    svac3_rpl_t rpl[2];                      /* explicit lists, when the flag is 0 */
    uint8_t  num_ref_active_override_flag;
    uint32_t num_ref_active_minus1[2];
    uint8_t  picture_qp;
    uint8_t  picture_privacy_enable_flag;
    int32_t  alpha_c_offset, beta_offset;
    uint8_t  chroma_quant_param_disable_flag;
    int32_t  chroma_quant_param_delta_cb, chroma_quant_param_delta_cr;
    uint8_t  picture_alf_enable_flag[3];
    uint8_t  picture_pmc_param_index, picture_umve_step_set_index;
    uint8_t  picture_obmc_blending_set_index, picture_inter_pc_skip_flag;
    uint8_t  picture_ts_enable_flag;
} svac3_ph_t;

/* 表32, header part */
typedef struct {
    uint8_t nal_unit_type;                   /* which kind of patch NAL unit carried it */
    uint8_t patch_index;
    uint8_t fixed_patch_qp_flag, patch_qp;
    uint8_t patch_sao_enable_flag[3];
    size_t  aec_data_offset;                 /* byte offset of the AEC data in the RBSP */
} svac3_patch_header_t;

/* Parser state. Each have_* flag is set only after its structure parsed
 * cleanly, and is cleared as soon as a NAL unit that replaces it, or one it
 * depends on, arrives: an SPS clears the PPS, picture header and patch
 * header; a PPS clears the picture and patch headers. */
typedef struct {
    svac3_sps_t sps;
    svac3_pps_t pps;
    svac3_ph_t  ph;
    svac3_patch_header_t patch;              /* last patch header parsed */
    int have_sps, have_pps, have_ph, have_patch;
} svac3_hls_t;

/* Annex-B: find the next NAL unit at or after *pos. Returns 1 with nal and
 * nal_len describing the unit (start code and trailing zero bytes
 * excluded) and *pos advanced past it, or 0 at the end of the buffer. */
int svac3_next_nal(const uint8_t *buf, size_t len, size_t *pos,
                   const uint8_t **nal, size_t *nal_len);

/* Remove emulation_prevention_three_byte (表12). dst must hold src_len
 * bytes; returns the RBSP length. */
size_t svac3_unescape(const uint8_t *src, size_t src_len, uint8_t *dst);

int svac3_parse_nal_header(const uint8_t *nal, size_t len, svac3_nal_header_t *h);

/* 1 if nal_unit_type carries patch data (表55): types 1, 2, 4, 12, 14, 17,
 * 18 and 19. */
int svac3_nal_is_patch(unsigned nal_unit_type);

/* Parse one NAL unit (without start code). When dump is not NULL, the fields
 * are written there as "key=value" lines in the format of
 * tools/svac3_parse.py. idx is the NAL's position in the stream, used only
 * for the dump. */
int svac3_parse_nal(svac3_hls_t *ctx, const uint8_t *nal, size_t len,
                    unsigned idx, FILE *dump);

#endif /* SVAC3_HLS_H */
