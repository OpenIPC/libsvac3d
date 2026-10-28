/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright (c) 2026 OpenIPC contributors
 *
 * SVAC3 patch data parser (表32-36): SAO parameters, the coding tree, coding
 * units and transform blocks, decoded with the entropy decoder in
 * svac3_aec.c. It parses only; reconstruction comes later.
 *
 * Supported tool set: the one HiSilicon Main-profile encoders use (MAEC,
 * SRCC, EIPM, MIPF, TMVP, SAO, deblocking, CU delta QP). A stream that
 * enables anything else is refused by svac3_patch_supported().
 */
#ifndef SVAC3_PATCH_H
#define SVAC3_PATCH_H

#include "svac3_aec.h"
#include "svac3_hls.h"
#include "svac3_recon.h"

#define SVAC3_NUM_CTX 445

typedef struct {
    unsigned lcus;           /* LCUs parsed */
    unsigned cus, intra_cus, skip_cus, direct_cus, chroma_cus;
    unsigned coded_blocks, coeffs;
    int stuffing_ok;         /* aec_lcu_stuffing_bit was 0 before the last LCU and 1 at it */
    unsigned first_bad_lcu;  /* first LCU whose stuffing bit was wrong, if any */
    size_t bytes_used, bytes_avail;
    size_t overrun_bits;     /* bits the decoder had to read past the end */
} svac3_patch_stats_t;

/* Returns SVAC3_OK if the SPS/PPS/PH use only tools this parser handles. */
int svac3_patch_supported(const svac3_hls_t *hls);

/* Parse the patch whose header is hls->patch. data/len is the patch RBSP
 * (after emulation prevention removal); the AEC data starts at
 * hls->patch.aec_data_offset. */
int svac3_patch_parse(const svac3_hls_t *hls, const uint8_t *data, size_t len,
                      svac3_patch_stats_t *stats);

/* As svac3_patch_parse(), and also reconstruct the patch into frame,
 * including deblocking and SAO. Only I pictures are reconstructed so far;
 * for other pictures frame is left untouched. */
int svac3_patch_decode(const svac3_hls_t *hls, const uint8_t *data, size_t len,
                       svac3_frame_t *frame, svac3_patch_stats_t *stats);

#endif /* SVAC3_PATCH_H */
