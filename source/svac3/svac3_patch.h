/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright (c) 2026 OpenIPC contributors
 *
 * SVAC3 patch data parser (表32-36): SAO parameters, the coding tree, coding
 * units and transform blocks, decoded with the entropy decoder in
 * svac3_aec.c, and reconstructs I and P pictures.
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

/* Parse the patch whose header is hls->patch, of any patch_index (a
 * neighbour in another patch is unavailable, so a patch parses alone).
 * data/len is the patch RBSP
 * (after emulation prevention removal); the AEC data starts at
 * hls->patch.aec_data_offset. */
int svac3_patch_parse(const svac3_hls_t *hls, const uint8_t *data, size_t len,
                      svac3_patch_stats_t *stats);

/* Temporal motion of a decoded picture, one per 16x16 luma unit in raster
 * order ((W + 15) / 16 units per row), 5.4.6.7.10.3. */
typedef struct {
    int16_t mv[2];           /* L0 motion vector, quarter samples */
    int8_t ref;              /* L0 reference index, -1 for 'PRED_I' */
    int32_t dist;            /* BlockDistance of that reference from the picture */
} svac3_tmv_t;

/* What a P picture is predicted from. */
typedef struct {
    int num;                                  /* pictures in reference list 0 */
    const svac3_frame_t *ref[SVAC3_MAX_REFS];
    int dist[SVAC3_MAX_REFS];                 /* BlockDistance: DistanceIndex of the
                                                 current picture minus the reference's */
    const svac3_tmv_t *col;                   /* temporal motion of ref[0] */
} svac3_refs_t;

/* Number of svac3_tmv_t a picture of this size stores. */
size_t svac3_tmv_count(int pic_w, int pic_h);

/* For a picture that is a single patch: as svac3_patch_parse(), and also
 * reconstruct it into frame,
 * including deblocking and SAO. frame must hold the coded picture in 4:2:0
 * (SVAC3_ERR_INVALID otherwise). A P picture needs refs, with at least
 * NumRefActive[0] pictures of the same size, each with its temporal motion
 * if TMVP is on; refs is ignored for an I picture. When tmv is not NULL it
 * receives the picture's temporal motion (svac3_tmv_count() entries), for
 * use as a later picture's refs->col. Unlike the parse-only call, a patch
 * that does not end exactly on its last LCU is an error, SVAC3_ERR_CORRUPT,
 * and an allocation failure is SVAC3_ERR_NOMEM. */
int svac3_patch_decode(const svac3_hls_t *hls, const uint8_t *data, size_t len,
                       svac3_frame_t *frame, const svac3_refs_t *refs, svac3_tmv_t *tmv,
                       svac3_patch_stats_t *stats);

/* A picture of several patches, decoded one patch at a time:
 * svac3_picture_begin() with the parameter sets and picture header in hls
 * (and frame, refs and tmv as for svac3_patch_decode()), then
 * svac3_picture_patch() for each patch in patch_index order, then
 * svac3_picture_end(), which runs the loop filters over the whole picture,
 * stores its temporal motion and frees it. hls must stay the same object
 * throughout, with only its patch header changing. */
typedef struct svac3_picture svac3_picture_t;

/* Number of patches in a picture with this SPS (uniform patches), or -1 */
int svac3_picture_patches(const svac3_hls_t *hls);

int svac3_picture_begin(const svac3_hls_t *hls, svac3_frame_t *frame, const svac3_refs_t *refs,
                        svac3_tmv_t *tmv, svac3_picture_t **pic);

/* Decode the patch whose header is hls->patch. Returns 1 when it was the
 * picture's last patch, 0 when more are due, SVAC3_ERR_CORRUPT for a patch
 * out of order or not ending on its last LCU. */
int svac3_picture_patch(svac3_picture_t *pic, const svac3_hls_t *hls, const uint8_t *data,
                        size_t len, svac3_patch_stats_t *stats);

/* Finish and free the picture: SVAC3_ERR_CORRUPT if patches are missing. */
int svac3_picture_end(svac3_picture_t *pic, const svac3_hls_t *hls);

/* Free a picture without finishing it. */
void svac3_picture_free(svac3_picture_t *pic);

#endif /* SVAC3_PATCH_H */
