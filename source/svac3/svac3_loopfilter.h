/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright (c) 2026 OpenIPC contributors
 *
 * SVAC3 in-loop filters for 8-bit 4:2:0: deblocking (5.4.11) and basic
 * sample adaptive offset (5.4.12.1).
 */
#ifndef SVAC3_LOOPFILTER_H
#define SVAC3_LOOPFILTER_H

#include <stdint.h>

#include "svac3_recon.h"
#include "svac3_pool.h"

/* What the deblocking filter needs per 4x4 luma unit. */
typedef struct {
    int pic_w, pic_h;            /* luma */
    int stride;                  /* units per row */
    const uint32_t *cu_id;       /* coding unit identity: edges lie between ids */
    const uint8_t *qp;           /* QPY */
    int alpha_offset, beta_offset;
    int filter_type;             /* DeblockingFilterType */
    int cb_delta, cr_delta;      /* chroma_quant_param_delta_cb/cr */
    /* inter pictures only (inter == 0 treats every unit as intra) */
    int inter;
    const uint8_t *cbf;          /* the coding unit has coefficients */
    const int8_t *ref;           /* L0 reference index, -1 when intra */
    const int16_t *mv;           /* L0 motion vector, two per unit */
} svac3_dbk_info_t;

/* All vertical edges of the picture are filtered before any horizontal
 * edge. 5.4.11.1 as printed filters each 8x8 block's vertical then
 * horizontal edge in raster order; only the picture-wide order reproduces
 * the reference decoder (docs/errata.md). */
void svac3_deblock(svac3_frame_t *f, const svac3_dbk_info_t *d, svac3_pool_t *pool);

/* Basic SAO parameters of one component of one LCU (5.4.12.1.3). */
typedef struct {
    uint8_t mode;                /* 0 off, 1 interval, 2 edge */
    int8_t offset[4];
    uint8_t start_pos, delta_pos_minus2;   /* interval mode */
    uint8_t edge_type;           /* edge mode: 0 horizontal, 1 vertical, 2 135 degrees, 3 45 degrees */
} svac3_sao_t;

/* Apply SAO to the deblocked frame. sao is [lcu][component]. Returns 0, or
 * -1 if a working copy of a plane could not be allocated (frame untouched
 * from that plane on). */
int svac3_sao(svac3_frame_t *f, const svac3_sao_t (*sao)[3], int lcu_size,
               int lcu_w, int lcu_h, svac3_pool_t *pool);

#endif /* SVAC3_LOOPFILTER_H */
