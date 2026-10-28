/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright (c) 2026 OpenIPC contributors
 *
 * SVAC3 entropy decoder (ae(v), 5.3.3), including the multi-hypothesis
 * probability models enabled by maec_enable_flag. Written for clarity
 * against the draft's pseudo-code, not for speed.
 */
#ifndef SVAC3_AEC_H
#define SVAC3_AEC_H

#include <stddef.h>
#include <stdint.h>

/* Binary symbol model, 5.3.3.2.1. Without MAEC only mps, cycno and lgPmps
 * are used; with MAEC, lgPmps0/lgPmps1 and the windows. */
typedef struct {
    uint8_t  mps;
    uint8_t  cycno;
    uint16_t lgPmps;
    uint16_t lgPmps0, lgPmps1;
} svac3_ctx_t;

typedef struct {
    /* bitstream */
    const uint8_t *buf;
    size_t bits, pos;
    size_t overrun;              /* bits read past the end (returned as 0) */

    /* engine state, 5.3.3.2.2 */
    uint32_t rS1, rT1, valueS, valueT;
    int bFlag, valueD;

    /* model parameters for the current picture */
    int maec;
    int counterThr1, counterThr2;
    int baseWin, diffWin0, diffWin1;
} svac3_aec_t;

#define SVAC3_AEC_BOUND_S 254

/* Start decoding at buf (the first byte after the patch header's
 * aec_byte_alignment_bit). picture_coding_type selects counterThr1/2. */
void svac3_aec_init(svac3_aec_t *a, const uint8_t *buf, size_t len, int maec,
                    int picture_coding_type);

/* 5.3.3.2.1: every model starts with mps = 0, cycno = 0 and the LPS
 * probability at its maximum. */
void svac3_ctx_init(svac3_ctx_t *ctx, size_t n);

int svac3_aec_decision(svac3_aec_t *a, svac3_ctx_t *ctx);
int svac3_aec_decision_w(svac3_aec_t *a, svac3_ctx_t *ctx, svac3_ctx_t *ctxW);
int svac3_aec_bypass(svac3_aec_t *a);
int svac3_aec_stuffing_bit(svac3_aec_t *a);

/* Re-initialise the engine (not the models) at the current byte position,
 * as after pcm_coeff (5.3.3.1). */
void svac3_aec_reinit(svac3_aec_t *a);

/* Bytes consumed so far, rounded up. */
size_t svac3_aec_bytes_read(const svac3_aec_t *a);

#endif /* SVAC3_AEC_H */
