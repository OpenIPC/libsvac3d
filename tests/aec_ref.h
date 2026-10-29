/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright (c) 2026 OpenIPC contributors
 *
 * The SVAC3 entropy decoder as the draft writes it (5.3.3.2-3, Annex M),
 * kept for tests/test_aec.c to check the decoder's faster engine against.
 */
#ifndef REF_AEC_H
#define REF_AEC_H

#include <stddef.h>
#include <stdint.h>

/* Binary symbol model, 5.3.3.2.1. Without MAEC only mps, cycno and lgPmps
 * are used; with MAEC, lgPmps0/lgPmps1 and the windows. */
typedef struct {
    uint8_t  mps;
    uint8_t  cycno;
    uint16_t lgPmps;
    uint16_t lgPmps0, lgPmps1;
} ref_ctx_t;

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
} ref_aec_t;

#define REF_AEC_BOUND_S 254

/* Start decoding at buf (the first byte after the patch header's
 * aec_byte_alignment_bit). picture_coding_type selects counterThr1/2. */
void ref_aec_init(ref_aec_t *a, const uint8_t *buf, size_t len, int maec,
                    int picture_coding_type);

/* 5.3.3.2.1: every model starts with mps = 0, cycno = 0 and the LPS
 * probability at its maximum. */
void ref_ctx_init(ref_ctx_t *ctx, size_t n);

int ref_aec_decision(ref_aec_t *a, ref_ctx_t *ctx);
int ref_aec_decision_w(ref_aec_t *a, ref_ctx_t *ctx, ref_ctx_t *ctxW);
int ref_aec_bypass(ref_aec_t *a);
int ref_aec_stuffing_bit(ref_aec_t *a);

/* Re-initialise the engine (not the models) at the current byte position,
 * as after pcm_coeff (5.3.3.1). */
void ref_aec_reinit(ref_aec_t *a);

/* Bytes consumed so far, rounded up. */
size_t ref_aec_bytes_read(const ref_aec_t *a);

#endif /* REF_AEC_H */
