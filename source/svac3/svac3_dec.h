/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright (c) 2026 OpenIPC contributors
 *
 * SVAC3 picture decoder: NAL units in, decoded pictures out. Keeps the
 * parameter sets, the decoded picture buffer (5.4.3.4) and the reference
 * picture lists (5.4.3.3) around svac3_patch_decode(). Low-delay streams
 * only (I and P pictures), so pictures come out in decoding order.
 */
#ifndef SVAC3_DEC_H
#define SVAC3_DEC_H

#include <stddef.h>
#include <stdint.h>

#include "svac3_hls.h"
#include "svac3_patch.h"

typedef struct svac3_dec svac3_dec_t;

svac3_dec_t *svac3_dec_create(void);
void svac3_dec_destroy(svac3_dec_t *d);

/* Decode one NAL unit (without its start code). When the unit completes a
 * picture, *out points at it, in the coded size (the display size is in
 * svac3_dec_hls()->sps), and stays valid until the next call; otherwise
 * *out is NULL. Errors are the SVAC3_ERR_* codes: SVAC3_ERR_CORRUPT also
 * covers a P picture whose reference is not in the buffer, as when
 * decoding starts after the I picture. */
int svac3_dec_nal(svac3_dec_t *d, const uint8_t *nal, size_t len, const svac3_frame_t **out);

/* Drop every decoded picture (after a seek); parameter sets are kept */
void svac3_dec_reset(svac3_dec_t *d);

const svac3_hls_t *svac3_dec_hls(const svac3_dec_t *d);

/* Statistics of the last patch decoded */
const svac3_patch_stats_t *svac3_dec_stats(const svac3_dec_t *d);

#endif /* SVAC3_DEC_H */
