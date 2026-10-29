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
 * decoding starts after the I picture, and a unit that reveals that an
 * earlier picture lost a patch. Such a unit is itself decoded, and may
 * complete a picture: *out is set alongside the error then. */
int svac3_dec_nal(svac3_dec_t *d, const uint8_t *nal, size_t len, const svac3_frame_t **out);

/* A picture decoded on a picture thread */
typedef struct {
    const svac3_frame_t *frame;  /* coded size; NULL when the picture failed */
    int width, height;           /* display size */
    int pict_type;               /* picture_coding_type */
    int64_t poi;
    int64_t tag;                 /* see svac3_dec_set_tag() */
    int err;                     /* 0, or why the picture failed */
} svac3_dec_pic_t;

typedef void (*svac3_dec_out_fn)(void *ctx, const svac3_dec_pic_t *pic);

/* Decode with this many threads (1, the default, decodes on the caller's).
 * Returns the number in use: fewer when threads cannot be started. With
 * two or more, pictures are decoded side by side, one per thread, and
 * svac3_dec_nal() never sets *out: each picture goes to the output
 * function instead, in decoding order, from a later call or from
 * svac3_dec_flush(), with any error its decoding met. Pictures still in
 * flight when the number changes are output first. */
int svac3_dec_set_threads(svac3_dec_t *d, int threads);

/* Where pictures decoded on picture threads go; frame is valid while the
 * function runs. */
void svac3_dec_set_output(svac3_dec_t *d, svac3_dec_out_fn fn, void *ctx);

/* Wait for every picture in flight and output it */
void svac3_dec_flush(svac3_dec_t *d);

/* Drop every decoded picture (after a seek), those in flight too;
 * parameter sets are kept */
void svac3_dec_reset(svac3_dec_t *d);

const svac3_hls_t *svac3_dec_hls(const svac3_dec_t *d);

/* POI of the last picture decoded (output, with picture threads) */
int64_t svac3_dec_poi(const svac3_dec_t *d);

/* A value each picture carries from the call that gave its first patch:
 * the one set when that call was made */
void svac3_dec_set_tag(svac3_dec_t *d, int64_t tag);

/* The tag of the last picture decoded (output, with picture threads) */
int64_t svac3_dec_tag(const svac3_dec_t *d);

/* Statistics of the last patch decoded (of the last picture output, with
 * picture threads) */
const svac3_patch_stats_t *svac3_dec_stats(const svac3_dec_t *d);

#endif /* SVAC3_DEC_H */
