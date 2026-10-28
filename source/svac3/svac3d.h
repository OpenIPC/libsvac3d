/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright (c) 2026 OpenIPC contributors
 *
 * libsvac3d: an SVAC 3.0 video decoder, written from the public-comment
 * draft of GB/T SVAC 3.0 (national standard plan 20243294-T-312).
 *
 * Supported: the Main profile (0x30) tool set HiSilicon encoders use,
 * 8-bit 4:2:0, I and P pictures, one patch per picture. Pictures come out
 * in decoding order, which for these low-delay streams is display order.
 * Anything else is refused with SVAC3D_ERR_UNSUPPORTED.
 *
 * This is the installed interface; the rest of source/svac3 is internal.
 */
#ifndef SVAC3D_H
#define SVAC3D_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SVAC3D_VERSION_MAJOR 0
#define SVAC3D_VERSION_MINOR 1
#define SVAC3D_VERSION_PATCH 0

/* Error codes; every call returns 0 or one of these. */
#define SVAC3D_ERR_EOF          (-1)  /* a NAL unit ends inside a syntax element */
#define SVAC3D_ERR_UNSUPPORTED  (-2)  /* valid syntax this decoder does not handle */
#define SVAC3D_ERR_INVALID      (-3)  /* value out of range, or a parameter set missing */
#define SVAC3D_ERR_CORRUPT      (-4)  /* patch data out of sync, or a reference missing */
#define SVAC3D_ERR_NOMEM        (-5)
#define SVAC3D_ERR_ARG          (-6)  /* bad argument */

typedef struct svac3d svac3d_t;

/* A decoded picture, cropped to the display size, 8-bit 4:2:0. The planes
 * belong to the decoder and stay valid only until the output callback
 * returns. */
typedef struct {
    const uint8_t *data[3];
    int linesize[3];
    int width, height;       /* luma; chroma is (width + 1) / 2 x (height + 1) / 2 */
    int pict_type;           /* 1 = I, 2 = P */
    int64_t poi;             /* picture output order index */
} svac3d_picture_t;

/* What the last sequence parameter set says. */
typedef struct {
    int profile_id, level_id;
    int width, height;       /* display size */
    int coded_width, coded_height;
    int frame_rate_num, frame_rate_den;   /* 0/0 when frame_rate_code is reserved */
} svac3d_info_t;

typedef void (*svac3d_output_fn)(void *opaque, const svac3d_picture_t *pic);

const char *svac3d_version(void);

svac3d_t *svac3d_open(void);
void svac3d_close(svac3d_t *d);

/* Forget every decoded picture, as after a seek; parameter sets are kept.
 * Decoding resumes at the next I picture. */
void svac3d_reset(svac3d_t *d);

/* Decode an Annex-B buffer (start codes, then NAL units) holding any whole
 * number of NAL units, typically one access unit. out is called once for
 * each picture completed, before the call returns. Decoding stops at the
 * first NAL unit in error, whose code is returned; the decoder stays usable
 * and later buffers decode normally once their references exist. */
int svac3d_decode(svac3d_t *d, const uint8_t *buf, size_t len, svac3d_output_fn out,
                  void *opaque);

/* 0 and *info filled once a sequence parameter set has been decoded,
 * SVAC3D_ERR_INVALID before. */
int svac3d_get_info(const svac3d_t *d, svac3d_info_t *info);

#ifdef __cplusplus
}
#endif

#endif /* SVAC3D_H */
