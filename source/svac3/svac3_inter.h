/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright (c) 2026 OpenIPC contributors
 *
 * SVAC3 motion compensation for 8-bit 4:2:0: the 8-tap luma (5.4.9.2.6.2)
 * and 4-tap chroma (5.4.9.2.7.2, 表130) interpolation of normal (not affine,
 * not scaled) prediction. Reference samples outside the picture take the
 * value of the nearest sample inside it.
 */
#ifndef SVAC3_INTER_H
#define SVAC3_INTER_H

#include <stdint.h>

#include "svac3_recon.h"

/* Predict the w x h luma block at (x0, y0) from ref with the quarter-sample
 * motion vector (mvx, mvy). */
void svac3_mc_luma(const svac3_frame_t *ref, int x0, int y0, int w, int h,
                   int mvx, int mvy, uint8_t *pred);

/* Predict the w x h block of chroma plane c at chroma position (cx0, cy0)
 * with the luma motion vector (mvx, mvy), which is 1/8 sample in chroma. */
void svac3_mc_chroma(const svac3_frame_t *ref, int c, int cx0, int cy0, int w, int h,
                     int mvx, int mvy, uint8_t *pred);

#endif /* SVAC3_INTER_H */
