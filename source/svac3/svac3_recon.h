/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright (c) 2026 OpenIPC contributors
 *
 * SVAC3 sample reconstruction for 8-bit 4:2:0: normal intra prediction
 * (5.4.8.1, with EIPM's 66 modes and MIPF), dequantisation (5.4.7.2) and
 * the DCT2 inverse transform (5.4.7.3.2).
 */
#ifndef SVAC3_RECON_H
#define SVAC3_RECON_H

#include <stdint.h>

typedef struct {
    uint8_t *plane[3];
    int stride[3];
    int width[3], height[3];     /* decoded (coded) size of each plane */
} svac3_frame_t;

/* Tells intra prediction whether a sample is available (5.4.6.4): inside
 * the picture, in the same patch, already decoded. Coordinates are luma. */
typedef int (*svac3_avail_fn)(void *opaque, int x, int y);

/* Normal intra prediction of an M x N block at (x0, y0) of plane c (0 = Y,
 * 1/2 = Cb/Cr, coordinates in that plane's samples). mode is
 * IntraLumaPredMode for luma; for chroma it is IntraChromaPredMode and
 * luma_mode the luma mode DM refers to. mipf is MipfEnableFlag. */
void svac3_intra_pred(const svac3_frame_t *f, int c, int x0, int y0, int M, int N,
                      int mode, int luma_mode, int mipf,
                      svac3_avail_fn avail, void *opaque, uint8_t *pred);

/* Dequantise and inverse-transform an M1 x M2 block of quantised
 * coefficients (row-major, [y * M1 + x]) into residuals. */
void svac3_dequant_itrans(const int16_t *quant, int M1, int M2, int qp, int16_t *resid);

/* QPcb / QPcr from CurrentQp and a chroma QP delta (5.4.6.2, 表103), 8 bit. */
int svac3_chroma_qp(int qp, int delta);

/* Add residuals to a prediction and write the block into the frame. */
void svac3_put_block(svac3_frame_t *f, int c, int x0, int y0, int M, int N,
                     const uint8_t *pred, const int16_t *resid);

#endif /* SVAC3_RECON_H */
