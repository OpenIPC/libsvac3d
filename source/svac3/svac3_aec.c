/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright (c) 2026 OpenIPC contributors
 *
 * SVAC3 entropy decoder: initialisation (5.3.3.2), the stuffing bit
 * (Annex M) and the bit accounting; the per-bin functions are in
 * svac3_aec.h.
 */
#include "svac3_aec.h"

void svac3_ctx_init(svac3_ctx_t *ctx, size_t n)
{
    for (size_t i = 0; i < n; i++) {
        ctx[i].mps = 0;
        ctx[i].cycno = 0;
        ctx[i].lgPmps = 1023;
        ctx[i].lgPmps0 = 1023;
        ctx[i].lgPmps1 = 1023;
    }
}

void svac3_aec_init(svac3_aec_t *a, const uint8_t *buf, size_t len, int maec,
                    int picture_coding_type)
{
    a->buf = buf;
    a->len = len;
    a->forced = 0;
    a->maec = maec;
    a->baseWin = 5;
    a->diffWin0 = 1;
    a->diffWin1 = 1;
    if (picture_coding_type == 1) {          /* I picture */
        a->counterThr1 = 0;
        a->counterThr2 = 8;
    } else {
        a->counterThr1 = 3;
        a->counterThr2 = 16;
    }
    /* rT1 = 0xFF and the first 9 bits as valueT: range 511, those bits at
     * the top of the window, 15 more below them and the marker */
    const uint32_t b0 = len > 0 ? buf[0] : 0, b1 = len > 1 ? buf[1] : 0, b2 = len > 2 ? buf[2] : 0;
    a->range = 511;
    a->low = (b0 << 18) | (b1 << 10) | (b2 << 2) | 2;
    a->pos = 3;
}

int svac3_aec_stuffing_bit(svac3_aec_t *a)
{
    /* a decision with an LPS range of 1 whose LPS is the 1 */
    return svac3_aec_decide(a, 0, 1);
}

size_t svac3_aec_bits_read(const svac3_aec_t *a)
{
    /* the window holds 16 - ctz(low) bits not yet consumed */
    return a->pos * 8 - 16 + (size_t)svac3_aec_ctz(a->low);
}

size_t svac3_aec_bytes_read(const svac3_aec_t *a)
{
    return (svac3_aec_bits_read(a) + 7) / 8;
}

size_t svac3_aec_overrun(const svac3_aec_t *a)
{
    const size_t bits = svac3_aec_bits_read(a), have = a->len * 8;
    return (bits > have ? bits - have : 0) + a->forced;
}
