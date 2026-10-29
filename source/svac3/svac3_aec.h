/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright (c) 2026 OpenIPC contributors
 *
 * SVAC3 entropy decoder (ae(v), 5.3.3), including the multi-hypothesis
 * probability models enabled by maec_enable_flag.
 *
 * The draft keeps the interval as a shift count and an 8-bit remainder
 * (rS1/rT1, valueS/valueT, 5.3.3.2.2). The same arithmetic is done here as
 * a 9-bit range and a 32-bit window of the bitstream, refilled 16 bits at
 * a time, the form AVS3 decoders use: one comparison per bin and a
 * leading-zero count to renormalise. The two give the same bins for every
 * stream with fewer than 254 consecutive zero bits in its AEC data, which
 * emulation prevention guarantees; tests/test_aec.c checks them against
 * each other. The per-bin functions are inline: the patch parser calls
 * them for every bin.
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
    size_t len;                  /* bytes */
    size_t pos;                  /* bytes loaded into low */
    uint32_t forced;             /* overrun bits added by svac3_aec_mark_corrupt() */

    /* engine: range in 256..511, low the bitstream window with the
     * fraction below bit 17 and a marker bit under the unread bits */
    uint32_t range, low;

    /* model parameters for the current picture */
    int maec;
    int counterThr1, counterThr2;
    int baseWin, diffWin0, diffWin1;
} svac3_aec_t;

/* Start decoding at buf (the first byte after the patch header's
 * aec_byte_alignment_bit). picture_coding_type selects counterThr1/2. */
void svac3_aec_init(svac3_aec_t *a, const uint8_t *buf, size_t len, int maec,
                    int picture_coding_type);

/* 5.3.3.2.1: every model starts with mps = 0, cycno = 0 and the LPS
 * probability at its maximum. */
void svac3_ctx_init(svac3_ctx_t *ctx, size_t n);

/* decode_aec_stuffing_bit (Annex M) */
int svac3_aec_stuffing_bit(svac3_aec_t *a);

/* Bits the engine has consumed so far, as the draft's decoder counts them
 * (the 9 it starts with, then one per renormalising shift). */
size_t svac3_aec_bits_read(const svac3_aec_t *a);

/* Bytes consumed so far, rounded up. */
size_t svac3_aec_bytes_read(const svac3_aec_t *a);

/* Bits consumed past the end of the data (read as 0), plus any added by
 * svac3_aec_mark_corrupt(). */
size_t svac3_aec_overrun(const svac3_aec_t *a);

/* Record that the parser met something impossible: overrun becomes > 0. */
static inline void svac3_aec_mark_corrupt(svac3_aec_t *a)
{
    a->forced++;
}

/* ------------------------------------------------------------------------ */
/* Per-bin decoding, inline */

static const int svac3_cwr2LGS[10] = { 427, 427, 427, 197, 95, 46, 23, 12, 6, 3 };

/* shifts that bring v (1 <= v < 512) to 256..511 */
static inline int svac3_aec_lead9(uint32_t v)
{
#if defined(__GNUC__) || defined(__clang__)
    return __builtin_clz(v) - 23;
#else
    int n = 0;
    while (v < 0x100) {
        v <<= 1;
        n++;
    }
    return n;
#endif
}

static inline int svac3_aec_ctz(uint32_t v)
{
#if defined(__GNUC__) || defined(__clang__)
    return __builtin_ctz(v);
#else
    int n = 0;
    while (!(v & 1)) {
        v >>= 1;
        n++;
    }
    return n;
#endif
}

/* The window has run out of unread bits (the marker reached bit 16):
 * append the next 16, bytes past the end reading as 0. */
static inline uint32_t svac3_aec_refill(svac3_aec_t *a, uint32_t low)
{
    const int i = svac3_aec_ctz(low) - 16;  /* the marker's distance past bit 16 */
    const uint32_t b0 = a->pos < a->len ? a->buf[a->pos] : 0;
    const uint32_t b1 = a->pos + 1 < a->len ? a->buf[a->pos + 1] : 0;
    a->pos += 2;
    return low + ((((b0 << 9) | (b1 << 1)) - 0xFFFFu) << i);
}

/* The arithmetic of decode_decision (5.3.3.3.3.2) for an MPS and an LPS
 * range already scaled to 8 bits */
static inline int svac3_aec_decide(svac3_aec_t *a, int predMps, uint32_t lps)
{
    uint32_t rMPS = a->range - lps;
    const int s_flag = rMPS < 256;              /* the draft's sFlag */
    rMPS |= 0x100;
    const uint32_t scaled = rMPS << (17 - s_flag);
    const uint32_t r = (a->range << s_flag) - rMPS;  /* the LPS range */
    int bin = predMps;
    if (a->low >= scaled) {                     /* the LPS */
        a->low -= scaled;
        a->range = r;
        bin = !predMps;
    } else {
        a->range = rMPS;
    }
    const int n = svac3_aec_lead9(a->range);
    a->range <<= n;
    a->low <<= n + s_flag;
    if (!(a->low & 0xFFFF))
        a->low = svac3_aec_refill(a, a->low);
    return bin;
}

/* 5.3.3.3.3.5. A probability that passes 1023 is reflected (2047 - p) and
 * the MPS flips; the draft prints the reflection but not the flip
 * (docs/errata.md). With MAEC the MPS flips when either estimate reflects:
 * that is the only reading under which the corpus parses. */
static inline void svac3_aec_update(svac3_aec_t *a, svac3_ctx_t *ctx, int binVal)
{
    if (a->maec) {
        const int A = a->baseWin - a->diffWin0;
        const int B = a->baseWin + a->diffWin1;
        int cwr = A;

        if (ctx->cycno < a->counterThr1)
            cwr = A - 1;
        else if (ctx->cycno < a->counterThr2)
            cwr = A;
        const int longUpdate = ctx->cycno < a->counterThr2 ? 0 : 1;
        if (binVal != ctx->mps) {
            if (ctx->cycno < a->counterThr2)
                ctx->cycno++;
            else
                ctx->cycno = (uint8_t)a->counterThr2;
        } else if (ctx->cycno == 0) {
            ctx->cycno = 1;
        }
        if (binVal == ctx->mps) {
            ctx->lgPmps0 = ctx->lgPmps0 - (ctx->lgPmps0 >> cwr) - (ctx->lgPmps0 >> (cwr + 2));
            ctx->lgPmps1 = longUpdate
                ? ctx->lgPmps1 - (ctx->lgPmps1 >> B) - (ctx->lgPmps1 >> (B + 2))
                : ctx->lgPmps0;
        } else {
            int p0 = ctx->lgPmps0 + svac3_cwr2LGS[cwr];
            int p1 = longUpdate ? ctx->lgPmps1 + svac3_cwr2LGS[B] : p0;
            int flip = 0;
            if (p0 > 1023) {
                p0 = 2047 - p0;
                flip = 1;
            }
            if (p1 > 1023) {
                p1 = 2047 - p1;
                flip = 1;
            }
            ctx->lgPmps0 = (uint16_t)p0;
            ctx->lgPmps1 = (uint16_t)p1;
            if (flip)
                ctx->mps = !ctx->mps;
        }
    } else {
        int cwr = ctx->cycno <= 1 ? 3 : ctx->cycno == 2 ? 4 : 5;
        if (binVal != ctx->mps) {
            if (ctx->cycno < 2)
                ctx->cycno++;
            else
                ctx->cycno = 3;
        } else if (ctx->cycno == 0) {
            ctx->cycno = 1;
        }
        if (binVal == ctx->mps) {
            ctx->lgPmps = ctx->lgPmps - (ctx->lgPmps >> cwr) - (ctx->lgPmps >> (cwr + 2));
        } else {
            int p = ctx->lgPmps + svac3_cwr2LGS[cwr];
            if (p > 1023) {
                p = 2047 - p;
                ctx->mps = !ctx->mps;
            }
            ctx->lgPmps = (uint16_t)p;
        }
    }
}

/* probability of the LPS on the 10-bit scale */
static inline uint32_t svac3_aec_lps(const svac3_aec_t *a, const svac3_ctx_t *ctx)
{
    return a->maec ? (uint32_t)(ctx->lgPmps0 + ctx->lgPmps1 + 1) >> 1 : ctx->lgPmps;
}

static inline int svac3_aec_decision(svac3_aec_t *a, svac3_ctx_t *ctx)
{
    const int bin = svac3_aec_decide(a, ctx->mps, svac3_aec_lps(a, ctx) >> 2);
    svac3_aec_update(a, ctx, bin);
    return bin;
}

static inline int svac3_aec_decision_w(svac3_aec_t *a, svac3_ctx_t *ctx, svac3_ctx_t *ctxW)
{
    const uint32_t p1 = svac3_aec_lps(a, ctx), p2 = svac3_aec_lps(a, ctxW);
    uint32_t lgPmps;
    int predMps;

    if (ctx->mps == ctxW->mps) {
        predMps = ctx->mps;
        lgPmps = (p1 + p2) >> 1;
    } else if (p1 < p2) {
        predMps = ctx->mps;
        lgPmps = 1023 - ((p2 - p1) >> 1);
    } else {
        predMps = ctxW->mps;
        lgPmps = 1023 - ((p1 - p2) >> 1);
    }
    const int bin = svac3_aec_decide(a, predMps, lgPmps >> 2);
    svac3_aec_update(a, ctx, bin);
    svac3_aec_update(a, ctxW, bin);
    return bin;
}

/* decode_bypass (Annex M): an even split, one bit */
static inline int svac3_aec_bypass(svac3_aec_t *a)
{
    const uint32_t scaled = a->range << 16;
    int bin = 0;
    if (a->low >= scaled) {
        a->low -= scaled;
        bin = 1;
    }
    a->low <<= 1;
    if (!(a->low & 0xFFFF))
        a->low = svac3_aec_refill(a, a->low);
    return bin;
}

#endif /* SVAC3_AEC_H */
