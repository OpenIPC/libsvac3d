/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright (c) 2026 OpenIPC contributors
 *
 * SVAC3 entropy decoder, transliterated from 5.3.3.2 (initialisation),
 * 5.3.3.3.3 (decode_decision, update_ctx) and Annex M (decode_bypass,
 * decode_aec_stuffing_bit).
 */
#include "aec_ref.h"

static const int cwr2LGS[10] = { 427, 427, 427, 197, 95, 46, 23, 12, 6, 3 };

static uint32_t read_bits(ref_aec_t *a, int n)
{
    uint32_t v = 0;
    for (int i = 0; i < n; i++) {
        int bit = 0;
        if (a->pos < a->bits)
            bit = (a->buf[a->pos >> 3] >> (7 - (a->pos & 7))) & 1;
        else
            a->overrun++;
        a->pos++;
        v = (v << 1) | (uint32_t)bit;
    }
    return v;
}

void ref_ctx_init(ref_ctx_t *ctx, size_t n)
{
    for (size_t i = 0; i < n; i++) {
        ctx[i].mps = 0;
        ctx[i].cycno = 0;
        ctx[i].lgPmps = 1023;
        ctx[i].lgPmps0 = 1023;
        ctx[i].lgPmps1 = 1023;
    }
}

void ref_aec_reinit(ref_aec_t *a)
{
    a->rS1 = 0;
    a->rT1 = 0xFF;
    a->valueS = 0;
    a->valueT = read_bits(a, 9);
    a->valueD = 1;
    a->bFlag = 0;
}

void ref_aec_init(ref_aec_t *a, const uint8_t *buf, size_t len, int maec,
                    int picture_coding_type)
{
    a->buf = buf;
    a->bits = len * 8;
    a->pos = 0;
    a->overrun = 0;
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
    ref_aec_reinit(a);
}

size_t ref_aec_bytes_read(const ref_aec_t *a)
{
    return (a->pos + 7) / 8;
}

/* The arithmetic part of decode_decision for a given MPS and an LPS range
 * already scaled to 8 bits. */
static int decide(ref_aec_t *a, int predMps, uint32_t lgPmps)
{
    uint32_t rS2, rT2, tRlps;
    int sFlag, binVal;

    if (a->valueD || (a->bFlag == 1 && a->rS1 == REF_AEC_BOUND_S)) {
        a->rS1 = 0;
        a->valueS = 0;
        while (a->valueT < 0x100 && a->valueS < REF_AEC_BOUND_S) {
            a->valueS++;
            a->valueT = (a->valueT << 1) | read_bits(a, 1);
        }
        a->bFlag = a->valueT < 0x100;
        a->valueT &= 0xFF;
    }
    if (a->rT1 >= lgPmps) {
        rS2 = a->rS1;
        rT2 = a->rT1 - lgPmps;
        sFlag = 0;
    } else {
        rS2 = a->rS1 + 1;
        rT2 = 256 + a->rT1 - lgPmps;
        sFlag = 1;
    }
    if ((rS2 > a->valueS || (rS2 == a->valueS && a->valueT >= rT2)) && a->bFlag == 0) {
        binVal = !predMps;
        tRlps = sFlag ? a->rT1 + lgPmps : lgPmps;
        if (rS2 == a->valueS)
            a->valueT = a->valueT - rT2;
        else
            a->valueT = 256 + ((a->valueT << 1) | read_bits(a, 1)) - rT2;
        while (tRlps < 0x100) {
            tRlps <<= 1;
            a->valueT = (a->valueT << 1) | read_bits(a, 1);
        }
        a->rT1 = tRlps & 0xFF;
        a->valueD = 1;
    } else {
        binVal = predMps;
        a->rS1 = rS2;
        a->rT1 = rT2;
        a->valueD = 0;
    }
    return binVal;
}

/* 5.3.3.3.3.5. A probability that passes 1023 is reflected (2047 - p) and
 * the MPS flips; the draft prints the reflection but not the flip
 * (docs/errata.md). With MAEC the MPS flips when either estimate reflects:
 * that is the only reading under which the corpus parses. */
static void update_ctx(ref_aec_t *a, ref_ctx_t *ctx, int binVal)
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
            int p0 = ctx->lgPmps0 + cwr2LGS[cwr];
            int p1 = longUpdate ? ctx->lgPmps1 + cwr2LGS[B] : p0;
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
            int p = ctx->lgPmps + cwr2LGS[cwr];
            if (p > 1023) {
                p = 2047 - p;
                ctx->mps = !ctx->mps;
            }
            ctx->lgPmps = (uint16_t)p;
        }
    }
}

static uint32_t ctx_lps(const ref_aec_t *a, const ref_ctx_t *ctx)
{
    /* probability of the LPS on the 10-bit scale */
    return a->maec ? (uint32_t)(ctx->lgPmps0 + ctx->lgPmps1 + 1) >> 1 : ctx->lgPmps;
}

int ref_aec_decision(ref_aec_t *a, ref_ctx_t *ctx)
{
    int bin = decide(a, ctx->mps, ctx_lps(a, ctx) >> 2);
    update_ctx(a, ctx, bin);
    return bin;
}

int ref_aec_decision_w(ref_aec_t *a, ref_ctx_t *ctx, ref_ctx_t *ctxW)
{
    const uint32_t p1 = ctx_lps(a, ctx), p2 = ctx_lps(a, ctxW);
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
    int bin = decide(a, predMps, lgPmps >> 2);
    update_ctx(a, ctx, bin);
    update_ctx(a, ctxW, bin);
    return bin;
}

/* Annex M */
int ref_aec_bypass(ref_aec_t *a)
{
    int binVal;

    if (a->valueD || (a->bFlag == 1 && a->rS1 == REF_AEC_BOUND_S)) {
        a->rS1 = 0;
        a->valueD = 1;
        a->valueT = (a->valueT << 1) | read_bits(a, 1);
        if (a->valueT >= 256 + a->rT1) {
            binVal = 1;
            a->valueT -= 256 + a->rT1;
        } else {
            binVal = 0;
        }
    } else {
        uint32_t rS2 = a->rS1 + 1;
        if ((rS2 > a->valueS || (rS2 == a->valueS && a->valueT >= a->rT1)) && a->bFlag == 0) {
            binVal = 1;
            if (rS2 == a->valueS)
                a->valueT = a->valueT - a->rT1;
            else
                a->valueT = 256 + ((a->valueT << 1) | read_bits(a, 1)) - a->rT1;
            a->rS1 = 0;
            a->valueD = 1;
        } else {
            binVal = 0;
            a->rS1 = rS2;
            a->valueD = 0;
        }
    }
    return binVal;
}

/* Annex M */
int ref_aec_stuffing_bit(ref_aec_t *a)
{
    uint32_t rS2, rT2;
    int binVal;

    if (a->valueD || (a->bFlag == 1 && a->rS1 == REF_AEC_BOUND_S)) {
        a->rS1 = 0;
        a->valueS = 0;
        while (a->valueT < 0x100 && a->valueS < REF_AEC_BOUND_S) {
            a->valueS++;
            a->valueT = (a->valueT << 1) | read_bits(a, 1);
        }
        a->bFlag = a->valueT < 0x100;
        a->valueT &= 0xFF;
    }
    if (a->rT1) {
        rS2 = a->rS1;
        rT2 = a->rT1 - 1;
    } else {
        rS2 = a->rS1 + 1;
        rT2 = 255;
    }
    if ((rS2 > a->valueS || (rS2 == a->valueS && a->valueT >= rT2)) && a->bFlag == 0) {
        binVal = 1;
        if (rS2 == a->valueS)
            a->valueT = a->valueT - rT2;
        else
            a->valueT = 256 + ((a->valueT << 1) | read_bits(a, 1)) - rT2;
        a->valueT = (a->valueT << 8) | read_bits(a, 8);
        a->rT1 = 0;
        a->valueD = 1;
    } else {
        binVal = 0;
        a->rS1 = rS2;
        a->rT1 = rT2;
        a->valueD = 0;
    }
    return binVal;
}
