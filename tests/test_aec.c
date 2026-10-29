/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright (c) 2026 OpenIPC contributors
 *
 * The decoder's entropy engine against the draft's (tests/aec_ref.c), on
 * random bitstreams and random sequences of decisions, two-model
 * decisions, bypass bins and stuffing bits: every bin and every model must
 * agree, and so must the bits consumed after a stuffing bit of 1, where a
 * patch ends (elsewhere the draft's decoder has read ahead over zeros).
 * usage: test_aec [iterations]
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "aec_ref.h"
#include "svac3_aec.h"

/* decode_bypass as 5.3.3.3.3.4 defines it: decode_decision with a model of
 * lgPmps 1024 and mps 0, a fresh one each time. Annex M (informative)
 * gives a shortcut for it; the test uses both, alternately. */
static int ref_bypass_normative(ref_aec_t *a)
{
    ref_ctx_t half = { .mps = 0, .cycno = 0, .lgPmps = 1024, .lgPmps0 = 1024, .lgPmps1 = 1024 };
    return ref_aec_decision(a, &half);
}

static uint32_t seed = 4242;
static uint32_t rnd(void)
{
    seed = seed * 1664525u + 1013904223u;
    return seed >> 8;
}

int main(int argc, char **argv)
{
    const int iters = argc > 1 ? atoi(argv[1]) : 2000;
    enum { LEN = 4096, NCTX = 24, OPS = 20000 };
    static uint8_t buf[LEN];
    int failures = 0;
    long bins = 0, ends = 0;

    for (int it = 0; it < iters && failures < 10; it++) {
        /* random bytes, with runs of zeros (up to 30 bytes, under the
         * draft's 254-bit bound) and of ones */
        for (int i = 0; i < LEN;) {
            const int kind = (int)(rnd() % 16);
            const int run = 1 + (int)(rnd() % 30);
            for (int k = 0; k < run && i < LEN; k++, i++)
                buf[i] = (uint8_t)(kind == 0 ? 0 : kind == 1 ? 0xFF : rnd());
        }
        /* AEC data an encoder can write starts with a value below the
         * initial range of 511: not nine 1 bits. (Past that, arithmetic
         * decoding keeps the value below the range; checked below.) */
        if (buf[0] == 0xFF && (buf[1] & 0x80))
            buf[1] &= 0x7F;
        const size_t len = 64 + rnd() % (LEN - 64);
        const int maec = (int)(rnd() & 1), pct = 1 + (int)(rnd() & 1);

        ref_aec_t ra;
        svac3_aec_t fa;
        ref_ctx_t rc[NCTX];
        svac3_ctx_t fc[NCTX];
        ref_aec_init(&ra, buf, len, maec, pct);
        svac3_aec_init(&fa, buf, len, maec, pct);
        ref_ctx_init(rc, NCTX);
        svac3_ctx_init(fc, NCTX);

        for (int op = 0; op < OPS; op++) {
            if ((fa.low >> 17) >= fa.range) {
                fprintf(stderr, "iteration %d op %d: value %u not below range %u\n", it, op,
                        fa.low >> 17, fa.range);
                failures++;
                break;
            }
            const int kind = (int)(rnd() % 64) < 63 ? (int)(rnd() % 15) : 15, c = (int)(rnd() % NCTX), w = (int)(rnd() % NCTX);
            int b1, b2, stuffed = 0;
            if (kind < 10) {
                b1 = ref_aec_decision(&ra, &rc[c]);
                b2 = svac3_aec_decision(&fa, &fc[c]);
            } else if (kind < 12 && w != c) {
                b1 = ref_aec_decision_w(&ra, &rc[c], &rc[w]);
                b2 = svac3_aec_decision_w(&fa, &fc[c], &fc[w]);
            } else if (kind < 15) {
                b1 = op & 1 ? ref_bypass_normative(&ra) : ref_aec_bypass(&ra);
                b2 = svac3_aec_bypass(&fa);
            } else if (op > 30) {
                /* between LCUs: rarely, so that patches run long */
                b1 = ref_aec_stuffing_bit(&ra);
                b2 = svac3_aec_stuffing_bit(&fa);
                stuffed = 1;
            } else {
                b1 = ref_bypass_normative(&ra);
                b2 = svac3_aec_bypass(&fa);
            }
            bins++;
            ends += stuffed && b1 == 1;
            /* every model, the second of a two-model decision included */
            int ctx_same = 1;
            for (int m = 0; m < NCTX; m++)
                ctx_same &= rc[m].mps == fc[m].mps && rc[m].cycno == fc[m].cycno &&
                            rc[m].lgPmps == fc[m].lgPmps && rc[m].lgPmps0 == fc[m].lgPmps0 &&
                            rc[m].lgPmps1 == fc[m].lgPmps1;
            const int end = stuffed && b1 == 1;
            if (b1 != b2 || !ctx_same || (end && ra.pos != svac3_aec_bits_read(&fa))) {
                fprintf(stderr, "iteration %d op %d (kind %d): bin %d/%d, bits %zu/%zu, model %s\n",
                        it, op, kind, b1, b2, ra.pos, svac3_aec_bits_read(&fa),
                        ctx_same ? "same" : "differs");
                failures++;
                break;
            }
            if (end)
                break;                      /* a stuffing bit of 1 ends a patch */
            if (ra.pos > len * 8 + 256)
                break;                      /* well past the end: both read zeros */
        }
    }
    if (failures)
        fprintf(stderr, "%d mismatch(es)\n", failures);
    else
        printf("%ld bins in %d streams (%ld patch ends) agree with the draft's engine\n", bins, iters, ends);
    return failures ? 1 : 0;
}
