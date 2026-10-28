/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright (c) 2026 OpenIPC contributors
 *
 * Unit tests for the SVAC3 high-level syntax parser.
 * usage: test_hls <stream.svac3>   (a Main-profile stream from tests/streams)
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "svac3_hls.h"

static int failures;

#define CHECK(cond)                                                       \
    do {                                                                  \
        if (!(cond)) {                                                    \
            fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__,        \
                    __LINE__, #cond);                                     \
            failures++;                                                   \
        }                                                                 \
    } while (0)

/* ---- bit writer producing an escaped NAL unit ---- */

typedef struct {
    uint8_t rbsp[4096];
    size_t bits;
} bw_t;

static void wu(bw_t *w, int n, uint32_t v)
{
    for (int i = n - 1; i >= 0; i--, w->bits++)
        if ((v >> i) & 1)
            w->rbsp[w->bits >> 3] |= 0x80 >> (w->bits & 7);
}

static void wue(bw_t *w, uint32_t v)
{
    uint64_t x = (uint64_t)v + 1;
    int len = 0;
    while ((x >> len) > 1)
        len++;
    wu(w, len, 0);
    for (int i = len; i >= 0; i--)
        wu(w, 1, (uint32_t)((x >> i) & 1));
}

/* rbsp_trailing_bits, the 2-byte NAL header and emulation prevention */
static size_t bw_nal(bw_t *w, uint8_t type, uint8_t *out)
{
    size_t n = 0, len;
    int zeros = 0;

    wu(w, 1, 1);
    while (w->bits & 7)
        wu(w, 1, 0);
    len = w->bits / 8;
    out[n++] = 0x00;
    out[n++] = type;
    for (size_t i = 0; i < len; i++) {
        if (zeros >= 2 && w->rbsp[i] <= 3) {
            out[n++] = 3;
            zeros = 0;
        }
        out[n++] = w->rbsp[i];
        zeros = w->rbsp[i] == 0 ? zeros + 1 : 0;
    }
    return n;
}

/* A Main-profile (0x30) 64x64 SPS with one RPL set of one reference.
 * wq_coeff > 0 enables a sequence weight-quant matrix filled with it. */
static size_t make_sps(uint8_t *out, uint32_t abs_delta_doi, uint32_t wq_coeff)
{
    bw_t w;
    memset(&w, 0, sizeof(w));
    wu(&w, 8, 0x30); wu(&w, 8, 0x50);
    wu(&w, 1, 0); wu(&w, 1, 0); wu(&w, 1, 0); wu(&w, 1, 0);     /* nesting, library x2, rapi */
    wu(&w, 14, 64); wu(&w, 14, 64); wu(&w, 14, 64); wu(&w, 14, 64);
    wu(&w, 2, 1); wu(&w, 3, 1); wu(&w, 4, 1); wu(&w, 4, 3);     /* 4:2:0, 8 bit, aspect, fps */
    wu(&w, 18, 1); wu(&w, 12, 0);                                /* bit rate */
    wu(&w, 1, 0); wu(&w, 18, 1); wu(&w, 4, 3);                   /* temporal id, bbv, max_dpb_size_minus1 */
    wu(&w, 1, 0); wu(&w, 1, 1);                                  /* rpl1_index_exist, rpl1_same_as_rpl0 */
    wue(&w, 1);                                                  /* num_ref_pic_list_set[0] */
    wue(&w, 1);                                                  /* num_of_ref_pic */
    wue(&w, abs_delta_doi);
    if (abs_delta_doi > 0)
        wu(&w, 1, 0);                                            /* sign_delta_doi */
    wue(&w, 0); wue(&w, 0);                                      /* num_ref_default_active_minus1 */
    wu(&w, 3, 3); wu(&w, 2, 0); wu(&w, 2, 1); wu(&w, 3, 0);
    wu(&w, 3, 0); wu(&w, 3, 3); wu(&w, 2, 0);                    /* LCU / partition sizes */
    wu(&w, 1, wq_coeff > 0);
    if (wq_coeff > 0) {
        wu(&w, 1, 1);                                            /* load_seq_weight_quant_data_flag */
        for (int i = 0; i < 80; i++)
            wue(&w, wq_coeff);
    }
    wu(&w, 7, 0);                                                /* st .. amvr */
    wu(&w, 4, 0);                                                /* num_of_hmvp_cand */
    wu(&w, 1, 0);                                                /* umve */
    wu(&w, 4, 0);                                                /* intra_pf, tscpm, dt, pbt */
    wu(&w, 4, 0);                                                /* pmc, iip, sawp, mipf */
    wu(&w, 2, 0);                                                /* awp, tmvp */
    wu(&w, 8, 0);                                                /* dmvr .. ist */
    wu(&w, 6, 0);                                                /* cu_dqp .. ccsao */
    wu(&w, 3, 0);                                                /* cross patch, colocated, stable */
    wu(&w, 2, 0);                                                /* reserved_bits */
    return bw_nal(&w, SVAC3_NAL_SPS, out);
}

static int parse(svac3_hls_t *ctx, const uint8_t *nal, size_t len)
{
    return svac3_parse_nal(ctx, nal, len, 0, NULL);
}

/* ---- tests on synthetic SPS ---- */

static void test_rpl_and_wq_ranges(void)
{
    uint8_t nal[8192];
    svac3_hls_t ctx;

    memset(&ctx, 0, sizeof(ctx));
    CHECK(parse(&ctx, nal, make_sps(nal, 1, 0)) == SVAC3_OK && ctx.have_sps);
    CHECK(ctx.sps.rpls[0][0].delta_doi[0] == 1);

    /* abs_delta_doi is 0..2^8-1 */
    CHECK(parse(&ctx, nal, make_sps(nal, 255, 0)) == SVAC3_OK);
    CHECK(ctx.sps.rpls[0][0].delta_doi[0] == 255);
    CHECK(parse(&ctx, nal, make_sps(nal, 256, 0)) == SVAC3_ERR_INVALID && !ctx.have_sps);
    /* 2^31 must be rejected before any signed conversion */
    CHECK(parse(&ctx, nal, make_sps(nal, 0x80000000u, 0)) == SVAC3_ERR_INVALID);

    /* weight_quant_coeff is 1..255 */
    CHECK(parse(&ctx, nal, make_sps(nal, 1, 255)) == SVAC3_OK);
    CHECK(ctx.sps.wq_matrix4x4[0] == 255 && ctx.sps.wq_matrix8x8[63] == 255);
    CHECK(parse(&ctx, nal, make_sps(nal, 1, 256)) == SVAC3_ERR_INVALID && !ctx.have_sps);
}

/* ---- tests on real NAL units ---- */

typedef struct {
    const uint8_t *nal;
    size_t len;
} unit_t;

static int find_units(const uint8_t *buf, size_t len, unit_t *sps, unit_t *pps,
                      unit_t *ph, unit_t *patch)
{
    size_t pos = 0, n;
    const uint8_t *p;
    int found = 0;

    while (svac3_next_nal(buf, len, &pos, &p, &n) && found != 0xf) {
        if (n < 2)
            continue;
        switch (p[1] & 31) {
        case SVAC3_NAL_SPS:      if (!(found & 1)) { *sps = (unit_t){ p, n }; found |= 1; } break;
        case SVAC3_NAL_PPS:      if (!(found & 2)) { *pps = (unit_t){ p, n }; found |= 2; } break;
        case SVAC3_NAL_PIC_HEAD: if (!(found & 4)) { *ph = (unit_t){ p, n }; found |= 4; } break;
        case SVAC3_NAL_IDR_PATCH:
            if ((found & 7) == 7 && !(found & 8)) { *patch = (unit_t){ p, n }; found |= 8; }
            break;
        }
    }
    return found == 0xf;
}

static void prime(svac3_hls_t *ctx, unit_t sps, unit_t pps, unit_t ph)
{
    memset(ctx, 0, sizeof(*ctx));
    CHECK(parse(ctx, sps.nal, sps.len) == SVAC3_OK);
    CHECK(parse(ctx, pps.nal, pps.len) == SVAC3_OK);
    CHECK(parse(ctx, ph.nal, ph.len) == SVAC3_OK);
    CHECK(ctx->have_sps && ctx->have_pps && ctx->have_ph);
}

static void test_real_units(const uint8_t *buf, size_t len)
{
    unit_t sps, pps, ph, patch;
    svac3_hls_t ctx;
    uint8_t tmp[4096];

    if (!find_units(buf, len, &sps, &pps, &ph, &patch)) {
        fprintf(stderr, "stream lacks SPS/PPS/PH/IDR patch\n");
        failures++;
        return;
    }

    /* the patch header is kept in the context */
    prime(&ctx, sps, pps, ph);
    CHECK(parse(&ctx, patch.nal, patch.len) == SVAC3_OK);
    CHECK(ctx.have_patch && ctx.patch.patch_index == 0 && ctx.patch.aec_data_offset > 0);

    /* a malformed stop bit fails the SPS and leaves nothing valid */
    CHECK(sps.len <= sizeof(tmp));
    memcpy(tmp, sps.nal, sps.len);
    tmp[sps.len - 1] ^= 0x01;                 /* break the trailing bits */
    CHECK(parse(&ctx, tmp, sps.len) == SVAC3_ERR_INVALID);
    CHECK(!ctx.have_sps && !ctx.have_pps && !ctx.have_ph && !ctx.have_patch);

    /* a corrupt replacement PPS drops the picture header */
    prime(&ctx, sps, pps, ph);
    memcpy(tmp, pps.nal, pps.len);
    tmp[pps.len - 1] ^= 0x01;
    CHECK(parse(&ctx, tmp, pps.len) != SVAC3_OK);
    CHECK(ctx.have_sps && !ctx.have_pps && !ctx.have_ph);
    CHECK(parse(&ctx, patch.nal, patch.len) == SVAC3_ERR_INVALID);

    /* a valid replacement PPS also drops the picture header */
    prime(&ctx, sps, pps, ph);
    CHECK(parse(&ctx, pps.nal, pps.len) == SVAC3_OK);
    CHECK(ctx.have_pps && !ctx.have_ph);
    CHECK(parse(&ctx, patch.nal, patch.len) == SVAC3_ERR_INVALID);

    /* an encrypted picture header cannot be parsed, so it invalidates the old one */
    prime(&ctx, sps, pps, ph);
    CHECK(ph.len <= sizeof(tmp));
    memcpy(tmp, ph.nal, ph.len);
    tmp[0] |= 1 << 5;                         /* encryption_idc = 1 */
    CHECK(parse(&ctx, tmp, ph.len) == SVAC3_OK);
    CHECK(ctx.have_pps && !ctx.have_ph);
    CHECK(parse(&ctx, patch.nal, patch.len) == SVAC3_ERR_INVALID);
}

int main(int argc, char **argv)
{
    FILE *f;
    long len;
    uint8_t *buf;

    if (argc != 2) {
        fprintf(stderr, "usage: %s stream.svac3\n", argv[0]);
        return 2;
    }
    test_rpl_and_wq_ranges();

    if (!(f = fopen(argv[1], "rb"))) {
        perror(argv[1]);
        return 1;
    }
    fseek(f, 0, SEEK_END);
    len = ftell(f);
    fseek(f, 0, SEEK_SET);
    buf = malloc((size_t)len);
    if (!buf || fread(buf, 1, (size_t)len, f) != (size_t)len) {
        fprintf(stderr, "cannot read %s\n", argv[1]);
        return 1;
    }
    fclose(f);
    test_real_units(buf, (size_t)len);
    free(buf);

    if (failures)
        fprintf(stderr, "%d check(s) failed\n", failures);
    else
        printf("all checks passed\n");
    return failures ? 1 : 0;
}
