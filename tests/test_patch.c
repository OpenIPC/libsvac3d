/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright (c) 2026 OpenIPC contributors
 *
 * Unit tests for the SVAC3 patch parser's input checks.
 * usage: test_patch <stream.svac3>   (a Main-profile stream from tests/streams)
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "svac3_hls.h"
#include "svac3_patch.h"

static int failures;

#define CHECK(cond)                                                       \
    do {                                                                  \
        if (!(cond)) {                                                    \
            fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__,        \
                    __LINE__, #cond);                                     \
            failures++;                                                   \
        }                                                                 \
    } while (0)

int main(int argc, char **argv)
{
    if (argc != 2) {
        fprintf(stderr, "usage: %s stream.svac3\n", argv[0]);
        return 2;
    }
    FILE *f = fopen(argv[1], "rb");
    if (!f) {
        perror(argv[1]);
        return 1;
    }
    fseek(f, 0, SEEK_END);
    long len = ftell(f);
    fseek(f, 0, SEEK_SET);
    uint8_t *buf = malloc((size_t)len), *rbsp = malloc((size_t)len);
    svac3_hls_t *hls = calloc(1, sizeof(*hls)), *t = calloc(1, sizeof(*t));
    if (!buf || !rbsp || !hls || !t || fread(buf, 1, (size_t)len, f) != (size_t)len)
        return 1;
    fclose(f);

    /* parse up to and including the first IDR patch header */
    size_t pos = 0, nal_len, rl = 0;
    const uint8_t *nal;
    while (svac3_next_nal(buf, (size_t)len, &pos, &nal, &nal_len)) {
        if (nal_len < 2 || svac3_parse_nal(hls, nal, nal_len, 0, NULL))
            continue;
        if ((nal[1] & 31) == SVAC3_NAL_IDR_PATCH) {
            rl = svac3_unescape(nal + 2, nal_len - 2, rbsp);
            break;
        }
    }
    CHECK(rl > 0 && hls->have_patch);

    svac3_patch_stats_t st;
    CHECK(svac3_patch_parse(hls, rbsp, rl, &st) == SVAC3_OK && st.stuffing_ok);

    /* no parsed patch header */
    *t = *hls; t->have_patch = 0;
    CHECK(svac3_patch_parse(t, rbsp, rl, &st) == SVAC3_ERR_INVALID);

    /* reserved picture_coding_type 0, and B pictures */
    *t = *hls; t->ph.picture_coding_type = 0;
    CHECK(svac3_patch_parse(t, rbsp, rl, &st) == SVAC3_ERR_INVALID);
    *t = *hls; t->ph.picture_coding_type = 3;
    CHECK(svac3_patch_parse(t, rbsp, rl, &st) == SVAC3_ERR_UNSUPPORTED);

    /* active reference counts that do not fit */
    *t = *hls; t->sps.num_ref_default_active_minus1[0] = 0x7fffffff;
    CHECK(svac3_patch_parse(t, rbsp, rl, &st) == SVAC3_ERR_INVALID);
    *t = *hls; t->ph.num_ref_active_override_flag = 1; t->ph.num_ref_active_minus1[0] = 0xffffffffu;
    CHECK(svac3_patch_parse(t, rbsp, rl, &st) == SVAC3_ERR_INVALID);

    /* huge patch dimensions are compared without overflow and still accepted
     * as covering the picture; small ones mean several patches */
    *t = *hls; t->sps.patch_width_minus1 = 0x7fffffff; t->sps.patch_height_minus1 = 0xffffffffu;
    CHECK(svac3_patch_parse(t, rbsp, rl, &st) == SVAC3_OK);
    *t = *hls; t->sps.patch_width_minus1 = 0;
    CHECK(svac3_patch_parse(t, rbsp, rl, &st) == SVAC3_ERR_UNSUPPORTED);

    /* patch kinds other than display-picture patches */
    *t = *hls; t->patch.nal_unit_type = SVAC3_NAL_CRR_PRIVACY;
    CHECK(svac3_patch_parse(t, rbsp, rl, &st) == SVAC3_ERR_UNSUPPORTED);

    /* every patch type the header parser knows is a patch to the classifier */
    const unsigned patch_types[] = { 1, 2, 4, 12, 14, 17, 18, 19 };
    for (unsigned i = 0; i < sizeof(patch_types) / sizeof(*patch_types); i++)
        CHECK(svac3_nal_is_patch(patch_types[i]));
    CHECK(!svac3_nal_is_patch(SVAC3_NAL_SPS) && !svac3_nal_is_patch(SVAC3_NAL_PIC_HEAD));

    free(t);
    free(hls);
    free(rbsp);
    free(buf);
    if (failures)
        fprintf(stderr, "%d check(s) failed\n", failures);
    else
        printf("all checks passed\n");
    return failures ? 1 : 0;
}
