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

#include "read_file.h"
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
    size_t len;
    uint8_t *buf = read_file(argv[1], &len), *rbsp = malloc(len ? len : 1);
    svac3_hls_t *hls = calloc(1, sizeof(*hls)), *t = calloc(1, sizeof(*t));
    if (!buf || !rbsp || !hls || !t)
        return 1;

    /* parse up to and including the first IDR patch header */
    size_t pos = 0, nal_len, rl = 0;
    const uint8_t *nal;
    while (svac3_next_nal(buf, len, &pos, &nal, &nal_len)) {
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

    /* MaxPartRatio above the Main profile's 8 */
    *t = *hls; t->sps.log2_max_part_ratio_minus2 = 2;
    CHECK(svac3_patch_parse(t, rbsp, rl, &st) == SVAC3_ERR_INVALID);

    /* reconstruction into a frame that cannot hold the coded picture */
    {
        const int W = hls->sps.pic_width_in_luma, H = hls->sps.pic_height_in_luma;
        uint8_t *pix = calloc((size_t)W * H * 3 / 2, 1);
        svac3_frame_t fr = {
            .plane = { pix, pix + (size_t)W * H, pix + (size_t)W * H * 5 / 4 },
            .stride = { W, W / 2, W / 2 },
            .width = { W, W / 2, W / 2 },
            .height = { H, H / 2, H / 2 },
        };
        CHECK(pix != NULL);
        CHECK(svac3_patch_decode(hls, rbsp, rl, &fr, &st) == SVAC3_OK);
        svac3_frame_t bad = fr;
        bad.height[0] = H - 8;
        CHECK(svac3_patch_decode(hls, rbsp, rl, &bad, &st) == SVAC3_ERR_INVALID);
        bad = fr;
        bad.stride[1] = W / 2 - 1;
        CHECK(svac3_patch_decode(hls, rbsp, rl, &bad, &st) == SVAC3_ERR_INVALID);
        bad = fr;
        bad.plane[2] = NULL;
        CHECK(svac3_patch_decode(hls, rbsp, rl, &bad, &st) == SVAC3_ERR_INVALID);
        /* a truncated patch is corrupt when reconstructing, only reported when parsing */
        CHECK(svac3_patch_decode(hls, rbsp, rl / 2, &fr, &st) == SVAC3_ERR_CORRUPT);
        CHECK(svac3_patch_parse(hls, rbsp, rl / 2, &st) == SVAC3_OK && !st.stuffing_ok);
        free(pix);
    }

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
