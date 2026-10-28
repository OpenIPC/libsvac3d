/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright (c) 2026 OpenIPC contributors
 *
 * libFuzzer target for the SVAC3 patch parser: every patch NAL unit that
 * follows a valid SPS, PPS and picture header is parsed.
 * Build: clang -g -O1 -fsanitize=fuzzer,address,undefined -Isource/svac3 \
 *          tests/fuzz/fuzz_patch.c source/svac3/svac3_hls.c \
 *          source/svac3/svac3_aec.c source/svac3/svac3_patch.c
 */
#include <stdlib.h>
#include <string.h>

#include "svac3_hls.h"
#include "svac3_patch.h"

int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size)
{
    static svac3_hls_t hls;
    size_t pos = 0, nal_len;
    const uint8_t *nal;
    uint8_t *rbsp = malloc(size ? size : 1);

    if (!rbsp)
        return 0;
    memset(&hls, 0, sizeof(hls));
    while (svac3_next_nal(data, size, &pos, &nal, &nal_len)) {
        if (nal_len < 2 || svac3_parse_nal(&hls, nal, nal_len, 0, NULL))
            continue;
        if (!svac3_nal_is_patch(nal[1] & 31u) || !hls.have_patch)
            continue;
        /* keep the fuzzer fast: bound the picture it may describe */
        if ((size_t)hls.sps.pic_width_in_luma * hls.sps.pic_height_in_luma > 1920 * 1088)
            continue;
        svac3_patch_stats_t st;
        size_t rl = svac3_unescape(nal + 2, nal_len - 2, rbsp);
        svac3_patch_parse(&hls, rbsp, rl, &st);
    }
    free(rbsp);
    return 0;
}
