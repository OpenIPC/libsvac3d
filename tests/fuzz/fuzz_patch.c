/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright (c) 2026 OpenIPC contributors
 *
 * libFuzzer target for the SVAC3 picture decoder: every NAL unit goes
 * through svac3_dec_nal(), so patches are parsed, reconstructed (I and P
 * pictures, against the decoded picture buffer) and loop-filtered.
 * Build: clang -g -O1 -fsanitize=fuzzer,address,undefined -Isource/svac3 \
 *          tests/fuzz/fuzz_patch.c source/svac3/svac3_hls.c source/svac3/svac3_aec.c \
 *          source/svac3/svac3_patch.c source/svac3/svac3_recon.c \
 *          source/svac3/svac3_loopfilter.c source/svac3/svac3_inter.c source/svac3/svac3_dec.c
 */
#include <stdlib.h>
#include <string.h>

#include "svac3_dec.h"

int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size)
{
    static svac3_hls_t peek;
    size_t pos = 0, nal_len;
    const uint8_t *nal;
    svac3_dec_t *dec = svac3_dec_create();
    int patches = 0;

    if (!dec)
        return 0;
    memset(&peek, 0, sizeof(peek));
    while (svac3_next_nal(data, size, &pos, &nal, &nal_len)) {
        if (nal_len < 2 || svac3_parse_nal(&peek, nal, nal_len, 0, NULL))
            continue;
        /* keep the fuzzer fast: bound the pictures an input may make the
         * decoder reconstruct, an I picture and a couple of P pictures */
        if (svac3_nal_is_patch(nal[1] & 31u)) {
            if ((size_t)peek.sps.pic_width_in_luma * peek.sps.pic_height_in_luma > 1920 * 1088)
                continue;
            if (++patches > 3)
                break;
        }
        const svac3_frame_t *fr;
        svac3_dec_nal(dec, nal, nal_len, &fr);
    }
    svac3_dec_destroy(dec);
    return 0;
}
