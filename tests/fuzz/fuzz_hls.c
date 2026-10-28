/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright (c) 2026 OpenIPC contributors
 *
 * libFuzzer target for the SVAC3 high-level syntax parser.
 * Build: clang -g -O1 -fsanitize=fuzzer,address,undefined \
 *          -Isource/svac3 tests/fuzz/fuzz_hls.c source/svac3/svac3_hls.c
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "svac3_hls.h"

int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size)
{
    static svac3_hls_t ctx;
    size_t pos = 0, nal_len;
    const uint8_t *nal;
    unsigned idx = 0;
    FILE *sink = fopen("/dev/null", "w");

    memset(&ctx, 0, sizeof(ctx));
    while (svac3_next_nal(data, size, &pos, &nal, &nal_len)) {
        if (nal_len >= 2)
            svac3_parse_nal(&ctx, nal, nal_len, idx, sink);
        idx++;
    }
    if (sink)
        fclose(sink);
    return 0;
}
