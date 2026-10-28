/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright (c) 2026 OpenIPC contributors
 *
 * svac3dump: print the high-level syntax of an SVAC3 elementary stream in
 * the format of tools/svac3_parse.py.
 */
#include <stdio.h>
#include <stdlib.h>

#include "read_file.h"
#include "svac3_hls.h"

int main(int argc, char **argv)
{
    uint8_t *buf;
    size_t n;
    size_t pos = 0, nal_len;
    const uint8_t *nal;
    unsigned idx = 0;
    svac3_hls_t *ctx;
    int ret = 0;

    if (argc != 2) {
        fprintf(stderr, "usage: %s stream.svac3\n", argv[0]);
        return 2;
    }
    if (!(buf = read_file(argv[1], &n))) {
        return 1;
    }
    ctx = calloc(1, sizeof(*ctx));
    if (!ctx) {
        free(buf);
        return 1;
    }

    while (svac3_next_nal(buf, n, &pos, &nal, &nal_len)) {
        if (nal_len >= 2) {
            ret = svac3_parse_nal(ctx, nal, nal_len, idx, stdout);
            if (ret) {
                fprintf(stderr, "nal %u: error %d\n", idx, ret);
                break;
            }
        }
        idx++;
    }
    free(ctx);
    free(buf);
    return ret ? 1 : 0;
}
