/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright (c) 2026 OpenIPC contributors
 *
 * svac3dump: print the high-level syntax of an SVAC3 elementary stream in
 * the format of tools/svac3_parse.py.
 */
#include <stdio.h>
#include <stdlib.h>

#include "svac3_hls.h"

int main(int argc, char **argv)
{
    FILE *f;
    uint8_t *buf;
    long len;
    size_t pos = 0, nal_len;
    const uint8_t *nal;
    unsigned idx = 0;
    svac3_hls_t *ctx;
    int ret = 0;

    if (argc != 2) {
        fprintf(stderr, "usage: %s stream.svac3\n", argv[0]);
        return 2;
    }
    if (!(f = fopen(argv[1], "rb"))) {
        perror(argv[1]);
        return 1;
    }
    fseek(f, 0, SEEK_END);
    len = ftell(f);
    fseek(f, 0, SEEK_SET);
    buf = malloc(len > 0 ? (size_t)len : 1);
    ctx = calloc(1, sizeof(*ctx));
    if (!buf || !ctx || fread(buf, 1, (size_t)len, f) != (size_t)len) {
        fprintf(stderr, "%s: cannot read\n", argv[1]);
        return 1;
    }
    fclose(f);

    while (svac3_next_nal(buf, (size_t)len, &pos, &nal, &nal_len)) {
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
