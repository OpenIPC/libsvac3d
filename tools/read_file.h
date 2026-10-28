/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright (c) 2026 OpenIPC contributors
 *
 * Read a whole (seekable) file into memory, for the command-line tools.
 */
#ifndef SVAC3_READ_FILE_H
#define SVAC3_READ_FILE_H

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

/* Returns a malloc'ed buffer (at least 1 byte) and sets *len, or NULL with
 * a message on stderr. */
static inline uint8_t *read_file(const char *path, size_t *len)
{
    FILE *f = fopen(path, "rb");
    uint8_t *buf = NULL;
    long n;

    if (!f) {
        perror(path);
        return NULL;
    }
    if (fseek(f, 0, SEEK_END) != 0 || (n = ftell(f)) < 0 || fseek(f, 0, SEEK_SET) != 0) {
        fprintf(stderr, "%s: not a seekable file\n", path);
        fclose(f);
        return NULL;
    }
    buf = malloc(n > 0 ? (size_t)n : 1);
    if (!buf || fread(buf, 1, (size_t)n, f) != (size_t)n) {
        fprintf(stderr, "%s: cannot read\n", path);
        free(buf);
        fclose(f);
        return NULL;
    }
    fclose(f);
    *len = (size_t)n;
    return buf;
}

#endif /* SVAC3_READ_FILE_H */
