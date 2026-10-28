/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright (c) 2026 OpenIPC contributors
 *
 * Unit tests for the SVAC3 picture decoder's buffer handling.
 * usage: test_dec <stream.svac3>   (a stream from tests/streams: SPS, PPS,
 *        an IDR picture, then P pictures each referencing the one before)
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "read_file.h"
#include "svac3_dec.h"

static int failures;

#define CHECK(cond)                                                       \
    do {                                                                  \
        if (!(cond)) {                                                    \
            fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__,        \
                    __LINE__, #cond);                                     \
            failures++;                                                   \
        }                                                                 \
    } while (0)

typedef struct {
    const uint8_t *p;
    size_t len;
    int patch;
} unit_t;

/* Feed the units [from, to) and return the first error, counting pictures */
static int feed(svac3_dec_t *d, const unit_t *u, int from, int to, int *pics)
{
    for (int i = from; i < to; i++) {
        const svac3_frame_t *f;
        int ret = svac3_dec_nal(d, u[i].p, u[i].len, &f);
        if (ret)
            return ret;
        *pics += f != NULL;
    }
    return 0;
}

int main(int argc, char **argv)
{
    if (argc != 2) {
        fprintf(stderr, "usage: %s stream.svac3\n", argv[0]);
        return 2;
    }
    size_t len;
    uint8_t *buf = read_file(argv[1], &len);
    unit_t *u = calloc(len / 3 + 1, sizeof(*u));
    if (!buf || !u)
        return 1;
    int n = 0;
    size_t pos = 0, nal_len;
    const uint8_t *nal;
    while (svac3_next_nal(buf, len, &pos, &nal, &nal_len))
        if (nal_len >= 2)
            u[n++] = (unit_t){ nal, nal_len, svac3_nal_is_patch(nal[1] & 31u) };

    /* patch indices: the IDR picture and the two P pictures after it */
    int patch[3], np = 0;
    for (int i = 0; i < n && np < 3; i++)
        if (u[i].patch)
            patch[np++] = i;
    CHECK(np == 3);
    if (np != 3)
        return 1;

    svac3_dec_t *d = svac3_dec_create();
    int pics = 0;
    CHECK(d != NULL);

    /* a P picture whose reference was never decoded */
    CHECK(feed(d, u, 0, patch[0], &pics) == 0);
    CHECK(feed(d, u, patch[0] + 1, patch[1] + 1, &pics) == SVAC3_ERR_CORRUPT && pics == 0);

    /* the IDR picture, then a truncated P picture: rejected, and the IDR
     * picture it refers to survives for the complete copy */
    svac3_dec_reset(d);
    CHECK(feed(d, u, 0, patch[0] + 1, &pics) == 0 && pics == 1);
    CHECK(feed(d, u, patch[0] + 1, patch[1], &pics) == 0);
    const svac3_frame_t *f;
    CHECK(svac3_dec_nal(d, u[patch[1]].p, u[patch[1]].len / 2, &f) == SVAC3_ERR_CORRUPT && !f);
    CHECK(svac3_dec_nal(d, u[patch[1]].p, u[patch[1]].len, &f) == 0 && f);
    CHECK(feed(d, u, patch[1] + 1, patch[2] + 1, &pics) == 0 && pics == 2);

    /* a reset forgets the references */
    svac3_dec_reset(d);
    CHECK(svac3_dec_nal(d, u[patch[2]].p, u[patch[2]].len, &f) == SVAC3_ERR_CORRUPT);

    svac3_dec_destroy(d);
    free(u);
    free(buf);
    if (failures)
        fprintf(stderr, "%d check(s) failed\n", failures);
    else
        printf("all checks passed\n");
    return failures ? 1 : 0;
}
