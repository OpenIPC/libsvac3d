/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright (c) 2026 OpenIPC contributors
 *
 * svac3patch: parse every patch of an SVAC3 stream and report whether each
 * one ends exactly on its last LCU.
 * usage: svac3patch [-v] stream.svac3
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "read_file.h"
#include "svac3_hls.h"
#include "svac3_patch.h"

int main(int argc, char **argv)
{
    const int verbose = argc == 3 && !strcmp(argv[1], "-v");
    if (argc != 2 && !verbose) {
        fprintf(stderr, "usage: %s [-v] stream.svac3\n", argv[0]);
        return 2;
    }
    const char *path = argv[argc - 1];
    size_t len;
    uint8_t *buf = read_file(path, &len);
    if (!buf)
        return 1;
    uint8_t *rbsp = malloc(len ? len : 1);
    svac3_hls_t *hls = calloc(1, sizeof(*hls));
    int rc = 1;
    if (!rbsp || !hls)
        goto out;

    size_t pos = 0, nal_len;
    const uint8_t *nal;
    unsigned idx = 0, patches = 0, good = 0, pic = 0;
    while (svac3_next_nal(buf, len, &pos, &nal, &nal_len)) {
        if (nal_len >= 2) {
            int t = nal[1] & 31;
            int ret = svac3_parse_nal(hls, nal, nal_len, idx, NULL);
            if (ret) {
                fprintf(stderr, "nal %u: header error %d\n", idx, ret);
                goto out;
            }
            if (t == SVAC3_NAL_PIC_HEAD)
                pic++;
            if (svac3_nal_is_patch((unsigned)t)) {
                svac3_patch_stats_t st;
                size_t rl = svac3_unescape(nal + 2, nal_len - 2, rbsp);
                ret = svac3_patch_parse(hls, rbsp, rl, &st);
                patches++;
                if (ret) {
                    printf("pic %u: parse error %d\n", pic - 1, ret);
                } else {
                    /* exact end: the right stuffing bits, and nothing left
                     * over but the rbsp_trailing_bits byte */
                    const int exact = st.stuffing_ok && st.overrun_bits == 0 &&
                                      st.bytes_used <= st.bytes_avail &&
                                      st.bytes_avail - st.bytes_used <= 1;
                    good += exact;
                    if (verbose || !exact)
                        printf("pic %u type %u: lcus=%u stuffing=%s%s cus=%u intra=%u skip=%u direct=%u "
                               "chroma=%u blocks=%u coeffs=%u bytes %zu/%zu overrun=%zu\n",
                               pic - 1, hls->ph.picture_coding_type, st.lcus,
                               st.stuffing_ok ? "ok" : "BAD", exact ? "" : " (not exact)",
                               st.cus, st.intra_cus, st.skip_cus, st.direct_cus, st.chroma_cus,
                               st.coded_blocks, st.coeffs, st.bytes_used, st.bytes_avail,
                               st.overrun_bits);
                    if (!exact && !st.stuffing_ok)
                        printf("    first bad LCU %u\n", st.first_bad_lcu);
                }
            }
        }
        idx++;
    }
    printf("%u/%u patches end exactly on their last LCU\n", good, patches);
    rc = patches && good == patches ? 0 : 1;
out:
    free(hls);
    free(rbsp);
    free(buf);
    return rc;
}
