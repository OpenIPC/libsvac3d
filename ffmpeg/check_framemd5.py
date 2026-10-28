#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
# Compare `ffmpeg -i stream.svac3 -f framemd5 out` with tests/golden/<stream>.framemd5.
# usage: check_framemd5.py <golden.framemd5> <ffmpeg.framemd5>
import sys

golden, got = sys.argv[1:3]
ref = [l.split()[1] for l in open(golden) if l.strip() and not l.startswith('#')]
out = [l.rsplit(',', 1)[1].strip() for l in open(got) if l.strip() and not l.startswith('#')]
bad = [i for i, (a, b) in enumerate(zip(ref, out)) if a != b]
for i in bad[:10]:
    print(f'MISMATCH: picture {i}')
print(f'{len(out)} frames from ffmpeg, {len(ref)} golden, {len(ref) - len(bad) if len(out) == len(ref) else 0} equal')
sys.exit(1 if bad or len(out) != len(ref) else 0)
