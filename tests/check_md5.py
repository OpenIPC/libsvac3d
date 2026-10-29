#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
# Decode a stream with svac3dec and compare every picture with
# tests/golden/<stream>.framemd5.
# usage: check_md5.py <svac3dec> <stream.svac3> <golden.framemd5> [threads]
import hashlib, os, subprocess, sys, tempfile

dec, stream, golden = sys.argv[1:4]
threads = sys.argv[4:5]
here = os.path.dirname(os.path.abspath(__file__))
ref = {}
for line in open(golden):
    if not line.startswith('#'):
        i, m = line.split()
        ref[int(i)] = m

dump = subprocess.run([sys.executable, os.path.join(here, '..', 'tools', 'svac3_parse.py'), stream],
                      capture_output=True, text=True, check=True).stdout.splitlines()
types = [int(l.split('=')[1]) for l in dump if l.startswith('picture_coding_type=')]
if sorted(ref) != list(range(len(types))):
    sys.exit(f'FAIL: {golden} has {len(ref)} hashes for {len(types)} pictures')
w = next(int(l.split('=')[1]) for l in dump if l.startswith('horizontal_size='))
h = next(int(l.split('=')[1]) for l in dump if l.startswith('vertical_size='))

with tempfile.TemporaryDirectory() as tmp:
    out = os.path.join(tmp, 'out.yuv')
    subprocess.run([dec] + (['-t'] + threads if threads else []) + [stream, out],
                   check=True, capture_output=True)
    data = open(out, 'rb').read()

fs = w * h * 3 // 2
if len(data) != fs * len(types):
    sys.exit(f'FAIL: {len(data) // fs} pictures decoded, {len(types)} in the stream')
bad = 0
for i, t in enumerate(types):
    if hashlib.md5(data[i * fs:(i + 1) * fs]).hexdigest() != ref[i]:
        bad += 1
        print(f'MISMATCH: picture {i} ({"IPB"[t - 1] if 1 <= t <= 3 else t})')
n_i = sum(1 for t in types if t == 1)
print(f'{os.path.basename(stream)}: {len(types) - bad}/{len(types)} pictures bit-exact '
      f'({n_i} I, {len(types) - n_i} P)')
sys.exit(1 if bad or not types else 0)
