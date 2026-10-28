#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
# Decode a stream with svac3dec and compare each picture that the decoder
# reconstructs (currently: I pictures) with tests/golden/<stream>.framemd5.
# usage: check_md5.py <svac3dec> <stream.svac3> <golden.framemd5>
import hashlib, os, subprocess, sys, tempfile

dec, stream, golden = sys.argv[1:4]
here = os.path.dirname(os.path.abspath(__file__))
ref = {}
for line in open(golden):
    if not line.startswith('#'):
        i, m = line.split()
        ref[int(i)] = m

dump = subprocess.run([sys.executable, os.path.join(here, '..', 'tools', 'svac3_parse.py'), stream],
                      capture_output=True, text=True, check=True).stdout.splitlines()
types = [int(l.split('=')[1]) for l in dump if l.startswith('picture_coding_type=')]
w = next(int(l.split('=')[1]) for l in dump if l.startswith('horizontal_size='))
h = next(int(l.split('=')[1]) for l in dump if l.startswith('vertical_size='))

with tempfile.TemporaryDirectory() as tmp:
    out = os.path.join(tmp, 'out.yuv')
    subprocess.run([dec, stream, out], check=True, capture_output=True)
    data = open(out, 'rb').read()

fs = w * h * 3 // 2
if len(data) != fs * len(types):
    sys.exit(f'FAIL: {len(data) // fs} pictures decoded, {len(types)} in the stream')
checked = bad = 0
for i, t in enumerate(types):
    if t != 1:
        continue                     # only I pictures are reconstructed so far
    checked += 1
    if hashlib.md5(data[i * fs:(i + 1) * fs]).hexdigest() != ref[i]:
        bad += 1
        print(f'MISMATCH: picture {i}')
print(f'{os.path.basename(stream)}: {checked - bad}/{checked} I pictures bit-exact')
sys.exit(1 if bad or not checked else 0)
