#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
# Generate source/svac3/svac3_tables.h from the SVAC 3.0 draft text.
# usage: tools/gen_tables.py draft.txt > source/svac3/svac3_tables.h
# draft.txt is `pdftotext -layout` of the public-comment draft of plan
# 20243294-T-312; tables are located by their captions, and every table is
# checked (row counts, filter tap sums, DCT basis orthogonality).
import re, sys
lines = open(sys.argv[1]).read().split('\n')

def caption(text):
    hits = [i for i, l in enumerate(lines) if text in l and '查表' not in l and '见表' not in l]
    assert hits, text
    return hits[-1]

out = []
out.append('/* SPDX-License-Identifier: BSD-3-Clause\n * Copyright (c) 2026 OpenIPC contributors\n *\n'
           ' * Tables of the SVAC 3.0 draft (plan 20243294-T-312), generated from the\n'
           ' * draft text by a script rather than typed in: 表118 angular directions,\n'
           ' * 表119 intra reference filters, Annex N.1 DCT2 inverse transforms.\n */\n'
           '#ifndef SVAC3_TABLES_H\n#define SVAC3_TABLES_H\n\n#include <stdint.h>\n')
t118 = {}
t118_at = caption('表118 角度预测模式的方向')
for l in lines[t118_at:t118_at + 120]:
    m = re.match(r'\s*(\d+)\s+([01])\s+(-?1)\s+\{(\d+),(\d+)\}\s+\{(\d+),(\d+)\}', l)
    if m:
        t118[int(m.group(1))] = tuple(int(x) for x in m.groups()[1:])
assert len(t118) == 60, len(t118)
out.append('/* 表118: dirXYFlag, dirXYSign, divDxy{m,s}, divDyx{m,s}; zero where not angular */')
out.append('static const int16_t svac3_dir_tab[66][6] = {')
for k in range(66):
    out.append('    { %d, %d, %d, %d, %d, %d },' % t118.get(k, (0,) * 6))
out.append('};\n')
rows01, rows23, cur = {}, {}, None
t119_at = caption('表119 参考像素滤波器系数')
for l in lines[t119_at:t119_at + 100]:
    # two half-tables: "filterIndex 0 1" and then "filterIndex 2 3"
    if re.search(r'filt\w*\s+2\s+3', l):
        cur = rows23
    elif cur is None and re.match(r'^\s*filterIn', l):
        cur = rows01
    if '改进型预测方法' in l:
        break
    m = re.match(r'\s*(\d+)((?:\s+-?\d+){8})\s*$', l)
    if m and cur is not None:
        cur[int(m.group(1))] = [int(x) for x in m.group(2).split()]
assert len(rows01) == 32 and len(rows23) == 32, (len(rows01), len(rows23))
filt = [[rows01[o][0:4] for o in range(32)], [rows01[o][4:8] for o in range(32)],
        [rows23[o][0:4] for o in range(32)], [rows23[o][4:8] for o in range(32)]]
for s, tot in ((0, 128), (1, 128), (2, 128), (3, 64)):
    for o in range(32):
        assert sum(filt[s][o]) == tot, (s, o, filt[s][o])
out.append('/* 表119: [filterIndex][offset][tap] */')
out.append('static const int8_t svac3_ipf_tab[4][32][4] = {')
for s in range(4):
    out.append('    {')
    for o in range(32):
        out.append('        { %d, %d, %d, %d },' % tuple(filt[s][o]))
    out.append('    },')
out.append('};\n')
txt = '\n'.join(lines)
start = txt.index('N.1 DCT2', txt.index('附 录 N'))
end = txt.index('N.2 DCT8', start)
body = txt[start:end]
for n in (4, 8, 16, 32, 64):
    i = body.index('DCT2%d = {' % n) + len('DCT2%d = {' % n)
    if n <= 16:
        j = body.index('}}', i) + 1
        rows = re.findall(r'\{([^{}]*)\}', body[i:j])
        nums = [int(x) for r in rows for x in re.findall(r'-?\d+', r)]
        assert len(nums) == n * n, (n, len(nums))
        m = [nums[k * n:(k + 1) * n] for k in range(n)]
    else:
        # printed as a grid of 16x16 blocks T11..Tgg; rows are the {...} groups
        g = n // 16
        m = [[None] * n for _ in range(n)]
        pos = i
        for bi in range(g):
            for bj in range(g):
                tag = 'T%d%d = {' % (bi + 1, bj + 1)
                pos = body.index(tag, pos) + len(tag)
                rows = []
                q = pos
                while len(rows) < 16:
                    a = body.index('{', q)
                    b = body.index('}', a)
                    vals = [int(x) for x in re.findall(r'-?\d+', body[a + 1:b])]
                    assert len(vals) == 16, (n, bi, bj, len(rows), vals)
                    rows.append(vals)
                    q = b + 1
                pos = q
                for r in range(16):
                    for c in range(16):
                        m[bi * 16 + r][bj * 16 + c] = rows[r][c]
    assert all(v == m[0][0] for v in m[0])
    # basis vectors must be (near) orthogonal: off-diagonal dot products small
    for a in range(n):
        for b in range(a + 1, n):
            d = sum(x * y for x, y in zip(m[a], m[b]))
            assert abs(d) <= 0.01 * n * 32 * 32, (n, a, b, d)
    out.append('/* Annex N.1: %dx%d DCT2, [k][n] (basis k, sample n) */' % (n, n))
    out.append('static const int8_t svac3_dct2_%d[%d][%d] = {' % (n, n, n))
    for k in range(n):
        out.append('    { ' + ', '.join(str(v) for v in m[k]) + ' },')
    out.append('};\n')
out.append('#endif /* SVAC3_TABLES_H */')
print('\n'.join(out))
