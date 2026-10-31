#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Sums r3d_light_cmp's per-frame lines: PSNR over all pixels, the p50 frame's
# PSNR, pixels more than 8 out on some channel, coverage disagreements.
import math
import sys
from collections import defaultdict

se = defaultdict(float); n = defaultdict(int); n8 = defaultdict(int); cov = defaultdict(int); fr = defaultdict(list)
for fn in sys.argv[1:]:
    for line in open(fn):
        w = line.split()
        if len(w) < 12 or w[2] != 'se': continue
        v = w[1]; s = float(w[3]); k = int(w[5])
        se[v] += s; n[v] += k; n8[v] += int(w[7]); cov[v] += int(w[9])
        fr[v].append(99.0 if s == 0 else 10 * math.log10(255 * 255 * 3 * k / s))
print('%-9s %7s %10s %10s %12s %12s' % ('variant', 'frames', 'PSNR dB', 'p50 frame', 'differ >8 %', 'coverage %'))
for v in sorted(n, key=lambda x: ('mame', 'vertex', 'fragment').index(x) if x in ('mame', 'vertex', 'fragment') else 9):
    p = sorted(fr[v])
    print('%-9s %7d %10.2f %10.2f %12.3f %12.3f' % (v, len(p), 10 * math.log10(255 * 255 * 3 * n[v] / se[v]), p[len(p) // 2],
                                                   100.0 * n8[v] / n[v], 100.0 * cov[v] / n[v]))
