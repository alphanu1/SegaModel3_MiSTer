#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Sega Model 3 core for MiSTer FPGA -- Copyright (C) 2026 alphanu1
#
# scsp_capsqueeze.py <capture> <out> <from s> <to s> <end s>
# A two-chip capture (c0/, c1/: regs.bin, ramw.bin) with [A, B) squeezed into
# [A, A + (B - A) / 50): every write is kept, in order, at an earlier sample,
# and the rest moved up; writes from END on are dropped. For the slow benches,
# which otherwise spend most of VF3's capture in its silent boot. The squeezed
# stream is not MAME's timing; tb_scsp_dual compares two designs on it.
import sys, os, numpy as np
src, dst, A, B, END = sys.argv[1], sys.argv[2], float(sys.argv[3]), float(sys.argv[4]), float(sys.argv[5])
a, b, e = int(A * 44100), int(B * 44100), int(END * 44100)
k = (b - a) // 50
def squeeze(n):
    n = n.astype(np.int64)
    out = n.copy()
    mid = (n >= a) & (n < b)
    out[mid] = a + (n[mid] - a) * k // (b - a)
    out[n >= b] = n[n >= b] - (b - a) + k
    return out
for c in (0, 1):
    os.makedirs(dst + '/c%d' % c, exist_ok=True)
    regs = np.fromfile(src + '/c%d/regs.bin' % c, dtype=[('n', '<u4'), ('off', '<u2'), ('data', '<u2'), ('mask', '<u2'), ('pad', '<u2')])
    regs = regs[regs['n'] < e]
    regs['n'] = squeeze(regs['n'])
    regs.tofile(dst + '/c%d/regs.bin' % c)
    ramw = np.fromfile(src + '/c%d/ramw.bin' % c, dtype=np.uint32).reshape(-1, 3)
    ramw = ramw[ramw[:, 0] < e]
    ramw[:, 0] = squeeze(ramw[:, 0])
    ramw.tofile(dst + '/c%d/ramw.bin' % c)
print('squeezed [%g, %g) s into %d samples; %g s of capture end at sample %d' % (A, B, k, END - B, e - (b - a) + k))
