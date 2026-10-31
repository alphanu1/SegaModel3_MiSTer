#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Sega Model 3 core for MiSTer FPGA -- Copyright (C) 2026 alphanu1
#
# scsp_dspmix.py <in capture> <out capture> <seed> [every]
# A random capture (scsp_randcap's, one chip) with random DSP programs merged
# in: RBL / RBP, COEF, MADRS, MPRO (IRA mostly <= 0x31, an abort now and then,
# memory on any step) and the 0xBF0 write that starts it, at sample 0 and again
# every `every` samples (default 3000). For tb_scsp_dual, which compares two
# designs and needs no reference of MAME's.
import sys, random, numpy as np
src, dst, seed = sys.argv[1], sys.argv[2], int(sys.argv[3])
every = int(sys.argv[4]) if len(sys.argv) > 4 else 3000
rng = random.Random(seed)
R = [('n', '<u4'), ('off', '<u2'), ('data', '<u2'), ('mask', '<u2'), ('pad', '<u2')]
regs = np.fromfile(src + '/regs.bin', dtype=R)
last = int(regs['n'].max()) if len(regs) else 0
add = []
for n in range(0, last + every, every):
    add.append((n, 0x402, ((rng.randrange(4)) << 7) | rng.randrange(64), 0xffff, 0))
    steps = 1 + rng.randrange(128)
    for k in range(64): add.append((n, 0x700 + 2 * k, rng.randrange(65536), 0xffff, 0))
    for k in range(32): add.append((n, 0x780 + 2 * k, rng.randrange(65536), 0xffff, 0))
    mp = []
    for k in range(512):
        v = 0
        if k // 4 < steps:
            v = rng.randrange(65536)
            if (k & 3) == 1:
                v = (v & ~(0x3f << 6)) | (rng.randrange(50) << 6)
                if rng.randrange(64) == 0: v |= 0x3f << 6
        mp.append(v)
    for k in range(512):
        if 0x800 + 2 * k != 0xBF0: add.append((n, 0x800 + 2 * k, mp[k], 0xffff, 0))
    add.append((n, 0xBF0, mp[(0xBF0 - 0x800) // 2], 0xffff, 0))
a = np.array(add, dtype=R)
out = np.concatenate([a, regs])
out = out[np.argsort(out['n'], kind='stable')]
import os, shutil
os.makedirs(dst, exist_ok=True)
out.tofile(dst + '/regs.bin')
for f in ('ramw.bin', 'midi.txt'):
    if os.path.exists(src + '/' + f): shutil.copy(src + '/' + f, dst + '/' + f)
print('%s: %d writes, %d DSP programs' % (dst, len(out), len(range(0, last + every, every))))
