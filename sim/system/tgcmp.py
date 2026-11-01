#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Sega Model 3 core for MiSTer FPGA
# Copyright (C) 2026 alphanu1
#
# The stage-A pass check for one screen:
#   tgcmp.py state  BENCH_BASE  SM_BASE     the tile generator's state (VRAM, palette,
#                                           registers) the bench reached against
#                                           Supermodel's dump, byte for byte
#   tgcmp.py pixels RTL.ppm REF.ppm         the RTL's video output against the tile
#                                           generator reference (tg_tool ppm), pixel
#                                           for pixel; with --overlay-box the debug
#                                           overlay's corner is left out
# BASE is a dump name without extension (fNNNNNN: .vram 0x120000 bytes, .regs 256).
import sys


def state(a, b):
    va, vb = open(a + '.vram', 'rb').read(), open(b + '.vram', 'rb').read()
    ra, rb = open(a + '.regs', 'rb').read(), open(b + '.regs', 'rb').read()
    dv = [i for i in range(0x100000) if va[i] != vb[i]]
    dp = [i for i in range(0x100000, 0x120000) if va[i] != vb[i]]
    # registers the tile generator uses: 20, 40, 44, 60-6C (08 and 0C: line and frame)
    used = [0x20, 0x40, 0x44, 0x60, 0x64, 0x68, 0x6C]
    dr = [o for o in used if ra[o:o + 4] != rb[o:o + 4]]
    print(f'VRAM bytes differing {len(dv)}, palette bytes {len(dp)}, used registers {[hex(o) for o in dr]}')
    for i in dv[:8]:
        print(f'  VRAM {i:05X}: bench {va[i]:02X} Supermodel {vb[i]:02X}')
    for i in dp[:8]:
        print(f'  palette byte {i - 0x100000:05X}: bench {va[i]:02X} Supermodel {vb[i]:02X}')
    return 0 if not (dv or dp or dr) else 1


def ppm(fn):
    d = open(fn, 'rb').read()
    parts = d.split(b'\n', 3)
    w, h = map(int, parts[1].split())
    return w, h, parts[3]


def pixels(a, b):
    wa, ha, pa = ppm(a)
    wb, hb, pb = ppm(b)
    assert (wa, ha) == (wb, hb), ((wa, ha), (wb, hb))
    n = 0
    first = None
    for i in range(wa * ha):
        if pa[3 * i:3 * i + 3] != pb[3 * i:3 * i + 3]:
            n += 1
            if first is None:
                first = (i % wa, i // wa, pa[3 * i:3 * i + 3].hex(), pb[3 * i:3 * i + 3].hex())
    print(f'{n} of {wa * ha} pixels differ' + (f'; first at {first}' if first else ''))
    return 0 if n == 0 else 1


if __name__ == '__main__':
    sys.exit({'state': state, 'pixels': pixels}[sys.argv[1]](sys.argv[2], sys.argv[3]))
