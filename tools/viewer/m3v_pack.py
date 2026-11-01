#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Sega Model 3 core for MiSTer FPGA
# Copyright (C) 2026 alphanu1
#
# Packs the M2.5 viewer's load files (study D-31). Both are ROM-derived: keep
# them under sm3-builds/ and on the board, never in the repository.
#
#   m3v_pack.py vrom   --zip vf3.zip --out vf3.v3v [--check vrom_vf3.bin]
#   m3v_pack.py frames --out vf3.v3f DUMP.bin.gz [DUMP.bin.gz ...]
#
# vrom: the video ROMs as the geometry reads them, 32-bit words little-endian,
# geometry word address 0x800000 upward (MAME's interleave_vroms puts a 16 MiB
# user3 region at the top half of its 64 MiB image; the lower half is zero).
# --check compares with MAME's saved image (mame0289-real3d-tris.patch,
# M3TRIS_VROM).
#
# frames: Supermodel dumps (supermodel-real3d-dump.patch) uncompressed and
# concatenated, 17 MiB a frame: culling RAM 8C (4 MiB), 8E (1 MiB), polygon RAM
# (4 MiB), texture RAM (8 MiB), each 32-bit word little-endian as dumped.

import argparse, gzip, hashlib, sys, zipfile

# MAME 0.289 model3.cpp ROM_START(vf3): user3 / user4, ROM_LOAD_VROM offsets
VF3_USER3 = [("mpr-19211.26", 0x2), ("mpr-19212.27", 0x0), ("mpr-19213.28", 0x6), ("mpr-19214.29", 0x4),
             ("mpr-19215.30", 0xa), ("mpr-19216.31", 0x8), ("mpr-19217.32", 0xe), ("mpr-19218.33", 0xc)]
VF3_USER4 = [("mpr-19219.34", 0x2), ("mpr-19220.35", 0x0), ("mpr-19221.36", 0x6), ("mpr-19222.37", 0x4),
             ("mpr-19223.38", 0xa), ("mpr-19224.39", 0x8), ("mpr-19225.40", 0xe), ("mpr-19226.41", 0xc)]
FRAME_BYTES = 0x1100000


def region(z, parts):
    # ROM_GROUPWORD | ROM_SKIP(14): two bytes of a ROM every 16 bytes
    r = bytearray(0x1000000)
    for name, off in parts:
        d = z.read(name)
        assert len(d) == 0x200000, name
        for k in range(2):
            r[off + k::16] = d[k::2]
    return r


def vrom(args):
    z = zipfile.ZipFile(args.zip)
    u3, u4 = region(z, VF3_USER3), region(z, VF3_USER4)
    # interleave_vroms: vrom16[i + x] = vrom1_16[(j + x) ^ 1], eight words of
    # each region in turn; uint16 in host (little-endian) order
    w3 = memoryview(u3).cast('H')
    w4 = memoryview(u4).cast('H')
    out = bytearray(0x2000000)
    o = memoryview(out).cast('H')
    for x in range(8):
        o[x::16] = w3[x ^ 1::8]
        o[8 + x::16] = w4[x ^ 1::8]
    if args.check:
        with open(args.check, 'rb') as f:
            f.seek(0x2000000)
            ref = f.read(0x2000000)
        if ref != bytes(out):
            sys.exit('VROM differs from ' + args.check)
        print('matches', args.check)
    with open(args.out, 'wb') as f:
        f.write(out)
    print(args.out, len(out), 'bytes, md5', hashlib.md5(out).hexdigest())


def frames(args):
    h = hashlib.md5()
    n = 0
    with open(args.out, 'wb') as f:
        for fn in args.dumps:
            d = gzip.open(fn, 'rb').read()
            assert len(d) == FRAME_BYTES, (fn, len(d))
            f.write(d)
            h.update(d)
            n += 1
    if n > 6:
        print('warning: a 128 MiB SDRAM holds 6 frames')
    print(args.out, n, 'frames,', n * FRAME_BYTES, 'bytes, md5', h.hexdigest())


def main():
    p = argparse.ArgumentParser()
    s = p.add_subparsers(dest='cmd', required=True)
    a = s.add_parser('vrom'); a.add_argument('--zip', required=True); a.add_argument('--out', required=True)
    a.add_argument('--check')
    b = s.add_parser('frames'); b.add_argument('--out', required=True); b.add_argument('dumps', nargs='+')
    args = p.parse_args()
    {'vrom': vrom, 'frames': frames}[args.cmd](args)


if __name__ == '__main__':
    main()
