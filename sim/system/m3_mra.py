#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Sega Model 3 core for MiSTer FPGA
# Copyright (C) 2026 alphanu1
#
# Writes the stage-A MRA for a Step 1.0 set (D-37) from MAME 0.289's
# ROM_START layout, and packs the SDRAM image an MRA describes (the same
# semantics as MiSTer Main's MRA loader: parts in order, <interleave
# output="64"> with map strings, rightmost digit = output byte 0, each digit
# the 1-based byte of the part; <part repeat="N">HH</part> a fill).
#
#   m3_mra.py mra vf3 --zip vf3.zip --out "mra/Virtua Fighter 3.mra"
#   m3_mra.py pack MRA --zip vf3.zip --out image.bin     (ROM-derived: sm3-builds only)
#
# SDRAM layout (bytes), m3_system.sv:
#   0x0000000 banked CROM (CROM0-3), 0x4000000 VROM, 0x6000000 fixed CROM,
#   0x6200000 backup RAM (00). Word form: the 32-bit value the PowerPC reads
#   at A is the little-endian word at SDRAM A, so MAME's ROM_LOAD64_WORD_SWAP
#   part at offset 2 gives bytes 0-1 of each 8, offset 0 bytes 2-3, offset 6
#   bytes 4-5, offset 4 bytes 6-7, each in file order.
#   VROM: MAME's interleave_vroms puts the sixteen 2 MiB ROMs in name order,
#   one 16-bit word each per 32-byte group. Here it is four 8 MiB blocks of
#   four ROMs (64-bit interleave, file order), so VROM 32-bit word i =
#   {g[19:0], b[1:0], h} is at 0x4000000 + {b, g, h, 2'b00}: a bit
#   permutation the Real3D's VROM reader makes (stage B).
import argparse, hashlib, sys, zipfile, zlib
import xml.etree.ElementTree as ET

# MAME 0.289 model3.cpp ROM_START(vf3), (name, ROM_LOAD64_WORD_SWAP offset)
VF3 = {
    'setname': 'vf3', 'name': 'Virtua Fighter 3 (Japan, Revision D)', 'year': '1996', 'zip': 'vf3.zip',
    'crom': [  # four banks of 16 MiB (CROM0 .. CROM3)
        [('mpr-19193.1', 6), ('mpr-19194.2', 4), ('mpr-19195.3', 2), ('mpr-19196.4', 0)],
        [('mpr-19197.5', 6), ('mpr-19198.6', 4), ('mpr-19199.7', 2), ('mpr-19200.8', 0)],
        [('mpr-19201.9', 6), ('mpr-19202.10', 4), ('mpr-19203.11', 2), ('mpr-19204.12', 0)],
        [('mpr-19205.13', 6), ('mpr-19206.14', 4), ('mpr-19207.15', 2), ('mpr-19208.16', 0)],
    ],
    'fixed': [('epr-19227d.17', 6), ('epr-19228d.18', 4), ('epr-19229d.19', 2), ('epr-19230d.20', 0)],
    'vrom': ['mpr-19211.26', 'mpr-19212.27', 'mpr-19213.28', 'mpr-19214.29', 'mpr-19215.30', 'mpr-19216.31',
             'mpr-19217.32', 'mpr-19218.33', 'mpr-19219.34', 'mpr-19220.35', 'mpr-19221.36', 'mpr-19222.37',
             'mpr-19223.38', 'mpr-19224.39', 'mpr-19225.40', 'mpr-19226.41'],
}
# ROM_START(bassdx): three CROM banks; CROM3 is unpopulated (00, as MAME's region)
BASSDX = {
    'setname': 'bassdx', 'name': 'Sega Bass Fishing Deluxe (USA)', 'year': '1997', 'zip': 'bassdx.zip',
    'category': 'Sports',
    'crom': [
        [('mpr-20256.1', 6), ('mpr-20257.2', 4), ('mpr-20258.3', 2), ('mpr-20259.4', 0)],
        [('mpr-20260.5', 6), ('mpr-20261.6', 4), ('mpr-20262.7', 2), ('mpr-20263.8', 0)],
        [('mpr-20264.9', 6), ('mpr-20265.10', 4), ('mpr-20266.11', 2), ('mpr-20267.12', 0)],
        None,
    ],
    'fixed': [('epr-20783.17', 6), ('epr-20784.18', 4), ('epr-20785.19', 2), ('epr-20786.20', 0)],
    'vrom': ['mpr-%d.%d' % (20270 + k, 26 + k) for k in range(16)],
}
SETS = {'vf3': VF3, 'bassdx': BASSDX}
# ROM_LOAD64_WORD_SWAP offset -> the output bytes of its pair in word form
OFF_MAP = {2: '00000021', 0: '00002100', 6: '00210000', 4: '21000000'}
SEQ_MAP = ['00000021', '00002100', '00210000', '21000000']


def mra(args):
    s = SETS[args.set]
    z = zipfile.ZipFile(args.zip)
    crc = {i.filename: '%08x' % i.CRC for i in z.infolist()}
    L = []
    w = L.append
    w('<misterromdescription>')
    w('    <!--')
    w('      Sega Model 3 core for MiSTer FPGA - Copyright (C) 2026 alphanu1')
    w('      SPDX-License-Identifier: GPL-3.0-or-later')
    w('')
    w('      %s. Stage A of study D-37: CPU, bus, tile generator, no sound,' % s['name'])
    w('      no 3D. From MAME 0.289 model3.cpp ROM_START(%s); sim/system/m3_mra.py' % s['setname'])
    w('      writes it, and packs the same image for the system bench.')
    w('')
    w('      SDRAM (bytes): 0x0000000 banked CROM, 64 MiB; 0x4000000 VROM, 32 MiB;')
    w('      0x6000000 fixed CROM, 2 MiB; 0x6200000 backup RAM, 128 KiB of 00.')
    w('      Word form: the PowerPC\'s 32-bit word at A is the little-endian word at')
    w('      SDRAM A. The sound ROMs (68000 program, samples) are not loaded (D-8).')
    w('    -->')
    w('    <name>%s</name>' % s['name'])
    w('    <setname>%s</setname>' % s['setname'])
    w('    <rbf>Model3</rbf>')
    w('    <mameversion>0289</mameversion>')
    w('    <year>%s</year>' % s['year'])
    w('    <manufacturer>Sega</manufacturer>')
    w('    <category>%s</category>' % s.get('category', 'Fighter'))
    w('')
    w('    <rom index="0" zip="%s" md5="none">' % s['zip'])
    for b, bank in enumerate(s['crom']):
        w('        <!-- 0x%07x  CROM%d, 16 MiB (banks %d-%d) -->' % (b * 0x1000000, b, 2 * b, 2 * b + 1))
        if bank is None:
            w('        <part repeat="0x1000000">00</part>')
            continue
        w('        <interleave output="64">')
        for name, off in sorted(bank, key=lambda p: OFF_MAP[p[1]][::-1]):
            w('            <part name="%s" crc="%s" map="%s"/>' % (name, crc[name], OFF_MAP[off]))
        w('        </interleave>')
    for blk in range(4):
        w('        <!-- 0x%07x  VROM block %d, 8 MiB -->' % (0x4000000 + blk * 0x800000, blk))
        w('        <interleave output="64">')
        for k in range(4):
            name = s['vrom'][4 * blk + k]
            w('            <part name="%s" crc="%s" map="%s"/>' % (name, crc[name], SEQ_MAP[k]))
        w('        </interleave>')
    w('        <!-- 0x6000000  fixed CROM, 2 MiB (FFE00000-FFFFFFFF) -->')
    w('        <interleave output="64">')
    for name, off in sorted(s['fixed'], key=lambda p: OFF_MAP[p[1]][::-1]):
        w('            <part name="%s" crc="%s" map="%s"/>' % (name, crc[name], OFF_MAP[off]))
    w('        </interleave>')
    w('        <!-- 0x6200000  backup RAM, 128 KiB of 00 (Supermodel\'s fresh state; MAME fills FF) -->')
    w('        <part repeat="0x20000">00</part>')
    w('    </rom>')
    w('</misterromdescription>')
    with open(args.out, 'w') as f:
        f.write('\n'.join(L) + '\n')
    print('wrote', args.out)


def pack(args):
    root = ET.parse(args.mra).getroot()
    rom = [r for r in root.findall('rom') if r.get('index') == '0'][0]
    z = zipfile.ZipFile(args.zip)
    out = bytearray()
    for el in rom:
        if el.tag == 'part':
            if el.get('repeat'):
                n = int(el.get('repeat'), 0)
                out += bytes.fromhex(el.text.strip()) * n
            else:
                d = z.read(el.get('name'))
                if el.get('crc') and '%08x' % zlib.crc32(d) != el.get('crc'):
                    sys.exit('crc mismatch ' + el.get('name'))
                out += d
        elif el.tag == 'interleave':
            width = int(el.get('output')) // 8
            parts = []
            for p in el.findall('part'):
                d = z.read(p.get('name'))
                if p.get('crc') and '%08x' % zlib.crc32(d) != p.get('crc'):
                    sys.exit('crc mismatch ' + p.get('name'))
                m = p.get('map')
                assert len(m) == width, m
                parts.append((d, m))
            per = [max(int(c) for c in m) for _, m in parts]
            n = len(parts[0][0]) // per[0]
            blk = bytearray(n * width)
            for (d, m), k in zip(parts, per):
                assert len(d) // k == n
                for ob in range(width):
                    c = int(m[width - 1 - ob])
                    if c:
                        blk[ob::width] = d[c - 1::k]
            out += blk
    with open(args.out, 'wb') as f:
        f.write(out)
    print(args.out, len(out), 'bytes, md5', hashlib.md5(out).hexdigest())


def main():
    p = argparse.ArgumentParser()
    s = p.add_subparsers(dest='cmd', required=True)
    a = s.add_parser('mra'); a.add_argument('set'); a.add_argument('--zip', required=True); a.add_argument('--out', required=True)
    b = s.add_parser('pack'); b.add_argument('mra'); b.add_argument('--zip', required=True); b.add_argument('--out', required=True)
    x = p.parse_args()
    {'mra': mra, 'pack': pack}[x.cmd](x)


if __name__ == '__main__':
    main()
