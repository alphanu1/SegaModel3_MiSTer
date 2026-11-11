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
#   m3_mra.py nvram SUPERMODEL.nv --out NAME.nvm          (the <nvram> file; never committed)
#
# pack writes index 0's image to --out and every other ROM index to
# OUT.iN.bin (1 the extra memory, 3 the fixed CROM, 4 the game configuration).
#
# Step 2.1 sets (daytona2; study R100) use several ioctl indexes: 4 the game
# configuration (step, flags, CROM bank map), 1 VROM in the extra memory
# (DDR3), 3 the 8 MiB fixed CROM (SDRAM 0x7300000), then 0 the banked CROM,
# and the <nvram> (index 2: backup RAM, then the 93C46). m3_loader.sv routes
# them.
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
import argparse, hashlib, struct, sys, zipfile, zlib
import xml.etree.ElementTree as ET

# the input profiles (configuration byte 20; rtl/io/m3_inputs.sv) and each one's button order, which the MRA's
# <buttons names> follows: 0 generic (VF3), 1 driving (m3_cab_inputs; Daytona's order on Model 2), 2 analog
# guns, 3 fishing
P_GEN, P_DRIVE, P_GUN, P_FISH = 0, 1, 2, 3
BTN_DRIVE = 'Coin,Start,Test,Service,VR1 Red,VR4 Green,Accel,Brake,Gear Up,Gear Down,VR2 Blue,VR3 Yellow'
PROF_NAME = {P_GEN: 'generic', P_DRIVE: 'driving', P_GUN: 'analog guns', P_FISH: 'fishing'}

# MAME 0.289 model3.cpp ROM_START(vf3), (name, ROM_LOAD64_WORD_SWAP offset)
VF3 = {
    'setname': 'vf3', 'name': 'Virtua Fighter 3 (Japan, Revision D)', 'year': '1996', 'zip': 'vf3.zip',
    'buttons': 'Punch,Kick,Guard,Escape,Start,Coin,Test,Service',
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
    'category': 'Sports', 'prof': P_FISH,
    'buttons': 'Cast,Select,Reel,Start,Coin,Test,Service',
    'crom': [
        [('mpr-20256.1', 6), ('mpr-20257.2', 4), ('mpr-20258.3', 2), ('mpr-20259.4', 0)],
        [('mpr-20260.5', 6), ('mpr-20261.6', 4), ('mpr-20262.7', 2), ('mpr-20263.8', 0)],
        [('mpr-20264.9', 6), ('mpr-20265.10', 4), ('mpr-20266.11', 2), ('mpr-20267.12', 0)],
        None,
    ],
    'fixed': [('epr-20783.17', 6), ('epr-20784.18', 4), ('epr-20785.19', 2), ('epr-20786.20', 0)],
    'vrom': ['mpr-%d.%d' % (20270 + k, 26 + k) for k in range(16)],
}
# ROM_START(daytona2) (Step 2.1, study R100): CROM0 and CROM1 are 8 MiB ROMs (32 MiB,
# four banks each), CROM2 and CROM3 4 MiB ROMs (16 MiB, two banks each); the fixed
# CROM is four 2 MiB ROMs filling FF800000-FFFFFFFF; VROM sixteen 4 MiB ROMs (64 MiB).
DAYTONA2 = {
    'setname': 'daytona2', 'name': 'Daytona USA 2 - Battle on the Edge (Japan, Revision A)', 'year': '1998',
    'zip': 'daytona2.zip', 'category': 'Driving', 'step': 0x21, 'prof': P_DRIVE, 'buttons': BTN_DRIVE,
    'idle_in': [0xAF, 0xFF],   # ports C, D at rest: the 4-speed shifter in first gear (Supermodel's default)
    'crom': [
        [('mpr-20845.1', 6), ('mpr-20846.2', 4), ('mpr-20847.3', 2), ('mpr-20848.4', 0)],
        [('mpr-20849.5', 6), ('mpr-20850.6', 4), ('mpr-20851.7', 2), ('mpr-20852.8', 0)],
        [('mpr-20853.9', 6), ('mpr-20854.10', 4), ('mpr-20855.11', 2), ('mpr-20856.12', 0)],
        [('mpr-20857.13', 6), ('mpr-20858.14', 4), ('mpr-20859.15', 2), ('mpr-20860.16', 0)],
    ],
    'crom_mib': [32, 32, 16, 16],
    'fixed': [('epr-20861a.17', 6), ('epr-20862a.18', 4), ('epr-20863a.19', 2), ('epr-20864a.20', 0)],
    'vrom': ['mpr-%d.%d' % (20870 + k, 26 + k) for k in range(16)],
}
# ROM_START(scuddxo) (Step 1.5, study D-46): VF3's layout, three CROM banks (CROM3 unpopulated,
# 00 as MAME's region), sixteen 2 MiB VROMs. The step goes in an index-4 configuration; the
# rest of it is the Step 1.0 default (banks 0-7 in slots 0-7). Port C rests at AF as on
# Daytona 2 (the 4-speed shifter in first gear; Supermodel's trace of scuddxo reads AF).
SCUDDXO = {
    'setname': 'scuddxo', 'name': 'Scud Race - Sega Super GT - Deluxe', 'year': '1996', 'zip': 'scuddxo.zip',
    'category': 'Driving', 'step': 0x15, 'idle_in': [0xAF, 0xFF], 'prof': P_DRIVE, 'buttons': BTN_DRIVE,
    'crom': [
        [('mpr-19592.1', 6), ('mpr-19591.2', 4), ('mpr-19590.3', 2), ('mpr-19589.4', 0)],
        [('mpr-19596.5', 6), ('mpr-19595.6', 4), ('mpr-19594.7', 2), ('mpr-19593.8', 0)],
        [('mpr-19600.9', 6), ('mpr-19599.10', 4), ('mpr-19598.11', 2), ('mpr-19597.12', 0)],
        None,
    ],
    'fixed': [('epr-19610.17', 6), ('epr-19609.18', 4), ('epr-19608.19', 2), ('epr-19607.20', 0)],
    # by IC position, .26 to .41, as VF3's (MAME offsets 2, 0, 6, 4, ...): the part numbers run in swapped pairs
    'vrom': ['mpr-19574.26', 'mpr-19573.27', 'mpr-19576.28', 'mpr-19575.29', 'mpr-19578.30', 'mpr-19577.31',
             'mpr-19580.32', 'mpr-19579.33', 'mpr-19582.34', 'mpr-19581.35', 'mpr-19584.36', 'mpr-19583.37',
             'mpr-19586.38', 'mpr-19585.39', 'mpr-19588.40', 'mpr-19587.41'],
}
# ROM_START(lostwsga) (Step 1.5): VF3's layout with all four CROMs (64 MiB, banks 0-7) and sixteen 2 MiB
# VROMs in IC order. The game selects banks 8-15 only (F0100008 written F0-F7 in Supermodel's trace),
# which both emulators mirror onto 0-7 (MAME's ROM_COPY into the upper 64 MiB, Supermodel's banked CROM
# copied twice): the bank map puts banks 8-15 in slots 0-7 too. Ports C and D read FF (Supermodel).
# Its DIP bank defaults to FF, all eight OFF (MAME's default): the game reads SW1 at start-up and leaves
# its I-cache off when it is ON, as Supermodel's 7F has it (study R133).
# Not loaded: the billboard's Z80 ROM (epr-18022.ic2), the 68000 program and the samples (D-8).
LOSTWSGA = {
    'setname': 'lostwsga', 'name': 'The Lost World - Jurassic Park (Revision A)', 'year': '1997', 'zip': 'lostwsga.zip',
    'category': 'Shooter', 'step': 0x15, 'crom_mirror': True, 'dsw': 0xFF, 'prof': P_GUN,
    'buttons': 'Trigger,Trigger 2,Start,Coin,Test,Service',
    'crom': [
        [('mpr-19918.1', 6), ('mpr-19919.2', 4), ('mpr-19920.3', 2), ('mpr-19921.4', 0)],
        [('mpr-19922.5', 6), ('mpr-19923.6', 4), ('mpr-19924.7', 2), ('mpr-19925.8', 0)],
        [('mpr-19926.9', 6), ('mpr-19927.10', 4), ('mpr-19928.11', 2), ('mpr-19929.12', 0)],
        [('mpr-19930.13', 6), ('mpr-19931.14', 4), ('mpr-19932.15', 2), ('mpr-19933.16', 0)],
    ],
    'fixed': [('epr-19939a.17', 6), ('epr-19938a.18', 4), ('epr-19937a.19', 2), ('epr-19936a.20', 0)],
    'vrom': ['mpr-%d.%d' % (19902 + k, 26 + k) for k in range(16)],
}
# ROM_START(srally2) (Step 2.0): Daytona 2's form (8 MiB fixed CROM of four 2 MiB ROMs, VROM sixteen 4 MiB ROMs,
# 64 MiB) but the banked CROM is three CROMs of four 4 MiB ROMs, 16 MiB each, with no CROM3: 48 MiB. Both emulators
# lay them out back to back (MAME's 0x800000 + bank x 8 MiB from CROM0 at 0x800000, CROM1 at 0x1800000; Supermodel's
# banked_crom offsets 0, 0x1000000, 0x2000000), so bank b is 8 MiB slot b, banks 0-5; Supermodel's trace selects
# banks 0, 1, 2, 4 and 5 (F0100008 written FF, FE, FD, FB, FA). No security board (no 315-5881 access in the trace).
# Port C: bit 0 the view button, bit 1 the hand brake (Supermodel's INPUT_VIEWCHANGE and INPUT_HANDBRAKE), 7:4 the
# 4-speed H; it rests at AF in Supermodel's trace, port D at FF. In the driving profile those are VR1 and VR2.
# Supermodel reaches the attract from a fresh NVRAM, so the <nvram> keeps settings and records only.
# Not loaded: the 68000 program and samples, the DSB2's 68000 program and MPEG ROMs (arcsnd reads them), the drive board.
SRALLY2 = {
    'setname': 'srally2', 'name': 'Sega Rally 2 - Sega Rally Championship', 'year': '1998', 'zip': 'srally2.zip',
    'category': 'Driving', 'step': 0x20, 'prof': P_DRIVE,
    'buttons': 'Coin,Start,Test,Service,View,-,Accel,Brake,Gear Up,Gear Down,Hand Brake',
    'idle_in': [0xAF, 0xFF],
    'crom': [
        [('mpr-20602.1', 6), ('mpr-20603.2', 4), ('mpr-20604.3', 2), ('mpr-20605.4', 0)],
        [('mpr-20606.5', 6), ('mpr-20607.6', 4), ('mpr-20608.7', 2), ('mpr-20609.8', 0)],
        [('mpr-20610.9', 6), ('mpr-20611.10', 4), ('mpr-20612.11', 2), ('mpr-20613.12', 0)],
    ],
    'crom_mib': [16, 16, 16],
    'crom_win': 16,             # MiB of bank space a CROM takes: two banks (Daytona 2's 32: four)
    'fixed': [('epr-20632.17', 6), ('epr-20633.18', 4), ('epr-20634.19', 2), ('epr-20635.20', 0)],
    'vrom': ['mpr-%d.%d' % (20616 + k, 26 + k) for k in range(16)],
    'nvnote': ['the 93C46. Supermodel reaches the attract from a fresh NVRAM;',
               'the file keeps the settings and records. m3_mra.py nvram converts a'],
}
SETS = {'vf3': VF3, 'bassdx': BASSDX, 'daytona2': DAYTONA2, 'scuddxo': SCUDDXO, 'lostwsga': LOSTWSGA, 'srally2': SRALLY2}

# game configuration (index 4) flags, m3_loader.sv
CF_VROM_X, CF_FIXED8, CF_VROM64 = 1, 2, 4
SDR_FIX8 = 0x7300000
XMEM_DDR3_BYTE = 0x38000000    # the DDR3 region for <rom address=...> (not used: the stream is the default)


def bank_map(crom_mib, win=32):
    """CROM bank b (0-15; CROMn is banks wn..wn+w-1, w = win / 8: four for Daytona 2's 32 MiB CROM windows, two
    for Sega Rally 2's 16 MiB) -> SDRAM slot, packed in order; 0x80 empty."""
    m, slot = [], 0
    for n, mib in enumerate(crom_mib):
        for k in range(win // 8):
            if k * 8 < mib:
                m.append(slot); slot += 1
            else:
                m.append(0x80)
    m += [0x80] * (16 - len(m))
    assert len(m) == 16
    return m
# ROM_LOAD64_WORD_SWAP offset -> the output bytes of its pair in word form
OFF_MAP = {2: '00000021', 0: '00002100', 6: '00210000', 4: '21000000'}
SEQ_MAP = ['00000021', '00002100', '00210000', '21000000']


def mra(args):
    s = SETS[args.set]
    z = zipfile.ZipFile(args.zip)
    crc = {i.filename: '%08x' % i.CRC for i in z.infolist()}
    if s.get('step', 0x10) >= 0x20:
        return mra_step2(args, s, crc)
    L = []
    w = L.append
    w('<misterromdescription>')
    w('    <!--')
    w('      Sega Model 3 core for MiSTer FPGA - Copyright (C) 2026 alphanu1')
    w('      SPDX-License-Identifier: GPL-3.0-or-later')
    w('')
    step = s.get('step', 0x10)
    if step == 0x10:
        w('      %s. Stage A of study D-37: CPU, bus, tile generator, no sound,' % s['name'])
        w('      no 3D. From MAME 0.289 model3.cpp ROM_START(%s); sim/system/m3_mra.py' % s['setname'])
        w('      writes it, and packs the same image for the system bench.')
    else:
        w('      %s. Step %X.%X (study D-46): no sound. From MAME 0.289' % (s['name'], step >> 4, step & 15))
        w('      model3.cpp ROM_START(%s); sim/system/m3_mra.py' % s['setname'])
        w('      writes it, and packs the same image for the system bench. Index 4, the')
        w('      game configuration (rtl/sys/m3_loader.sv), gives the step and the input profile; the rest of it')
        if s.get('crom_mirror'):
            w('      is the Step 1.0 default but for CROM banks 8-15, which mirror 0-7, and')
            w('      index 0 is laid out as VF3\'s.')
        else:
            w('      is the Step 1.0 default, and index 0 is laid out as VF3\'s.')
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
    switches(w, s)
    buttons(w, s)
    w('')
    if step != 0x10 or s.get('prof', P_GEN) != P_GEN:
        mirror = s.get('crom_mirror', False)
        cfg = [step, 0x00] + [b & 7 if mirror else b if b < 8 else 0x80 for b in range(16)] + s.get('idle_in', [0xFF, 0xFF]) \
              + [s.get('prof', P_GEN), s.get('pflg', 0)]
        w('    <!-- game configuration: step %02X, flags 00, banks 0-7 %sin slots 0-7, idle ports C and D, input profile %02X (%s) -->'
          % (step, 'and their mirror 8-15 ' if mirror else '', cfg[20], PROF_NAME[cfg[20]]))
        w('    <rom index="4" md5="none">')
        w('        <part>%s</part>' % ' '.join('%02X' % b for b in cfg))
        w('    </rom>')
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



def switches(w, s):
    """The CPU board's DIP bank (315-5649 port G) as MiSTer's DIP Switches page: index 254, byte 0, bit 7
    SW1 ... bit 0 SW8 (MAME's DIP locations), 0 = ON. The default is the game's 'dsw', else 7F
    (Supermodel's, SW1 ON); MAME's is FF. SW1 ON is a debug setting (MAME: vs215o's debug stats; Lost World
    leaves its I-cache off), the other seven have no known use (study R133)."""
    w('    <!-- the CPU board\'s DIP bank, 315-5649 port G (index 254); Reset & apply after a change -->')
    w('    <switches default="%02X">' % s.get('dsw', 0x7F))
    for n in range(1, 9):
        w('        <dip name="SW%d%s" bits="%d" ids="On,Off"/>' % (n, ' (debug)' if n == 1 else '', 8 - n))
    w('    </switches>')

def buttons(w, s):
    """The MiSTer buttons in the input profile's order (rtl/io/m3_inputs.sv); the joystick's directions are 3:0."""
    if 'buttons' in s:
        w('    <!-- the controls: input profile %02X (%s), rtl/io/m3_inputs.sv -->' % (s.get('prof', P_GEN), PROF_NAME[s.get('prof', P_GEN)]))
        w('    <buttons names="%s"/>' % s['buttons'])

def mra_step2(args, s, crc):
    L = []
    w = L.append
    nb = sum(s['crom_mib']) // 8
    bm = bank_map(s['crom_mib'], s.get('crom_win', 32))
    cfg = [s['step'], CF_VROM_X | CF_FIXED8 | CF_VROM64] + bm + s.get('idle_in', [0xFF, 0xFF]) + [s.get('prof', P_GEN), s.get('pflg', 0)]
    w('<misterromdescription>')
    w('    <!--')
    w('      Sega Model 3 core for MiSTer FPGA - Copyright (C) 2026 alphanu1')
    w('      SPDX-License-Identifier: GPL-3.0-or-later')
    w('')
    w('      %s. Step %X.%X (study R100): no sound.' % (s['name'], s['step'] >> 4, s['step'] & 15))
    w('      From MAME 0.289 model3.cpp ROM_START(%s); sim/system/m3_mra.py writes it,' % s['setname'])
    w('      and packs the same images for the system bench. One download per index')
    w('      (rtl/sys/m3_loader.sv):')
    w('        4  the game configuration: step, flags, CROM bank map, idle inputs, input profile')
    w('        1  VROM, 64 MiB, in the extra memory (DDR3 byte 0x38000000)')
    w('        3  fixed CROM, 8 MiB, SDRAM 0x7300000 (FF800000-FFFFFFFF)')
    w('        0  banked CROM, %d MiB in %d slots of 8 MiB from SDRAM 0; backup RAM' % (8 * nb, nb))
    w('           at 0x6200000, 128 KiB of 00; last, so its end releases the reset')
    w('        2  <nvram>: config/nvram/<this file>.nvm, if present: backup RAM, then')
    for t in s.get('nvnote', ['the 93C46. The game needs its link setting at Single, or it stops',
                              'at "NETWORK BOARD NOT PRESENT"; m3_mra.py nvram converts a']):
        w('           ' + t)
    w('           Supermodel NVRAM file. No NVRAM data is in this repository.')
    w('      Word form: the PowerPC\'s 32-bit word at A is the little-endian word at')
    w('      the memory\'s A. The sound ROMs are not loaded (D-8).')
    w('    -->')
    w('    <name>%s</name>' % s['name'])
    w('    <setname>%s</setname>' % s['setname'])
    w('    <rbf>Model3</rbf>')
    w('    <mameversion>0289</mameversion>')
    w('    <year>%s</year>' % s['year'])
    w('    <manufacturer>Sega</manufacturer>')
    w('    <category>%s</category>' % s.get('category', 'Fighter'))
    switches(w, s)
    buttons(w, s)
    w('')
    w('    <!-- game configuration: step %02X, flags %02X (VROM in the extra memory, fixed CROM 8 MiB, VROM 64 MiB), bank map, idle ports C and D, input profile %02X (%s) -->' % (cfg[0], cfg[1], cfg[20], PROF_NAME[cfg[20]]))
    w('    <rom index="4" md5="none">')
    w('        <part>%s</part>' % ' '.join('%02X' % b for b in cfg))
    w('    </rom>')
    w('    <rom index="1" zip="%s" md5="none">' % s['zip'])
    for blk in range(4):
        w('        <!-- 0x%07x  VROM block %d, 16 MiB -->' % (blk * 0x1000000, blk))
        w('        <interleave output="64">')
        for k in range(4):
            name = s['vrom'][4 * blk + k]
            w('            <part name="%s" crc="%s" map="%s"/>' % (name, crc[name], SEQ_MAP[k]))
        w('        </interleave>')
    w('    </rom>')
    w('    <rom index="3" zip="%s" md5="none">' % s['zip'])
    w('        <!-- fixed CROM, 8 MiB (FF800000-FFFFFFFF) -->')
    w('        <interleave output="64">')
    for name, off in sorted(s['fixed'], key=lambda p: OFF_MAP[p[1]][::-1]):
        w('            <part name="%s" crc="%s" map="%s"/>' % (name, crc[name], OFF_MAP[off]))
    w('        </interleave>')
    w('    </rom>')
    w('    <rom index="0" zip="%s" md5="none">' % s['zip'])
    a = 0
    bw = s.get('crom_win', 32) // 8
    for n, bank in enumerate(s['crom']):
        mib = s['crom_mib'][n]
        w('        <!-- 0x%07x  CROM%d, %d MiB (banks %s) -->' % (a, n, mib, ', '.join(str(b) for b in range(bw * n, bw * n + mib // 8))))
        w('        <interleave output="64">')
        for name, off in sorted(bank, key=lambda p: OFF_MAP[p[1]][::-1]):
            w('            <part name="%s" crc="%s" map="%s"/>' % (name, crc[name], OFF_MAP[off]))
        w('        </interleave>')
        a += mib << 20
    if a < 0x6000000:
        w('        <!-- 0x%07x  slots %d-11 unused, %d MiB of 00, so the backup RAM lands at 0x6200000 -->' % (a, a >> 23, (0x6000000 - a) >> 20))
        w('        <part repeat="0x%x">00</part>' % (0x6000000 - a))
        a = 0x6000000
    assert a == 0x6000000
    w('        <!-- 0x6000000  (the Step 1.0 fixed CROM\'s place, unused here), 2 MiB of 00 -->')
    w('        <part repeat="0x200000">00</part>')
    w('        <!-- 0x6200000  backup RAM, 128 KiB of 00 (Supermodel\'s fresh state); the NVRAM file overwrites it -->')
    w('        <part repeat="0x20000">00</part>')
    w('    </rom>')
    w('    <nvram index="2" size="%d"/>' % (0x20000 + 128))
    w('</misterromdescription>')
    with open(args.out, 'w') as f:
        f.write('\n'.join(L) + '\n')
    print('wrote', args.out)


def pack_rom(rom, z):
    out = bytearray()
    for el in rom:
        if el.tag == 'part':
            if el.get('repeat'):
                n = int(el.get('repeat'), 0)
                out += bytes.fromhex(el.text.strip()) * n
            elif el.get('name') is None:
                out += bytes.fromhex(el.text.strip())
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
    return out


def pack(args):
    root = ET.parse(args.mra).getroot()
    z = zipfile.ZipFile(args.zip)
    for rom in root.findall('rom'):
        ix = int(rom.get('index'))
        out = pack_rom(rom, z)
        fn = args.out if ix == 0 else '%s.i%d.bin' % (args.out[:-4] if args.out.endswith('.bin') else args.out, ix)
        with open(fn, 'wb') as f:
            f.write(out)
        print('index %d:' % ix, fn, len(out), 'bytes, md5', hashlib.md5(out).hexdigest())
    # <switches>: Main's 8 bytes (its 64-bit dip_cur, little-endian) at their default, as index 254
    sw = root.find('switches')
    if sw is not None:
        out = bytes.fromhex(sw.get('default', '').replace(',', '')).ljust(8, b'\0')[:8]
        fn = '%s.i254.bin' % (args.out[:-4] if args.out.endswith('.bin') else args.out)
        with open(fn, 'wb') as f:
            f.write(out)
        print('index 254:', fn, 'switches', out.hex())


def nvram(args):
    """Supermodel's NVRAM file (CBlockFile: blocks of {u32 length, u32 name length, u32 comment length,
    name, comment, data}) -> the core's <nvram> file: the backup RAM (128 KiB, Supermodel's byte order is
    the core's word form) and then the 93C46's 64 words, little-endian."""
    d = open(args.nv, 'rb').read()
    blocks, o = {}, 0
    while o + 12 <= len(d):
        ln, nl, cl = struct.unpack_from('<III', d, o)
        if ln < 12 + nl + cl or o + ln > len(d):
            break
        name = d[o + 12:o + 12 + nl].rstrip(b'\0').decode('latin-1')
        blocks[name] = d[o + 12 + nl + cl:o + ln]
        o += ln
    if 'Backup RAM' not in blocks or '93C46' not in blocks:
        sys.exit('not a Supermodel NVRAM file (blocks: %s)' % ', '.join(blocks))
    out = blocks['Backup RAM'][:0x20000] + blocks['93C46'][:128]
    assert len(out) == 0x20000 + 128
    with open(args.out, 'wb') as f:
        f.write(out)
    print(args.out, len(out), 'bytes; EEPROM starts', out[0x20000:0x20008].hex())


def main():
    p = argparse.ArgumentParser()
    s = p.add_subparsers(dest='cmd', required=True)
    a = s.add_parser('mra'); a.add_argument('set'); a.add_argument('--zip', required=True); a.add_argument('--out', required=True)
    b = s.add_parser('pack'); b.add_argument('mra'); b.add_argument('--zip', required=True); b.add_argument('--out', required=True)
    c = s.add_parser('nvram'); c.add_argument('nv'); c.add_argument('--out', required=True)
    x = p.parse_args()
    {'mra': mra, 'pack': pack, 'nvram': nvram}[x.cmd](x)


if __name__ == '__main__':
    main()
