#!/usr/bin/env python3
# cap2vec.py <sm|mame> <capture.log> <out.vec>
#
# Turns a 53C810 capture (tools/mame-census/*-scripts-log.patch) into replay
# vectors for tb_ref and tb_m3_scripts. One record a line, hex fields:
#   F frame               frame marker
#   P idx le              PCI configuration write: dword index, PCI (little-endian) value
#   p idx expect          PCI configuration read
#   L addr word           memory word (bus, big-endian) that the next access's run fetches
#   W off be data         register write: word offset, lanes (bit 3 = byte 0), bus word
#   R off be expect       register read, expected bus word (lanes not read are 0)
#   X dsp w0 w1 [w2]      an instruction the access before ran (PCI dwords, as the chip sees them)
#   M src dst len         a memory move it made
#   Q level               the interrupt line after that access
# Supermodel logs each access's width; MAME logs bytes, one time stamp a CPU access.
# The output holds RAM contents (the SCRIPTS words), so it lives in sm3-builds/.
import sys

def swap(x):
    return ((x & 0xFF) << 24) | ((x & 0xFF00) << 8) | ((x >> 8) & 0xFF00) | (x >> 24)

def parse(form, line):
    p = line.split()
    if len(p) < 3:
        return None
    fr, st, k = int(p[0]), p[1], p[2]
    if form == 'sm' and k[0] in 'RW' and k[1:].isdigit():
        return dict(k=k[0], fr=fr, st=st, reg=int(p[3], 16), val=int(p[4], 16), key=(st, k, p[5]))
    if form == 'mame' and k in ('R', 'W'):
        return dict(k=k, fr=fr, st=st, reg=int(p[3], 16), val=int(p[4], 16), key=(st, k, p[6]))
    return dict(k=k, fr=fr, st=st, p=p)

def main():
    form, src, dst = sys.argv[1], sys.argv[2], sys.argv[3]
    out = open(dst, 'w')
    st = dict(level=0, frame=-1)
    cur = dict(pre=[], acc=None, post=[])

    def emit():
        if cur['acc'] or cur['pre'] or cur['post']:
            for r in cur['pre']:
                out.write(r + '\n')
            if cur['acc']:
                k, base, be, data, _ = cur['acc']
                out.write('%s %02X %X %08X\n' % (k, base, be, data))
            for r in cur['post']:
                out.write(r + '\n')
            if cur['acc']:
                out.write('Q %d\n' % st['level'])
        cur['pre'], cur['acc'], cur['post'] = [], None, []

    nxt = None
    stream = open(src)
    def events():
        for raw in stream:
            e = parse(form, raw)
            if e:
                yield e
    it = events()
    e = next(it, None)
    while e is not None:
        nx = next(it, None)
        k = e['k']
        if k in ('R', 'W'):
            base, lane = e['reg'] & 0x7C, e['reg'] & 3
            bit, sh = 1 << (3 - lane), 8 * (3 - lane)
            a = cur['acc']
            if a and not cur['post'] and a[4] == e['key'] and a[0] == k and a[1] == base and not (a[2] & bit):
                a[2] |= bit; a[3] |= e['val'] << sh
            else:
                emit()
                if e['fr'] != st['frame']:
                    st['frame'] = e['fr']; out.write('F %X\n' % e['fr'])
                cur['acc'] = [k, base, bit, e['val'] << sh, e['key']]
        elif k == 'IRQ':
            lv = int(e['p'][3])
            if lv == 0 and nx and nx['k'] == 'R' and nx['st'] == e['st']:
                emit()            # released inside the read that follows: the access before keeps the old level
            st['level'] = lv
        elif k == 'X':
            p = e['p']; dsp = int(p[3], 16); w = [int(x, 16) for x in p[4:]]
            n = 3 if (w[0] >> 24) & 0xE0 == 0xC0 else 2
            for i in range(n):
                cur['pre'].append('L %08X %08X' % (dsp + 4 * i, swap(w[i])))
            cur['post'].append('X %08X ' % dsp + ' '.join('%08X' % x for x in w[:n]))
        elif k == 'M':
            p = e['p']; cur['post'].append('M %s %s %s' % (p[3], p[4], p[5]))
        elif k == 'PCIW':
            emit(); p = e['p']
            if form == 'mame':
                idx, le = int(p[3], 16), int(p[4], 16)
            else:
                le = swap(int(p[6], 16))
                # Supermodel folds dwords 4-7 into 4 and 0-3 into 0; the values say which
                idx = 5 if int(p[3], 16) == 4 else (3 if le == 0x0000FF00 else 1)
            out.write('P %X %08X\n' % (idx, le))
        elif k == 'PCIR':
            emit(); out.write('p 0 00011000\n')
        e = nx
    emit()
    out.close()

if __name__ == '__main__':
    main()
