#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Sega Model 3 core for MiSTer FPGA
# Copyright (C) 2026 alphanu1
#
# Bus traces: Supermodel's (supermodel-bus-trace.patch, M3BUS_TRACE) and the
# system bench's (tb_m3_system --trace) share one 24-byte record:
#   frame u32, pc u32, addr u32, kind u8 (0 read, 1 write, 2 event),
#   size u8 (bytes; for events 1 IRQ assert, 2 ping-pong flip, 3 frame start),
#   master u8 (0 CPU, 1 53C810), pad u8, data u64
#
#   bustrace.py summary TRACE [--frames A-B]
#   bustrace.py dump TRACE [--frames A-B] [--region NAME] [--limit N]
#   bustrace.py compare REF TEST [--frames A-B] [--region NAME] [--writes]
#
# compare lines up the two ordered streams of one master and region: writes
# exactly, reads with runs of identical (address, value) polls collapsed, since
# how many times a game polls depends on timing. It prints the first
# difference with context.
import argparse, struct, sys
from collections import Counter, defaultdict

REC = struct.Struct('<IIIBBBBQ')

REGIONS = [
    ('ram',     0x00000000, 0x00800000),
    ('r3dstat', 0x84000000, 0x84000040),
    ('r3dcmd',  0x88000000, 0x88000008),
    ('cull_lo', 0x8C000000, 0x8C400000),
    ('cull_hi', 0x8E000000, 0x8E100000),
    ('vromport',0x90000000, 0x91000000),
    ('texfifo', 0x94000000, 0x94100000),
    ('polyram', 0x98000000, 0x98400000),
    ('r3dcfg',  0x9C000000, 0x9D000000),
    ('scsi',    0xC0000000, 0xC0000100),
    ('r3ddma',  0xC2000000, 0xC2000100),
    ('io',      0xF0040000, 0xF0040040),
    ('snd',     0xF0080000, 0xF0090000),
    ('backup',  0xF00C0000, 0xF00E0000),
    ('sys',     0xF0100000, 0xF0100040),
    ('rtc',     0xF0140000, 0xF0140040),
    ('pciaddr', 0xF0800CF8, 0xF0800D00),
    ('pcidata', 0xF0C00CF8, 0xF0C00D00),
    ('vram',    0xF1000000, 0xF1100000),
    ('pal',     0xF1100000, 0xF1120000),
    ('tgreg',   0xF1180000, 0xF1180100),
    ('mpc105',  0xF8FFF000, 0xF8FFF100),
    ('crom',    0xFF000000, 0x100000000),
]


def region(a):
    for n, lo, hi in REGIONS:
        if lo <= a < hi:
            return n
    return 'other'


def load(fn, frames=None):
    out = []
    with open(fn, 'rb') as f:
        data = f.read()
    for i in range(0, len(data) - REC.size + 1, REC.size):
        r = REC.unpack_from(data, i)
        if frames and not (frames[0] <= r[0] <= frames[1]):
            continue
        out.append(r)
    return out


def rng(s):
    if not s:
        return None
    a, b = s.split('-')
    return (int(a), int(b))


def fmt(r):
    fr, pc, a, k, sz, m, _, d = r
    if k == 2:
        ev = {1: 'irq', 2: 'pingpong', 3: 'frame'}.get(sz, '?')
        return f'f{fr:5d} {"cpu" if m == 0 else "dma"} EVENT {ev} {a:x} {d:x}'
    return f'f{fr:5d} pc {pc:08x} {"cpu" if m == 0 else "dma"} {"W" if k else "R"}{sz} {a:08x} {region(a):8s} {d:0{sz*2}x}'


def summary(args):
    t = load(args.trace, rng(args.frames))
    c = Counter()
    first = {}
    addrs = defaultdict(set)
    for r in t:
        if r[3] == 2:
            c[('event', r[4])] += 1
            continue
        key = (region(r[2]), 'W' if r[3] else 'R', r[4], r[5])
        c[key] += 1
        addrs[key].add(r[2])
        first.setdefault(key, r)
    print(f'{len(t)} records, frames {t[0][0]}-{t[-1][0]}' if t else 'empty')
    for k in sorted(c, key=str):
        if k[0] == 'event':
            print(f'  event {k[1]}: {c[k]}')
        else:
            print(f'  {k[0]:9s} {k[1]}{k[2]} {"dma" if k[3] else "cpu"}: {c[k]:9d} accesses, {len(addrs[k]):7d} addresses, first {fmt(first[k])}')


def dump(args):
    n = 0
    for r in load(args.trace, rng(args.frames)):
        if args.region and (r[3] == 2 or region(r[2]) not in args.region.split(',')):
            continue
        print(fmt(r))
        n += 1
        if args.limit and n >= args.limit:
            break


def stream(t, args):
    regs = args.region.split(',') if args.region else None
    out = []
    for r in t:
        if r[3] == 2:
            continue
        if regs and region(r[2]) not in regs:
            continue
        if args.exclude and region(r[2]) in args.exclude.split(','):
            continue
        if args.writes and r[3] == 0:
            continue
        if args.master is not None and r[5] != args.master:
            continue
        key = (r[3], r[4], r[2], r[7])
        if r[3] == 0 and out and out[-1][0] == key:
            continue        # a repeated poll
        out.append((key, r))
    return out


def compare(args):
    a = stream(load(args.ref, rng(args.frames)), args)
    b = stream(load(args.test, rng(args.tframes) if args.tframes else None), args)
    n = min(len(a), len(b))
    for i in range(n):
        if a[i][0] != b[i][0]:
            print(f'first difference at record {i} of {len(a)} / {len(b)}')
            for j in range(max(0, i - 8), min(n, i + 8)):
                mark = '>>' if j == i else '  '
                print(f'{mark} ref  {fmt(a[j][1])}')
                print(f'{mark} test {fmt(b[j][1])}')
            return 1
    print(f'identical over {n} records (ref {len(a)}, test {len(b)})')
    return 0


def regions(args):
    # every region's own ordered stream, one pass over each trace: interrupt
    # timing reorders accesses between regions, not within one
    def split(fn, frames):
        out = defaultdict(list)
        for r in load(fn, frames):
            if r[3] == 2:
                continue
            g = region(r[2]) + ('/dma' if r[5] else '')
            key = (r[3], r[4], r[2], r[7])
            lst = out[g]
            if r[3] == 0 and lst and lst[-1][0] == key:
                continue
            lst.append((key, r))
        return out
    a = split(args.ref, rng(args.frames))
    b = split(args.test, rng(args.tframes) if args.tframes else None)
    bad = 0
    for g in sorted(set(a) | set(b)):
        x, y = a.get(g, []), b.get(g, [])
        n = min(len(x), len(y))
        d = next((i for i in range(n) if x[i][0] != y[i][0]), None)
        if d is None:
            print(f'{g:14s} identical over {n} (ref {len(x)}, test {len(y)})')
        else:
            bad += 1
            print(f'{g:14s} DIFFER at {d} of {n}: ref {fmt(x[d][1])} | test {fmt(y[d][1])}')
    return 1 if bad else 0


def main():
    p = argparse.ArgumentParser()
    s = p.add_subparsers(dest='cmd', required=True)
    x = s.add_parser('summary'); x.add_argument('trace'); x.add_argument('--frames')
    x = s.add_parser('dump'); x.add_argument('trace'); x.add_argument('--frames'); x.add_argument('--region'); x.add_argument('--limit', type=int)
    x = s.add_parser('compare'); x.add_argument('ref'); x.add_argument('test'); x.add_argument('--frames'); x.add_argument('--tframes')
    x.add_argument('--region'); x.add_argument('--exclude'); x.add_argument('--writes', action='store_true'); x.add_argument('--master', type=int)
    x = s.add_parser('regions'); x.add_argument('ref'); x.add_argument('test'); x.add_argument('--frames'); x.add_argument('--tframes')
    a = p.parse_args()
    sys.exit({'summary': summary, 'dump': dump, 'compare': compare, 'regions': regions}[a.cmd](a) or 0)


if __name__ == '__main__':
    main()
