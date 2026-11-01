#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Sega Model 3 core for MiSTer FPGA
# Copyright (C) 2026 alphanu1
#
# Stage B: the system bench's composed frames (tb_m3_system --ppm) against
# Supermodel's finished frames (supermodel-stageb-dump.patch, full_NNNNNN.bmp).
# The bench runs 23-36 frames ahead of Supermodel's numbering (sim/system/
# notes.md), and animation phases drift, so each bench frame is matched to the
# Supermodel frame nearest in content within a window. Reported per frame: the
# match, PSNR, pixels equal and within 9 a channel (Supermodel's GL path
# converts 5-bit colour its own way), and a side-by-side PNG (bench | Supermodel).
#
#   sb_grabcmp.py PPM_DIR GRAB_DIR OUT_DIR [--lo -60] [--hi 0]
import argparse, glob, os, re, subprocess, sys
import numpy as np


def ppm(fn):
    d = open(fn, 'rb').read()
    m = re.match(rb'P6\s+(\d+)\s+(\d+)\s+255\s', d)
    w, h = int(m.group(1)), int(m.group(2))
    return np.frombuffer(d[m.end():], np.uint8).reshape(h, w, 3).astype(np.int32)


def bmp(fn, cache):
    if fn not in cache:
        r = subprocess.run(['magick', '-define', 'bmp:ignore-filesize=true', fn, '-depth', '8', 'rgb:-'],
                           capture_output=True, check=True).stdout
        cache[fn] = np.frombuffer(r, np.uint8).reshape(384, 496, 3).astype(np.int32)
    return cache[fn]


def main():
    p = argparse.ArgumentParser()
    p.add_argument('ppm_dir'); p.add_argument('grab_dir'); p.add_argument('out_dir')
    p.add_argument('--lo', type=int, default=-60); p.add_argument('--hi', type=int, default=10)
    a = p.parse_args()
    os.makedirs(a.out_dir, exist_ok=True)
    cache = {}
    for fn in sorted(glob.glob(os.path.join(a.ppm_dir, 'f*.ppm'))):
        f = int(re.search(r'f(\d+)', os.path.basename(fn)).group(1))
        b = ppm(fn)
        best = None
        for g in range(f + a.lo, f + a.hi + 1):
            gf = os.path.join(a.grab_dir, 'full_%06d.bmp' % g)
            if not os.path.exists(gf):
                continue
            s = bmp(gf, cache)
            mse = float(((b - s) ** 2).mean())
            if best is None or mse < best[1]:
                best = (g, mse)
        if best is None:
            print(f'bench frame {f}: no grab in the window'); continue
        g, mse = best
        s = bmp(os.path.join(a.grab_dir, 'full_%06d.bmp' % g), cache)
        d = np.abs(b - s).max(axis=2)
        psnr = 10 * np.log10(255 ** 2 / mse) if mse > 0 else 99.0
        print(f'bench frame {f}: Supermodel frame {g}, PSNR {psnr:.2f} dB, equal {100 * (d == 0).mean():.2f}%, '
              f'within 9 {100 * (d <= 9).mean():.2f}%')
        img = np.concatenate([b, s], axis=1).astype(np.uint8)
        out = os.path.join(a.out_dir, 'cmp_%06d_sm%06d.png' % (f, g))
        subprocess.run(['magick', '-size', '992x384', '-depth', '8', 'rgb:-', out], input=img.tobytes(), check=True)


if __name__ == '__main__':
    sys.exit(main())
