#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Sega Model 3 core for MiSTer FPGA
# Copyright (C) 2026 alphanu1
#
# ---------------------------------------------------------------------------
#
# The frsqrte estimate's table (study D-22): writes the same integers into
# rtl/cpu/ppc_fpu_rsqe.sv and sim/fpu/ppc_fpu_ref.h, between their
# "BEGIN rsqe table" / "END rsqe table" markers, and prints the worst
# relative error of the estimate over every input.
#
# The estimate. frB = m * 2^e, m in [1, 2) (a denormal normalised first);
# x = m (e even) or 2m (e odd), so 1/sqrt(frB) = y * 2^-floor(e/2) with
# y = 1/sqrt(x) in (0.5, 1]. The index is the parity of e and the top K bits
# of m's fraction; the next J bits are dx. Each entry is a line over its
# segment: y_fix = B * 2^10 - S * dx in units of 2^-32, B in units of 2^-22,
# S per segment in units of 2^-20. The result is 2y truncated to 23 fraction
# bits, so its significand fits a single.
#
#   gen_rsqe_tab.py <rtl file> <model file>      (rewrite both)
#   gen_rsqe_tab.py                              (only print the error)

import math
import re
import sys

K, J = 6, 12          # index bits of the fraction, interpolation bits
BB, SB = 22, 20       # B, S scales (fraction bits)
YB = 32               # y_fix scale


def f(x):
    return 1.0 / math.sqrt(x)


def build():
    tab = []
    for p in (0, 1):
        for i in range(1 << K):
            x0 = (1 + i / (1 << K)) * (1 + p)
            x1 = (1 + (i + 1) / (1 << K)) * (1 + p)
            s = f(x0) - f(x1)                      # the drop over the segment
            # the point where f's slope equals the chord's: f'(x) = -x^-1.5 / 2
            xs = ((x1 - x0) / s / 2) ** (2.0 / 3.0)
            dev = f(xs) - (f(x0) - s * (xs - x0) / (x1 - x0))   # below the chord, < 0
            base = f(x0) + dev / 2                 # minimax line
            base -= s / (2 << J)                   # dx is truncated: centre each cell
            bq = round(base * (1 << BB))
            sq = round(s * (1 << SB))
            # y stays at or above 0.5 at the segment's end (x near 4), so the
            # result's exponent never moves
            while y_fix(bq, sq, (1 << J) - 1) < (1 << 31):
                bq += 1
            tab.append((bq, sq))
    return tab


def y_fix(bq, sq, dx):
    return (bq << (YB - BB)) - sq * dx


def worst(tab):
    w = 0.0
    for p in (0, 1):
        for i in range(1 << K):
            bq, sq = tab[p * (1 << K) + i]
            for dx in range(1 << J):
                y = y_fix(bq, sq, dx)
                assert (1 << 31) <= y < (1 << 32), (p, i, dx, y)
                frac23 = (y >> 8) & ((1 << 23) - 1)
                est = (1 + frac23 / (1 << 23)) / 2          # y truncated as delivered
                lo = (1 + (i + dx / (1 << J)) / (1 << K)) * (1 + p)
                hi = (1 + (i + (dx + 1) / (1 << J)) / (1 << K)) * (1 + p)
                for x in (lo, hi):
                    w = max(w, abs(est * math.sqrt(x) - 1))
    return w


def splice(path, body):
    s = open(path).read()
    pat = re.compile(r"(// BEGIN rsqe table[^\n]*\n)(.*?)(\s*// END rsqe table)", re.S)
    assert pat.search(s), path
    s = pat.sub(lambda m: m.group(1) + body + m.group(3), s)
    open(path, "w").write(s)


def main():
    tab = build()
    w = worst(tab)
    bmax = max(b for b, _ in tab).bit_length()
    smax = max(s for _, s in tab).bit_length()
    print("entries %d, B %d bits, S %d bits, worst relative error %.4g = 2^%.2f (bound 1/32)"
          % (len(tab), bmax, smax, w, math.log2(w)))
    if len(sys.argv) < 3:
        return
    sv = "".join("      7'd%-3d: t = {22'd%d, 13'd%d};\n" % (k, b, s) for k, (b, s) in enumerate(tab))
    splice(sys.argv[1], sv + "      default: t = '0;")
    cc = "".join("  {%du, %du},\n" % (b, s) for b, s in tab)
    splice(sys.argv[2], cc.rstrip("\n"))


if __name__ == "__main__":
    main()
