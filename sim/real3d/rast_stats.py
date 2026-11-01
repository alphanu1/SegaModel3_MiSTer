#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
#
# rast_stats.py LOG...: the whole-rasteriser bench's FRAME lines pooled, p50 /
# p99 / max of the replay and binning cycles (share of a 100 MHz, 57.52 Hz
# frame), the store's DDR3 bytes a frame (binning writes + replay reads) as
# MB/s at 57.52 Hz, texture lines, the layer's bytes (FRAME lwr, when the
# bench prints it), and the wrong pixels.
import sys
F = 1738530.0
rows = []
for fn in sys.argv[1:]:
    for l in open(fn):
        if l.startswith("FRAME "):
            w = l.split()
            d = {w[i]: int(w[i + 1]) for i in range(2, len(w) - 1, 2)}
            rows.append(d)
def pc(v, p):
    v = sorted(v)
    return v[-1] if p >= 100 else v[min(len(v) - 1, len(v) * p // 100)]
print("%d frames, %d shaded pixels, %d wrong" % (len(rows), sum(r["shaded"] for r in rows), sum(r["wrong"] for r in rows)))
for k, nm in (("replay", "replay cycles"), ("bin", "binning cycles"), ("eobw", "end-of-band drain")):
    v = [r[k] for r in rows]
    print("  %-18s p50 %9d (%5.1f%%)  p99 %9d (%5.1f%%)  max %9d (%5.1f%%)" % (nm, pc(v, 50), 100 * pc(v, 50) / F, pc(v, 99), 100 * pc(v, 99) / F, pc(v, 100), 100 * pc(v, 100) / F))
s = [r["swr"] + r["srd"] for r in rows]
print("  store bytes a frame p50 %d p99 %d max %d: %.1f / %.1f / %.1f MB/s at 57.52 Hz (writes p99 %.1f, reads p99 %.1f)" % (
    pc(s, 50), pc(s, 99), pc(s, 100), pc(s, 50) * 57.52e-6, pc(s, 99) * 57.52e-6, pc(s, 100) * 57.52e-6,
    pc([r["swr"] for r in rows], 99) * 57.52e-6, pc([r["srd"] for r in rows], 99) * 57.52e-6))
t = [r["tex"] for r in rows]
print("  texture lines a frame p50 %d p99 %d max %d: %.1f / %.1f / %.1f MB/s at 57.52 Hz" % (
    pc(t, 50), pc(t, 99), pc(t, 100), pc(t, 50) * 32 * 57.52e-6, pc(t, 99) * 32 * 57.52e-6, pc(t, 100) * 32 * 57.52e-6))
if rows and "lwr" in rows[0]:
    l = [r["lwr"] for r in rows]
    print("  layer bytes a frame p50 %d p99 %d: %.1f / %.1f MB/s at 57.52 Hz" % (pc(l, 50), pc(l, 99), pc(l, 50) * 57.52e-6, pc(l, 99) * 57.52e-6))
    a = [r["swr"] + r["srd"] + r["tex"] * 32 + r["lwr"] for r in rows]
    print("  all three ports, bytes a frame p50 %d p99 %d max %d: %.1f / %.1f / %.1f MB/s (the frame's own sum; p99 of the sum)" % (
        pc(a, 50), pc(a, 99), pc(a, 100), pc(a, 50) * 57.52e-6, pc(a, 99) * 57.52e-6, pc(a, 100) * 57.52e-6))
