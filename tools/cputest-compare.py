#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Sega Model 3 core for MiSTer FPGA -- Copyright (C) 2026 alphanu1
#
# Check a board capture of the CPU bring-up build against the simulator's
# expected report (make test_cputest writes build/cputest/expected.txt), and
# print CPI and the stall breakdown.
#
#   tools/cputest-compare.py capture.txt [build/cputest/expected.txt]
#
# Every report on the board must equal the expected one in every counter and
# in the retire signature; only the run number (0A) differs. A difference is a
# hardware or timing fault in the build, since the board's memories answer in
# one cycle exactly as the simulation's do.
import sys

NAMES = {0: "cycles", 1: "instructions", 2: "uops", 3: "decode starved", 4: "load-use",
         5: "mul/div wait", 6: "data wait", 7: "redirects", 8: "fetch wait", 9: "signature", 10: "run"}

def reports(path):
    out, cur = [], {}
    for raw in open(path, "rb").read().decode("ascii", "replace").splitlines():
        f = raw.strip().split()
        if len(f) != 3 or f[0] != "P":
            continue
        try:
            k, v = int(f[1], 16), int(f[2], 16)
        except ValueError:
            continue
        cur[k] = v
        if k == 10:
            if len(cur) == 11:          # a capture can start partway through a report
                out.append(cur)
            cur = {}
    return out

cap = reports(sys.argv[1])
exp = reports(sys.argv[2] if len(sys.argv) > 2 else "build/cputest/expected.txt")
if not exp:
    sys.exit("no expected report")
want = exp[0]
bad = 0
for r in cap:
    diff = [k for k in range(10) if r.get(k) != want.get(k)]
    if diff:
        bad += 1
        print(f"run {r.get(10, '?')}: " + ", ".join(f"{NAMES[k]} {r.get(k, 0):08X} expected {want[k]:08X}" for k in diff))
print(f"{len(cap)} reports, {len(cap) - bad} identical to the simulation, {bad} different")
if cap:
    r = cap[-1]
    cyc, ins = r.get(0, 0), max(1, r.get(1, 1))
    print(f"CPI {cyc / ins:.3f} ({cyc} cycles, {ins} instructions, {r.get(2, 0)} uops)")
    for k in (3, 4, 5, 6, 7, 8):
        print(f"  {NAMES[k]:15} {r.get(k, 0):8d}  {100.0 * r.get(k, 0) / max(1, cyc):5.1f}% of cycles")
sys.exit(1 if bad or not cap else 0)
