#!/usr/bin/env python3
# Planted faults for m3_tilegen (M0-E). Each fault is a `define in the RTL; the
# bench must fail on it. Run from the repository root:
#   python3 sim/tilegen/faults.py [name ...]
import subprocess, sys, os
R = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
T = os.path.join(R, "sm3-builds/tilegen")
# Every fault runs first on the real dumps (12 per folder); if those miss it,
# on two synthetic variants of each as well (tg::mutate). ACK runs behind a
# memory with acknowledges 1-40 clocks late.
SM_DIRS = ["sm_vf3", "sm_bassdx"]
MAME_DIRS = ["mame_vf3"]
SM_FAULTS = ["SCROLL", "ROWSCROLL", "RSSWAP", "STENCIL", "PAIR", "STALE", "BPP", "SUPPRESS", "PRIO", "OFFSET",
             "EXPAND", "ACK"]
MAME_FAULTS = ["SCROLL", "ROWSCROLL", "STENCIL", "PAIR", "BPP", "SUPPRESS", "PRIO", "ACK"]
want = sys.argv[1:]
rows = [(n, "sm") for n in SM_FAULTS] + [(n, "mame") for n in MAME_FAULTS]
caught = 0; total = 0
for n, c in rows:
    if want and n not in want: continue
    obj = f"f_{n}_{c}"
    b = subprocess.run([os.path.join(T, "build.sh"), obj, c, f"+define+TG_FAULT_{n}"], capture_output=True, text=True)
    if b.returncode:
        print(f"{n:10s} {c:4s} BUILD FAILED\n{b.stdout}{b.stderr}"); continue
    base = ["--max", "12"] + (["--lat", "1", "40"] if n == "ACK" else [])
    dirs = SM_DIRS if c == "sm" else MAME_DIRS
    how = "MISSED"; summ = ""
    for extra, label in (([], "real dumps"), (["--mutate", "2"], "synthetic variants")):
        r = subprocess.run([os.path.join(T, f"obj_{obj}", "Vm3_tilegen"), "--cfg", c] + base + extra + dirs,
                           cwd=T, capture_output=True, text=True)
        out = r.stdout.strip().splitlines()
        summ = next((l for l in out if l.startswith("frames")), "")
        if out and out[-1] == "FAIL":
            how = "caught on " + label; break
    total += 1; caught += how != "MISSED"
    print(f"{n:10s} {c:4s} {how:30s} {summ}", flush=True)
    subprocess.run(["rm", "-rf", os.path.join(T, f"obj_{obj}")])
print(f"{caught} of {total} caught")
