#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Planted faults for the 100 MHz rasteriser blocks: each fault is one edit to
# a copy of the RTL in sm3-builds/rast/faults100/<name>/, built against its
# block's bench and run on 20 VF3 frames and a synthetic or random set. A
# fault is caught when the bench reports a mismatch.
#
#   python3 sim/real3d/faults_rast.py [name ...]

import os, re, shutil, subprocess, sys

R = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
OUT = os.path.join(R, "sm3-builds/rast/faults100")
TRIS = os.path.join(R, "sm3-builds/rast/tris_vf3")
VFLAGS = ["--cc", "--exe", "--build", "-j", "16", "-O3", "-Wno-fatal", "-Wno-UNUSEDSIGNAL",
          "-Wno-DECLFILENAME", "-Wno-WIDTHTRUNC"]
SIM = ["sim/real3d/r3d_rast.cpp", "sim/real3d/r3d_rast_rtl.cpp"]

FRONT = ["rtl/real3d/r3d_div_lane.sv", "rtl/real3d/r3d_rast_div.sv", "rtl/real3d/r3d_rast_edge.sv",
         "rtl/real3d/r3d_rast_front.sv"]
BLOCKS = {
    "front": dict(top="r3d_rast_front", files=FRONT, tb="sim/real3d/tb_r3d_rast_edge.cpp", defs="-DFRONT",
                  runs=[["--tris", TRIS, "--frames", "20", "--band", "16"],
                        ["--synthetic", "20000", "--band", "16"]]),
    "div": dict(top="r3d_rast_div", files=FRONT[:2], tb="sim/real3d/tb_r3d_rast_div.cpp", defs="",
                runs=[["--tris", TRIS, "--frames", "20"], ["--random", "200000"]]),
    "zspan": dict(top="r3d_rast_zspan", files=["rtl/real3d/r3d_rast_zspan.sv"], tb="sim/real3d/tb_r3d_rast_zspan.cpp", defs="",
                  runs=[["--tris", TRIS, "--frames", "20", "--oow"], ["--random", "200000"]]),
    "intake": dict(top="r3d_rast_intake", files=["rtl/real3d/r3d_rast_intake.sv"],
                   tb="sim/real3d/tb_r3d_rast_intake.cpp", defs="",
                   runs=[["--tris", TRIS, "--frames", "20", "--oow"], ["--random", "200000"]]),
}

# name: (block, file, old, new)
FAULTS = {
    "D1_neg_bit":     ("div", "rtl/real3d/r3d_rast_div.sv", "neg[j]  <= dxs[j][32];", "neg[j]  <= dxs[j][31];"),
    "D2_skip_r0":     ("front", "rtl/real3d/r3d_rast_div.sv", "g_r[j] <= {16'd0, nn[53:38]};", "g_r[j] <= {16'd0, nn[52:37]};"),
    "D3_exact_tie":   ("front", "rtl/real3d/r3d_div_lane.sv", "if (!s[32]) step", "if (!s[32] && s[31:0] != 32'd0) step"),
    "E1_prod_short":  ("front", "rtl/real3d/r3d_rast_edge.sv", "if (m_d1[31:2] == 30'd0 && m_d2[31:2] == 30'd0)",
                       "if (m_d1[31:4] == 28'd0 && m_d2[31:4] == 28'd0)"),
    "E2_band_start":  ("front", "rtl/real3d/r3d_rast_edge.sv", "wire signed [16:0] ys  = (z_r1 > b0s) ? z_r1 : b0s;",
                       "wire signed [16:0] ys  = (z_r1 > b0s + 17'sd1) ? z_r1 : b0s;"),
    "E3_upper_late":  ("front", "rtl/real3d/r3d_rast_edge.sv", "w_up <= (w_y + 17'sd1 < w_r2);", "w_up <= (w_y + 17'sd1 <= w_r2);"),
    "E4_clamp_hi":    ("front", "rtl/real3d/r3d_rast_edge.sv", "wire signed [19:0] hi  = (hi0 > W20) ? W20 : hi0;",
                       "wire signed [19:0] hi  = (hi0 > W20 - 20'sd1) ? W20 - 20'sd1 : hi0;"),
    "I1_no_round":    ("intake", "rtl/real3d/r3d_rast_intake.sv", "{38'd0, s_q[0]}", "{38'd0, 1'b0}"),
    "I2_grad_exp":    ("intake", "rtl/real3d/r3d_rast_intake.sv", "(grad ? 9'd9 : 9'd0)", "(grad ? 9'd8 : 9'd0)"),
    "I3_vtx_sat":     ("intake", "rtl/real3d/r3d_rast_intake.sv", "f_w[30:23] >= 8'd142", "f_w[30:23] >= 8'd143"),
    "I4_zero_e":      ("intake", "rtl/real3d/r3d_rast_intake.sv", "(mx == 9'd0) ? 9'sd2 :", "(mx == 9'd0) ? 9'sd1 :"),
    "I5_nan_sign":    ("intake", "rtl/real3d/r3d_rast_intake.sv", "else if (r_kind == K_VTX && r_nan) o_data <= 40'h00_8000_0000;",
                       "else if (r_kind == K_VTX && r_nan) o_data <= 40'h00_7fff_ffff;"),
    "Z1_man_window":  ("zspan", "rtl/real3d/r3d_rast_zspan.sv", "zw2 = {ex[7:0], sh[40:25]};", "zw2 = {ex[7:0], sh[39:24]};"),
    "Z2_exp_bias":    ("zspan", "rtl/real3d/r3d_rast_zspan.sv", "11'sd86;", "11'sd87;"),
    "Z3_lane1_init":  ("zspan", "rtl/real3d/r3d_rast_zspan.sv", "w_l1 <= n_acc + {n_dx[40:0], 1'b0};", "w_l1 <= n_acc + n_dx;"),
    "Z4_zero_test":   ("zspan", "rtl/real3d/r3d_rast_zspan.sv", "    if (v[41]) n = 7'd64;\n", "    if (v[41]) n = 7'd64;\n    if (v == 42'sd0) n = 7'd0;\n"),
}


def run(name):
    blk, fn, old, new = FAULTS[name]
    b = BLOCKS[blk]
    d = os.path.join(OUT, name)
    shutil.rmtree(d, ignore_errors=True)
    os.makedirs(d)
    files = []
    for f in b["files"]:
        src = open(os.path.join(R, f)).read()
        if f == fn:
            assert src.count(old) == 1, (name, old)
            src = src.replace(old, new)
        dst = os.path.join(d, os.path.basename(f))
        open(dst, "w").write(src)
        files.append(dst)
    cmd = ["verilator"] + VFLAGS + ["--top-module", b["top"], "-CFLAGS",
           "-O2 -std=c++17 -ffp-contract=off -I%s/sim/real3d %s" % (R, b["defs"]), "-LDFLAGS", "-lz",
           "--Mdir", os.path.join(d, "obj")] + files + [os.path.join(R, b["tb"])] + [os.path.join(R, s) for s in SIM]
    subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    exe = os.path.join(d, "obj", "V" + b["top"])
    res = []
    for a in b["runs"]:
        p = subprocess.run([exe] + a, capture_output=True, text=True)
        m = re.search(r"(\d+) wrong", p.stdout)
        res.append(int(m.group(1)) if m else -1)
    shutil.rmtree(os.path.join(d, "obj"), ignore_errors=True)
    caught = any(x != 0 for x in res)
    print("%-16s %-7s %-40s %s" % (name, blk, " / ".join("%d wrong" % x for x in res), "caught" if caught else "NOT caught"), flush=True)
    return caught


if __name__ == "__main__":
    names = sys.argv[1:] or list(FAULTS)
    n = sum(run(x) for x in names)
    print("%d of %d caught" % (n, len(names)))
