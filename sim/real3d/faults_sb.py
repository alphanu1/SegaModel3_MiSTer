#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Planted faults for D-37 stage B's blocks (r3d_cmd, r3d_mem, r3d_texup,
# m3_compose): each fault is one edit to a copy of the RTL in
# sm3-builds/sb/faults/<name>/, built against its block's bench and run. A
# fault is caught when the bench fails.
#   host bench: VF3's trace to frame 1125 against Supermodel's memories
#   compose bench: frames 1400 / 1000 and two random layers
#
#   python3 sim/real3d/faults_sb.py [name ...]

import os, shutil, subprocess, sys
from concurrent.futures import ThreadPoolExecutor

R = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
OUT = os.path.join(R, "sm3-builds/sb/faults")
ORA = os.path.join(R, "sm3-builds/sb_oracle")
VFLAGS = ["--cc", "--exe", "--build", "-j", "4", "-O3", "-Wno-fatal", "-Wno-UNUSEDSIGNAL", "-Wno-DECLFILENAME",
          "-Wno-WIDTHTRUNC", "--x-initial", "unique", "--x-assign", "unique"]
HOST = ["rtl/real3d/r3d_cmd.sv", "rtl/real3d/r3d_mem.sv", "rtl/real3d/r3d_texup.sv", "rtl/mem/m3_ddr_arb.sv",
        "rtl/mem/m2_sdram.sv", "rtl/mem/m3_ddr3p.sv", "sim/real3d/r3d_host_tb.sv"]
COMP = ["rtl/video/m3_compose.sv", "rtl/video/m2_fb_read.sv", "rtl/video/m2_video_timing.sv", "rtl/mem/m3_ddr3p.sv",
        "sim/video/m3_compose_tb.sv"]
LIST = open(os.path.join(ORA, "list.txt")).read().strip()
TRACE = ["--trace", os.path.join(R, "sm3-builds/sys/sm_bus.bin"), "--sdram", os.path.join(R, "sm3-builds/sys/vf3_sdram.bin"),
         "--dump", os.path.join(ORA, "dump"), "--quiet", "+verilator+rand+reset+2"]
# the synthetic uploads first (every type and depth: VF3 uses one), then the
# trace with the geometry stand-in longer than the rest of the frame, so the
# next frame's writes land while bank R is read (the deferral and the copies)
# (to 1500: the attract fights from about 1125 change their data every frame,
# so a write that reaches bank R early shows; the scenes before are still)
HOST_RUNS = [TRACE + ["--list", "1", "--to", "300", "--synth-tex", "300"],
             TRACE + ["--list", LIST, "--to", "1500", "--geo", "150000", "400000", "--rast", "20000", "150000", "--minchk", "20"]]
COMP_RUNS = [["--surf", f"{ORA}/tg/f001399.surf", "--layer", f"{ORA}/layer_1400.bin", "--buf", "1", "+verilator+rand+reset+2"],
             ["--surf", f"{ORA}/tg/f000999.surf", "--layer", f"{ORA}/layer_1000.bin", "--show", "0", "--buf", "3", "+verilator+rand+reset+2"],
             ["--surf", f"{ORA}/tg/f001399.surf", "--synth", "7", "--buf", "0", "--busy", "40", "--lat", "8", "100", "+verilator+rand+reset+2"],
             ["--surf", f"{ORA}/tg/f001374.surf", "--synth", "9", "--buf", "2", "+verilator+rand+reset+2"],
             ["--surf", f"{ORA}/tg/f001399.surf", "--synth", "5", "--show", "0", "--buf", "1", "+verilator+rand+reset+2"]]
BENCH = {"host": ("r3d_host_tb", HOST, "sim/real3d/tb_r3d_host.cpp", HOST_RUNS),
         "comp": ("m3_compose_tb", COMP, "sim/video/tb_m3_compose.cpp", COMP_RUNS)}

# name: (bench, file, old, new, what it breaks)
FAULTS = {
    "C1": ("host", "rtl/real3d/r3d_cmd.sv", "{6'h3F, pingpong, 25'h1FF_FFFF}", "{6'h3F, !pingpong, 25'h1FF_FFFF}", "status: ping-pong bit inverted"),
    "C2": ("host", "rtl/real3d/r3d_cmd.sv", "if (sw_wait && !b_v && m_can_swap)", "if (sw_wait && m_can_swap)", "swap before the held write goes in"),
    "C3": ("host", "rtl/real3d/r3d_cmd.sv", "if (cmd_seen && tg_seen) begin", "if (cmd_seen) begin", "draw without tile register 0C"),
    "C4": ("host", "rtl/real3d/r3d_cmd.sv", "m_swap <= 1'b1; m_rbusy <= 1'b1;", "m_swap <= 1'b1; m_rbusy <= 1'b0;", "bank R not held during the geometry"),
    "C5": ("host", "rtl/real3d/r3d_cmd.sv", "if (!x_run && !t_busy && inf_n == 2'd0 && !t_mark) begin", "if (!x_run && !t_busy && !t_mark) begin", "upload while an earlier frame is drawn"),
    "C6": ("host", "rtl/real3d/r3d_cmd.sv", "x_need <= 1'b1; t_mark <= 1'b1; blk <= merc_bc;", "x_need <= 1'b1; blk <= merc_bc;", "no mark at the draw"),
    "C7": ("host", "rtl/real3d/r3d_cmd.sv", "assign m_addr  = (b_hi == 8'h8E) ? {2'd0, b_a[19:2]} : b_a[21:2];", "assign m_addr  = b_a[21:2];", "8E not mirrored to 1 MiB"),
    "M1": ("host", "rtl/real3d/r3d_mem.sv", "dfr <= r_busy || pending;", "dfr <= pending;", "no deferral while R is read"),
    "M2": ("host", "rtl/real3d/r3d_mem.sv", "(CB+1)'(left) + 1'b1;", "(CB+1)'(left);", "a run's last beat not copied"),
    "M3": ("host", "rtl/real3d/r3d_mem.sv", "if (sk == 2'd3) begin ca <= ca + 21'd2;", "if (sk == 2'd3) begin ca <= ca + 21'd4;", "8E copy skips pairs"),
    "M4": ("host", "rtl/real3d/r3d_mem.sv", "d_be <= ca[0] ? 8'hF0 : 8'h0F;", "d_be <= ca[0] ? 8'h0F : 8'hF0;", "polygon RAM lanes swapped"),
    "M5": ("host", "rtl/real3d/r3d_mem.sv", "s_din <= sh ? cd[31:16] : cd[15:0];", "s_din <= sh ? cd[15:0] : cd[31:16];", "culling RAM halves swapped"),
    "M6": ("host", "rtl/real3d/r3d_mem.sv", "ca[19:0] == o_st + 20'(o_n)", "ca[19:0] == o_st + 20'(o_n) + 20'd1", "runs never extend (performance only)"),
    "M7": ("host", "rtl/real3d/r3d_mem.sv", "assign can_swap = (st == S_IDLE) && !pending;", "assign can_swap = (st == S_IDLE);", "swap with copies pending"),
    "M8": ("host", "rtl/real3d/r3d_mem.sv", "if (o_v) rf_wp <= rf_wp + 1'b1;", "if (o_v && 1'b0) rf_wp <= rf_wp + 1'b1;", "a closed run is lost"),
    "T1": ("host", "rtl/real3d/r3d_texup.sv", "h = (b16 ? xx[0] : yy[0]) ? rword[15:0] : rword[31:16];", "h = (b16 ? xx[0] : yy[0]) ? rword[31:16] : rword[15:0];", "texel halves swapped"),
    "T2": ("host", "rtl/real3d/r3d_texup.sv", "b = xx[0] ? h[7:0] : h[15:8];", "b = xx[0] ? h[15:8] : h[7:0];", "8-bit bytes swapped"),
    "T3": ("host", "rtl/real3d/r3d_texup.sv", "(11'd0 - (11'd1 << (4'd11 - lv)))", "(11'd0 - (11'd1 << (4'd10 - lv)))", "mip x origin"),
    "T4": ("host", "rtl/real3d/r3d_texup.sv", "+ {5'd0, yy[0]} + {3'd0, xx[2:1], 1'b0}", "+ {3'd0, xx[2:1], 1'b0}", "16-bit tile row parity dropped"),
    "T5": ("host", "rtl/real3d/r3d_texup.sv", "AW'({v_a[2:1], v_a[22:3], 2'b00})", "AW'({v_a[22:3], v_a[2:1], 2'b00})", "VROM permutation"),
    "T6": ("host", "rtl/real3d/r3d_texup.sv", "tnext <= dp + PW'(({1'b0, rword[21:0]} + 23'd4) >> 3);", "tnext <= dp + PW'(({1'b0, rword[21:0]} + 23'd4) >> 2);", "texture size formula"),
    "T7": ("host", "rtl/real3d/r3d_texup.sv", "if ($signed(fp_m - dp) <= $signed(PW'(2))) begin", "if ($signed(fp - dp) <= $signed(PW'(2))) begin", "upload past the mark"),
    "T8": ("host", "rtl/real3d/r3d_texup.sv", "busy <= 1'b1; dp <= rp;\n          rs = 1'b1; rsa = rp;", "busy <= 1'b1; dp <= rp;", "no fresh fetch at an upload"),
    "T9": ("host", "rtl/real3d/r3d_texup.sv", "else if (htype == 8'h01) us <= U_JMP;", "else if (htype == 8'h02) us <= U_JMP;", "type 1 given mip levels"),
    "T10": ("host", "rtl/real3d/r3d_texup.sv", "dp <= dp + PW'(tw); adv = {5'd0, tw};", "dp <= dp + PW'(tw) + PW'(1); adv = {5'd0, tw};", "tile end one word late"),
    "T12": ("host", "rtl/real3d/r3d_texup.sv", "wire           f_room = have <= 11'sd480;", "wire           f_room = have <= 11'sd500;", "a burst may overwrite held words"),
    "T13": ("host", "rtl/real3d/r3d_texup.sv", "dp <= dp + PW'(2); adv = 11'd2;", "dp <= dp + PW'(2);", "the header's words not taken from have"),
    "T11": ("host", "rtl/real3d/r3d_texup.sv", "y0 <= {hpage,", "y0 <= {1'b0,", "page ignored"),
    "P1": ("comp", "rtl/video/m3_compose.sv", "if (o_drawn && o_above)  px <= {o_r, o_g, o_b};\n    else if (show_3d && hit) px <= col;",
           "if (show_3d && hit) px <= col;\n    else if (o_drawn && o_above)  px <= {o_r, o_g, o_b};", "3D over above-3D tiles"),
    "P2": ("comp", "rtl/video/m3_compose.sv", ".rd_buf(y_l), .rd_x(x_l)", ".rd_buf(vcnt[1:0]), .rd_x(hcnt[8:0])", "3D read at the counters (one pixel late)"),
    "P3": ("comp", "rtl/video/m3_compose.sv", "(buf_q[1] ? 25'h4_0000 : 25'h0)", "25'h0", "buffer high bit dropped"),
    "P4": ("comp", "rtl/video/m3_compose.sv", "else if (show_3d && hit) px <= col;", "else if (hit) px <= col;", "show_3d ignored"),
}


def run(name):
    bench, f, old, new, what = FAULTS[name]
    top, files, tb, runs = BENCH[bench]
    d = os.path.join(OUT, name)
    shutil.rmtree(d, ignore_errors=True)
    os.makedirs(d)
    src = open(os.path.join(R, f)).read()
    if src.count(old) != 1:
        return name, what, f"edit not found ({src.count(old)})"
    for g in files:
        os.makedirs(os.path.dirname(os.path.join(d, g)), exist_ok=True)
        shutil.copy(os.path.join(R, g), os.path.join(d, g))
    open(os.path.join(d, f), "w").write(src.replace(old, new))
    b = subprocess.run(["verilator"] + VFLAGS + ["--top-module", top, "-CFLAGS", "-O2 -std=c++17",
                        "-CFLAGS", f"-I{R}/sim/real3d -I{R}/sim/system", "--Mdir", "obj"] + files + [os.path.join(R, tb)],
                       cwd=d, capture_output=True, text=True)
    exe = os.path.join(d, "obj", "V" + top)
    if not os.path.exists(exe):
        return name, what, "build failed: " + b.stderr[-300:]
    for a in runs:
        p = subprocess.run([exe] + a, cwd=d, capture_output=True, text=True)
        if p.returncode != 0 or "FAIL" in p.stdout:
            ls = p.stdout.splitlines()
            line = next((l for l in ls if l.rstrip().endswith("FAIL") and l.strip() != "FAIL"), None) or \
                   next((l for l in ls if "fewer than" in l or "not drawn" in l or "stuck" in l), None) or (ls[-1] if ls else f"exit {p.returncode}")
            shutil.rmtree(os.path.join(d, "obj"), ignore_errors=True)
            return name, what, "caught: " + line.strip()[:120]
    shutil.rmtree(os.path.join(d, "obj"), ignore_errors=True)
    return name, what, "MISSED"


if __name__ == "__main__":
    names = sys.argv[1:] or list(FAULTS)
    with ThreadPoolExecutor(4) as ex:
        res = list(ex.map(run, names))
    caught = 0
    for n, w, r in res:
        print(f"{n:4s} {w:45s} {r}")
        caught += r.startswith("caught")
    print(f"{caught} of {len(res)} caught")
