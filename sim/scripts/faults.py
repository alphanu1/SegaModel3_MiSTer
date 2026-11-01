#!/usr/bin/env python3
# Planted faults for m3_scripts (M0-F). Each fault is a text substitution in a
# copy of the RTL (sm3-builds/scripts_cap/faults/); the bench must fail on it.
# Run from the repository root:
#   python3 sim/scripts/faults.py [name ...]
# Each fault is tried on the captures first (every 400th chain, quick memory),
# then on 1,500 synthetic programs behind a hostile memory (acknowledges 0-400
# cycles late, BUSY 30%), and is caught by whichever fails first.
import os, subprocess, sys
R = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
T = os.path.join(R, "sm3-builds/scripts_cap")
RTL = os.path.join(R, "rtl/io/m3_scripts.sv")
PKG = os.path.join(R, "rtl/cpu/ppc_cache_pkg.sv")
TB = os.path.join(R, "sim/scripts/tb_m3_scripts.cpp")
VEC = ["vec/sm_vf3.vec", "vec/sm_bassdx.vec", "vec/sm_vf3tb.vec", "vec/mame_vf3.vec"]

# name, area, [(old, new), ...]
FAULTS = [
  ("D1", "opcode decode: MMOV ignores the reserved bits 29:25",
   [("wire is_mmov  = dcmd[7:5] == 3'b110 && dcmd[4:1] == 4'd0;", "wire is_mmov  = dcmd[7:5] == 3'b110;")]),
  ("D2", "opcode decode: MMOV taken from bits 31:30 only (load/store too)",
   [("wire is_mmov  = dcmd[7:5] == 3'b110 && dcmd[4:1] == 4'd0;", "wire is_mmov  = dcmd[7:6] == 2'b11 && dcmd[4:1] == 4'd0;")]),
  ("D3", "opcode decode: RETURN decoded as INT",
   [("wire is_int   = INT_OP && dcmd[7:3] == 5'b10011;", "wire is_int   = INT_OP && dcmd[7:4] == 4'b1001;")]),
  ("D4", "opcode decode: INT's carry test not refused",
   [("wire int_bad  = dbc[21] | dbc[20] | dbc[18] | dbc[17];", "wire int_bad  = dbc[20] | dbc[18] | dbc[17];")]),
  ("D5", "opcode decode: INT taken when its jump-if-true bit is clear",
   [("if (dbc[19]) begin e_int = 1'b1;", "if (1'b1) begin e_int = 1'b1;")]),
  ("L1", "move length: one word fewer read than written",
   [("rd_left <= dbc[23:2];", "rd_left <= dbc[23:2] - 22'd1;")]),
  ("L2", "move length: the read count's top byte dropped",
   [("rd_left <= dbc[23:2];", "rd_left <= {6'd0, dbc[17:2]};")]),
  ("L3", "move length: a line write with four words left",
   [("wire wr_line_ok = LINE && temp[4:2] == 3'd0 && dbc[23:5] != 19'd0;", "wire wr_line_ok = LINE && temp[4:2] == 3'd0 && dbc[23:4] != 20'd0;")]),
  ("L4", "move length: a zero count not refused",
   [("wire mov_bad  = dbc == 24'd0 || ", "wire mov_bad  = ")]),
  ("A1", "address increment: a line advances seven words",
   [("wire [29:0] step = p_line ? 30'd8 : 30'd1;", "wire [29:0] step = p_line ? 30'd7 : 30'd1;")]),
  ("A2", "address increment: always one word",
   [("wire [29:0] step = p_line ? 30'd8 : 30'd1;", "wire [29:0] step = 30'd1;")]),
  ("A3", "address increment: a read advances the destination",
   [("K_RD:    dsps[31:2] <= dsps[31:2] + step;", "K_RD:    temp[31:2] <= temp[31:2] + step;")]),
  ("A4", "address increment: DSP not advanced by a fetch",
   [("K_FETCH: dsp  <= dsp + 30'd1;", "K_FETCH: dsp  <= dsp;")]),
  ("A5", "address increment: line read taken at an unaligned source",
   [("wire do_lrd = LINE && dsps[4:2] == 3'd0 &&", "wire do_lrd = LINE && dsps[4:3] == 2'd0 &&")]),
  ("S1", "DSP auto-start: ignores DMODE.MAN",
   [("wire start_dsp = w_dsp3 && !dmode[0];", "wire start_dsp = w_dsp3;")]),
  ("S2", "DSP auto-start: never",
   [("wire start_dsp = w_dsp3 && !dmode[0];", "wire start_dsp = 1'b0 && w_dsp3 && !dmode[0];")]),
  ("S3", "DSP auto-start: on byte 2C instead of 2F",
   [("wire w_dsp3  = w_dsp && s_be[0];", "wire w_dsp3  = w_dsp && s_be[3];")]),
  ("S4", "DSP auto-start: the run ignores SSM",
   [("E_END: if (dcntl[4]) begin", "E_END: if (dcntl[4] && !dmode_was0) begin"),
    ("  logic       e_int;", "  logic       e_int;\n  logic dmode_was0; always_ff @(posedge clk) if (start) dmode_was0 <= start_dsp;"),
    ("E_END: if (dcntl[4]) est <= E_IDLE;", "E_END: if (dcntl[4] && !dmode_was0) est <= E_IDLE;")]),
  ("T1", "single step: never stops",
   [("E_END: if (dcntl[4]) begin", "E_END: if (1'b0) begin"),
    ("E_END: if (dcntl[4]) est <= E_IDLE;", "E_END: if (1'b0) est <= E_IDLE;")]),
  ("T2", "single step: SSM read from the wrong bit",
   [("E_END: if (dcntl[4]) begin", "E_END: if (dcntl[3]) begin"),
    ("E_END: if (dcntl[4]) est <= E_IDLE;", "E_END: if (dcntl[3]) est <= E_IDLE;")]),
  ("T3", "single step: STD starts a halted processor",
   [("wire start_std = w_dcntl && s_wdata[2] && !halted && !busy && !srst;", "wire start_std = w_dcntl && s_wdata[2] && !busy && !srst;")]),
  ("T4", "single step: STD not cleared by the interrupt",
   [("dcntl[2] <= 1'b0;                                    // STD", "// dcntl[2] <= 1'b0;")]),
  ("T5", "single step: a DSP write does not release the halt",
   [("if (w_dsp3) halted <= 1'b0;", "")]),
  ("C1", "status: a DSTAT read leaves SSI set",
   [("st_ssi <= 1'b0; st_sir <= 1'b0; st_iid <= 1'b0;\n      irq_l  <= 1'b0;", "st_sir <= 1'b0; st_iid <= 1'b0;\n      irq_l  <= 1'b0;")]),
  ("C2", "status: a DSTAT read leaves IRQ asserted",
   [("st_ssi <= 1'b0; st_sir <= 1'b0; st_iid <= 1'b0;\n      irq_l  <= 1'b0;", "st_ssi <= 1'b0; st_sir <= 1'b0; st_iid <= 1'b0;")]),
  ("C3", "status: ISTAT.DIP misses SSI",
   [("wire dip = st_ssi | st_sir | st_iid;", "wire dip = st_sir | st_iid;")]),
  ("C4", "status: DSTAT's SSI and SIR swapped",
   [("5'h03: s_rdata <= {1'b1, 3'b000, st_ssi, st_sir,", "5'h03: s_rdata <= {1'b1, 3'b000, st_sir, st_ssi,")]),
  ("C5", "status: the interrupt ignores DIEN",
   [("if ((e_bits[3] && dien[3]) ||", "if ((e_bits[3]) ||")]),
  ("C6", "status: DCNTL.IRQD does not mask the pin",
   [("assign irq      = irq_l && !dcntl[1];", "assign irq      = irq_l;")]),
  ("C7", "status: INT does not set SIR",
   [("e_bits = {dcntl[4], 1'b1, 2'b00};", "e_bits = {dcntl[4], 1'b0, 2'b00};")]),
  ("C8", "status: an illegal instruction does not halt",
   [("if (e_bits[2] || e_bits[0]) halted <= 1'b1;", "if (e_bits[2]) halted <= 1'b1;")]),
  ("P1", "port: m_req not dropped at its acknowledge",
   [("      m_req <= 1'b0;\n      p_acked <= m_cmd != MC_WR;", "      p_acked <= m_cmd != MC_WR;")]),
  ("P2", "port: a line counted as seven beats",
   [("p_beats <= p_line ? 3'd7 : 3'd0;", "p_beats <= p_line ? 3'd6 : 3'd0;")]),
  ("P3", "port: a single write pops the FIFO before its acknowledge",
   [("assign f_pop  = (p_beat && p_kind == K_WR) || p_wack;", "assign f_pop  = (p_beat && p_kind == K_WR) || (m_req && m_cmd == MC_WR);")]),
  ("P4", "port: the move asks again before the port is idle",
   [("E_MOVE: if (!p_out && !p_done) begin", "E_MOVE: if (!p_done) begin")]),
  ("P5", "port: busy falls with the last write still open (R73 bug 2)",
   [("assign busy = est != E_IDLE || p_out;", "assign busy = est != E_IDLE;")]),
  ("P6", "port: a fetch issued with the last beat of the one before (no idle cycle)",
   [("        if (!p_out) begin\n          m_req <= 1'b1; m_cmd <= MC_RD; m_addr <= {dsp, 2'b00};",
     "        if (!p_out || p_done) begin\n          m_req <= 1'b1; m_cmd <= MC_RD; m_addr <= {dsp, 2'b00};")]),
  ("G1", "config: BAR1 bits 31:24 from the wrong lane",
   [("if (c_be[3]) bar1[31:24] <= c_wdata[31:24];", "if (c_be[3]) bar1[31:24] <= c_wdata[23:16];")]),
  ("G2", "config: the window enabled by the I/O bit",
   [("assign win_en   = cmd[1];", "assign win_en   = cmd[0];")]),
  ("G3", "config: reset does not clear the register window's DMODE",
   [("      dmode <= 8'd0; dien <= 8'd0; dcntl <= 8'd0;\n      st_ssi <= 1'b0; st_sir <= 1'b0; st_iid <= 1'b0;\n      halted <= 1'b0; irq_l <= 1'b0;\n    end",
     "      dien <= 8'd0; dcntl <= 8'd0;\n      st_ssi <= 1'b0; st_sir <= 1'b0; st_iid <= 1'b0;\n      halted <= 1'b0; irq_l <= 1'b0;\n    end")]),
]

def build(name, subs):
    d = os.path.join(T, "faults", name)
    os.makedirs(d, exist_ok=True)
    src = open(RTL).read()
    for old, new in subs:
        n = src.count(old)
        if n != 1:
            return f"substitution found {n} times: {old[:50]!r}"
        src = src.replace(old, new)
    open(os.path.join(d, "m3_scripts.sv"), "w").write(src)
    obj = os.path.join(d, "obj")
    r = subprocess.run(["verilator", "--cc", "--exe", "--build", "-j", "4", "-O3", "-Wno-fatal", "-Wno-WIDTHEXPAND",
                        "-Wno-UNUSEDSIGNAL", "--top-module", "m3_scripts", "-CFLAGS", "-O2", "-CFLAGS", "-std=c++17",
                        "-CFLAGS", "-I" + os.path.join(R, "sim/scripts"), "--Mdir", obj, PKG,
                        os.path.join(d, "m3_scripts.sv"), TB], capture_output=True, text=True)
    return None if r.returncode == 0 else r.stdout[-400:] + r.stderr[-400:]

def run(name, args):
    exe = os.path.join(T, "faults", name, "obj", "Vm3_scripts")
    try:
        r = subprocess.run([exe, "--timeout", "60000000"] + args, cwd=T, capture_output=True, text=True, timeout=900)
    except subprocess.TimeoutExpired:
        return True, "bench timed out"
    out = r.stdout.strip().splitlines()
    err = next((l.strip() for l in out if "ERROR" in l), "")
    return (bool(out) and out[-1].startswith("FAIL")) or r.returncode != 0, err

want = sys.argv[1:]
caught = total = 0
for name, what, subs in FAULTS:
    if want and name not in want:
        continue
    total += 1
    e = build(name, subs)
    if e:
        print(f"{name:3s} BUILD FAILED: {e}", flush=True)
        continue
    how, first = "MISSED", ""
    for args, label in ((["--every", "400"] + VEC, "captures"),
                        (["--seed", "3", "--ack", "0", "400", "--busy", "30", "--synth", "1500"], "synthetic, hostile")):
        bad, first = run(name, args)
        if bad:
            how = "caught on " + label
            break
    caught += how != "MISSED"
    print(f"{name:3s} {how:26s} {what}  [{first[:110]}]", flush=True)
    subprocess.run(["rm", "-rf", os.path.join(T, "faults", name)])
print(f"{caught} of {total} caught")
