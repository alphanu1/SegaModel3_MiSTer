// SPDX-License-Identifier: GPL-3.0-or-later
//
// Sega Model 3 core for MiSTer FPGA
// Copyright (C) 2026 alphanu1
//
// This program is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by the Free
// Software Foundation, either version 3 of the License, or (at your option)
// any later version. See LICENSE for the full text.
//
// ---------------------------------------------------------------------------
//
// PowerPC 603e FPU arithmetic with the fast single-precision port (study
// D-21, D-22): ppc_fpu's double multiply-add pipeline (ppc_fpu_fma, latency
// L) and iterative unit (ppc_fpu_div, fdiv / fdivs / fres), plus
//  - ppc_fpu_sfma: the single-precision multiply-add path, latency LS;
//  - ppc_fpu_rsqe: frsqrte as a pipelined estimate, latency LS too.
// All results leave through the double unit's output register (one port).
//
// Same results either way. A single op whose operands are all single values
// may go to either unit: both compute the exact result and round it once, so
// they agree bit for bit (checked against each other by the unit bench).
//
// Issue rules for the core (one op a cycle on the f_* port):
//  1. Route: f_fast = fpu_fast_route(f_op, f_single, f_a, f_b, f_c)
//     (ppc_fpu_pkg). It may be formed a cycle early from the operands as
//     they are captured, as long as it is for these operand values. frsqrte
//     (FMA_RSQRTE) is always fast; f_fast = 0 for an eligible single op is
//     allowed (it then takes the double unit, slower, same result).
//  2. Write port: an op issued in cycle t writes the result port in cycle
//     t + L (f_fast = 0) or t + LS (f_fast = 1); a divide FDD cycles after
//     its hand-over (d_inj_req's last cycle). A fast op may issue only when
//     no result is due in cycle t + LS or later: f_fast_ok says so. In the
//     core's terms (fr_at[k]: the port writes k cycles from now): no fr_at[k]
//     set for k >= LS, and no divide between its start and its hand-over;
//     the fast op then marks fr_at[LS - 1] in the next cycle's vector (a
//     double op marks fr_at[L - 1] as now). That is enough for both rules
//     below; f_fast_ok is the same test from this unit's own registers.
//  3. In-order results, so the FPSCR is updated in program order: rule 2
//     keeps a fast op behind everything issued before it; a double op or a
//     divide issued after a fast op finishes after it (L > LS, and a divide
//     takes at least 9 cycles from start to result, more than LS). The
//     divider's own rules are ppc_fpu's, unchanged: no f_* op between a
//     divide's start and its hand-over, a divide starts once every older
//     double op is past S4b (fast ops are never in the way of either).
//  4. frsqrte runs here, not on the divider (DIV_RSQRTE is not built:
//     ppc_fpu_div RSQRT = 0); its significand fits a single, so its
//     consumers stay on the fast port when its exponent is in single range.
//  5. Forwarding as before: a result's write enable is always 1 while
//     FPSCR[VE] = FPSCR[ZE] = 0.
//  6. EF = 1 (the single unit's early forward, study R117): the single unit takes its operands by source,
//     f_sel per operand (0 f_aq ..., the operand registers; 1 f_ax ..., their held copies; 2 this unit's
//     output register; 3 the single unit's R5). An op issued in cycle t whose operand is the result of a
//     single-unit op (not frsqrte) issued in cycle t - LS + 1 with OE = UE = 0 may take it with f_sel = 3;
//     f_a ... f_c are then that op's operands for the other units and the route only (the R5 operand's own
//     value is not there yet).
//  7. CV != 0 (cpu140 kit 1a): frsp, fctiw and fctiwz may go to the conversion port (f_cvt, ppc_fpu_cvt), which writes
//     the result port CV cycles after issue. A conversion issues only when no result is due CV or more cycles ahead
//     (f_cvt_ok; the core: fr_at), and then marks fr_at[CV - 1]; so it stays behind everything issued before it,
//     and everything issued after it (latency LS or L, both above CV) finishes after it. frsp with FPSCR[UE] = 1
//     stays on the double unit (the port does not normalise a double denormal). Same results either way.
//  8. DS != 0 (cpu140, dfront): an op goes to the double unit (f_fast = f_cvt = 0) only when f_dbl_ok: DS = 2 keeps
//     them two cycles apart, DS = 3 three (the iterative front); its latency stays L.

module ppc_fpu_fast
  import ppc_fpu_pkg::*;
#(
  parameter int TAGW   = 6,
  parameter bit P1A    = 1'b1,
  parameter bit P2     = 1'b1,
  parameter bit P3     = 1'b1,
  parameter bit P4A    = 1'b1,
  parameter bit P5A    = 1'b1,
  parameter bit P5     = 1'b1,
  parameter bit P6A    = 1'b1,
  parameter bit P6     = 1'b1,
  parameter bit P7     = 1'b1,
  // the single unit's stage boundaries (ppc_fpu_sfma); LS = sfma_latency()
  parameter bit SQ1    = 1'b1,
  parameter bit SQ2    = 1'b1,
  parameter bit SQ3    = 1'b1,
  parameter bit SQ4    = 1'b1,
  parameter bit SQ5    = 1'b1,
  parameter bit EF     = 1'b0,        // the single unit's early forward (rule 6)
  // CV (cpu140 kit 1a): frsp / fctiw / fctiwz on a port of their own (ppc_fpu_cvt, latency 4) where f_cvt says;
  // 0: they take the double unit, as before (rule 7)
  parameter int CV     = 0,
  // DS (cpu140, dfront): the double unit's issue spacing (rule 8: f_dbl_ok). 0: an op every cycle; 2: at most every
  // other cycle, its S4a / S4b sharing one adder pair (ppc_fpu_fma SH4); 3: at most one in three cycles, the
  // iterative front (ppc_fpu_fma IT)
  parameter int DS     = 0,
  parameter bit SF     = 1'b1         // 0: no single unit (f_fast only for frsqrte; ppc_core FAST = 0)
) (
  input  logic            clk,
  input  logic            rst,

  input  logic            f_valid,
  input  fma_op_t         f_op,
  input  logic            f_single,
  input  logic            f_fast,     // fpu_fast_route() of this op
  input  logic [7:0]      f_ctl,
  input  logic [63:0]     f_a,
  input  logic [63:0]     f_b,
  input  logic [63:0]     f_c,
  // the double unit's operands: the same values as f_a, f_b, f_c whenever an op goes to it (f_fast = 0); a
  // core may give the fast port its operands through a later select than these (ppc_core's FP operand bypass)
  input  logic [63:0]     f_ad,
  input  logic [63:0]     f_bd,
  input  logic [63:0]     f_cd,
  input  logic [TAGW-1:0] f_tag,
  // EF: the single unit's operands by source (rule 6)
  input  logic [2:0][1:0] f_sel,
  input  logic [63:0]     f_aq, f_bq, f_cq, f_ax, f_bx, f_cx,
  output logic            f_fast_ok,  // a fast op may issue this cycle (rule 2)
  input  logic            f_cvt,      // CV: this op (frsp, fctiw, fctiwz) goes to the conversion port
  output logic            f_cvt_ok,   // CV: a conversion may issue this cycle (rule 7)
  output logic            f_dbl_ok,   // DS: an op may go to the double unit this cycle (rule 8)

  input  logic            d_start,
  input  div_op_t         d_op,       // DIV_DIV or DIV_RES
  input  logic            d_single,
  input  logic [7:0]      d_ctl,
  input  logic [63:0]     d_a,
  input  logic [63:0]     d_b,
  input  logic [TAGW-1:0] d_tag,
  output logic            d_busy,
  output logic            d_inj_req,

  output logic            out_valid,
  output logic [TAGW-1:0] out_tag,
  output logic            out_div,
  output logic [63:0]     out_res,
  output fpu_flags_t      out_flags
);

  localparam int L   = fma_latency(P1A, P2, P3, P4A, P5A, P5, P6A, P6, P7);
  localparam int LS  = sfma_latency(SQ1, SQ2, SQ3, SQ4, SQ5);
  localparam int FDD = 2 + int'(P5A) + int'(P5) + int'(P6A) + int'(P6) + int'(P7);  // hand-over cycle to result (ppc_core's FDL)
  // the estimate pipe matched to LS: 3 + QA + QC + DLY
  localparam bit RQA  = (LS >= 4);
  localparam bit RQC  = (LS >= 5);
  localparam int RDLY = (LS > 5) ? LS - 5 : 0;

  logic            inj_valid, inj_next, inj_ready, inj_special, inj_sign, inj_sticky, inj_single;
  logic            inj_xx_en, inj_invalid, inj_zdiv;
  logic [51:0]     inj_payload;
  logic [55:0]     inj_q;
  logic [13:0]     inj_e, inj_e0c;
  logic [7:0]      inj_sden;
  logic            inj_cap;
  logic [7:0]      inj_ctl;
  fpu_exc_t        inj_exc;
  logic [TAGW-1:0] inj_tag;

  ppc_fpu_div #(.TAGW(TAGW), .RSQRT(1'b0)) u_div (
    .clk (clk), .rst (rst),
    .start (d_start), .op (d_op), .single (d_single), .ctl (d_ctl), .a (d_a), .b (d_b), .tag (d_tag),
    .busy (d_busy),
    .inj_valid (inj_valid), .inj_next (inj_next), .inj_ready (inj_ready), .inj_special (inj_special),
    .inj_payload (inj_payload), .inj_sign (inj_sign), .inj_q (inj_q), .inj_sticky (inj_sticky),
    .inj_e (inj_e), .inj_e0c (inj_e0c), .inj_sden (inj_sden), .inj_cap (inj_cap), .inj_single (inj_single), .inj_ctl (inj_ctl), .inj_xx_en (inj_xx_en),
    .inj_exc (inj_exc), .inj_invalid (inj_invalid), .inj_zdiv (inj_zdiv), .inj_tag (inj_tag)
  );
  assign d_inj_req = inj_valid;

  // ------------------------------------------------------- the fast port
  wire is_rsq = (f_op == FMA_RSQRTE);
  logic            sf_v, rq_v;
  logic [TAGW-1:0] sf_tag, rq_tag;
  logic [63:0]     sf_res, rq_res;
  fpu_flags_t      sf_flags, rq_flags;

  // (SF = 0: never valid, so synthesis removes the unit; the instance stays for its state's names)
  ppc_fpu_sfma #(.TAGW(TAGW), .Q1(SQ1), .Q2(SQ2), .Q3(SQ3), .Q4(SQ4), .Q5(SQ5), .EF(EF)) u_sfma (
    .clk (clk), .rst (rst),
    .in_valid (SF & f_valid & f_fast & ~is_rsq), .in_op (f_op), .in_ctl (f_ctl),
    .in_a (f_a), .in_b (f_b), .in_c (f_c), .in_tag (f_tag),
    .in_sel (f_sel), .in_aq (f_aq), .in_bq (f_bq), .in_cq (f_cq), .in_ax (f_ax), .in_bx (f_bx), .in_cx (f_cx),
    .in_fo (out_res),
    .o_valid (sf_v), .o_tag (sf_tag), .o_res (sf_res), .o_flags (sf_flags)
  );

  ppc_fpu_rsqe #(.TAGW(TAGW), .QA(RQA), .QC(RQC), .DLY(RDLY)) u_rsqe (
    .clk (clk), .rst (rst),
    .in_valid (f_valid & f_fast & is_rsq), .in_ctl (f_ctl), .in_b (f_b), .in_tag (f_tag),
    .o_valid (rq_v), .o_tag (rq_tag), .o_res (rq_res), .o_flags (rq_flags)
  );

  // the conversion port (CV): its results join the fast port's at the double unit's output register
  logic            cv_v;
  logic [TAGW-1:0] cv_tag;
  logic [63:0]     cv_res;
  fpu_flags_t      cv_flags;
  generate if (CV != 0) begin : g_cvt
    ppc_fpu_cvt #(.TAGW(TAGW)) u_cvt (
      .clk (clk), .rst (rst),
      .in_valid (f_valid & f_cvt), .in_op (f_op), .in_ctl (f_ctl), .in_b (f_b), .in_tag (f_tag),
      .o_valid (cv_v), .o_tag (cv_tag), .o_res (cv_res), .o_flags (cv_flags)
    );
  end else begin : g_nocvt
    assign cv_v = 1'b0; assign cv_tag = '0; assign cv_res = '0; assign cv_flags = '0;
  end endgenerate

  // the three never finish in one cycle (the single unit and the estimate pipe have one latency, one issue a cycle;
  // the conversion port's rule 7 keeps it off their cycles)
  wire             x_valid = sf_v | rq_v | cv_v;
  wire [TAGW-1:0]  x_tag   = sf_v ? sf_tag : cv_v ? cv_tag : rq_tag;
  wire [63:0]      x_res   = sf_v ? sf_res : cv_v ? cv_res : rq_res;
  fpu_flags_t      x_flags;
  assign x_flags = sf_v ? sf_flags : cv_v ? cv_flags : rq_flags;

  ppc_fpu_fma #(.TAGW(TAGW), .P1A(P1A), .P2(P2), .P3(P3), .P4A(P4A), .P5A(P5A), .P5(P5), .P6A(P6A), .P6(P6), .P7(P7), .SH4(DS == 2), .IT(DS == 3)) u_fma (
    .clk (clk), .rst (rst),
    .in_valid (f_valid & ~f_fast & ~f_cvt), .in_op (f_op), .in_single (f_single), .in_ctl (f_ctl),
    .in_a (f_ad), .in_b (f_bd), .in_c (f_cd), .in_tag (f_tag),
    .inj_valid (inj_valid), .inj_next (inj_next), .inj_ready (inj_ready), .inj_special (inj_special),
    .inj_payload (inj_payload), .inj_sign (inj_sign), .inj_q (inj_q), .inj_sticky (inj_sticky),
    .inj_e (inj_e), .inj_e0c (inj_e0c), .inj_sden (inj_sden), .inj_cap (inj_cap), .inj_single (inj_single), .inj_ctl (inj_ctl), .inj_xx_en (inj_xx_en),
    .inj_exc (inj_exc), .inj_invalid (inj_invalid), .inj_zdiv (inj_zdiv), .inj_tag (inj_tag),
    .x_valid (x_valid), .x_tag (x_tag), .x_res (x_res), .x_flags (x_flags),
    .out_valid (out_valid), .out_tag (out_tag), .out_div (out_div), .out_res (out_res),
    .out_flags (out_flags)
  );

  // ------------------------------------------------- write-port schedule
  // due[k]: the port carries a result k cycles from now (k = 0 is out_valid
  // in this cycle, not tracked)
  logic [L-1:0] due;
  always_ff @(posedge clk)
    if (rst) due <= '0;
    else     due <= (due >> 1) |
                    ({L{f_valid & ~f_fast & ~f_cvt}} & (L'(1) << (L - 1))) |
                    ({L{f_valid &  f_fast}} & (L'(1) << (LS - 1))) |
                    ({L{f_valid &  f_cvt}} & (L'(1) << ((CV != 0 ? CV : LS) - 1))) |
                    ({L{inj_ready}}         & (L'(1) << (FDD - 1)));
  assign f_fast_ok = ((due >> LS) == '0) && !d_busy;
  // rule 7 (CV): as rule 2 for the conversion port's latency CV: no result due CV or more cycles ahead, so results
  // stay in issue order and the port carries one a cycle
  assign f_cvt_ok  = CV != 0 && ((due >> (CV != 0 ? CV : LS)) == '0) && !d_busy;
  // rule 8 (DS): no double-unit op in the DS - 1 cycles after another; dbl_b: one may not go this cycle
  wire  dbl_i = f_valid && !f_fast && !f_cvt;
  logic dbl_q, dbl_b;
  always_ff @(posedge clk) begin
    dbl_q <= !rst && DS >= 2 && dbl_i;
    dbl_b <= !rst && ((DS >= 2 && dbl_i) || (DS >= 3 && dbl_q));
  end
  assign f_dbl_ok = !dbl_b;
  // synthesis translate_off
  always_ff @(posedge clk) if (!rst && dbl_b && dbl_i) $fatal(1, "ppc_fpu_fast: rule 8");
  initial if (DS != 0 && DS != 2 && DS != 3) $error("ppc_fpu_fast: DS = %0d is not supported", DS);
  // synthesis translate_on

  // synthesis translate_off
  initial if (LS < 3 || LS >= L) $error("ppc_fpu_fast: LS = %0d is not supported", LS);
  initial if (EF && !SQ5) $error("ppc_fpu_fast: EF needs the single unit's R5 (SQ5)");
  initial if (EF && !SF) $error("ppc_fpu_fast: EF needs the single unit (SF)");
  initial if (CV != 0 && (CV != 4 || CV >= LS)) $error("ppc_fpu_fast: CV = %0d is not supported", CV);
`ifdef R104_CHECK
  // EF: f_sel names the same values as f_a ... f_c wherever it is not 3
  always_ff @(posedge clk)
    if (!rst && EF && f_valid)
      for (int p = 0; p < 3; p++)
        if (f_sel[p] != 2'd3 && (p == 0 ? f_a : p == 1 ? f_b : f_c) !=
            (f_sel[p] == 2'd0 ? (p == 0 ? f_aq : p == 1 ? f_bq : f_cq) :
             f_sel[p] == 2'd1 ? (p == 0 ? f_ax : p == 1 ? f_bx : f_cx) : out_res))
          $fatal(1, "ppc_fpu_fast: operand %0d by source %0d differs from the port's", p, f_sel[p]);
`endif
  always_ff @(posedge clk)
    if (!rst && f_valid && f_cvt && (f_fast || CV == 0 || !(f_op == FMA_RSP || f_op == FMA_CTIW || f_op == FMA_CTIWZ)))
      $fatal(1, "ppc_fpu_fast: f_cvt with op %0d (fast %b, CV %0d)", f_op, f_fast, CV);
  always_ff @(posedge clk)
    if (!rst && f_valid && !f_fast && !f_cvt && {f_ad, f_bd, f_cd} != {f_a, f_b, f_c})
      $fatal(1, "ppc_fpu_fast: the double unit's operands differ from the port's");
  // synthesis translate_on

endmodule

// ===========================================================================
// ppc_fpu_cvt: frsp, fctiw and fctiwz on a port of their own (cpu140 kit item 1a, review140's plan; the 603e runs
// them 1-1-1, table 6-5, where the double unit takes twelve). Fully pipelined, one op a cycle, latency 4 to the
// result port: three registers (C1 ... C3) and S4 combinational into the double unit's output register, as the
// single unit's S6 is. Results are the double unit's bit for bit (PEM D.4.1-D.4.2, as ppc_fpu_ref.h has them).
//   S1  classes; fctiw's right shift (1075 - e, clamped to 63) and range; frsp's rounding place P in the significand
//       (29, or 926 - e for a tiny value, clamped to 54), tininess, the overflow candidates
//   S2  fctiw: the integer, guard and sticky; frsp: the kept bits, guard, sticky, LSB, the increment's place; both:
//       the rounding decision
//   S3  the increment: frsp's 54-bit add (its carry is the exponent's step), fctiw's 33-bit add and range check
//   S4  frsp: overflow (OE = 1 scales by 2^-192, else Table 3-15's defaults), a tiny result's flush (NI), the
//       result, FPRF; fctiw: the sign, saturation, frD's undefined high word as the double unit gives it
// frsp never shifts: a tiny result stays a normalised double with its low bits cleared at P (PEM D.4.1), so no
// leading-zero count is needed. The one case that would need one, frsp of a double denormal with FPSCR[UE] = 1, is
// never routed here (ppc_core: frsp goes to the double unit while UE = 1); in_ctl's UE is not read.
module ppc_fpu_cvt
  import ppc_fpu_pkg::*;
#(
  parameter int TAGW = 6
) (
  input  logic            clk,
  input  logic            rst,
  input  logic            in_valid,
  input  fma_op_t         in_op,      // FMA_RSP, FMA_CTIW, FMA_CTIWZ
  input  logic [7:0]      in_ctl,     // FPSCR[24:31]: VE OE UE ZE XE NI RN
  input  logic [63:0]     in_b,
  input  logic [TAGW-1:0] in_tag,
  output logic            o_valid,
  output logic [TAGW-1:0] o_tag,
  output logic [63:0]     o_res,
  output fpu_flags_t      o_flags
);
  // ------------------------------------------------------------------ S1
  wire        s1  = in_b[63];
  wire [10:0] e1v = in_b[62:52];
  wire [51:0] f1v = in_b[51:0];
  wire        e0  = e1v == 11'd0;
  wire        eo  = e1v == 11'h7FF;
  wire        fz  = f1v == 52'd0;
  wire        rsp = in_op == FMA_RSP;
  // fctiw: x = m * 2^(e - 1075); the integer is m >> (1075 - e). e < 1012 (or a denormal): shift 63, everything sticky
  wire [11:0] rsw = 12'd1075 - {1'b0, e1v};
  wire [5:0]  rs1 = (e1v < 11'd1012) ? 6'd63 : rsw[5:0];
  wire        big1 = e1v >= 11'd1055;                  // |x| >= 2^32 (finite): out of range whatever the rounding
  // frsp: E = e - 1023 < -126 is tiny; its LSB moves up from bit 29 by -126 - E: P = 926 - e
  wire        tny1 = e1v < 11'd897;                    // (a denormal too)
  wire [11:0] plw  = 12'd926 - {1'b0, e1v};
  wire [5:0]  p1   = !tny1 ? 6'd29 : (e1v <= 11'd872) ? 6'd54 : plw[5:0];
  typedef struct packed {
    logic            v, rsp, cvz, s, nan, snan, inf, zero, tny, big, ve, oe, ni;
    logic [1:0]      rn;
    logic [10:0]     e;
    logic [52:0]     m;
    logic [5:0]      rs, p;
    logic [TAGW-1:0] tag;
  } c1_t;
  c1_t c1;
  always_ff @(posedge clk) begin
    c1.v    <= !rst && in_valid;
    c1.rsp  <= rsp;
    c1.cvz  <= in_op == FMA_CTIWZ;
    c1.s    <= s1;
    c1.nan  <= eo && !fz;
    c1.snan <= eo && !fz && !f1v[51];
    c1.inf  <= eo && fz;
    c1.zero <= e0 && fz;
    c1.tny  <= tny1;
    c1.big  <= big1;
    c1.ve   <= in_ctl[7];
    c1.oe   <= in_ctl[6];
    c1.ni   <= in_ctl[2];
    c1.rn   <= (in_op == FMA_CTIWZ) ? 2'b01 : in_ctl[1:0];
    c1.e    <= e1v;
    c1.m    <= {!e0, f1v};
    c1.rs   <= rs1;
    c1.p    <= rsp ? p1 : rs1;                         // (k1s) one rounding place for both: fctiw's is its shift
    c1.tag  <= in_tag;
  end
  // ------------------------------------------------------------------ S2
  // fctiw: the integer (at most 33 bits matter: rs >= 21 wherever the value is in range), the guard m[rs - 1], the
  // sticky m[rs - 2 : 0]
  wire [52:0] ish = c1.m >> c1.rs;
  // (k1s) the rounding place P is frsp's place or fctiw's shift (the integer's LSB sits at m[rs]): one thermometer
  // (keep: j >= P) and one one-hot (ph: j == P) give the LSB, the guard (ph one up) and the sticky (keep one up, not)
  logic [53:0] keep_r, ph_r;
  always_comb
    for (int j = 0; j < 54; j++) begin
      keep_r[j] = 7'(j) >= {1'b0, c1.p};
      ph_r[j]   = 7'(j) == {1'b0, c1.p};
    end
  wire        g2   = |(c1.m & ph_r[53:1]);              // m[P - 1]
  wire        st2  = |(c1.m & ~keep_r[53:1]);           // m[P - 2 : 0]
  wire        lsb2 = |(c1.m & ph_r[52:0]);              // m[P]
  logic       up2;
  always_comb
    case (c1.rn)
      2'b00:   up2 = g2 & (st2 | lsb2);
      2'b01:   up2 = 1'b0;
      2'b10:   up2 = !c1.s & (g2 | st2);
      default: up2 = c1.s & (g2 | st2);
    endcase
  typedef struct packed {
    logic            v, rsp, s, nan, snan, inf, zero, tny, big, ve, oe, ni, up, ix, pz;
    logic [1:0]      rn;
    logic [10:0]     e;
    logic [52:0]     kv, inc;                           // frsp: the kept bits and the increment at P
    logic [32:0]     iv;                                // fctiw: the integer
    logic [51:0]     f;                                 // frB's fraction (a NaN's payload)
    logic [TAGW-1:0] tag;
  } c2_t;
  c2_t c2;
  always_ff @(posedge clk) begin
    c2.v    <= !rst && c1.v;
    c2.rsp  <= c1.rsp;  c2.s <= c1.s;  c2.nan <= c1.nan;  c2.snan <= c1.snan;  c2.inf <= c1.inf;
    c2.zero <= c1.zero; c2.tny <= c1.tny; c2.big <= c1.big;
    c2.ve   <= c1.ve;   c2.oe <= c1.oe;  c2.ni <= c1.ni;   c2.rn <= c1.rn;    c2.e <= c1.e;
    c2.up   <= up2;
    c2.ix   <= g2 | st2;
    c2.pz   <= c1.p >= 6'd53;                          // frsp: nothing kept (the result is 0 or 2^-149)
    c2.kv   <= c1.m & keep_r[52:0];
    c2.inc  <= ph_r[52:0];
    c2.iv   <= ish[32:0];
    c2.f    <= c1.m[51:0];
    c2.tag  <= c1.tag;
  end
  // ------------------------------------------------------------------ S3
  wire [53:0] rr3 = {1'b0, c2.kv} + {1'b0, c2.up ? c2.inc : 53'd0};
  wire [33:0] ir3 = {1'b0, c2.iv} + {33'd0, c2.up};
  typedef struct packed {
    logic            v, rsp, s, nan, snan, inf, zero, tny, ovr, ve, oe, ni, up, ix, pz, c;
    logic [1:0]      rn;
    logic [10:0]     e;
    logic [51:0]     r;                                 // frsp: the rounded fraction (0 on a carry)
    logic [31:0]     ia;                                // fctiw: the magnitude
    logic [51:0]     f;
    logic [TAGW-1:0] tag;
  } c3_t;
  c3_t c3;
  always_ff @(posedge clk) begin
    c3.v   <= !rst && c2.v;
    c3.rsp <= c2.rsp;  c3.s <= c2.s;  c3.nan <= c2.nan;  c3.snan <= c2.snan;  c3.inf <= c2.inf;  c3.zero <= c2.zero;
    c3.tny <= c2.tny;  c3.ve <= c2.ve;  c3.oe <= c2.oe;  c3.ni <= c2.ni;  c3.rn <= c2.rn;
    c3.up  <= c2.up;   c3.ix <= c2.ix;  c3.pz <= c2.pz;  c3.e <= c2.e;  c3.f <= c2.f;  c3.tag <= c2.tag;
    // frsp: a carry out of the significand (bit 53) is the exponent's step, the fraction then 0
    c3.c   <= rr3[53];
    c3.r   <= rr3[53] ? 52'd0 : rr3[51:0];
    // fctiw: out of range after rounding (D.4.2): above 2^31 - 1, or below -2^31
    c3.ovr <= c2.big || (c2.s ? ir3 > 34'h080000000 : ir3 > 34'h07FFFFFFF);
    c3.ia  <= ir3[31:0];
  end
  // ------------------------------------------------------------------ S4
  wire [11:0] eb   = {1'b0, c3.e} + {11'd0, c3.c};     // frsp: the rounded result's biased double exponent
  wire        ovf  = !c3.tny && eb > 12'd1150;         // E > 127 after rounding
  wire        toinf = (c3.rn == 2'b00) || (c3.rn == 2'b10 && !c3.s) || (c3.rn == 2'b11 && c3.s);
  wire        fin  = !c3.nan && !c3.inf && !c3.zero;
  logic [63:0] rs_r;                                   // frsp's result
  logic        rs_fr, rs_fi, rs_ox, rs_ux;
  always_comb begin
    rs_fr = 1'b0; rs_fi = 1'b0; rs_ox = 1'b0; rs_ux = 1'b0;
    if (c3.nan)       rs_r = {c3.s, 11'h7FF, 1'b1, c3.f[50:29], 29'd0};   // quiet(frB) & ~0x1FFFFFFF
    else if (!fin)    rs_r = {c3.s, c3.e, c3.f};                          // infinity, zero: frB
    else begin
      rs_fr = c3.up; rs_fi = c3.ix;
      if (ovf) begin
        rs_ox = 1'b1;
        if (c3.oe) rs_r = {c3.s, 11'(eb - 12'd192), c3.r};
        else begin
          rs_r  = toinf ? {c3.s, 11'h7FF, 52'd0} : {c3.s, 63'h47EF_FFFF_E000_0000};
          rs_fr = 1'b0; rs_fi = 1'b1;
        end
      end else if (c3.tny) begin
        rs_ux = c3.ix;
        if (c3.pz)                                    // n = up: 0 or 2^-149, which NI flushes
          rs_r = (c3.up && !c3.ni) ? {c3.s, 11'd874, 52'd0} : {c3.s, 63'd0};
        else if (c3.ni && eb < 12'd897)               // still denormal in single: delivered as zero
          rs_r = {c3.s, 63'd0};
        else
          rs_r = {c3.s, eb[10:0], c3.r};
      end else
        rs_r = {c3.s, eb[10:0], c3.r};
    end
  end
  // fctiw
  wire        iv_inv = c3.nan || c3.inf || (fin && c3.ovr);
  wire        bigp   = c3.inf ? !c3.s : !c3.nan && !c3.s;
  wire [31:0] ilo    = iv_inv ? (bigp ? 32'h7FFF_FFFF : 32'h8000_0000) : (c3.s ? 32'd0 - c3.ia : c3.ia);
  wire [63:0] ir_r   = {(iv_inv && ilo[31]) ? 32'hFFFF_FFFF : 32'd0, ilo};
  // the flags (ppc_fpu_ref.h: frsp as an arithmetic op on single, fctiw as D.4.2)
  wire        inv    = c3.rsp ? c3.snan : iv_inv;
  wire        wr     = !(inv && c3.ve);
  fpu_flags_t fl;
  always_comb begin
    fl = '0;
    fl.wr      = wr;
    fl.frfi_we = 1'b1;
    if (c3.rsp) begin
      fl.fr = wr && rs_fr;  fl.fi = wr && rs_fi;
      fl.fprf_we = wr;
      fl.fprf    = fprf_of(rs_r, 1'b1);
      fl.exc.ox  = rs_ox;  fl.exc.ux = rs_ux;  fl.exc.xx = wr && rs_fi;
      fl.exc.vxsnan = c3.snan;
    end else begin
      fl.fr = !iv_inv && c3.up;  fl.fi = !iv_inv && c3.ix;
      fl.exc.xx  = !iv_inv && c3.ix;
      fl.exc.vxsnan = c3.snan;
      fl.exc.vxcvi  = iv_inv;
    end
  end
  assign o_valid = c3.v;
  assign o_tag   = c3.tag;
  assign o_res   = c3.rsp ? rs_r : ir_r;
  assign o_flags = fl;
endmodule
