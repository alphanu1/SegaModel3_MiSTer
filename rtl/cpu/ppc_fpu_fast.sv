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
  parameter bit EF     = 1'b0         // the single unit's early forward (rule 6)
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

  ppc_fpu_sfma #(.TAGW(TAGW), .Q1(SQ1), .Q2(SQ2), .Q3(SQ3), .Q4(SQ4), .Q5(SQ5), .EF(EF)) u_sfma (
    .clk (clk), .rst (rst),
    .in_valid (f_valid & f_fast & ~is_rsq), .in_op (f_op), .in_ctl (f_ctl),
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

  // the two never finish in one cycle (same latency, one issue a cycle)
  wire             x_valid = sf_v | rq_v;
  wire [TAGW-1:0]  x_tag   = sf_v ? sf_tag : rq_tag;
  wire [63:0]      x_res   = sf_v ? sf_res : rq_res;
  fpu_flags_t      x_flags;
  assign x_flags = sf_v ? sf_flags : rq_flags;

  ppc_fpu_fma #(.TAGW(TAGW), .P1A(P1A), .P2(P2), .P3(P3), .P4A(P4A), .P5A(P5A), .P5(P5), .P6A(P6A), .P6(P6), .P7(P7)) u_fma (
    .clk (clk), .rst (rst),
    .in_valid (f_valid & ~f_fast), .in_op (f_op), .in_single (f_single), .in_ctl (f_ctl),
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
                    ({L{f_valid & ~f_fast}} & (L'(1) << (L - 1))) |
                    ({L{f_valid &  f_fast}} & (L'(1) << (LS - 1))) |
                    ({L{inj_ready}}         & (L'(1) << (FDD - 1)));
  assign f_fast_ok = ((due >> LS) == '0) && !d_busy;

  // synthesis translate_off
  initial if (LS < 3 || LS >= L) $error("ppc_fpu_fast: LS = %0d is not supported", LS);
  initial if (EF && !SQ5) $error("ppc_fpu_fast: EF needs the single unit's R5 (SQ5)");
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
    if (!rst && f_valid && !f_fast && {f_ad, f_bd, f_cd} != {f_a, f_b, f_c})
      $fatal(1, "ppc_fpu_fast: the double unit's operands differ from the port's");
  // synthesis translate_on

endmodule
