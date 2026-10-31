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
// PowerPC 603e FPU arithmetic: the multiply-add pipeline (ppc_fpu_fma) and the
// iterative unit (ppc_fpu_div) behind one result port.
//
// Issue rules for the core:
//  - f_*: one operation a cycle; its result appears on out_* exactly L cycles
//    later, L = fma_latency(P1A, P2, P3, P4A, P5A, P5, P6A, P6, P7) from ppc_fpu_pkg,
//    with its tag.
//  - d_*: start when d_busy is low. The result appears on out_* with its tag
//    and out_div set, after the iterative unit has handed it to the
//    multiply-add unit's back end (cycle counts in ppc_fpu_div).
//  - The hand-over needs one free slot at the back end's R4. In the cycle
//    d_inj_req rises the core leaves one bubble in f_valid; the hand-over
//    then happens when that bubble reaches S4b. The 603e itself dispatches no
//    FP instruction while fdiv, fdivs or fres runs [603e 6.4.3]; a core that
//    copies that has the slot free anyway.
//  - Results from the two units may complete out of issue order; the tag
//    says which is which.
//  - ppc_fpu_misc (combinational, or two cycles with PIPE = 1, which 120 MHz
//    needs), ppc_fpu_ldst and ppc_fpu_fpscr are instantiated by the core
//    where it needs them. The FPSCR logic takes the small unit's flags
//    registered: compare and FPSCR update in one cycle measured 8.66 ns.

module ppc_fpu
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
  parameter bit P7     = 1'b1
) (
  input  logic            clk,
  input  logic            rst,

  input  logic            f_valid,
  input  fma_op_t         f_op,
  input  logic            f_single,
  input  logic [7:0]      f_ctl,
  input  logic [63:0]     f_a,
  input  logic [63:0]     f_b,
  input  logic [63:0]     f_c,
  input  logic [TAGW-1:0] f_tag,

  input  logic            d_start,
  input  div_op_t         d_op,
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

  ppc_fpu_div #(.TAGW(TAGW)) u_div (
    .clk (clk), .rst (rst),
    .start (d_start), .op (d_op), .single (d_single), .ctl (d_ctl), .a (d_a), .b (d_b), .tag (d_tag),
    .busy (d_busy),
    .inj_valid (inj_valid), .inj_next (inj_next), .inj_ready (inj_ready), .inj_special (inj_special),
    .inj_payload (inj_payload), .inj_sign (inj_sign), .inj_q (inj_q), .inj_sticky (inj_sticky),
    .inj_e (inj_e), .inj_e0c (inj_e0c), .inj_sden (inj_sden), .inj_cap (inj_cap), .inj_single (inj_single), .inj_ctl (inj_ctl), .inj_xx_en (inj_xx_en),
    .inj_exc (inj_exc), .inj_invalid (inj_invalid), .inj_zdiv (inj_zdiv), .inj_tag (inj_tag)
  );

  assign d_inj_req = inj_valid;

  ppc_fpu_fma #(.TAGW(TAGW), .P1A(P1A), .P2(P2), .P3(P3), .P4A(P4A), .P5A(P5A), .P5(P5), .P6A(P6A), .P6(P6), .P7(P7)) u_fma (
    .clk (clk), .rst (rst),
    .in_valid (f_valid), .in_op (f_op), .in_single (f_single), .in_ctl (f_ctl),
    .in_a (f_a), .in_b (f_b), .in_c (f_c), .in_tag (f_tag),
    .inj_valid (inj_valid), .inj_next (inj_next), .inj_ready (inj_ready), .inj_special (inj_special),
    .inj_payload (inj_payload), .inj_sign (inj_sign), .inj_q (inj_q), .inj_sticky (inj_sticky),
    .inj_e (inj_e), .inj_e0c (inj_e0c), .inj_sden (inj_sden), .inj_cap (inj_cap), .inj_single (inj_single), .inj_ctl (inj_ctl), .inj_xx_en (inj_xx_en),
    .inj_exc (inj_exc), .inj_invalid (inj_invalid), .inj_zdiv (inj_zdiv), .inj_tag (inj_tag),
    .x_valid (1'b0), .x_tag ('0), .x_res (64'd0), .x_flags ('0),
    .out_valid (out_valid), .out_tag (out_tag), .out_div (out_div), .out_res (out_res),
    .out_flags (out_flags)
  );

endmodule
