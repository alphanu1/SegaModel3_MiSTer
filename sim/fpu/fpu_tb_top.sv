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
// Test harness for the FPU units (tb_fpu.cpp): the arithmetic unit (the
// multiply-add pipeline with the iterative unit injecting into it), the small
// unit, the load/store conversions and the FPSCR logic, each with its ports
// brought out. The FPSCR logic is fed by whichever unit the bench selects.
// FAST = 1: the arithmetic unit is ppc_fpu_fast (the single-precision path
// and the frsqrte estimate beside the double unit, study D-21 / D-22), and
// f_route is the RTL's own route for the f_* inputs.

module fpu_tb_top
  import ppc_fpu_pkg::*;
#(
  parameter bit MISC_PIPE = 1'b0,
  parameter bit FAST = 1'b0,
  // the single unit's stages: unused with FAST = 0
  /* verilator lint_off UNUSEDPARAM */
  parameter bit SQ1 = 1'b1, SQ2 = 1'b1, SQ3 = 1'b1, SQ4 = 1'b1, SQ5 = 1'b1,
  /* verilator lint_on UNUSEDPARAM */
  parameter bit P1A = 1'b1, P2 = 1'b1, P3 = 1'b1, P4A = 1'b1, P5A = 1'b1, P5 = 1'b1, P6A = 1'b1, P6 = 1'b1, P7 = 1'b1
) (
  input  logic        clk,
  input  logic        rst,

  // multiply-add issue
  input  logic        f_valid,
  input  logic [3:0]  f_op,
  input  logic        f_single,
  input  logic [7:0]  f_ctl,
  input  logic [63:0] f_a, f_b, f_c,
  input  logic [7:0]  f_tag,
  input  logic        f_fast,
  output logic        f_fast_ok,
  output logic        f_route,

  // iterative unit
  input  logic        d_start,
  input  logic [1:0]  d_op,
  input  logic        d_single,
  input  logic [7:0]  d_ctl,
  input  logic [63:0] d_a, d_b,
  input  logic [7:0]  d_tag,
  output logic        d_busy,
  output logic        d_inj_req,

  // result port
  output logic        o_valid,
  output logic [7:0]  o_tag,
  output logic        o_div,
  output logic [63:0] o_res,
  output logic [31:0] o_flags,

  // small unit (combinational)
  input  logic [2:0]  m_op,
  input  logic [7:0]  m_ctl,
  input  logic [63:0] m_a, m_b, m_c,
  input  logic [31:0] m_fpscr,
  output logic [63:0] m_res,
  output logic [3:0]  m_cr,
  output logic [31:0] m_flags,

  // loads and stores (combinational)
  input  logic [31:0] l_word,
  output logic [63:0] l_dbl,
  input  logic [63:0] s_dbl,
  output logic [31:0] s_word,

  // FPSCR logic (combinational); fs_sel: 0 result port, 1 small unit
  input  logic [2:0]  fs_op,
  input  logic        fs_sel,
  input  logic [31:0] fs_in,
  input  logic [7:0]  fs_fm,
  input  logic [31:0] fs_frb,
  input  logic [2:0]  fs_fld,
  input  logic [3:0]  fs_imm,
  input  logic [4:0]  fs_bit,
  output logic [31:0] fs_out,
  output logic [3:0]  fs_cr1,
  output logic [3:0]  fs_crf
);

  fpu_flags_t of, mf;
  assign o_flags = 32'(of);
  assign m_flags = 32'(mf);

  assign f_route = fpu_fast_route(fma_op_t'(f_op), f_single, f_a, f_b, f_c);

  generate if (FAST) begin : g_fast
    ppc_fpu_fast #(.TAGW(8), .P1A(P1A), .P2(P2), .P3(P3), .P4A(P4A), .P5A(P5A), .P5(P5), .P6A(P6A), .P6(P6), .P7(P7),
                   .SQ1(SQ1), .SQ2(SQ2), .SQ3(SQ3), .SQ4(SQ4), .SQ5(SQ5)) u_fpu (
      .clk (clk), .rst (rst),
      .f_valid (f_valid), .f_op (fma_op_t'(f_op)), .f_single (f_single), .f_fast (f_fast), .f_ctl (f_ctl),
      .f_a (f_a), .f_b (f_b), .f_c (f_c), .f_tag (f_tag), .f_fast_ok (f_fast_ok),
      .d_start (d_start), .d_op (div_op_t'(d_op)), .d_single (d_single), .d_ctl (d_ctl),
      .d_a (d_a), .d_b (d_b), .d_tag (d_tag), .d_busy (d_busy), .d_inj_req (d_inj_req),
      .out_valid (o_valid), .out_tag (o_tag), .out_div (o_div), .out_res (o_res), .out_flags (of)
    );
  end else begin : g_unified
    assign f_fast_ok = 1'b0;
    ppc_fpu #(.TAGW(8), .P1A(P1A), .P2(P2), .P3(P3), .P4A(P4A), .P5A(P5A), .P5(P5), .P6A(P6A), .P6(P6), .P7(P7)) u_fpu (
      .clk (clk), .rst (rst),
      .f_valid (f_valid), .f_op (fma_op_t'(f_op)), .f_single (f_single), .f_ctl (f_ctl),
      .f_a (f_a), .f_b (f_b), .f_c (f_c), .f_tag (f_tag),
      .d_start (d_start), .d_op (div_op_t'(d_op)), .d_single (d_single), .d_ctl (d_ctl),
      .d_a (d_a), .d_b (d_b), .d_tag (d_tag), .d_busy (d_busy), .d_inj_req (d_inj_req),
      .out_valid (o_valid), .out_tag (o_tag), .out_div (o_div), .out_res (o_res), .out_flags (of)
    );
  end endgenerate

  ppc_fpu_misc #(.PIPE(MISC_PIPE)) u_misc (
    .clk (clk), .op (misc_op_t'(m_op)), .ctl (m_ctl), .a (m_a), .b (m_b), .c (m_c), .fpscr (m_fpscr),
    .res (m_res), .cr (m_cr), .flags (mf)
  );

  ppc_fpu_ldst #(.PIPE(MISC_PIPE)) u_ldst (.clk (clk), .lfs_word (l_word), .lfs_dbl (l_dbl), .stfs_dbl (s_dbl), .stfs_word (s_word));

  ppc_fpu_fpscr u_fpscr (
    .op (fs_op_t'(fs_op)), .fpscr_in (fs_in), .flags (fs_sel ? mf : of),
    .fm (fs_fm), .frb_lo (fs_frb), .fld (fs_fld), .imm (fs_imm), .bitn (fs_bit),
    .fpscr_out (fs_out), .cr1 (fs_cr1), .crf (fs_crf)
  );

endmodule
