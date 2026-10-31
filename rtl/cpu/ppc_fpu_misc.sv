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
// PowerPC 603e small FP operations, combinational (the core registers the
// result in its EX stage, as it does the integer ALU's): fcmpu, fcmpo, fsel,
// fmr, fneg, fabs, fnabs, mffs. None of them rounds, so none goes through the
// multiply-add pipeline and its latency.
//
//  - fcmpu / fcmpo [PEM 8-60, 8-61]: CR field LT GT EQ UN, also written to
//    FPCC (FPRF's C bit, FR and FI unchanged). +0 = -0. VXSNAN for an SNaN;
//    fcmpo also VXVC for a QNaN, or for an SNaN when VE = 0.
//  - fsel [PEM 8-84]: frA >= 0 (either zero) selects frC, else frB; a NaN
//    frA selects frB. No exception, no FPSCR change.
//  - fmr, fneg, fabs, fnabs: bit copies, the FPSCR untouched [PEM 4.2.2.6].
//  - mffs: frD[32-63] = FPSCR. UNDEFINED frD[0-31] [PEM 8-125]: Supermodel
//    writes zero.
//
// The compare is a magnitude compare of the two images with the signs
// applied, so no subtractor is spent on it.
//
// PIPE = 0: combinational. From registered operands to a registered result
// it measured 8.85 ns standalone (the NaN test steering a 64-bit select, and
// the 63-bit compare), 0.52 ns over 120 MHz. PIPE = 1 registers the operand
// tests and the compare, and makes the select a cycle later: two cycles.

module ppc_fpu_misc
  import ppc_fpu_pkg::*;
#(
  parameter bit PIPE = 1'b0
) (
  input  logic        clk,
  input  misc_op_t    op,
  input  logic [7:0]  ctl,     // FPSCR[24:31] (VE for fcmpo)
  input  logic [63:0] a,       // frA
  input  logic [63:0] b,       // frB
  input  logic [63:0] c,       // frC (fsel)
  input  logic [31:0] fpscr,   // mffs
  output logic [63:0] res,
  output logic [3:0]  cr,      // compares: LT GT EQ UN
  output fpu_flags_t  flags
);

  // ----------------------------------------------- operand tests, compare
  typedef struct packed {
    misc_op_t    op;
    logic        ve;
    logic [63:0] b, c;
    logic [31:0] fpscr;
    logic        a_s, b_s, a_nan, b_nan, a_snan, b_snan, a_zero, b_zero;
    logic        hi_lt, hi_eq, lo_lt, lo_eq;   // the magnitude compare in halves
  } t_t;

  t_t t0, t;
  always_comb begin
    t0.op     = op;
    t0.ve     = ctl[7];
    t0.b      = b;
    t0.c      = c;
    t0.fpscr  = fpscr;
    t0.a_s    = a[63];
    t0.b_s    = b[63];
    t0.a_nan  = (a[62:52] == 11'h7FF) && (a[51:0] != 0);
    t0.b_nan  = (b[62:52] == 11'h7FF) && (b[51:0] != 0);
    t0.a_snan = t0.a_nan & ~a[51];
    t0.b_snan = t0.b_nan & ~b[51];
    t0.a_zero = (a[62:0] == 63'd0);
    t0.b_zero = (b[62:0] == 63'd0);
    // ordered compare of two non-NaN images: magnitudes as integers, then
    // signs; in two halves, joined after the (optional) register
    t0.hi_lt  = a[62:31] < b[62:31];
    t0.hi_eq  = a[62:31] == b[62:31];
    t0.lo_lt  = a[30:0] < b[30:0];
    t0.lo_eq  = a[30:0] == b[30:0];
  end

  generate if (PIPE) begin : g_pipe
    always_ff @(posedge clk) t <= t0;
  end else begin : g_comb
    assign t = t0;
  end endgenerate

  // ----------------------------------------------------- result, flags
  wire a_nan = t.a_nan, b_nan = t.b_nan, a_snan = t.a_snan, b_snan = t.b_snan;
  wire a_zero = t.a_zero, b_zero = t.b_zero;
  wire mag_lt = t.hi_lt | (t.hi_eq & t.lo_lt);
  wire mag_eq = t.hi_eq & t.lo_eq;
  logic lt, gt, eq;
  always_comb begin
    eq = mag_eq & (t.a_s == t.b_s) | (a_zero & b_zero);
    if (a_zero & b_zero)      lt = 1'b0;
    else if (t.a_s != t.b_s)  lt = t.a_s;
    else                      lt = t.a_s ? (~mag_lt & ~mag_eq) : mag_lt;
    gt = ~lt & ~eq;
  end

  wire unord = a_nan | b_nan;
  wire [3:0] crv = unord ? 4'b0001 : {lt, gt, eq, 1'b0};

  always_comb begin
    res   = 64'd0;
    cr    = crv;
    flags = '0;
    case (t.op)
      MSC_FCMPU, MSC_FCMPO: begin
        flags.fpcc_we    = 1'b1;
        flags.fprf       = {1'b0, crv};
        flags.exc.vxsnan = a_snan | b_snan;
        if (t.op == MSC_FCMPO)
          flags.exc.vxvc = (a_snan | b_snan) ? ~t.ve : unord;
      end
      MSC_FSEL:  begin flags.wr = 1'b1; res = (!a_nan && (a_zero || !t.a_s)) ? t.c : t.b; end
      MSC_FMR:   begin flags.wr = 1'b1; res = t.b; end
      MSC_FNEG:  begin flags.wr = 1'b1; res = {~t.b[63], t.b[62:0]}; end
      MSC_FABS:  begin flags.wr = 1'b1; res = {1'b0, t.b[62:0]}; end
      MSC_FNABS: begin flags.wr = 1'b1; res = {1'b1, t.b[62:0]}; end
      MSC_MFFS:  begin flags.wr = 1'b1; res = {32'd0, t.fpscr}; end
      default: ;
    endcase
  end

endmodule
