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
// PowerPC 603e floating-point unit: shared types.
//
// Every unit hands the FPSCR logic (ppc_fpu_fpscr) the same flag bundle: what
// the instruction wrote (FR/FI, FPRF or only FPCC) and which exception causes
// it raised. The FPSCR register itself lives in the core.
//
// Bit numbering in this file is LSB-0. The manuals number the FPSCR and the
// FPRs from the MSB (IBM bit 0 = sign / FX).

package ppc_fpu_pkg;

  // Operations of the multiply-add unit (ppc_fpu_fma). fadd and fsub run as
  // frA * 1.0 +- frB; frsp, fctiw and fctiwz use the addend path alone.
  typedef enum logic [3:0] {
    FMA_ADD, FMA_SUB, FMA_MUL,
    FMA_MADD, FMA_MSUB, FMA_NMADD, FMA_NMSUB,
    FMA_RSP, FMA_CTIW, FMA_CTIWZ,
    FMA_RSQRTE   // frsqrte: only on the fast port (ppc_fpu_fast, ppc_fpu_rsqe)
  } fma_op_t;

  // Operations of the iterative unit (ppc_fpu_div)
  typedef enum logic [1:0] { DIV_DIV, DIV_RES, DIV_RSQRTE } div_op_t;

  // Operations of the small unit (ppc_fpu_misc)
  typedef enum logic [2:0] {
    MSC_FCMPU, MSC_FCMPO, MSC_FSEL, MSC_FMR, MSC_FNEG, MSC_FABS, MSC_FNABS, MSC_MFFS
  } misc_op_t;

  // Operations of the FPSCR logic (ppc_fpu_fpscr)
  typedef enum logic [2:0] {
    FS_NONE,     // FPSCR unchanged (fsel, moves, mffs)
    FS_FLAGS,    // apply a unit's flag bundle (arithmetic, conversion, compare)
    FS_MTFSF, FS_MTFSFI, FS_MTFSB0, FS_MTFSB1, FS_MCRFS
  } fs_op_t;

  // Exception causes, in FPSCR order (LSB-0 bit of the FPSCR in brackets)
  typedef struct packed {
    logic ox;      // [28]
    logic ux;      // [27]
    logic zx;      // [26]
    logic xx;      // [25]
    logic vxsnan;  // [24]
    logic vxisi;   // [23]
    logic vxidi;   // [22]
    logic vxzdz;   // [21]
    logic vximz;   // [20]
    logic vxvc;    // [19]
    logic vxsqrt;  // [9]
    logic vxcvi;   // [8]
  } fpu_exc_t;

  typedef struct packed {
    logic       wr;       // frD is written (0: enabled invalid or zero divide)
    logic       frfi_we;  // FR and FI written
    logic       fr, fi;
    logic       fprf_we;  // FPRF written (all five bits)
    logic       fpcc_we;  // FPCC written (compares: C unchanged)
    logic [4:0] fprf;     // C < > = ?
    fpu_exc_t   exc;
  } fpu_flags_t;

  // FPRF codes [PEM Table 2-5]
  localparam logic [4:0] FPRF_QNAN  = 5'h11;
  localparam logic [4:0] FPRF_NINF  = 5'h09, FPRF_PINF  = 5'h05;
  localparam logic [4:0] FPRF_NNORM = 5'h08, FPRF_PNORM = 5'h04;
  localparam logic [4:0] FPRF_NDEN  = 5'h18, FPRF_PDEN  = 5'h14;
  localparam logic [4:0] FPRF_NZERO = 5'h12, FPRF_PZERO = 5'h02;

  // The multiply-add unit's latency, issue to result, for its stage
  // parameters: R1, R4 and R8 are always registered.
  function automatic int fma_latency(input bit p1a, p2, p3, p4a, p5a, p5, p6a, p6, p7);
    return 3 + int'(p1a) + int'(p2) + int'(p3) + int'(p4a) + int'(p5a) + int'(p5) + int'(p6a) + int'(p6) + int'(p7);
  endfunction

  // FPRF of an FPR value. Single-precision results are classed against the
  // single format: a biased double exponent below 897 is below 2^-126.
  function automatic logic [4:0] fprf_of(input logic [63:0] x, input logic single);
    logic s, e0, e1, f0, den;
    s   = x[63];
    e0  = (x[62:52] == 11'd0);
    e1  = (x[62:52] == 11'h7FF);
    f0  = (x[51:0] == 52'd0);
    den = e0 || (single && (x[62:52] < 11'd897));
    if (e1 && !f0)      fprf_of = FPRF_QNAN;
    else if (e1)        fprf_of = s ? FPRF_NINF  : FPRF_PINF;
    else if (e0 && f0)  fprf_of = s ? FPRF_NZERO : FPRF_PZERO;
    else if (den)       fprf_of = s ? FPRF_NDEN  : FPRF_PDEN;
    else                fprf_of = s ? FPRF_NNORM : FPRF_PNORM;
  endfunction

  // ------------------------------------------------------------------
  // The fast port (ppc_fpu_fast, study D-21). A single-precision multiply-add
  // family op whose operands are all held exactly in single goes to the
  // single unit (ppc_fpu_sfma), and frsqrte to its estimate pipe
  // (ppc_fpu_rsqe); everything else to the double unit. An operand counts as
  // single when it is a zero, or when its low 29 fraction bits are zero and
  // its exponent is infinity / NaN or within single range including the
  // single denormals held as doubles (2^-149 ... 2^127, biased 874 ... 1150).
  // Only the operands the op reads are tested: fadds and fsubs read A and B,
  // fmuls A and C.
  function automatic logic fpu_sgl_val(input logic [63:0] x);
    logic [10:0] e;
    e = x[62:52];
    if (e == 11'd0) fpu_sgl_val = (x[51:0] == 52'd0);
    else            fpu_sgl_val = (x[28:0] == 29'd0) && (e == 11'h7FF || (e >= 11'd874 && e <= 11'd1150));
  endfunction

  function automatic logic fpu_fast_route(input fma_op_t op, input logic single,
                                          input logic [63:0] a, input logic [63:0] b, input logic [63:0] c);
    logic ra, rb, rc;
    ra = fpu_sgl_val(a);  rb = fpu_sgl_val(b);  rc = fpu_sgl_val(c);
    case (op)
      FMA_RSQRTE:                               fpu_fast_route = 1'b1;
      FMA_ADD, FMA_SUB:                         fpu_fast_route = single & ra & rb;
      FMA_MUL:                                  fpu_fast_route = single & ra & rc;
      FMA_MADD, FMA_MSUB, FMA_NMADD, FMA_NMSUB: fpu_fast_route = single & ra & rb & rc;
      default:                                  fpu_fast_route = 1'b0;
    endcase
  endfunction

  // The single unit's latency for its optional stage boundaries (R1 ... R5),
  // issue to result; the result leaves through the double unit's output
  // register, which is the last of them
  function automatic int sfma_latency(input bit q1, q2, q3, q4, q5);
    return 1 + int'(q1) + int'(q2) + int'(q3) + int'(q4) + int'(q5);
  endfunction

endpackage
