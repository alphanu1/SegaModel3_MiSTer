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
// PowerPC integer ALU, single cycle.
//
// One adder serves add, subtract, compare and the memory address (study 8.4:
// built once, shared by groups that never overlap). x is rA (or rS for the
// logical, shift and rotate groups); y is rB or the immediate.
//
// Each group's result comes out on its own port and the core picks one with a
// one-hot select registered in decode, merged with its own sources (multiply,
// divide, move-from): one level of selection instead of three (R23 timing).
// The logical group is a 4-bit truth table per bit, chosen in decode, as the CR
// logic instructions are: one LUT level for all eight logical operations.
//
// Bit numbering in comments follows the manual: bit 0 is the MSB.

module ppc_alu
  import ppc_pkg::*;
(
  input  alu_op_t     op,
  input  logic [31:0] x,
  input  logic [31:0] y,
  input  logic [31:0] a_ins,      // rlwimi: the old rA under the mask
  input  cin_sel_t    cin,
  input  logic        ca_in,
  input  logic        so_in,
  input  logic [4:0]  sh_imm,
  input  logic        sh_from_rb,
  input  logic [4:0]  mb,
  input  logic [4:0]  me,
  input  logic        rlw_insert,
  input  logic [3:0]  ltt,        // logical truth table, indexed {x, y}
  input  logic [1:0]  shk,        // 0 slw, 1 srw, 2 sraw / srawi
  input  logic [1:0]  mk,         // 0 cntlzw, 1 extsb, 2 extsh
  output logic [31:0] r_sum,
  output logic [31:0] r_logic,
  output logic [31:0] r_shift,
  output logic [31:0] r_rot,
  output logic [31:0] r_misc,
  output logic        ca_out,
  output logic        ov_out,
  output logic [3:0]  cmp_out     // LT GT EQ SO
);

  // ------------------------------------------------------------------ adder
  wire        sub   = (op == OP_SUBF);
  wire [31:0] ax    = sub ? ~x : x;
  wire        c0    = (cin == CIN_ONE) ? 1'b1 : (cin == CIN_CA) ? ca_in : 1'b0;
  wire [32:0] sum   = {1'b0, ax} + {1'b0, y} + {32'd0, c0};
  wire        sum_ov = (ax[31] == y[31]) && (sum[31] != ax[31]);
  assign r_sum = sum[31:0];

  // ---------------------------------------------------------------- compare
  wire        lt_s  = $signed(x) < $signed(y);
  wire        lt_u  = x < y;
  wire        eq    = (x == y);

  // ---------------------------------------------------------------- logical
  always_comb
    for (int i = 0; i < 32; i++) r_logic[i] = ltt[{x[i], y[i]}];

  // --------------------------------------------------------------- rotator
  wire [4:0]  rsh   = sh_from_rb ? y[4:0] : sh_imm;
  wire [63:0] rdbl  = {x, x} << rsh;
  wire [31:0] rot   = rdbl[63:32];
  wire [31:0] m_lo  = 32'hFFFF_FFFF >> mb;          // bits mb..31
  wire [31:0] m_hi  = 32'hFFFF_FFFF << (5'd31 - me); // bits 0..me
  wire [31:0] mask  = (mb <= me) ? (m_lo & m_hi) : (m_lo | m_hi);
  assign r_rot = (rot & mask) | (rlw_insert ? (a_ins & ~mask) : 32'd0);

  // ---------------------------------------------------------------- shifts
  wire [31:0] shl   = y[5] ? 32'd0 : (x << y[4:0]);
  wire [31:0] shr   = y[5] ? 32'd0 : (x >> y[4:0]);
  // the arithmetic shift on its own line: inside a conditional with an unsigned
  // arm, >>> on a $signed operand silently becomes a logical shift
  wire signed [31:0] xs     = $signed(x);
  wire signed [31:0] sra_s  = xs >>> y[4:0];
  wire [31:0] sra   = y[5] ? {32{x[31]}} : sra_s;
  wire [31:0] lost  = y[5] ? x : (x & ~(32'hFFFF_FFFF << y[4:0]));
  wire        sra_ca = x[31] && (lost != 32'd0);
  assign r_shift = (shk == 2'd0) ? shl : (shk == 2'd1) ? shr : sra;

  // ---------------------------------------------------- count leading zeros
  // A balanced tree, not a priority loop (which synthesised as a chain and
  // limited the core, R23): each level halves the field, picking the upper
  // half unless it is all zeros.
  logic [5:0]  clz;
  logic [15:0] c16;
  logic [7:0]  c8;
  logic [3:0]  c4;
  logic [1:0]  c2;
  always_comb begin
    clz[5] = (x == 32'd0);
    clz[4] = (x[31:16] == 16'd0);
    c16    = clz[4] ? x[15:0] : x[31:16];
    clz[3] = (c16[15:8] == 8'd0);
    c8     = clz[3] ? c16[7:0] : c16[15:8];
    clz[2] = (c8[7:4] == 4'd0);
    c4     = clz[2] ? c8[3:0] : c8[7:4];
    clz[1] = (c4[3:2] == 2'd0);
    c2     = clz[1] ? c4[1:0] : c4[3:2];
    clz[0] = !c2[1];
    if (clz[5]) clz[4:0] = 5'd0;                       // 32: only bit 5
  end
  assign r_misc = (mk == 2'd0) ? {26'd0, clz} :
                  (mk == 2'd1) ? {{24{x[7]}}, x[7:0]} : {{16{x[15]}}, x[15:0]};

  // ------------------------------------------------- carry, overflow, compare
  always_comb begin
    ca_out  = (op == OP_SRAW) ? sra_ca : sum[32];
    ov_out  = sum_ov;
    cmp_out = {1'b0, 1'b0, eq, so_in};
    if (op == OP_CMP)  cmp_out = {lt_s, !lt_s && !eq, eq, so_in};
    if (op == OP_CMPL) cmp_out = {lt_u, !lt_u && !eq, eq, so_in};
  end

endmodule
