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
// Each group is gated by a one-hot select registered in decode and the core
// ORs them with its own sources (multiply, divide, move-from): no group mux
// and no select level after the units (R23 timing). The adder is gated at its
// inputs, so it gives zero when not selected with no gate after its carry
// chain. The logical group is a 4-bit truth table per bit, chosen in decode,
// as the CR logic instructions are: one LUT level for all eight logical
// operations, and an all-zero table when the group is not selected.
//
// The groups come out in four parts by how late they are: the sum, the two
// rotations under their masks, count-leading-zeros, and the rest (logical,
// extend, the sign and insert fills), which is early. The core meets the late
// parts in the last LUT before its operand registers (ppc_core.sv).
//
// cmp and cmpl subtract on the adder (y - x), and the compare is its carry
// with an equality test beside it: no separate magnitude comparator, which
// Quartus built as a five-level LUT tree (R23 timing).
//
// The shifts and rotates share two rotators by y[4:0], one left and one
// right, and differ only in the masks applied to them (study 8.4: built once):
// slw is the left rotation under a mask of the bits the shift keeps, srw the
// right rotation under its mask, sraw the same with the sign bit in the rest,
// rlw* the left rotation under MB..ME. rlwinm and rlwimi carry SH in y as an
// immediate (ppc_decode.sv), so the amount needs no mux. The rotator's last
// level and the mask meet in one LUT (R23 timing: four barrel shifters and
// their select before).
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
  input  logic [31:0] mask,       // rlw*: MB..ME, formed in ID (ppc_core.sv)
  input  logic        rlw_insert,
  input  logic [3:0]  ltt,        // logical truth table, indexed {x, y}; 0 when not logical
  input  logic        ext_h,      // extsh, else extsb
  // one-hot group select: 0 sum, 1 slw, 2 srw, 3 sraw, 4 rlw*, 5 cntlzw, 6 extsb/extsh
  input  logic [6:0]  sel,
  output logic [31:0] r_sum,      // the address too; zero unless selected
  output logic [31:0] r_rl,       // left rotation under its mask (slw, rlw*)
  output logic [31:0] r_rr,       // right rotation under its mask (srw, sraw)
  output logic [31:0] r_clz,      // cntlzw
  output logic [31:0] r_mid,      // logical, extend, sraw's sign fill, rlwimi's insert
  output logic        ca_out,
  output logic        ov_out,
  output logic [3:0]  cmp_out     // LT GT EQ SO
);

  // ------------------------------------------------------------------ adder
  wire        cmp   = (op == OP_CMP) || (op == OP_CMPL);
  wire        sub   = (op == OP_SUBF) || cmp;
  wire [31:0] ax    = sel[0] ? (sub ? ~x : x) : 32'd0;
  wire [31:0] ay    = sel[0] ? y : 32'd0;
  wire        c0    = sel[0] && ((cin == CIN_ONE) || (cin == CIN_CA && ca_in) || cmp);
  wire [32:0] sum   = {1'b0, ax} + {1'b0, ay} + {32'd0, c0};
  wire        sum_ov = (ax[31] == ay[31]) && (sum[31] != ax[31]);
  assign r_sum = sum[31:0];

  // ---------------------------------------------------------------- compare
  // for cmp / cmpl the adder forms y - x: its carry is y >= x unsigned; with
  // the signs different the signed order is the sign bits'
  wire        eq    = (x == y);
  wire        y_ge  = sum[32];
  wire        sdiff = x[31] ^ y[31];
  wire        lt_u  = y_ge && !eq;
  wire        gt_u  = !y_ge;
  wire        lt_s  = sdiff ? x[31] : lt_u;
  wire        gt_s  = sdiff ? y[31] : gt_u;

  // ---------------------------------------------------------------- logical
  logic [31:0] r_logic;
  always_comb
    for (int i = 0; i < 32; i++) r_logic[i] = ltt[{x[i], y[i]}];

  // -------------------------------------------------------- rotators, masks
  wire [4:0]  n     = y[4:0];
  wire [63:0] rdl   = {x, x} << n;
  wire [63:0] rdr   = {x, x} >> n;
  wire [31:0] rotl  = rdl[63:32];
  wire [31:0] rotr  = rdr[31:0];
  // what each shift keeps of its rotation; a shift amount of 32 or more (y[5])
  // keeps nothing, so sraw is then all sign
  wire [31:0] keep_l = y[5] ? 32'd0 : (32'hFFFF_FFFF << n);
  wire [31:0] keep_r = y[5] ? 32'd0 : (32'hFFFF_FFFF >> n);
  wire [31:0] k_l   = ({32{sel[1]}} & keep_l) | ({32{sel[4]}} & mask);
  wire [31:0] k_r   = {32{sel[2] || sel[3]}} & keep_r;
  wire [31:0] k_s   = {32{sel[3]}} & ~keep_r;
  wire [31:0] k_a   = {32{sel[4] && rlw_insert}} & ~mask;
  assign r_rl = rotl & k_l;
  assign r_rr = rotr & k_r;

  // CA of sraw / srawi: a negative value lost one-bits
  wire [31:0] lost  = y[5] ? x : (x & ~(32'hFFFF_FFFF << n));
  wire        sra_ca = x[31] && (lost != 32'd0);

  // ---------------------------------------------------- count leading zeros
  // A balanced tree, not a priority loop (which synthesised as a chain and
  // limited the core, R23). Each nibble gives its zero flag and its own count
  // in one LUT level; pairs, quads and the word then pick the upper half's
  // count unless that half is all zeros, so every output bit is four levels
  // from x (the halving tree before it chained its compares, seven levels).
  logic [7:0]  nz;                                      // nibble k is x[4k+3:4k]
  logic [1:0]  nc [0:7];
  logic [3:0]  pz;
  logic [2:0]  pc [0:3];
  logic [1:0]  qz;
  logic [3:0]  qc [0:1];
  logic [5:0]  clz;
  always_comb begin
    for (int k = 0; k < 8; k++) begin
      nz[k] = (x[4*k +: 4] == 4'd0);
      nc[k] = x[4*k + 3] ? 2'd0 : x[4*k + 2] ? 2'd1 : x[4*k + 1] ? 2'd2 : 2'd3;
    end
    for (int k = 0; k < 4; k++) begin
      pz[k] = nz[2*k + 1] && nz[2*k];
      pc[k] = nz[2*k + 1] ? {1'b1, nc[2*k]} : {1'b0, nc[2*k + 1]};
    end
    for (int k = 0; k < 2; k++) begin
      qz[k] = pz[2*k + 1] && pz[2*k];
      qc[k] = pz[2*k + 1] ? {1'b1, pc[2*k]} : {1'b0, pc[2*k + 1]};
    end
    clz = !qz[1] ? {2'b00, qc[1]} : !qz[0] ? {2'b01, qc[0]} : 6'd32;
  end
  wire [31:0] r_ext = ext_h ? {{16{x[15]}}, x[15:0]} : {{24{x[7]}}, x[7:0]};

  assign r_clz = {26'd0, {6{sel[5]}} & clz};
  assign r_mid = r_logic | ({32{sel[6]}} & r_ext) | ({32{x[31]}} & k_s) | (a_ins & k_a);

  // ------------------------------------------------- carry, overflow, compare
  always_comb begin
    ca_out  = (op == OP_SRAW) ? sra_ca : sum[32];
    ov_out  = sum_ov;
    cmp_out = {1'b0, 1'b0, eq, so_in};
    if (op == OP_CMP)  cmp_out = {lt_s, gt_s, eq, so_in};
    if (op == OP_CMPL) cmp_out = {lt_u, gt_u, eq, so_in};
  end

endmodule
