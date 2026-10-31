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
// Bench wrapper: the Real3D FP units as the geometry engine instantiates them,
// with and without subnormals, and the divide / reciprocal unit.

module tb_r3d_fp_top (
  input  logic        clk,
  input  logic        rst,
  input  logic        m_valid,
  input  logic [31:0] m_a, m_b,
  output logic        m_ovalid,
  output logic [31:0] m_res,
  input  logic        a_valid,
  input  logic [31:0] a_a, a_b,
  output logic        a_ovalid,
  output logic [31:0] a_res,
  input  logic        r_valid,
  input  logic        r_div,
  input  logic [31:0] r_a,
  input  logic [31:0] r_x,
  output logic        r_ovalid,
  output logic [31:0] r_res,
  output logic        r_tiny,
  // the flush-to-zero units (SUBNORM = 0), fed the same operands
  output logic [31:0] mz_res,
  output logic [31:0] az_res
);
  r3d_fp_mul #(.SUBNORM(1'b1)) u_mul (
    .clk, .rst_n(!rst), .in_valid(m_valid), .a(m_a), .b(m_b),
    .out_valid(m_ovalid), .result(m_res), .overflow(), .underflow(), .invalid());
  r3d_fp_add #(.SUBNORM(1'b1)) u_add (
    .clk, .rst_n(!rst), .in_valid(a_valid), .a(a_a), .b(a_b), .sub(1'b0),
    .out_valid(a_ovalid), .result(a_res), .overflow(), .underflow(), .invalid());
  r3d_recip u_recip (.clk, .in_valid(r_valid), .div(r_div), .a(r_a), .x(r_x), .out_valid(r_ovalid), .result(r_res), .tiny(r_tiny));
  r3d_fp_mul #(.SUBNORM(1'b0)) u_mulz (
    .clk, .rst_n(!rst), .in_valid(m_valid), .a(m_a), .b(m_b),
    .out_valid(), .result(mz_res), .overflow(), .underflow(), .invalid());
  r3d_fp_add #(.SUBNORM(1'b0)) u_addz (
    .clk, .rst_n(!rst), .in_valid(a_valid), .a(a_a), .b(a_b), .sub(1'b0),
    .out_valid(), .result(az_res), .overflow(), .underflow(), .invalid());
endmodule
