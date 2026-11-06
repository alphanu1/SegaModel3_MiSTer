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
// Real3D rasteriser: fog's depth term, P = floor(w K 1024) saturated at 4096,
// bit for bit r3d_rast_rtl.cpp fog_p at 24 bits. w is the perspective
// reciprocal the interpolants already hold (r, n of the 1/w plane value), K
// the viewport's density / kz (FP32). One a clock, three stages on en.
//
//   o2 <= 0: 4096. K not > 0, or subnormal: 0.
//   Else P = min(floor(r km 2^sh), 4096), km K's significand, sh = ke - eo - n
//   - 123. r km lies in [2^46, 2^49), so sh >= -34 gives 4096 and sh <= -50
//   gives 0; between, a shift of 35 to 49.
//
// It runs beside the texel fetch and its 13 bits go on with the pixel to the
// colour stage, where the lerp is applied.

module r3d_rast_fogp (
  input  logic               clk,
  input  logic               en,
  input  logic        [24:0] i_r,
  input  logic        [5:0]  i_n,
  input  logic signed [8:0]  i_eo,
  input  logic               i_pos,
  input  logic        [31:0] i_k,
  output logic        [12:0] o_p
);

  logic [48:0] pr;
  logic signed [10:0] ix;                       // 49 - nsh, nsh the right shift: 0..14 in range
  logic        sat, zero;
  logic [12:0] qm;                              // stage 2: the shifted product's low 13 bits
  logic        f4k, f0;                         // stage 2: 4096, 0 (else qm)

  // the right shift nsh = 49 - ix for ix 0..14 (35..49), as a 16-way select of 13 bits
  function automatic logic [12:0] shr(input logic [48:0] p, input logic [3:0] i);
    shr = 13'(p >> (6'd49 - {2'd0, i}));
  endfunction

  always_ff @(posedge clk) if (en) begin
    logic kz;
    // 1: the product, the shift's index, the special cases
    kz = i_k[31] || i_k[30:23] == 8'd0 || (i_k[30:23] == 8'hff && i_k[22:0] != 23'd0);
    pr <= 49'(i_r) * 49'({1'b1, i_k[22:0]});
    ix <= {3'd0, i_k[30:23]} - {{2{i_eo[8]}}, i_eo} - {5'd0, i_n} - 11'sd74;
    sat  <= !i_pos;
    zero <= i_pos && kz;
    // 2: the shift and the decision (study R109: the clamp's compare is stage 3's select)
    //    nsh >= 50 (ix < 0): 0; nsh <= 34 (ix > 14): 4096; else q = pr >> nsh, where q[48:12] != 0
    //    only for nsh 35 (ix 14: pr[48:47]) and 36 (ix 13: pr[48]). It was a 49-bit shift by a 6-bit
    //    amount and a 37-bit compare in one stage, into p2's clear (R104's fix 6 went a step of it)
    qm  <= shr(pr, ix[3:0]);
    f4k <= sat || (!zero && !ix[10] && ix > 11'sd14) ||
           (!zero && ix == 11'sd14 && pr[48:47] != 2'd0) || (!zero && ix == 11'sd13 && pr[48]);
    f0  <= zero || ix[10];
    // 3: out
    o_p <= f4k ? 13'd4096 : f0 ? 13'd0 : qm;
  end

endmodule
