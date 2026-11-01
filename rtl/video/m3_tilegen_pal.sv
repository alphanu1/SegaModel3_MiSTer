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
// The colour half of the tile generator's palette: 32 K x 15, one write port
// (CPU), one read port (emitter), data the cycle after. The suppress bit is a
// separate r3d_ram in m3_tilegen: two arrays in one module can be merged by
// Quartus 17 into one (r3d_ram's note), and a merged 16-bit array at 2 K deep
// would take 64 blocks.
//
// M10K PACKING. An M10K holds 10,240 bits only in its x5 / x10 / x20 / x40
// shapes; in the x1-x32 power-of-two shapes it holds 8,192. Left to itself
// Quartus 17 built the 15 colour bits from 8 K x 1 slices, 60 blocks (seed 1,
// first fit). Built as sixteen 2 K x 15 memories, each packs as 2 K x 5 x 3:
// 48 blocks, at the cost of a 16:1 read mux. (The max_depth attribute on one
// 32 K array is what I tried first: Quartus 17 ignored it, MAXIMUM_DEPTH 0.)
// The suppress bit is 32 K x 1, 4 blocks of 8 K x 1 either way. 52 in all,
// the least that holds 32 K x 16.

module m3_tilegen_pal (
  input  logic        clk,
  input  logic        we,
  input  logic [14:0] wa,
  input  logic [14:0] wd,
  input  logic [14:0] ra,
  output logic [14:0] q
);

  logic [14:0] bq [16];
  logic [3:0]  rsel;

  genvar gk;
  generate
    for (gk = 0; gk < 16; gk++) begin : g_bank
      localparam logic [3:0] K = gk;
      r3d_ram #(.W(15), .A(11), .SYNC(1'b1), .BLOCK(1'b1)) u_c (
        .clk, .we(we && (wa[14:11] == K)), .wa(wa[10:0]), .wd(wd), .ra(ra[10:0]), .rq(bq[gk]));
    end
  endgenerate

  always_ff @(posedge clk) rsel <= ra[14:11];
  assign q = bq[rsel];

endmodule
