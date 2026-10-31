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
// Leading-zero count as a balanced tree: log2(W) levels of a 2:1 mux, not a
// priority chain (the integer core's cntlzw is built the same way, R26). The
// input is padded on the right with ones to a power of two, so an all-zero
// input counts exactly W. A trailing-zero count is this module on the
// bit-reversed input.
//
// The Model 2 core's clz16 (m2_persp_recip) carries the warning that a loop
// scanning in the wrong direction finds the lowest set bit and passes every
// power-of-two test; the tree has no scan direction to get wrong.
//
// REG = 1 puts a register across the tree after level CUT (the count then
// comes a cycle later); clk is unused otherwise.

module ppc_fpu_lzc #(
  parameter int W   = 162,
  parameter int LW  = $clog2(W + 1),
  parameter bit REG = 1'b0,
  parameter int CUT = 4
) (
  input  logic          clk,
  input  logic [W-1:0]  x,
  output logic [LW-1:0] n
);

  localparam int LV = $clog2(W);     // levels
  localparam int N2 = 1 << LV;       // padded width

  logic [N2-1:0] xp;
  assign xp = {x, {(N2 - W){1'b1}}};

  localparam int K = REG ? CUT : 0;   // the level the second half starts from

  // level l: N2 >> l groups of 2^l bits, each with "has a one" and a count
  logic          v [LV+1][N2];
  logic [LV-1:0] c [LV+1][N2];
  // level K as the second half sees it: registered, or straight through
  logic          vk [N2 >> K];
  logic [LV-1:0] ck [N2 >> K];

  always_comb begin
    for (int l = 0; l <= LV; l++)
      for (int g = 0; g < N2; g++) begin
        v[l][g] = 1'b0;
        c[l][g] = '0;
      end
    for (int g = 0; g < N2; g++) v[0][g] = xp[g];
    for (int l = 1; l <= LV; l++)
      for (int g = 0; g < (N2 >> l); g++) begin
        if (l == K + 1 && K > 0) begin
          // the second half starts from the (registered) level K
          v[l][g] = vk[2*g+1] | vk[2*g];
          c[l][g] = vk[2*g+1] ? ck[2*g+1] : (ck[2*g] | (LV'(1) << (l - 1)));
        end else begin
          // group g at level l is groups 2g+1 (upper) and 2g (lower) of level l-1
          v[l][g] = v[l-1][2*g+1] | v[l-1][2*g];
          c[l][g] = v[l-1][2*g+1] ? c[l-1][2*g+1]
                                  : (c[l-1][2*g] | (LV'(1) << (l - 1)));
        end
      end
  end

  generate if (K > 0) begin : g_reg
    always_ff @(posedge clk)
      for (int g = 0; g < (N2 >> K); g++) begin
        vk[g] <= v[K][g];
        ck[g] <= c[K][g];
      end
  end else begin : g_comb
    always_comb
      for (int g = 0; g < N2; g++) begin
        vk[g] = 1'b0;
        ck[g] = '0;
      end
  end endgenerate

  // the root: count, or W for an all-zero input (only reachable with W = N2)
  assign n = v[LV][0] ? LW'(c[LV][0]) : LW'(W);

endmodule
