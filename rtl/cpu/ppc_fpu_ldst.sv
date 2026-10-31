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
// Single-precision load and store conversions, combinational, for the core's
// load/store path. No exceptions, no FPSCR [PEM 3.3.4]. lfd, stfd and stfiwx
// need none (copies; stfiwx stores frS[32-63]).
//
// lfs [PEM D.6]: bit replication of WORD[1] into the exponent for normal,
// zero, infinity and NaN (an SNaN stays an SNaN); a single denormal is
// normalised into a double with its exponent below -126.
//
// stfs [PEM D.7]: bit selection, never rounding. Biased exponent above 896
// (or a zero): WORD = frS[0-1] || frS[5-34], which also covers values too big
// for single. 874..896: shifted right into a single denormal, the bits shifted
// out lost. UNDEFINED below 874 [PEM D-16]: Supermodel stores (float)x in the
// host's round-to-nearest -- the smallest denormal for a value strictly
// between 2^-150 and 2^-149 (biased 873, fraction nonzero), else zero.
//
// The 603e may take up to 24 cycles for a single denormal [603e 2.3.4.2].
// PIPE = 0: one cycle of logic. lfs (count, shift, pack) measured 8.89 ns
// from a register to a register, 0.56 ns over 120 MHz; PIPE = 1 registers the
// count and the classification and shifts and packs a cycle later.

module ppc_fpu_ldst #(
  parameter bit PIPE = 1'b0
) (
  input  logic        clk,
  input  logic [31:0] lfs_word,
  output logic [63:0] lfs_dbl,
  input  logic [63:0] stfs_dbl,
  output logic [31:0] stfs_word
);

  // ---------------------------------------------------------------- lfs
  logic [4:0] llz0;
  ppc_fpu_lzc #(.W(23)) u_lz (.clk (1'b0), .x(lfs_word[22:0]), .n(llz0));
  wire lden0 = (lfs_word[30:23] == 8'd0) && (lfs_word[22:0] != 23'd0);
  wire lspc0 = (lfs_word[30:23] == 8'd0) || (lfs_word[30:23] == 8'hFF);

  // --------------------------------------------------------------- stfs
  wire [10:0] se0 = stfs_dbl[62:52];
  // case 0: no denormalisation (also zero, infinity, NaN, too big);
  // 1: 874..896, shift 1.f right by 897 - e (1..23); 2: below, undefined
  wire [1:0] scase0 = (se0 > 11'd896 || stfs_dbl[62:0] == 63'd0) ? 2'd0 : (se0 >= 11'd874) ? 2'd1 : 2'd2;
  wire [4:0] dsh0   = 5'(11'd897 - se0);
  wire       s873   = (se0 == 11'd873) && (stfs_dbl[51:0] != 52'd0);

  logic [31:0] lw;
  logic [4:0]  llz, dsh;
  logic        lden, lspc, sn873;
  logic [1:0]  scase;
  logic [63:0] sd;
  generate if (PIPE) begin : g_pipe
    always_ff @(posedge clk) begin
      lw <= lfs_word;  llz <= llz0;  lden <= lden0;  lspc <= lspc0;
      sd <= stfs_dbl;  scase <= scase0;  dsh <= dsh0;  sn873 <= s873;
    end
  end else begin : g_comb
    always_comb begin
      lw = lfs_word;  llz = llz0;  lden = lden0;  lspc = lspc0;
      sd = stfs_dbl;  scase = scase0;  dsh = dsh0;  sn873 = s873;
    end
  end endgenerate

  wire [22:0] lfn = lw[22:0] << llz;      // leading one at bit 22
  // the denormal 0.f * 2^-126 = 1.f' * 2^(-127 - llz): biased 896 - llz
  wire [10:0] lde = 11'd896 - {6'd0, llz};
  always_comb begin
    if (lden)
      lfs_dbl = {lw[31], lde, lfn[21:0], 30'd0};
    else
      lfs_dbl = {lw[31], lw[30], {3{lspc ? lw[30] : ~lw[30]}}, lw[29:0], 29'd0};
  end

  wire [52:0] sden = {1'b1, sd[51:0]} >> dsh;
  always_comb begin
    case (scase)
      2'd0:    stfs_word = {sd[63:62], sd[58:29]};
      2'd1:    stfs_word = {sd[63], 8'd0, sden[51:29]};
      default: stfs_word = {sd[63], 30'd0, sn873};
    endcase
  end

endmodule
