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
// GPR file for a dual-issue core (study D-24 prototype, not in the core): 64
// x 32 in MLAB (32 GPRs and the TGPR shadows, as ppc_regfile.sv), NW write
// ports (1 or 2) and NR read ports, built by the study 8.4 technique: a bank
// per write port, a copy of each bank per read port, and a live-value table
// (LVT, flip-flops) saying which bank last wrote each register. A read is
// the bank the LVT names; the core forwards a same-edge write, so no
// read-during-write logic is needed (as ppc_regfile.sv). Two writes to one
// register on one edge are excluded by the pairing rule (no WAW in a pair).
//
// PMUX = 1 is the port-sharing form for pairs that read at most three GPRs
// between them: three ports whose addresses are each chosen from the six
// fields of the two instructions in decode by a registered 3-bit map, so no
// port is added; the operand side chooses its port the same way.

module ppc_regfile2w #(
  parameter int NW   = 2,
  parameter int NR   = 5,
  parameter int PMUX = 0
)(
  input  logic                clk,
  input  logic [NR-1:0][5:0]  ra,         // PMUX = 0: one address per read port
  input  logic [5:0][5:0]     fa,         // PMUX = 1: the six candidate fields
  input  logic [2:0][2:0]     pmap,       // PMUX = 1: field feeding each of three ports
  output logic [NR-1:0][31:0] q,
  input  logic [NW-1:0]       we,
  input  logic [NW-1:0][5:0]  wa,
  input  logic [NW-1:0][31:0] wd
);

  logic [NR-1:0][5:0] rad;
  always_comb begin
    rad = ra;
    if (PMUX != 0)
      for (int p = 0; p < 3 && p < NR; p++) rad[p] = fa[pmap[p]];
  end

  logic [NW-1:0][NR-1:0][31:0] bq;
  genvar gw, gr;
  generate
    for (gw = 0; gw < NW; gw++) begin : g_w
      for (gr = 0; gr < NR; gr++) begin : g_r
        ppc_mlab64x32 u_m (.clk (clk), .we (we[gw]), .wa (wa[gw]), .wd (wd[gw]), .ra (rad[gr]), .q (bq[gw][gr]));
      end
    end
  endgenerate

  generate
    if (NW == 1) begin : g_one
      assign q = bq[0];
    end else begin : g_lvt
      logic [63:0] lvt;                       // 1: bank 1 holds the live value
      always_ff @(posedge clk) begin
        if (we[0]) lvt[wa[0]] <= 1'b0;
        if (we[1]) lvt[wa[1]] <= 1'b1;
      end
      always_comb
        for (int r = 0; r < NR; r++) q[r] = lvt[rad[r]] ? bq[1][r] : bq[0][r];
    end
  endgenerate

endmodule

// one bank copy: 64 x 32 in MLAB, asynchronous read, as ppc_regfile.sv's arrays
module ppc_mlab64x32 (
  input  logic        clk,
  input  logic        we,
  input  logic [5:0]  wa,
  input  logic [31:0] wd,
  input  logic [5:0]  ra,
  output logic [31:0] q
);
  (* ramstyle = "MLAB, no_rw_check" *) logic [31:0] m [0:63];
  always_ff @(posedge clk) if (we) m[wa] <= wd;
  assign q = m[ra];
endmodule
