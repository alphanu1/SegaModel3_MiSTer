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
// One write port, one read port. SYNC = 0: unregistered read, an MLAB.
// SYNC = 1: registered read address, data the cycle after (M10K, or MLAB with
// BLOCK = 0). A module of its own so that two copies written alike (a second
// read port) stay two memories; inline arrays were merged by Quartus 17 into
// one, and the second port into registers. Read-during-write is don't-care:
// callers never read a word in the cycle it is written.

module r3d_ram #(
  parameter int W = 32,
  parameter int A = 5,
  parameter bit SYNC = 1'b0,
  parameter bit BLOCK = 1'b0
) (
  input  logic         clk,
  input  logic         we,
  input  logic [A-1:0] wa,
  input  logic [W-1:0] wd,
  input  logic [A-1:0] ra,
  output logic [W-1:0] rq
);
  generate if (BLOCK) begin : g_m10k
    (* ramstyle = "M10K, no_rw_check" *) logic [W-1:0] m [0:(1 << A) - 1];
    always_ff @(posedge clk) begin
      if (we) m[wa] <= wd;
      rq <= m[ra];
    end
  end else if (SYNC) begin : g_mlab_sync
    (* ramstyle = "MLAB, no_rw_check" *) logic [W-1:0] m [0:(1 << A) - 1];
    logic [A-1:0] ra_q;
    always_ff @(posedge clk) begin
      if (we) m[wa] <= wd;
      ra_q <= ra;
    end
    assign rq = m[ra_q];
  end else begin : g_mlab
    (* ramstyle = "MLAB, no_rw_check" *) logic [W-1:0] m [0:(1 << A) - 1];
    always_ff @(posedge clk) if (we) m[wa] <= wd;
    assign rq = m[ra];
  end endgenerate
endmodule

// A fixed delay of D steps (D >= 3) for a W-bit word, as a chain of D registers all enabled by en would give it
// (q is d of D enabled clocks before), in an M10K (area pass, R1xx): the first D - 2 steps in the block's memory,
// the last two in its address and output registers, so q is the block's output register and nothing of the line
// is in ALMs. Each enabled clock writes the block at p and reads it at p - (D - 2): the word written D - 2 steps
// before, which with 2^A >= D has not been written over, at an address that is never the one being written. q
// then follows it by the output register's step. Nothing resets: as the chain, q is only defined D steps after the
// clock starts, whatever value p starts at. (The first form, an MLAB with its output register, freed no ALMs: an
// MLAB is a whole LAB, here used 4 to 10 deep of its 32.)
module r3d_dline #(
  parameter int W = 32,
  parameter int D = 4
) (
  input  logic         clk,
  input  logic         en,
  input  logic [W-1:0] d,
  output logic [W-1:0] q
);
  localparam int A = $clog2(D);
  logic [A-1:0] p;                 // any start: the delay holds from D steps on whatever p starts at
  (* ramstyle = "M10K, no_rw_check" *) logic [W-1:0] m [0:(1 << A) - 1];
  logic [W-1:0] q1;
  always_ff @(posedge clk) if (en) begin
    p  <= p + 1'b1;
    m[p] <= d;
    q1 <= m[p - A'(D - 2)];
    q  <= q1;
  end
endmodule
