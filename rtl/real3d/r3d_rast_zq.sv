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
// Real3D rasteriser: the pairs that passed the z band, queued for the pixel
// stage (study R149). The z band tests a pair a clock and the pixel stage
// draws a pixel a clock. In overdraw-heavy frames, where most pairs fail Z,
// the pixel stage starved behind runs of rejected pairs while the z band
// waited on it behind runs of passing ones. The queue lets the z band run
// ahead and the pixel stage catch up.
//
// First word fall-through: a word pushed is the head the next clock, as a
// register would be, so the queue adds no clock to the z band -> pixel path,
// and it holds D words in all. Every word goes into the RAM (M10K or MLAB, by
// RAM), and the RAM's read is the head: it reads the next clock's head every
// clock (rp, one on when this clock pops), so its registered address is
// always a clock ahead. The one word it cannot give is one written in the
// clock before it is the head: a push with nothing left after the pop. That
// word is the head for one clock from bd, which takes every word pushed
// (study R156: one register word, where the head and the next word in
// registers were two, and the same clocks as an ideal queue).

module r3d_rast_zq #(
  parameter int D   = 64,        // words held, a power of two, 4 or more
  parameter int W   = 113,
  parameter     RAM = "M10K"     // the RAM's style: "M10K" or "MLAB"
) (
  input  logic         clk,
  input  logic         rst,
  input  logic         i_valid,
  output logic         i_ready,
  input  logic [W-1:0] i_data,
  output logic         o_valid,
  input  logic         o_ready,
  output logic [W-1:0] o_data
);
  localparam int A = $clog2(D);

  logic [A-1:0] wp, rp;
  logic [A:0]   n;               // words held
  logic         o_v;             // n != 0, as a register
  logic         byp;             // the head is bd: it was pushed last clock, into a queue left empty
  logic [W-1:0] bd, q;

  assign i_ready = n != (A+1)'(D);

  wire          push = i_valid && i_ready;
  wire          pop  = o_v && o_ready;
  wire [A-1:0]  ra   = rp + A'(pop);              // the next clock's head
  wire          none = n == (A+1)'(pop);          // nothing held after this clock's pop

  assign o_valid = o_v;
  assign o_data  = byp ? bd : q;

  generate if (RAM == "MLAB") begin : g_mlab
    (* ramstyle = "MLAB, no_rw_check" *) logic [W-1:0] m [D];
    always_ff @(posedge clk) begin
      if (push) m[wp] <= i_data;
      q <= m[ra];
    end
  end else begin : g_m10k
    (* ramstyle = "M10K, no_rw_check" *) logic [W-1:0] m [D];
    always_ff @(posedge clk) begin
      if (push) m[wp] <= i_data;
      q <= m[ra];
    end
  end endgenerate

  always_ff @(posedge clk) begin
    if (rst) begin
      o_v <= 1'b0; byp <= 1'b0; wp <= '0; rp <= '0; n <= '0;
    end else begin
      o_v <= push || !none;
      byp <= push && none;                        // (the read this clock is of the word being written)
      if (push) wp <= wp + 1'b1;
      if (pop)  rp <= rp + 1'b1;
      n <= n + (A+1)'(push) - (A+1)'(pop);
    end
  end
  // the word itself holds no reset
  always_ff @(posedge clk) bd <= i_data;

`ifdef VERILATOR
  always_ff @(posedge clk)
    if (!rst && (o_v != (n != '0) || wp != rp + A'(n))) $fatal(1, "r3d_rast_zq: the count");
`endif
endmodule
