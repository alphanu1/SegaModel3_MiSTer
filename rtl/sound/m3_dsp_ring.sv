// SPDX-License-Identifier: GPL-3.0-or-later
//
// Sega Model 3 core for MiSTer FPGA
// Copyright (C) 2026 alphanu1
//
// ONE SLICE OF THE SOUND DSP'S RING (m3_scsp_dsp): W bits of every ring word, in M10K, with its own
// port registers. m3_scsp_dsp gathers the ring's port (address, write, word) into one register stage;
// each slice takes it into its own registers, kept apart (one copy a slice), so the slice's M10K and
// its registers can sit together wherever the fitter puts them, and the hop from the gathered stage
// is register to register. The word read is registered twice after the RAM (the second register is
// the slice's own, so the hop back to the engine starts at a register): an address in a_i in clock X
// is read at the end of X + 1, and its word is in q in X + 4.

`timescale 1ns/1ps

module m3_dsp_ring #(
  parameter int AW = 16,
  parameter int W  = 4
) (
  input  logic          clk,
  input  logic [AW-1:0] a_i,
  input  logic          we_i,
  input  logic [W-1:0]  d_i,
  output logic [W-1:0]  q
);

  (* preserve *) logic [AW-1:0] a;
  (* preserve *) logic          we;
  (* preserve *) logic [W-1:0]  d;
  // single address: a step's access, read or write, is at one offset; a word read during its own
  // write is never used, so no bypass (no_rw_check). Quartus builds it of 8 K x 1 blocks (W x 2^(AW - 13)
  // of them), as it built the whole ring: a 3-bit decode and an 8:1 select at the slice
  (* ramstyle = "M10K, no_rw_check" *) logic [W-1:0] ram [2 ** AW];
  logic [W-1:0] q1, q2;

  always_ff @(posedge clk) begin
    a  <= a_i;
    we <= we_i;
    d  <= d_i;
    if (we) ram[a] <= d;
    q1 <= ram[a];
    q2 <= q1;
    q  <= q2;
  end

endmodule
