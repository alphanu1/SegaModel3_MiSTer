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
// sys140 (D-81): one register stage between an m2_sdram's pin outputs and the pins (the commands, the address,
// the byte mask, the write data and its output enable, all of them by one clock, so every interval between them
// is the controller's own). These registers are the ones that go into the I/O cells (FAST_OUTPUT_REGISTER), and
// the controller's own become core registers the fitter may place by the controller: at 140 MHz its state into
// the I/O-cell registers was -2.0 to -3.3 ns, 8-11 ns of route from where the controller sits to the second
// SDRAM's pins. The controller is built with XLAT 1, so its reads are captured a clock deeper.
// Out of reset (and before the controller's first edge) the stage shows a NOP with the chip deselected.

module m3_sdr_opipe (
  input  logic        clk,
  input  logic        i_cke, i_cs_n, i_ras_n, i_cas_n, i_we_n,
  input  logic [1:0]  i_ba,
  input  logic [12:0] i_a,
  input  logic [1:0]  i_dqm,
  input  logic [15:0] i_dq_o,
  input  logic        i_dq_oe,
  output logic        sd_cke, sd_cs_n, sd_ras_n, sd_cas_n, sd_we_n,
  output logic [1:0]  sd_ba,
  output logic [12:0] sd_a,
  output logic [1:0]  sd_dqm,
  output logic [15:0] sd_dq_o,
  output logic        sd_dq_oe
);
  initial begin
    sd_cke = 1'b1; {sd_cs_n, sd_ras_n, sd_cas_n, sd_we_n} = 4'b1111; sd_dq_oe = 1'b0; sd_dqm = 2'b11;
    sd_ba = '0; sd_a = '0; sd_dq_o = '0;
  end
  always_ff @(posedge clk) begin
    sd_cke <= i_cke; sd_cs_n <= i_cs_n; sd_ras_n <= i_ras_n; sd_cas_n <= i_cas_n; sd_we_n <= i_we_n;
    sd_ba <= i_ba; sd_a <= i_a; sd_dqm <= i_dqm; sd_dq_o <= i_dq_o; sd_dq_oe <= i_dq_oe;
  end
endmodule
