// SPDX-License-Identifier: GPL-3.0-or-later
//
// Sega Model 3 core for MiSTer FPGA
// Copyright (C) 2026 alphanu1
//
// Standalone fit of ppc_core (M0-A2): ALM, M10K, DSP and Fmax at 100 MHz.
// Every port but the clock is a virtual pin (cpu_spike.qsf); the retire trace
// is left unconnected so it is trimmed, as it is in the core build.

module cpu_spike_top (
  input  logic        clk,
  input  logic        rst,
  output logic        i_req,
  output logic [31:0] i_addr,
  input  logic        i_valid,
  input  logic [31:0] i_data,
  output logic        d_req,
  output logic        d_we,
  output logic [31:0] d_addr,
  output logic [3:0]  d_be,
  output logic [31:0] d_wdata,
  input  logic        d_valid,
  input  logic [31:0] d_rdata,
  input  logic        tb_tick
);

  ppc_core u_cpu (
    .clk (clk), .rst (rst),
    .i_req (i_req), .i_addr (i_addr), .i_valid (i_valid), .i_data (i_data),
    .d_req (d_req), .d_we (d_we), .d_addr (d_addr), .d_be (d_be), .d_wdata (d_wdata),
    .d_valid (d_valid), .d_rdata (d_rdata),
    .tb_tick (tb_tick),
    .rt_valid (), .rt_last (), .rt_pc (), .rt_insn (), .rt_gpr_we (), .rt_gpr (), .rt_gpr_val (),
    .rt_cr (), .rt_xer (), .rt_lr (), .rt_ctr (), .rt_msr (), .rt_srr0 (), .rt_srr1 ()
  );

endmodule
