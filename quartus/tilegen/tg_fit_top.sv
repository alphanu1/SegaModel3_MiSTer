// SPDX-License-Identifier: GPL-3.0-or-later
//
// Sega Model 3 core for MiSTer FPGA
// Copyright (C) 2026 alphanu1
//
// Standalone fit wrapper for m3_tilegen: every port but clk is a virtual pin,
// and the block sits one level down so the fitter's entity table gives its ALM
// without the pins.

module tg_fit_top #(
  parameter bit SM = 1'b1   // 1: Supermodel switches (the default build), 0: MAME's
) (
  input  logic          clk,
  input  logic          rst_n,
  output logic          ready,
  input  logic          reg_we,
  input  logic [5:0]    reg_addr,
  input  logic [31:0]   reg_wdata,
  input  logic          pal_we,
  input  logic [14:0]   pal_addr,
  input  logic [15:0]   pal_wdata,
  input  logic          swap,
  input  logic          go,
  input  logic [8:0]    go_y,
  output logic          busy,
  output logic          overrun,
  input  logic          sc_rd,
  input  logic [8:0]    sc_x,
  output logic          o_v,
  output logic          o_drawn,
  output logic          o_above,
  output logic [7:0]    o_r,
  output logic [7:0]    o_g,
  output logic [7:0]    o_b,
  output logic          p_req,
  output logic [25:1]   p_addr,
  input  logic [63:0]   p_dout,
  input  logic          p_ack,
  output logic [15:0]   st_cycles,
  output logic [15:0]   st_reqs
);
  m3_tilegen #(
    .MASK_SCREEN(SM), .SCROLL_SM(SM), .RS_SWAP(SM), .STALE(SM), .CODE_WRAP(!SM), .EXPAND_SM(SM), .OFFSET(SM),
    .AW(25), .VBASE(25'h0)
  ) u_tg (.*);
endmodule
