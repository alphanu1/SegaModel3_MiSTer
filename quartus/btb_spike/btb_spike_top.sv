// SPDX-License-Identifier: GPL-3.0-or-later
//
// Sega Model 3 core for MiSTer FPGA
// Copyright (C) 2026 alphanu1
//
// Standalone fit of rtl/cpu/ppc_btb.sv: every input and output registered
// once at the top, so the BTB's own paths are timed register to register.

module btb_spike_top #(
  parameter int N = 256, parameter int XW = 0, parameter int STYLE = 0
)(
  input  logic          clk,
  input  logic          rst,
  input  logic [31:2]   rd_pc,
  input  logic [1:0]    ctx,
  input  logic          cap, p_take, kill,
  input  logic          w_en, w_v, w_reg,
  input  logic [31:2]   w_pc, w_tgt,
  input  logic [1:0]    w_ctx,
  input  logic [((XW > 0) ? XW : 1)-1:0] w_x,
  input  logic          flush,
  output logic          p_v, p_reg, busy,
  output logic [31:2]   p_tgt,
  output logic [((XW > 0) ? XW : 1)-1:0] p_x
);
  localparam int XB = (XW > 0) ? XW : 1;
  logic          r_rst, r_cap, r_take, r_kill, r_wen, r_wv, r_wreg, r_flush;
  logic [31:2]   r_pc, r_wpc, r_wtgt;
  logic [1:0]    r_ctx, r_wctx;
  logic [XB-1:0] r_wx;
  logic          o_v, o_reg, o_busy;
  logic [31:2]   o_tgt;
  logic [XB-1:0] o_x;
  always_ff @(posedge clk) begin
    r_rst <= rst; r_pc <= rd_pc; r_ctx <= ctx; r_cap <= cap; r_take <= p_take; r_kill <= kill;
    r_wen <= w_en; r_wv <= w_v; r_wreg <= w_reg; r_wpc <= w_pc; r_wtgt <= w_tgt; r_wctx <= w_ctx;
    r_wx <= w_x; r_flush <= flush;
    p_v <= o_v; p_reg <= o_reg; busy <= o_busy; p_tgt <= o_tgt; p_x <= o_x;
  end
  ppc_btb #(.N (N), .XW (XW), .STYLE (STYLE)) u_btb (
    .clk (clk), .rst (r_rst), .rd_pc (r_pc), .ctx (r_ctx), .cap (r_cap),
    .p_v (o_v), .p_tgt (o_tgt), .p_reg (o_reg), .p_x (o_x), .p_take (r_take), .kill (r_kill),
    .w_en (r_wen), .w_v (r_wv), .w_pc (r_wpc), .w_ctx (r_wctx), .w_reg (r_wreg), .w_tgt (r_wtgt), .w_x (r_wx),
    .flush (r_flush), .busy (o_busy));
endmodule
