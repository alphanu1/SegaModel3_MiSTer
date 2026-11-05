// SPDX-License-Identifier: GPL-3.0-or-later
//
// Sega Model 3 core for MiSTer FPGA
// Copyright (C) 2026 alphanu1
//
// Bench top for m3_sdr_port: the port on m2_sdram's port 0 (as m3_system
// builds it), port 1 a second reader the C++ drives for contention (the tile
// generator's 4-word bursts), the SDRAM at its pins.
module m3_sdr_tb (
  input  logic        clk,
  input  logic        rst_n,
  output logic        ready,
  input  logic        s_req,
  input  logic [1:0]  s_cmd,
  input  logic [26:0] s_sa,
  input  logic        s_swap,
  input  logic        s_zero,
  input  logic [3:0]  s_be,
  input  logic [31:0] s_wdata,
  output logic        s_ack,
  output logic        s_wnext,
  output logic        s_rvalid,
  output logic [31:0] s_rdata,
  output logic        s_idle,
  input  logic        q_req,
  input  logic [26:1] q_addr,
  output logic        q_ack,
  output logic [63:0] q_dout,
  output logic        sd_cke, sd_cs_n, sd_ras_n, sd_cas_n, sd_we_n,
  output logic [1:0]  sd_ba,
  output logic [12:0] sd_a,
  output logic [15:0] sd_dq_o,
  input  logic [15:0] sd_dq_i
);
  logic        p_req, p_we, p_long, p_lo;
  logic [26:1] p_addr;
  logic [15:0] p_din;
  logic [1:0]  p_be;
  logic [3:0]  wb_col;
  logic [15:0] wb_din;
  logic [1:0]  wb_be;
  logic [1:0]  ack, lo;
  logic [1:0][63:0] dout;

  m3_sdr_port #(.AW(26)) u_port (
    .clk, .rst(!rst_n),
    .s_req, .s_cmd, .s_sa, .s_swap, .s_zero, .s_be, .s_wdata, .s_ack, .s_wnext, .s_rvalid, .s_rdata, .s_idle,
    .p_req, .p_we, .p_addr, .p_din, .p_be, .p_long, .p_dout(dout[0]), .p_lo(lo[0]), .p_ack(ack[0]), .wb_col, .wb_din, .wb_be);

  m2_sdram #(.COL_BITS(10), .NP(2), .T_REFI(770), .ACK_HOLD(1), .DUAL(1'b1), .MASK_ON_A(1'b1), .INIT_NOP(200), .WB(2'b01)) u_sdram (
    .clk, .rst_n, .ready, .rd_lat_sel(3'd2),
    .sd_cke, .sd_cs_n, .sd_ras_n, .sd_cas_n, .sd_we_n, .sd_ba, .sd_a, .sd_dqm(), .sd_dq_o, .sd_dq_oe(), .sd_dq_i,
    .wr_req(1'b0), .wr_addr('0), .wr_din('0), .wr_be('0), .wr_ack(),
    .p_req({q_req, p_req}), .p_we({1'b0, p_we}), .p_addr({q_addr, p_addr}), .p_din({16'd0, p_din}), .p_be({2'b00, p_be}),
    .p_dout(dout), .p_ack(ack), .p_long({1'b0, p_long}), .p_lo(lo), .wb_col, .wb_din, .wb_be, .dbg_req(), .dbg_grant());
  assign q_ack = ack[1];
  assign q_dout = dout[1];
endmodule
