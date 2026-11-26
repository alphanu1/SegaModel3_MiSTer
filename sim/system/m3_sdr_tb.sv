// SPDX-License-Identifier: GPL-3.0-or-later
//
// Sega Model 3 core for MiSTer FPGA
// Copyright (C) 2026 alphanu1
//
// Bench top for m3_sdr_port: the port on m2_sdram's port 0 (as m3_system
// builds it), port 1 a second reader the C++ drives for contention (the tile
// generator's 4-word bursts), the SDRAM at its pins.
module m3_sdr_tb #(parameter bit L16 = 1'b0, parameter bit BXOR = 1'b0, parameter bit LW1 = 1'b0, parameter bit PAIR = 1'b0, parameter bit RA = 1'b0, parameter bit PW = 1'b0, parameter bit RA_CONF = 1'b0,
  parameter bit FWD = 1'b0, parameter bit FREQ = 1'b0, parameter bit FDSP = 1'b0, parameter bit HITRD = 1'b0, parameter bit FAST = 1'b0, parameter int PW_DLY = 0, parameter bit R3P = 1'b0,
  parameter bit BR = 1'b0, parameter int CL = 2,
  parameter bit CAPPIPE = 1'b0, parameter int T_RCD = 2, parameter int T_RP = 2, parameter bit OPIPE = 1'b0) (   // (sys140: m2_sdram's CAPPIPE, T_RCD, T_RP) FAST, R3P: the bus's (no bus here); BR, the bridges pass: m2_sdram SHD
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
  input  logic [3:0]  s_wbe,
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

  m3_sdr_port #(.AW(26), .L16(L16), .LW1(LW1), .PAIR(PAIR), .RA(RA), .PW(PW), .RA_CONF(RA_CONF), .FWD(FWD), .FREQ(FREQ), .PW_DLY(PW_DLY)) u_port (
    .clk, .rst(!rst_n), .ra_clr(1'b0),
    .s_req, .s_cmd, .s_sa, .s_swap, .s_zero, .s_be, .s_wdata, .s_wbe, .s_ack, .s_wnext, .s_rvalid, .s_rdata, .s_idle,
    .p_req, .p_we, .p_addr, .p_din, .p_be, .p_long, .p_dout(dout[0]), .p_lo(lo[0]), .p_ack(ack[0]), .wb_col, .wb_din, .wb_be);

  // (sys140) OPIPE: the pin stage, as m3_system builds it
  logic q1_cke, q1_cs_n, q1_ras_n, q1_cas_n, q1_we_n; logic [1:0] q1_ba; logic [12:0] q1_a; logic [15:0] q1_dq_o;
  generate if (OPIPE) begin : g_op1
    m3_sdr_opipe u_op1 (.clk, .i_cke(q1_cke), .i_cs_n(q1_cs_n), .i_ras_n(q1_ras_n), .i_cas_n(q1_cas_n), .i_we_n(q1_we_n),
      .i_ba(q1_ba), .i_a(q1_a), .i_dqm(2'b00), .i_dq_o(q1_dq_o), .i_dq_oe(1'b0),
      .sd_cke, .sd_cs_n, .sd_ras_n, .sd_cas_n, .sd_we_n, .sd_ba, .sd_a, .sd_dqm(), .sd_dq_o, .sd_dq_oe());
  end else begin : g_nop1
    assign {sd_cke, sd_cs_n, sd_ras_n, sd_cas_n, sd_we_n, sd_ba, sd_a, sd_dq_o} = {q1_cke, q1_cs_n, q1_ras_n, q1_cas_n, q1_we_n, q1_ba, q1_a, q1_dq_o};
  end endgenerate
  m2_sdram #(.COL_BITS(10), .NP(2), .CL(CL), .T_REFI(770), .CAPPIPE(CAPPIPE), .XLAT(OPIPE ? 1 : 0), .T_RCD(T_RCD), .T_RP(T_RP), .ACK_HOLD(1), .DUAL(1'b1), .MASK_ON_A(1'b1), .INIT_NOP(200), .WB(2'b01), .L16({1'b0, L16}), .BXOR(BXOR), .PAIR({1'b0, PAIR}), .FDSP(FDSP), .HITRD(HITRD), .SHD(BR ? 2'b11 : 2'b00)) u_sdram (
    .clk, .clk_cap(clk), .rst_n, .ready, .rd_lat_sel(3'd2),
    .sd_cke(q1_cke), .sd_cs_n(q1_cs_n), .sd_ras_n(q1_ras_n), .sd_cas_n(q1_cas_n), .sd_we_n(q1_we_n), .sd_ba(q1_ba), .sd_a(q1_a),
    .sd_dqm(), .sd_dq_o(q1_dq_o), .sd_dq_oe(), .sd_dq_i,
    .wr_req(1'b0), .wr_addr('0), .wr_din('0), .wr_be('0), .wr_ack(),
    .p_req({q_req, p_req}), .p_we({1'b0, p_we}), .p_addr({q_addr, p_addr}), .p_din({16'd0, p_din}), .p_be({2'b00, p_be}),
    .p_dout(dout), .p_ack(ack), .p_long({1'b0, p_long}), .p_lo(lo), .wb_col, .wb_din, .wb_be, .dbg_req(), .dbg_grant());
  assign q_ack = ack[1];
  assign q_dout = dout[1];
endmodule
