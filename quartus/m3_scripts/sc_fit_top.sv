// SPDX-License-Identifier: GPL-3.0-or-later
//
// Sega Model 3 core for MiSTer FPGA
// Copyright (C) 2026 alphanu1
//
// Standalone fit of m3_scripts (M0-F). Every port but clk is a virtual pin
// (fit.sh), so the block's own row in the entity table is its ALM without pins.

module sc_fit_top #(
  parameter bit ENABLE = 1'b1,
  parameter bit LINE   = 1'b1,
  parameter bit INT_OP = 1'b1,
  parameter bit RD_ALL = 1'b0
)(
  input  logic        clk,
  input  logic        rst,
  input  logic        c_req, c_we,
  input  logic [5:0]  c_idx,
  input  logic [3:0]  c_be,
  input  logic [31:0] c_wdata,
  output logic        c_ack,
  output logic [31:0] c_rdata,
  output logic        win_en,
  output logic [31:8] win_base,
  input  logic        s_req, s_we,
  input  logic [6:2]  s_addr,
  input  logic [3:0]  s_be,
  input  logic [31:0] s_wdata,
  output logic        s_ack,
  output logic [31:0] s_rdata,
  output logic        m_req,
  output logic [1:0]  m_cmd,
  output logic [31:0] m_addr,
  output logic [3:0]  m_be,
  output logic [31:0] m_wdata,
  input  logic        m_ack, m_wnext, m_rvalid,
  input  logic [31:0] m_rdata,
  output logic        irq,
  output logic        busy
);
  m3_scripts #(.ENABLE(ENABLE), .LINE(LINE), .INT_OP(INT_OP), .RD_ALL(RD_ALL)) u_sc (.*);
endmodule
