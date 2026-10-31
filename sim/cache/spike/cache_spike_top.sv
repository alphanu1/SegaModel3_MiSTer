// SPDX-License-Identifier: GPL-3.0-or-later
//
// Sega Model 3 core for MiSTer FPGA
// Copyright (C) 2026 alphanu1
//
// Standalone fit of ppc_icache and ppc_dcache. The CPU-side inputs come from
// registers, as ppc_core drives them; the memory side is virtual pins, its
// inputs registered once as a controller would present them.

module cache_spike_top (
  input  logic        clk,
  input  logic        rst,
  input  logic        c_i_req, c_i_seq,
  input  logic [31:0] c_i_addr,
  input  logic [3:0]  c_i_wimg,
  output logic        c_i_valid,
  output logic [31:0] c_i_data,
  input  logic        c_d_req, c_d_we,
  input  logic [31:0] c_d_addr, c_d_wdata,
  input  logic [3:0]  c_d_be, c_d_wimg,
  input  logic [2:0]  c_d_op,
  output logic        c_d_valid,
  output logic        c_d_rdy,
  output logic [31:0] c_d_rdata,
  input  logic [6:0]  c_hid,
  input  logic        c_icbi_req,
  input  logic [6:0]  c_icbi_set,
  output logic        c_icbi_ack, c_icfi_ack, c_dcfi_ack, c_i_idle, c_d_idle,
  output logic        mi_req,
  output logic [1:0]  mi_cmd,
  output logic [31:0] mi_addr,
  output logic [3:0]  mi_be,
  input  logic        mi_ack, mi_rvalid,
  input  logic [31:0] mi_rdata,
  output logic        md_req,
  output logic [1:0]  md_cmd,
  output logic [31:0] md_addr, md_wdata,
  output logic [3:0]  md_be,
  input  logic        md_ack, md_wnext, md_rvalid,
  input  logic [31:0] md_rdata
);

  logic        i_req, i_seq, d_req, d_we, icbi_req;
  logic [31:0] i_addr, d_addr, d_wdata;
  logic [3:0]  i_wimg, d_be, d_wimg;
  logic [2:0]  d_op;
  logic [6:0]  hid, icbi_set;
  logic        mi_ack_q, mi_rvalid_q, md_ack_q, md_wnext_q, md_rvalid_q;
  logic [31:0] mi_rdata_q, md_rdata_q;
  always_ff @(posedge clk) begin
    i_req <= c_i_req; i_seq <= c_i_seq; i_addr <= c_i_addr; i_wimg <= c_i_wimg;
    d_req <= c_d_req; d_we <= c_d_we; d_addr <= c_d_addr; d_wdata <= c_d_wdata; d_be <= c_d_be;
    d_wimg <= c_d_wimg; d_op <= c_d_op; hid <= c_hid; icbi_req <= c_icbi_req; icbi_set <= c_icbi_set;
    mi_ack_q <= mi_ack; mi_rvalid_q <= mi_rvalid; mi_rdata_q <= mi_rdata;
    md_ack_q <= md_ack; md_wnext_q <= md_wnext; md_rvalid_q <= md_rvalid; md_rdata_q <= md_rdata;
  end

  ppc_icache u_ic (
    .clk (clk), .rst (rst),
    .i_req (i_req), .i_addr (i_addr), .i_seq (i_seq), .i_wimg (i_wimg), .i_valid (c_i_valid), .i_data (c_i_data),
    .hid_ice (hid[0]), .hid_ilock (hid[1]), .hid_icfi (hid[2]), .icfi_ack (c_icfi_ack),
    .icbi_req (icbi_req), .icbi_set (icbi_set), .icbi_ack (c_icbi_ack), .i_idle (c_i_idle),
    .m_req (mi_req), .m_cmd (mi_cmd), .m_addr (mi_addr), .m_be (mi_be),
    .m_ack (mi_ack_q), .m_rvalid (mi_rvalid_q), .m_rdata (mi_rdata_q),
    .t_alloc (), .t_alloc_set (), .t_alloc_way (), .t_alloc_tag ()
  );

  ppc_dcache u_dc (
    .clk (clk), .rst (rst),
    .d_req (d_req), .d_we (d_we), .d_addr (d_addr), .d_be (d_be), .d_wdata (d_wdata),
    .d_wimg (d_wimg), .d_op (d_op), .d_rdy (c_d_rdy), .d_valid (c_d_valid), .d_rdata (c_d_rdata),
    .hid_dce (hid[3]), .hid_dlock (hid[4]), .hid_dcfi (hid[5]), .hid_noopti (hid[6]),
    .dcfi_ack (c_dcfi_ack), .d_idle (c_d_idle),
    .m_req (md_req), .m_cmd (md_cmd), .m_addr (md_addr), .m_be (md_be), .m_wdata (md_wdata),
    .m_ack (md_ack_q), .m_wnext (md_wnext_q), .m_rvalid (md_rvalid_q), .m_rdata (md_rdata_q),
    .t_alloc (), .t_alloc_set (), .t_alloc_way (), .t_alloc_tag ()
  );

endmodule
