// SPDX-License-Identifier: GPL-3.0-or-later
//
// Sega Model 3 core for MiSTer FPGA
// Copyright (C) 2026 alphanu1
//
// Standalone fit of the I-cache as built (PR = 0, rtl/cpu/ppc_icache.sv) or
// with the pre-read prototype (PR = 1, rtl/cpu/ppc_icache_pr.sv), every input
// and output registered once here, so the two compare like for like.

module icache_spike_top #(parameter int PR = 0) (
  input  logic        clk, rst,
  input  logic        i_req, i_seq, i_pv, i_pre,
  input  logic [31:0] i_addr,
  input  logic [11:2] i_pidx,
  input  logic [3:0]  i_wimg,
  output logic        i_valid,
  output logic [31:0] i_data,
  input  logic [2:0]  hid,
  input  logic        icbi_req,
  input  logic [6:0]  icbi_set,
  output logic        icbi_ack, icfi_ack, i_idle,
  output logic        m_req,
  output logic [1:0]  m_cmd,
  output logic [31:0] m_addr,
  output logic [3:0]  m_be,
  input  logic        m_ack, m_rvalid,
  input  logic [31:0] m_rdata
);
  logic        r_rst, r_req, r_seq, r_pv, r_pre, r_icbi, r_ack, r_rv;
  logic [31:0] r_addr, r_mrd;
  logic [11:2] r_pidx;
  logic [3:0]  r_wimg;
  logic [2:0]  r_hid;
  logic [6:0]  r_set;
  logic        o_valid, o_iack, o_fack, o_idle, o_mreq;
  logic [31:0] o_data, o_maddr;
  logic [1:0]  o_mcmd;
  logic [3:0]  o_mbe;
  always_ff @(posedge clk) begin
    r_rst <= rst; r_req <= i_req; r_seq <= i_seq; r_pv <= i_pv; r_pre <= i_pre; r_addr <= i_addr; r_pidx <= i_pidx;
    r_wimg <= i_wimg; r_hid <= hid; r_icbi <= icbi_req; r_set <= icbi_set; r_ack <= m_ack; r_rv <= m_rvalid; r_mrd <= m_rdata;
    i_valid <= o_valid; i_data <= o_data; icbi_ack <= o_iack; icfi_ack <= o_fack; i_idle <= o_idle;
    m_req <= o_mreq; m_cmd <= o_mcmd; m_addr <= o_maddr; m_be <= o_mbe;
  end
  generate
    if (PR != 0) begin : g_pr
      ppc_icache_pr u_ic (.clk, .rst (r_rst), .i_req (r_req), .i_addr (r_addr), .i_seq (r_seq), .i_pv (r_pv), .i_pidx (r_pidx),
        .i_pre (r_pre), .i_wimg (r_wimg), .i_valid (o_valid), .i_data (o_data), .hid_ice (r_hid[0]), .hid_ilock (r_hid[1]),
        .hid_icfi (r_hid[2]), .icfi_ack (o_fack), .icbi_req (r_icbi), .icbi_set (r_set), .icbi_ack (o_iack), .i_idle (o_idle),
        .m_req (o_mreq), .m_cmd (o_mcmd), .m_addr (o_maddr), .m_be (o_mbe), .m_ack (r_ack), .m_rvalid (r_rv), .m_rdata (r_mrd),
        .t_alloc (), .t_alloc_set (), .t_alloc_way (), .t_alloc_tag ());
    end else begin : g_base
      ppc_icache u_ic (.clk, .rst (r_rst), .i_req (r_req), .i_addr (r_addr), .i_seq (r_seq),
        .i_wimg (r_wimg), .i_valid (o_valid), .i_data (o_data), .hid_ice (r_hid[0]), .hid_ilock (r_hid[1]),
        .hid_icfi (r_hid[2]), .icfi_ack (o_fack), .icbi_req (r_icbi), .icbi_set (r_set), .icbi_ack (o_iack), .i_idle (o_idle),
        .m_req (o_mreq), .m_cmd (o_mcmd), .m_addr (o_maddr), .m_be (o_mbe), .m_ack (r_ack), .m_rvalid (r_rv), .m_rdata (r_mrd),
        .t_alloc (), .t_alloc_set (), .t_alloc_way (), .t_alloc_tag ());
    end
  endgenerate
endmodule
