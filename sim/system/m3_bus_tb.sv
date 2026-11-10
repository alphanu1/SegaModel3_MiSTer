// SPDX-License-Identifier: GPL-3.0-or-later
//
// Sega Model 3 core for MiSTer FPGA
// Copyright (C) 2026 alphanu1
//
// Bench top for m3_bus: the bus with its SDRAM target as m3_system builds it
// (m3_sdr_port on m2_sdram, the SDRAM at its pins), the two masters, the
// 53C810, I/O and JTAG sides and the video line events driven by the C++.
module m3_bus_tb
  import ppc_cache_pkg::*;
#(parameter bit L16 = 1'b0, parameter bit BXOR = 1'b0, parameter bit LW1 = 1'b0, parameter bit PAIR = 1'b0, parameter bit RA = 1'b0, parameter bit PW = 1'b0, parameter bit RA_CONF = 1'b0,
  parameter bit FWD = 1'b0, parameter bit FREQ = 1'b0, parameter bit FDSP = 1'b0, parameter bit HITRD = 1'b0, parameter bit FAST = 1'b0, parameter int PW_DLY = 0, parameter bit R3P = 1'b0, parameter bit TGT = 1'b0,   // m3_sdr_port / m2_sdram (study R107)
  parameter bit BR = 1'b0)   // the bridges pass: m3_bus OWIRE and WD_DEC, m2_sdram SHD
(
  input  logic        clk,
  input  logic        rst_n,
  input  logic        rst,
  output logic        ready,
  input  logic [1:0]        m_req,
  input  logic [1:0][1:0]   m_cmd,
  input  logic [1:0][31:0]  m_addr,
  input  logic [1:0][3:0]   m_be,
  input  logic [1:0][31:0]  m_wdata,
  output logic [1:0]        m_ack,
  output logic [1:0]        m_wnext,
  output logic [1:0]        m_rvalid,
  output logic [31:0]       m_rdata,
  output logic        sn_req, sn_inv,
  output logic [31:5] sn_addr,
  input  logic        sn_ack,
  output logic        sc_req, sc_we,
  output logic [6:2]  sc_addr,
  output logic [3:0]  sc_be,
  output logic [31:0] sc_wdata,
  input  logic        sc_ack,
  input  logic [31:0] sc_rdata,
  input  logic        sc_win_en,
  input  logic [31:8] sc_win_base,
  output logic        cf_req, cf_we,
  output logic [5:0]  cf_idx,
  output logic [3:0]  cf_be,
  output logic [31:0] cf_wdata,
  input  logic        cf_ack,
  input  logic [31:0] cf_rdata,
  input  logic        scsi_irq,
  output logic        io_sel, io_we,
  output logic [3:0]  io_off,
  output logic [7:0]  io_wbyte,
  input  logic [7:0]  io_rbyte,
  output logic        tg_reg_we,
  output logic [5:0]  tg_reg_addr,
  output logic [31:0] tg_reg_wdata,
  output logic        tg_pal_we,
  output logic [14:0] tg_pal_addr,
  output logic [15:0] tg_pal_wdata,
  input  logic        ln_start,
  input  logic [9:0]  ln,
  output logic        ext_int,
  output logic [7:0]  irq_state_o,
  output logic [23:0] n_submit, n_cull_lo, n_poly,
  output logic [15:0] n_unmapped,
  output logic        jt_wr,
  output logic [7:0]  jt_wdata,
  input  logic        jt_busy,
  input  logic        jt_tdo,
  output logic        sd_cke, sd_cs_n, sd_ras_n, sd_cas_n, sd_we_n,
  output logic [1:0]  sd_ba,
  output logic [12:0] sd_a,
  output logic [15:0] sd_dq_o,
  input  logic [15:0] sd_dq_i,
  // the Real3D's host port, for the C++ (R112): ready as driven, every strobe seen
  input  logic        h_rdy_i,
  output logic        h_go_o, h_we_o,
  output logic [31:0] h_addr_o, h_wdata_o
);
  logic        s_req, s_swap, s_zero, s_ack, s_wnext, s_rvalid, s_idle;
  logic [1:0]  s_cmd;
  logic [26:0] s_sa;
  logic [3:0]  s_be;
  logic [31:0] s_wdata, s_rdata;
  logic        p_req, p_we, p_long, p_lo, p_ack;
  logic [26:1] p_addr;
  logic [15:0] p_din;
  logic [1:0]  p_be;
  // r3d_cmd's host port (stage B): always ready, a status read answered a
  // clock later with FFFFFFFD
  logic        h_go, h_we, h_rv;
  logic [31:0] h_rdata;
  always_ff @(posedge clk) begin h_rv <= h_go && h_rdy_i && !h_we; h_rdata <= 32'hFFFF_FFFD; end
  assign h_go_o = h_go; assign h_we_o = h_we;
  logic [3:0]  wb_col;
  logic [15:0] wb_din;
  logic [1:0]  wb_be;
  logic [63:0] p_dout;

  m3_bus #(.AW(26), .MIDI_GAP(500), .FAST(FAST), .R3P(R3P), .TGT(TGT), .OWIRE(BR), .WD_DEC(BR)) u_bus (
    .clk, .rst, .sec_tick(1'b0),
    .m_req, .m_cmd, .m_addr, .m_be, .m_wdata, .m_ack, .m_wnext, .m_rvalid, .m_rdata,
    .sn_req, .sn_addr, .sn_inv, .sn_ack,
    .s_req, .s_cmd, .s_sa, .s_swap, .s_zero, .s_be, .s_wdata, .s_ack, .s_wnext, .s_rvalid, .s_rdata,
    .sc_req, .sc_we, .sc_addr, .sc_be, .sc_wdata, .sc_ack, .sc_rdata, .sc_win_en, .sc_win_base,
    .cf_req, .cf_we, .cf_idx, .cf_be, .cf_wdata, .cf_ack, .cf_rdata, .scsi_irq,
    .io_sel, .io_we, .io_off, .io_wbyte, .io_rbyte,
    .tg_reg_we, .tg_reg_addr, .tg_reg_wdata, .tg_pal_we, .tg_pal_addr, .tg_pal_wdata,
    .ln_start, .ln, .ext_int,
    .irq_state_o, .irq_enable_o(), .crom_bank_o(),
    .n_submit, .n_cull_lo, .n_cull_hi(), .n_poly, .n_tex(), .n_vrom(), .n_unmapped, .last_unmapped(),
    .pingpong_o(), .pingpong_i(1'b0), .h_go, .h_rdy(h_rdy_i), .h_we, .h_addr(h_addr_o), .h_wdata(h_wdata_o), .h_rv, .h_rdata,
    .r3d_cfg0(), .r3d_cfg1(), .r3d_cfg2(),
    .jt_wr, .jt_wdata, .jt_busy, .jt_tdo, .jt_cfg_we(1'b0), .jt_cfg0('0), .jt_cfg1('0), .jt_cfg2('0),
    .tr_v(), .tr_master(), .tr_we(), .tr_addr(), .tr_be(), .tr_data());

  m3_sdr_port #(.AW(26), .L16(L16), .LW1(LW1), .PAIR(PAIR), .RA(RA), .PW(PW), .RA_CONF(RA_CONF), .FWD(FWD), .FREQ(FREQ), .PW_DLY(PW_DLY)) u_port (
    .clk, .rst(!rst_n), .ra_clr(1'b0),
    .s_req, .s_cmd, .s_sa, .s_swap, .s_zero, .s_be, .s_wdata, .s_ack, .s_wnext, .s_rvalid, .s_rdata, .s_idle,
    .p_req, .p_we, .p_addr, .p_din, .p_be, .p_long, .p_dout, .p_lo, .p_ack, .wb_col, .wb_din, .wb_be);

  logic [1:0] pk, pl;
  logic [1:0][63:0] pd;
  assign p_ack = pk[0]; assign p_lo = pl[0]; assign p_dout = pd[0];
  m2_sdram #(.COL_BITS(10), .NP(2), .T_REFI(770), .ACK_HOLD(1), .DUAL(1'b1), .MASK_ON_A(1'b1), .INIT_NOP(200), .WB(2'b01), .L16({1'b0, L16}), .BXOR(BXOR), .PAIR({1'b0, PAIR}), .FDSP(FDSP), .HITRD(HITRD), .SHD(BR ? 2'b11 : 2'b00)) u_sdram (
    .clk, .rst_n, .ready, .rd_lat_sel(3'd2),
    .sd_cke, .sd_cs_n, .sd_ras_n, .sd_cas_n, .sd_we_n, .sd_ba, .sd_a, .sd_dqm(), .sd_dq_o, .sd_dq_oe(), .sd_dq_i,
    .wr_req(1'b0), .wr_addr('0), .wr_din('0), .wr_be('0), .wr_ack(),
    .p_req({1'b0, p_req}), .p_we({1'b0, p_we}), .p_addr({26'd0, p_addr}), .p_din({16'd0, p_din}), .p_be({2'b00, p_be}),
    .p_dout(pd), .p_ack(pk), .p_long({1'b0, p_long}), .p_lo(pl), .wb_col, .wb_din, .wb_be, .dbg_req(), .dbg_grant());
endmodule
