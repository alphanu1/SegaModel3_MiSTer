// SPDX-License-Identifier: GPL-3.0-or-later
//
// Sega Model 3 core for MiSTer FPGA
// Copyright (C) 2026 alphanu1
//
// Bench top for D-37 stage B's Real3D host side (tb_r3d_host.cpp): r3d_cmd,
// r3d_mem and r3d_texup on the real m2_sdram (two ports) and m3_ddr3p (port
// L behind m3_ddr_arb: 0 a load generator standing in for the rest of the
// Real3D, 1 r3d_mem, 2 r3d_texup). The geometry and the rasteriser are the
// bench's: it drives g_busy, f_ready and r_done.

module r3d_host_tb #(
  parameter int COL_BITS = 10
) (
  input  logic        clk,
  input  logic        mem_rst_n,
  input  logic        rst,

  input  logic        h_go,
  output logic        h_rdy,
  input  logic        h_we,
  input  logic [31:0] h_addr,
  input  logic [31:0] h_wdata,
  output logic        h_rv,
  output logic [31:0] h_rdata,
  input  logic        tg_reg_we,
  input  logic [5:0]  tg_reg_addr,
  input  logic [31:0] tg_reg_wdata,
  input  logic        ln_start,
  input  logic [9:0]  ln,
  input  logic        mw_we,
  input  logic [3:0]  mw_dev,
  input  logic [31:0] mw_val,

  output logic        g_start,
  input  logic        g_busy,
  input  logic        f_ready,
  output logic        f_begin,
  output logic        f_end,
  output logic [1:0]  f_buf,
  input  logic        r_done,
  input  logic        r_busy,
  output logic [1:0]  show_buf,
  output logic        show_3d,
  output logic        pingpong,
  output logic        flip,
  output logic [15:0] n_draw,
  output logic [15:0] n_drop,
  output logic        bank,
  output logic        m_swap,
  output logic        t_go,
  output logic        t_busy,
  output logic        mem_idle,

  // the load generator on DDR3 port 0
  input  logic        x_req,
  input  logic [24:0] x_addr,
  input  logic [7:0]  x_blen,
  output logic        x_ack,

  input  logic [2:0]  rd_lat_sel,
  output logic        sd_cke, sd_cs_n, sd_ras_n, sd_cas_n, sd_we_n,
  output logic [1:0]  sd_ba,
  output logic [12:0] sd_a,
  output logic [1:0]  sd_dqm,
  output logic [15:0] sd_dq_o,
  output logic        sd_dq_oe,
  input  logic [15:0] sd_dq_i,
  output logic        sd_ready,

  input  logic        DDRAM_BUSY,
  output logic [7:0]  DDRAM_BURSTCNT,
  output logic [28:0] DDRAM_ADDR,
  output logic [63:0] DDRAM_DIN,
  output logic [7:0]  DDRAM_BE,
  output logic        DDRAM_WE,
  output logic        DDRAM_RD,
  input  logic [63:0] DDRAM_DOUT,
  input  logic        DDRAM_DOUT_READY
);

  localparam int AW = 1 + 2 + 13 + COL_BITS;

  logic        m_valid, m_ready, m_can_swap, m_rbusy;
  logic [1:0]  m_reg;
  logic [19:0] m_addr;
  logic [31:0] m_data;
  logic        t_valid, t_ready, t_vrom, t_flush, t_pend, t_mark, t_mpend;
  logic [31:0] t_data;

  r3d_cmd u_cmd (
    .clk, .rst, .h_go, .h_rdy, .h_we, .h_addr, .h_wdata, .h_rv, .h_rdata,
    .tg_reg_we, .tg_reg_addr, .tg_reg_wdata, .ln_start, .ln, .mw_we, .mw_dev, .mw_val,
    .m_valid, .m_ready, .m_reg, .m_addr, .m_data, .m_swap, .m_can_swap, .m_rbusy,
    .t_valid, .t_ready, .t_vrom, .t_data, .t_flush, .t_pend, .t_mark, .t_mpend, .t_go, .t_busy,
    .g_start, .g_busy, .f_ready, .f_begin, .f_end, .f_buf, .r_done, .r_busy,
    .show_buf, .show_3d, .pingpong, .flip, .n_draw, .n_drop);

  // SDRAM: port 0 r3d_mem, port 1 r3d_texup
  logic [1:0]       p_req, p_we, p_ack, p_lo;
  logic [1:0][AW:1] p_addr;
  logic [1:0][15:0] p_din;
  logic [1:0][1:0]  p_be;
  logic [1:0][63:0] p_dout;
  m2_sdram #(.COL_BITS(COL_BITS), .NP(2), .T_REFI(770), .ACK_HOLD(1), .DUAL(1'b1), .MASK_ON_A(1'b1)) u_sdram (
    .clk, .rst_n(mem_rst_n), .ready(sd_ready), .rd_lat_sel,
    .sd_cke, .sd_cs_n, .sd_ras_n, .sd_cas_n, .sd_we_n, .sd_ba, .sd_a, .sd_dqm, .sd_dq_o, .sd_dq_oe, .sd_dq_i,
    .wr_req(1'b0), .wr_addr('0), .wr_din('0), .wr_be('0), .wr_ack(),
    .p_req, .p_we, .p_addr, .p_din, .p_be, .p_dout, .p_ack, .p_long(2'b00), .p_lo,
    .dbg_req(), .dbg_grant());

  // DDR3
  logic [2:0]       d_req, d_we, d_wacc, d_rvalid, d_ack;
  logic [2:0][24:0] d_addr;
  logic [2:0][7:0]  d_blen, d_be;
  logic [2:0][63:0] d_din;
  logic        dm_req, dm_we, dm_wacc, dm_rvalid, dm_ack;
  logic [24:0] dm_addr;
  logic [7:0]  dm_blen, dm_be;
  logic [63:0] dm_din, dm_dout;
  m3_ddr_arb #(.NP(3)) u_arb (
    .clk, .rst(!mem_rst_n), .req(d_req), .we(d_we), .addr(d_addr), .blen(d_blen), .din(d_din), .be(d_be),
    .wacc(d_wacc), .rvalid(d_rvalid), .ack(d_ack),
    .m_req(dm_req), .m_we(dm_we), .m_addr(dm_addr), .m_blen(dm_blen), .m_din(dm_din), .m_be(dm_be),
    .m_wacc(dm_wacc), .m_rvalid(dm_rvalid), .m_ack(dm_ack));
  m3_ddr3p #(.PIPE(1'b1), .NP(1)) u_ddr (
    .clk, .rst_n(mem_rst_n), .base(29'd0),
    .req(dm_req), .we(dm_we), .addr(dm_addr), .blen(dm_blen), .din(dm_din), .be(dm_be),
    .wnext(), .wacc(dm_wacc), .rvalid(dm_rvalid), .ack(dm_ack),
    .p_req(1'b0), .p_addr('0), .p_blen('0), .p_we(1'b0), .p_din('0), .p_ack(), .p_rvalid(), .p_wacc(),
    .dout(dm_dout),
    .DDRAM_CLK(), .DDRAM_BUSY, .DDRAM_BURSTCNT, .DDRAM_ADDR, .DDRAM_DIN, .DDRAM_BE,
    .DDRAM_WE, .DDRAM_RD, .DDRAM_DOUT, .DDRAM_DOUT_READY,
    .dbg_lat_v(), .dbg_lat(), .dbg_lat_src(), .dbg_lat_max(), .dbg_acc(), .dbg_acc_max(),
    .dbg_take(), .dbg_take_src(), .dbg_done(), .dbg_done_src(), .dbg_idle(), .dbg_st(), .dbg_rdy(), .dbg_rq());

  assign d_req[0] = x_req; assign d_we[0] = 1'b0; assign d_addr[0] = x_addr; assign d_blen[0] = x_blen;
  assign d_din[0] = '0; assign d_be[0] = 8'hFF; assign x_ack = d_ack[0];

  logic s0_req, s0_we;
  logic [AW:1] s0_addr;
  logic [15:0] s0_din;
  logic [1:0]  s0_be;
  r3d_mem #(.AW(AW)) u_mem (
    .clk, .rst, .w_valid(m_valid), .w_ready(m_ready), .w_reg(m_reg), .w_addr(m_addr), .w_data(m_data),
    .swap(m_swap), .r_busy(m_rbusy), .bank, .can_swap(m_can_swap),
    .s_req(s0_req), .s_we(s0_we), .s_addr(s0_addr), .s_din(s0_din), .s_be(s0_be), .s_ack(p_ack[0]), .s_dout(p_dout[0]),
    .d_req(d_req[1]), .d_we(d_we[1]), .d_addr(d_addr[1]), .d_blen(d_blen[1]), .d_din(d_din[1]), .d_be(d_be[1]),
    .d_wacc(d_wacc[1]), .d_rvalid(d_rvalid[1]), .d_ack(d_ack[1]), .d_dout(dm_dout),
    .n_runs(), .n_copy());
  assign p_req[0] = s0_req; assign p_we[0] = s0_we; assign p_addr[0] = s0_addr; assign p_din[0] = s0_din; assign p_be[0] = s0_be;

  logic s1_req;
  logic [AW:1] s1_addr;
  r3d_texup #(.AW(AW)) u_tex (
    .clk, .rst, .t_valid, .t_ready, .t_vrom, .t_data, .t_flush, .t_pend, .mark(t_mark), .t_mpend, .go(t_go), .busy(t_busy),
    .s_req(s1_req), .s_addr(s1_addr), .s_ack(p_ack[1]), .s_dout(p_dout[1]),
    .d_req(d_req[2]), .d_we(d_we[2]), .d_addr(d_addr[2]), .d_blen(d_blen[2]), .d_din(d_din[2]), .d_be(d_be[2]),
    .d_wacc(d_wacc[2]), .d_rvalid(d_rvalid[2]), .d_ack(d_ack[2]), .d_dout(dm_dout),
    .n_tex(), .n_skip());
  assign p_req[1] = s1_req; assign p_we[1] = 1'b0; assign p_addr[1] = s1_addr; assign p_din[1] = '0; assign p_be[1] = '0;

  assign mem_idle = h_rdy && m_can_swap && t_ready && !t_busy && !t_pend;

endmodule
