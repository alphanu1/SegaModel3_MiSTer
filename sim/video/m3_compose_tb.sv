// SPDX-License-Identifier: GPL-3.0-or-later
//
// Sega Model 3 core for MiSTer FPGA
// Copyright (C) 2026 alphanu1
//
// Bench top for m3_compose (tb_m3_compose.cpp): stage A's video timing and
// pixel enable, m3_compose on m3_ddr3p's port L. The bench plays the tile
// generator: it answers each sc_rd with o_* three cycles later.

module m3_compose_tb (
  input  logic        clk,
  input  logic        rst_n,
  input  logic        ddr_ok,
  input  logic [1:0]  show_buf,
  input  logic        show_3d,
  output logic        sc_rd,
  output logic [9:0]  hcnt, vcnt,
  input  logic        o_v, o_drawn, o_above,
  input  logic [7:0]  o_r, o_g, o_b,
  output logic [23:0] px,
  output logic [15:0] n_late,
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
  logic [6:0] ce_acc;
  logic       ce_pix;
  always_ff @(posedge clk) begin
    if (!rst_n) begin ce_acc <= '0; ce_pix <= 1'b0; end
    else if (ce_acc + 7'd16 >= 7'd100) begin ce_acc <= ce_acc + 7'd16 - 7'd100; ce_pix <= 1'b1; end
    else begin ce_acc <= ce_acc + 7'd16; ce_pix <= 1'b0; end
  end
  logic hblank, vblank;
  m2_video_timing u_vt (
    .clk, .ce_pix, .rst_n, .interlace(1'b0), .ntsc_lines(1'b0), .mid_lines(1'b0), .narrow(1'b0),
    .hcnt, .vcnt, .hblank, .vblank, .hsync(), .vsync(), .visible(), .line_start(), .line_number(),
    .vblank_start(), .field(), .ypos(), .vtotal());
  assign sc_rd = ce_pix && !hblank && !vblank;

  logic        m_req, m_rvalid, m_ack;
  logic [24:0] m_addr;
  logic [7:0]  m_blen;
  logic [63:0] m_dout;
  m3_compose #(.FB_BASE(25'h0)) u_c (
    .clk, .rst_n, .ddr_ok, .hcnt, .vcnt, .sc_rd, .show_buf, .show_3d,
    .o_v, .o_drawn, .o_above, .o_r, .o_g, .o_b, .px,
    .m_req, .m_addr, .m_blen, .m_rvalid, .m_ack, .m_dout, .n_late);
  m3_ddr3p #(.PIPE(1'b1), .NP(1)) u_ddr (
    .clk, .rst_n, .base(29'd0),
    .req(m_req), .we(1'b0), .addr(m_addr), .blen(m_blen), .din('0), .be(8'hFF),
    .wnext(), .wacc(), .rvalid(m_rvalid), .ack(m_ack),
    .p_req(1'b0), .p_addr('0), .p_blen('0), .p_we(1'b0), .p_din('0), .p_ack(), .p_rvalid(), .p_wacc(),
    .dout(m_dout),
    .DDRAM_CLK(), .DDRAM_BUSY, .DDRAM_BURSTCNT, .DDRAM_ADDR, .DDRAM_DIN, .DDRAM_BE,
    .DDRAM_WE, .DDRAM_RD, .DDRAM_DOUT, .DDRAM_DOUT_READY,
    .dbg_lat_v(), .dbg_lat(), .dbg_lat_src(), .dbg_lat_max(), .dbg_acc(), .dbg_acc_max(),
    .dbg_take(), .dbg_take_src(), .dbg_done(), .dbg_done_src(), .dbg_idle(), .dbg_st(), .dbg_rdy(), .dbg_rq());
endmodule
