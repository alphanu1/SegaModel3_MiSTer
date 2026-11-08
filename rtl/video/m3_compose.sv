// SPDX-License-Identifier: GPL-3.0-or-later
//
// Sega Model 3 core for MiSTer FPGA
// Copyright (C) 2026 alphanu1
//
// This program is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by the Free
// Software Foundation, either version 3 of the License, or (at your option)
// any later version. See LICENSE for the full text.
//
// ---------------------------------------------------------------------------
//
// The compositor (D-37 stage B; study 5.3, step 10): the 3D layer scanned out
// of DDR3 and the tile generator's pixel, at the tile generator's timing.
//
// Supermodel draws its bottom tile surface, then the 3D, then its top
// surface; MAME the priority-0 layers, the 3D, the priority-1 layers. The
// tile generator's line buffer already holds the topmost tile pixel with its
// above-3D bit (R77), so a pixel is: an above tile; else the 3D where the
// layer was painted (and the frame is shown: r3d_cmd's show_3d, false before
// the first frame and under the Mercury's block culling); else a below tile;
// else black.
//
// The layer is Model 2's m2_fb_read form (as the viewer, R64): 32-bit pixels
// {7'd0, painted, RGB}, 512 to a line, four buffers at FB_BASE + 0x20000 k
// (r3d_cmd's show_buf). A line is fetched three lines ahead of the beam into
// one of four M10K line buffers (Model 2 R655); the read is by the tile
// generator's own read (sc_rd), so the 3D pixel is ready two cycles after it
// and the tile pixel three (o_v). Nothing is
// fetched before ddr_ok (R71: hps_io's first command and RESET low for
// 100 ms).
//
// The buffer changes at the line-384 start, when no line of the next frame
// is in flight (they are fetched from line 385).

`timescale 1ns/1ps

module m3_compose #(
  parameter logic [24:0] FB_BASE = 25'h0
) (
  input  logic        clk,
  input  logic        rst_n,          // memory reset (PLL lock)
  input  logic        ddr_ok,

  // video timing (m2_video_timing): counters, advanced on ce_pix; sc_rd the
  // tile generator's read (the pixel at hcnt, vcnt is the one o_v gives)
  input  logic [9:0]  hcnt,
  input  logic [9:0]  vcnt,
  input  logic        sc_rd,

  // the frame to show (r3d_cmd)
  input  logic [1:0]  show_buf,
  input  logic        show_3d,

  // the tile generator's pixel
  input  logic        o_v,
  input  logic        o_drawn,
  input  logic        o_above,
  input  logic [7:0]  o_r, o_g, o_b,
  output logic [23:0] px,             // the composed pixel, from the cycle after o_v

  // DDR3 (m2_ddr3's contract, reads)
  output logic        m_req,
  output logic [24:0] m_addr,
  output logic [7:0]  m_blen,
  input  logic        m_rvalid,
  input  logic        m_ack,
  input  logic [63:0] m_dout,

  output logic [15:0] n_late          // lines shown before they landed
);

  // the buffer: taken through line 384; its high bit is an offset. r3d_cmd
  // moves show_buf at the line-384 start (its vbl, a clock after vcnt reaches
  // 384), so a take on vcnt's first clock at 384 got the buffer before it, the
  // one r3d_cmd had already freed: the scan-out showed a frame late, and a
  // frame ending in the next one could be drawn into the buffer on screen.
  logic [1:0] buf_q;
  logic [9:0] vq;
  always_ff @(posedge clk) begin
    vq <= vcnt;
    if (!rst_n) buf_q <= 2'd0;
    else if (vcnt == 10'd384) buf_q <= show_buf;
  end

  // the line three ahead of the beam (m3v_core, Model 2 R655 / R657)
  logic       line_req;
  logic [8:0] line_y;
  wire        fl_top = (vcnt >= 10'd385) && (vcnt < 10'd389);
  wire  [9:0] fl_tgt = fl_top ? vcnt - 10'd385 : (vcnt + 10'd3 < 10'd384) ? vcnt + 10'd3 : 10'd384;
  always_ff @(posedge clk) begin
    if (!rst_n) line_req <= 1'b0;
    else begin
      line_req <= 1'b0;
      if (vcnt != vq && fl_tgt < 10'd384 && ddr_ok) begin line_req <= 1'b1; line_y <= fl_tgt[8:0]; end
    end
  end

  // the pixel being read: the 3D is ready two cycles after sc_rd, the tile at three
  logic [8:0]  x_l;
  logic [1:0]  y_l;
  always_ff @(posedge clk) if (sc_rd) begin x_l <= hcnt[8:0]; y_l <= vcnt[1:0]; end

  logic [24:0] fr_addr;
  logic [23:0] col;
  logic        hit;
  m2_fb_read #(.WIDTH(496), .STRIDE(512), .LA(3), .V_TOTAL(424)) u_fbr (
    .clk, .rd_clk(clk), .rst_n,
    .fb_sel(buf_q[0]), .line_req, .line_y, .il(1'b0), .line_f(1'b0), .line_ready(), .hungry(),
    .m_req, .m_we(), .m_addr(fr_addr), .m_blen,
    .m_rvalid, .m_dout, .m_ack,
    .rd_buf(y_l), .rd_x(x_l), .rd_col(col), .rd_hit(hit),
    .dbg_lines(), .dbg_late(n_late), .dbg_st(), .dbg_busy(), .dbg_acks_seen());
  assign m_addr = FB_BASE + fr_addr + (buf_q[1] ? 25'h4_0000 : 25'h0);

  always_ff @(posedge clk) if (o_v) begin
    if (o_drawn && o_above)  px <= {o_r, o_g, o_b};
    else if (show_3d && hit) px <= col;
    else if (o_drawn)        px <= {o_r, o_g, o_b};
    else                     px <= 24'h000000;
  end

endmodule
