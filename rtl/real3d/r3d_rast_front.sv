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
// Real3D rasteriser front: y-sorted 16.16 vertices and a band in, the spans in
// that band out. The slope divider (r3d_rast_div) feeds the edge walker
// (r3d_rast_edge); the divider works on the next triangle while the walker's
// multiply and walk run. In the rasteriser the divider runs once a triangle in
// the set-up and the walker once a (triangle, band) pair; this chain is their
// bench and fit together.

module r3d_rast_front (
  input  logic               clk,
  input  logic               rst,

  input  logic               t_valid,
  output logic               t_ready,
  input  logic signed [31:0] t_x1, t_y1, t_x2, t_y2, t_x3, t_y3,
  input  logic        [8:0]  t_b0, t_b1,

  output logic               s_valid,
  input  logic               s_ready,
  output logic [8:0]         s_y,
  output logic [8:0]         s_x0,
  output logic [8:0]         s_x1,

  output logic               busy
);

  logic               d_valid, d_ready, e_busy;
  logic signed [31:0] d_x1, d_y1, d_x2, d_y2, d_x3, d_y3;
  logic signed [53:0] d_s12, d_s13, d_s23;
  logic        [8:0]  d_b0, d_b1;

  // the band rides beside the divider, which holds one triangle at a time
  always_ff @(posedge clk)
    if (t_valid && t_ready) begin
      d_b0 <= t_b0;
      d_b1 <= t_b1;
    end

  r3d_rast_div u_div (
    .clk, .rst,
    .i_valid(t_valid), .i_ready(t_ready),
    .i_x1(t_x1), .i_y1(t_y1), .i_x2(t_x2), .i_y2(t_y2), .i_x3(t_x3), .i_y3(t_y3),
    .o_valid(d_valid), .o_ready(d_ready),
    .o_x1(d_x1), .o_y1(d_y1), .o_x2(d_x2), .o_y2(d_y2), .o_x3(d_x3), .o_y3(d_y3),
    .o_s12(d_s12), .o_s13(d_s13), .o_s23(d_s23)
  );

  r3d_rast_edge u_edge (
    .clk, .rst,
    .t_valid(d_valid), .t_ready(d_ready),
    .t_x1(d_x1), .t_y1(d_y1), .t_x2(d_x2), .t_y2(d_y2), .t_x3(d_x3), .t_y3(d_y3),
    .t_s12(d_s12), .t_s13(d_s13), .t_s23(d_s23),
    .t_b0(d_b0), .t_b1(d_b1),
    .s_valid, .s_ready, .s_y, .s_x0, .s_x1,
    .busy(e_busy)
  );

  assign busy = !t_ready || d_valid || e_busy;

endmodule
