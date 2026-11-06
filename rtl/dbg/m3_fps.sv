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
// Frame counter: the Real3D frames begun in each second of clk (CLK_HZ
// clocks, r3d_cmd's n_draw), shown as two decimal digits at the top right
// on a dark box, the glyphs doubled (10 x 14). It saturates at 99 and leaves
// a leading zero blank. Unlike the debug overlay it stays in release builds,
// behind the OSD switch on en. box and ink follow x and y by two clocks;
// x and y hold for a pixel, six clocks or more.

module m3_fps #(
  parameter int CLK_HZ = 110_000_000,
  parameter int X0 = 460,             // box left: 28 wide, 8 from the 496-pixel line's end
  parameter int Y0 = 8                // box top: 18 high
) (
  input  logic       clk,
  input  logic       rst,
  input  logic       en,
  input  logic       draw,            // one clock per frame begun
  input  logic [9:0] x,
  input  logic [9:0] y,
  output logic       box,
  output logic       ink
);

  localparam int TW = $clog2(CLK_HZ);

  // ---- count: BCD, one window a second, the shown pair latched at its end
  logic [TW-1:0] tcnt;
  logic [3:0]    c1, c0, d1, d0;
  always_ff @(posedge clk) begin
    if (rst) begin
      tcnt <= '0; c1 <= 4'd0; c0 <= 4'd0; d1 <= 4'd0; d0 <= 4'd0;
    end else if (tcnt == TW'(CLK_HZ - 1)) begin
      tcnt <= '0;
      d1 <= c1; d0 <= c0;
      c1 <= 4'd0; c0 <= {3'd0, draw};
    end else begin
      tcnt <= tcnt + 1'd1;
      if (draw && !(c1 == 4'd9 && c0 == 4'd9)) begin
        if (c0 == 4'd9) begin c0 <= 4'd0; c1 <= c1 + 4'd1; end
        else c0 <= c0 + 4'd1;
      end
    end
  end

  function automatic logic [4:0] glyph(input logic [3:0] d, input logic [2:0] r);
    logic [34:0] g;
    case (d)
      4'd0: g = 35'b01110_10001_10011_10101_11001_10001_01110;
      4'd1: g = 35'b00100_01100_00100_00100_00100_00100_01110;
      4'd2: g = 35'b01110_10001_00001_00010_00100_01000_11111;
      4'd3: g = 35'b11111_00010_00100_00010_00001_10001_01110;
      4'd4: g = 35'b00010_00110_01010_10010_11111_00010_00010;
      4'd5: g = 35'b11111_10000_11110_00001_00001_10001_01110;
      4'd6: g = 35'b00110_01000_10000_11110_10001_10001_01110;
      4'd7: g = 35'b11111_00001_00010_00100_01000_01000_01000;
      4'd8: g = 35'b01110_10001_10001_01110_10001_10001_01110;
      4'd9: g = 35'b01110_10001_10001_01111_00001_00010_01100;
      default: g = 35'd0;
    endcase
    glyph = g[34 - 5 * r -: 5];
  endfunction

  // ---- draw: box 28 x 18; tens glyph at +2, units at +16, both from y +2
  wire [9:0] rx = x - 10'(X0);
  wire [9:0] ry = y - 10'(Y0);
  wire       in_b = (x >= 10'(X0)) && (rx < 10'd28) && (y >= 10'(Y0)) && (ry < 10'd18);
  wire [9:0] gx1 = rx - 10'd2;          // tens
  wire [9:0] gx0 = rx - 10'd16;         // units
  wire [9:0] gy  = ry - 10'd2;
  wire       on_y = (ry >= 10'd2) && (gy < 10'd14);
  wire       on1  = (rx >= 10'd2)  && (gx1 < 10'd10) && on_y && (d1 != 4'd0);
  wire       on0  = (rx >= 10'd16) && (gx0 < 10'd10) && on_y;

  logic       box_p, on_p;
  logic [3:0] dig_p;
  logic [2:0] col_p, row_p;
  always_ff @(posedge clk) begin
    box_p <= en && in_b;
    on_p  <= en && in_b && (on1 || on0);
    dig_p <= on1 ? d1 : d0;
    col_p <= on1 ? gx1[3:1] : gx0[3:1];
    row_p <= gy[3:1];
  end
  wire [4:0] grow = glyph(dig_p, row_p);
  always_ff @(posedge clk) begin
    box <= box_p;
    ink <= on_p && grow[3'd4 - col_p];
  end

endmodule
