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
// Frame counter: the Real3D frames begun a second (r3d_cmd's n_draw), the
// mean over a window of 1, 5 or 10 seconds of clk (avg, the OSD's "FPS
// average"), rounded to the nearest whole frame (a half up), shown as two
// decimal digits at the top right on a dark box, the glyphs doubled (10 x 14).
// It saturates at 99 and leaves a leading zero blank. Unlike the debug overlay it stays in release builds,
// behind the OSD switch on en. box and ink follow x and y by two clocks;
// x and y hold for a pixel, six clocks or more.

module m3_fps #(
  parameter int CLK_HZ = 110_000_000,
  parameter int X0 = 460,             // box left: 28 wide, 8 from the 496-pixel line's end
  parameter int Y0 = 8,               // box top: 18 high
  parameter bit EXT_SEC = 1'b0        // the second from sec_tick (m3_system's counter, the RTC's too), not tcnt
) (
  input  logic       clk,
  input  logic       rst,
  input  logic       sec_tick,        // EXT_SEC: high in the clock tcnt would have read CLK_HZ - 1
  input  logic       en,
  input  logic [1:0] avg,             // the window: 0 5 s (the OSD's default), 1 10 s, 2 1 s, 3 5 s
  input  logic       draw,            // one clock per frame begun
  input  logic [9:0] x,
  input  logic [9:0] y,
  output logic       box,
  output logic       ink
);

  localparam int TW = $clog2(CLK_HZ);

  // ---- count: frames x 10 / W over a window of W seconds, in three BCD digits (ch ct cu), so no divider: a frame
  // adds 10 in a 1 s window, 2 in a 5 s one, 1 in a 10 s one. At the window's end the shown pair is {ch, ct},
  // one more when cu is 5 or over (the mean rounded), 99 at most. The count holds at 999; a new setting starts a
  // new window.
  logic [TW-1:0] tcnt;
  logic [3:0]    ch, ct, cu, d1, d0, wcnt;
  logic [1:0]    avg_q;
  wire wrap = EXT_SEC ? sec_tick : (tcnt == TW'(CLK_HZ - 1));
  wire w1   = avg == 2'd2;                       // 1 s: x 10
  wire w10  = avg == 2'd1;                       // 10 s: x 1
  wire wend = wrap && (w1 || wcnt == (w10 ? 4'd9 : 4'd4));
  wire [3:0] k = w10 ? 4'd1 : 4'd2;              // the units' step (5 s, 10 s)
  wire [4:0] us = {1'b0, cu} + {1'b0, k};
  wire       uc = us >= 5'd10;                   // the units carry
  wire       tinc = w1 || uc;                    // the tens step
  wire       full = ch == 4'd9 && ct == 4'd9 && tinc;
  wire       rup  = cu >= 4'd5;                  // the shown pair one up
  always_ff @(posedge clk) begin
    avg_q <= avg;
    if (!EXT_SEC) tcnt <= (rst || wrap) ? '0 : tcnt + 1'd1;
    if (rst) begin d1 <= 4'd0; d0 <= 4'd0; end
    if (rst || avg != avg_q) begin
      ch <= 4'd0; ct <= 4'd0; cu <= 4'd0; wcnt <= 4'd0;
    end else if (wend) begin
      // the window's mean, rounded; this clock's frame is the next window's first
      wcnt <= 4'd0;
      d1 <= ch; d0 <= ct;
      if (rup && !(ch == 4'd9 && ct == 4'd9)) begin
        if (ct == 4'd9) begin d1 <= ch + 4'd1; d0 <= 4'd0; end else d0 <= ct + 4'd1;
      end
      ch <= 4'd0;
      ct <= {3'd0, draw && w1};
      cu <= (draw && !w1) ? k : 4'd0;
    end else begin
      if (wrap) wcnt <= wcnt + 4'd1;
      if (draw && !full) begin
        if (!w1) cu <= uc ? 4'(us - 5'd10) : us[3:0];
        if (tinc) begin
          if (ct == 4'd9) begin ct <= 4'd0; ch <= ch + 4'd1; end else ct <= ct + 4'd1;
        end
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
