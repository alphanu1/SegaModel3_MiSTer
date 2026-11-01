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
// The viewer's status overlay: NR rows of four 32-bit values as hex digits,
// top left. A value is eight 8-pixel cells and a blank one; a row is 9
// pixels. The 5 x 7 glyphs sit in the cells' top-left corner, on a dark box.
// The value under the beam is asked for on idx (row x 4 + column) and must
// come back on val the next cycle (m3v_core reads it from its registers or
// from the probe's MLAB).

module m3v_overlay #(
  parameter int NR = 4
) (
  input  logic              clk,
  input  logic              en,
  input  logic [9:0]        x,
  input  logic [9:0]        y,
  output logic [5:0]        idx,
  input  logic [31:0]       val,
  output logic              box,        // inside the box: darken
  output logic              ink         // a glyph pixel
);

  localparam int X0 = 8, Y0 = 8;

  function automatic logic [4:0] glyph(input logic [3:0] d, input logic [2:0] r);
    logic [34:0] g;
    case (d)
      4'h0: g = 35'b01110_10001_10011_10101_11001_10001_01110;
      4'h1: g = 35'b00100_01100_00100_00100_00100_00100_01110;
      4'h2: g = 35'b01110_10001_00001_00010_00100_01000_11111;
      4'h3: g = 35'b11111_00010_00100_00010_00001_10001_01110;
      4'h4: g = 35'b00010_00110_01010_10010_11111_00010_00010;
      4'h5: g = 35'b11111_10000_11110_00001_00001_10001_01110;
      4'h6: g = 35'b00110_01000_10000_11110_10001_10001_01110;
      4'h7: g = 35'b11111_00001_00010_00100_01000_01000_01000;
      4'h8: g = 35'b01110_10001_10001_01110_10001_10001_01110;
      4'h9: g = 35'b01110_10001_10001_01111_00001_00010_01100;
      4'hA: g = 35'b01110_10001_10001_11111_10001_10001_10001;
      4'hB: g = 35'b11110_10001_10001_11110_10001_10001_11110;
      4'hC: g = 35'b01110_10001_10000_10000_10000_10001_01110;
      4'hD: g = 35'b11100_10010_10001_10001_10001_10010_11100;
      4'hE: g = 35'b11111_10000_10000_11110_10000_10000_11111;
      default: g = 35'b11111_10000_10000_11110_10000_10000_10000;
    endcase
    glyph = g[34 - 5 * r -: 5];
  endfunction

  // where the beam is: ocell column, row, and the pixel in the ocell
  wire [9:0] rx = x - 10'(X0);
  wire [9:0] ry = y - 10'(Y0);
  wire       in_x = (x >= 10'(X0)) && (rx < 10'd288);     // 4 values x 9 cells x 8
  wire       in_y = (y >= 10'(Y0)) && (ry < 10'(9 * NR));
  wire [5:0] ocell = rx[8:3];                               // 0..35
  logic [1:0] col;
  logic [3:0] dig;                                         // 0..8 in the value
  always_comb begin
    if (ocell < 6'd9) begin col = 2'd0; dig = 4'(ocell); end
    else if (ocell < 6'd18) begin col = 2'd1; dig = 4'(ocell - 6'd9); end
    else if (ocell < 6'd27) begin col = 2'd2; dig = 4'(ocell - 6'd18); end
    else begin col = 2'd3; dig = 4'(ocell - 6'd27); end
  end
  logic [3:0] row;
  logic [3:0] ly;
  always_comb begin
    row = '0; ly = 4'(ry);
    for (int k = 1; k < NR; k++) if (ry >= 10'(9 * k)) begin row = 4'(k); ly = 4'(ry - 10'(9 * k)); end
  end
  assign idx = {row, col};

  // registered once: the value and nibble, then the glyph bit
  logic [3:0]  dig_q, ly_q;
  logic [2:0]  px_q;
  logic        in_q;
  always_ff @(posedge clk) begin
    dig_q <= dig; ly_q <= ly; px_q <= rx[2:0];
    in_q  <= en && in_x && in_y;
  end
  wire [3:0] nib = val[31 - 4 * dig_q[2:0] -: 4];
  wire [4:0] grow = glyph(nib, ly_q[2:0]);
  always_ff @(posedge clk) begin
    box <= in_q;
    ink <= in_q && dig_q != 4'd8 && ly_q < 4'd7 && px_q < 3'd5 && grow[3'd4 - px_q];
  end

endmodule
