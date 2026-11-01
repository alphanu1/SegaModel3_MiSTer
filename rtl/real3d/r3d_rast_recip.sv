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
// Real3D rasteriser: the perspective reciprocal, r = round(2^47 / m) for a
// 24-bit m with its top bit set (2^24 for m = 2^23), r3d_rast_rtl.cpp persp
// and fog_p at 24 bits. LATENCY 10, one a clock, every stage on en.
//
// The arithmetic of r3d_recip.sv (the geometry's divide) cut to the
// reciprocal: a 128-entry seed, two Newton steps to about 2^-46, the
// estimate r2 >> 26 within [-1, 0] of the rounded quotient, then the
// remainder test m (2q - 1) <= 2^48 < m (2q + 1) on the low 30 bits. 2^47 / m
// is never a tie. Five products.

/* verilator lint_off UNUSEDSIGNAL */
module r3d_rast_recip (
  input  logic        clk,
  input  logic        en,
  input  logic [23:0] m,
  output logic [24:0] r
);

  function automatic logic [11:0] seed(input logic [6:0] i);
    case (i)
      7'd0: seed = 12'd2040;
      7'd1: seed = 12'd2024;
      7'd2: seed = 12'd2009;
      7'd3: seed = 12'd1993;
      7'd4: seed = 12'd1978;
      7'd5: seed = 12'd1964;
      7'd6: seed = 12'd1949;
      7'd7: seed = 12'd1935;
      7'd8: seed = 12'd1920;
      7'd9: seed = 12'd1907;
      7'd10: seed = 12'd1893;
      7'd11: seed = 12'd1879;
      7'd12: seed = 12'd1866;
      7'd13: seed = 12'd1853;
      7'd14: seed = 12'd1840;
      7'd15: seed = 12'd1827;
      7'd16: seed = 12'd1814;
      7'd17: seed = 12'd1802;
      7'd18: seed = 12'd1789;
      7'd19: seed = 12'd1777;
      7'd20: seed = 12'd1765;
      7'd21: seed = 12'd1753;
      7'd22: seed = 12'd1742;
      7'd23: seed = 12'd1730;
      7'd24: seed = 12'd1719;
      7'd25: seed = 12'd1708;
      7'd26: seed = 12'd1697;
      7'd27: seed = 12'd1686;
      7'd28: seed = 12'd1675;
      7'd29: seed = 12'd1664;
      7'd30: seed = 12'd1654;
      7'd31: seed = 12'd1644;
      7'd32: seed = 12'd1633;
      7'd33: seed = 12'd1623;
      7'd34: seed = 12'd1613;
      7'd35: seed = 12'd1603;
      7'd36: seed = 12'd1594;
      7'd37: seed = 12'd1584;
      7'd38: seed = 12'd1574;
      7'd39: seed = 12'd1565;
      7'd40: seed = 12'd1556;
      7'd41: seed = 12'd1547;
      7'd42: seed = 12'd1538;
      7'd43: seed = 12'd1529;
      7'd44: seed = 12'd1520;
      7'd45: seed = 12'd1511;
      7'd46: seed = 12'd1502;
      7'd47: seed = 12'd1494;
      7'd48: seed = 12'd1485;
      7'd49: seed = 12'd1477;
      7'd50: seed = 12'd1469;
      7'd51: seed = 12'd1460;
      7'd52: seed = 12'd1452;
      7'd53: seed = 12'd1444;
      7'd54: seed = 12'd1436;
      7'd55: seed = 12'd1429;
      7'd56: seed = 12'd1421;
      7'd57: seed = 12'd1413;
      7'd58: seed = 12'd1406;
      7'd59: seed = 12'd1398;
      7'd60: seed = 12'd1391;
      7'd61: seed = 12'd1383;
      7'd62: seed = 12'd1376;
      7'd63: seed = 12'd1369;
      7'd64: seed = 12'd1362;
      7'd65: seed = 12'd1355;
      7'd66: seed = 12'd1348;
      7'd67: seed = 12'd1341;
      7'd68: seed = 12'd1334;
      7'd69: seed = 12'd1327;
      7'd70: seed = 12'd1321;
      7'd71: seed = 12'd1314;
      7'd72: seed = 12'd1307;
      7'd73: seed = 12'd1301;
      7'd74: seed = 12'd1295;
      7'd75: seed = 12'd1288;
      7'd76: seed = 12'd1282;
      7'd77: seed = 12'd1276;
      7'd78: seed = 12'd1269;
      7'd79: seed = 12'd1263;
      7'd80: seed = 12'd1257;
      7'd81: seed = 12'd1251;
      7'd82: seed = 12'd1245;
      7'd83: seed = 12'd1239;
      7'd84: seed = 12'd1234;
      7'd85: seed = 12'd1228;
      7'd86: seed = 12'd1222;
      7'd87: seed = 12'd1216;
      7'd88: seed = 12'd1211;
      7'd89: seed = 12'd1205;
      7'd90: seed = 12'd1200;
      7'd91: seed = 12'd1194;
      7'd92: seed = 12'd1189;
      7'd93: seed = 12'd1183;
      7'd94: seed = 12'd1178;
      7'd95: seed = 12'd1173;
      7'd96: seed = 12'd1168;
      7'd97: seed = 12'd1163;
      7'd98: seed = 12'd1157;
      7'd99: seed = 12'd1152;
      7'd100: seed = 12'd1147;
      7'd101: seed = 12'd1142;
      7'd102: seed = 12'd1137;
      7'd103: seed = 12'd1132;
      7'd104: seed = 12'd1128;
      7'd105: seed = 12'd1123;
      7'd106: seed = 12'd1118;
      7'd107: seed = 12'd1113;
      7'd108: seed = 12'd1108;
      7'd109: seed = 12'd1104;
      7'd110: seed = 12'd1099;
      7'd111: seed = 12'd1095;
      7'd112: seed = 12'd1090;
      7'd113: seed = 12'd1085;
      7'd114: seed = 12'd1081;
      7'd115: seed = 12'd1077;
      7'd116: seed = 12'd1072;
      7'd117: seed = 12'd1068;
      7'd118: seed = 12'd1063;
      7'd119: seed = 12'd1059;
      7'd120: seed = 12'd1055;
      7'd121: seed = 12'd1051;
      7'd122: seed = 12'd1046;
      7'd123: seed = 12'd1042;
      7'd124: seed = 12'd1038;
      7'd125: seed = 12'd1034;
      7'd126: seed = 12'd1030;
      7'd127: seed = 12'd1026;
      default: seed = 12'd0;
    endcase
  endfunction

  logic [23:0] m0, m1, m2, m3, m4, m5, m6, m7, m8;
  logic        p0, p1q, p2q, p3, p4, p5, p6, p7, p8;   // m = 2^23
  logic [11:0] t0, t1, t2;
  logic [35:0] pr1;
  logic [19:0] e1s;
  logic [20:0] r1s, r1s4, r1s5, r1s6;
  logic [44:0] pr2;
  logic signed [24:0] ds;
  logic signed [46:0] c6p;
  logic [24:0] q07, q08;
  logic [48:0] mq;
  logic        up9, dn9;
  logic [24:0] q9;

  always_ff @(posedge clk) if (en) begin
    logic [35:0] e1;
    logic [31:0] r1;
    logic signed [45:0] d;
    logic signed [46:0] csh;
    logic signed [52:0] r2;
    logic [29:0] d10, dlo, dhi;
    logic [24:0] q;
    // 0: seed
    m0 <= m; p0 <= (m[22:0] == 23'd0); t0 <= seed(m[22:16]);
    // 1: m t (Q2.34)
    m1 <= m0; p1q <= p0; t1 <= t0; pr1 <= m0 * t0;
    // 2: e1s = (2^35 - m t) >> 16
    m2 <= m1; p2q <= p1q; t2 <= t1; e1 = 36'h8_0000_0000 - pr1; e1s <= e1[35:16];
    // 3: r1s = (t e1s) >> 9 (Q.20)
    m3 <= m2; p3 <= p2q; r1 = t2 * e1s; r1s <= r1[29:9];
    // 4: m r1s (Q.43)
    m4 <= m3; p4 <= p3; r1s4 <= r1s; pr2 <= m3 * r1s;
    // 5: ds = (2^43 - m r1s) >>> 4
    m5 <= m4; p5 <= p4; r1s5 <= r1s4; d = 46'sh800_0000_0000 - $signed({1'b0, pr2}); ds <= d[28:4];
    // 6: r1s ds (Q.59)
    m6 <= m5; p6 <= p5; r1s6 <= r1s5; c6p <= $signed({1'b0, r1s5}) * ds;
    // 7: r2 = (r1s << 30) + (c >>> 9) ~ 2^73 / m; q0 = r2 >> 26
    m7 <= m6; p7 <= p6; csh = c6p >>> 9;
    r2 = $signed({2'b00, r1s6, 30'd0}) + $signed({{6{csh[46]}}, csh});
    q07 <= r2[50:26];
    // 8: m q0
    m8 <= m7; p8 <= p7; q08 <= q07; mq <= m7 * q07;
    // 9: d = 2^48 - 2 m q0 on 30 bits; round up if d >= m, down if d < -m
    q9 <= q08; d10 = 30'd0 - {mq[28:0], 1'b0};
    dlo = d10 - {6'd0, m8}; dhi = d10 + {6'd0, m8};
    up9 <= !dlo[29] && !p8; dn9 <= dhi[29] && !p8;
    if (p8) q9 <= 25'h100_0000;
    // 10: pick
    if (up9) q = q9 + 25'd1; else if (dn9) q = q9 - 25'd1; else q = q9;
    r <= q;
  end

endmodule
