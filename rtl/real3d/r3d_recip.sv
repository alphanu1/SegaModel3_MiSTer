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
// Correctly rounded FP32 divide a / x, and reciprocal 1.0f / x (a = 1.0), for
// the Real3D engine: the perspective divide, the clip's t and the set-up's
// divides all run here, on the pool. Fully pipelined, LATENCY 12, one result
// a cycle.
//
// Mantissa M of x in [2^23, 2^24): a 128-entry seed (Q1.11), one Newton step
// to Q.20, a second step whose correction term is cut to Q.39, giving r2 =
// 2^73 / M to about 2^-46. The quotient estimate is Q0 = A * (r2 >> 24),
// shifted to 24 or 25 bits; it lies within [-1, 0] of the rounded quotient on
// 67 M mantissa pairs (every M, four A each). A remainder test,
// M (2Q - 1) <= 2 A 2^s < M (2Q + 1), then picks Q0 - 1, Q0 or Q0 + 1: round
// to nearest (a quotient of two 24-bit significands is never a tie). Every
// multiply fits a 27 x 27 DSP: M x seed, seed x e1, M x r1, r1 x delta,
// A x r, M x Q0 (6 DSP).
//
// Flush to zero: a subnormal operand is taken as zero and a result below
// 2^-126 flushes to zero; "tiny" flags either case (the bench asserts it never
// happens). 0/0, inf/inf and NaN give the canonical NaN, x/0 and inf/x
// infinity, 0/x and x/inf zero, all with the sign of a xor x.

module r3d_recip (
  input  logic        clk,
  input  logic        in_valid,
  input  logic        div,          // 1: a / x, 0: 1.0 / x
  input  logic [31:0] a,
  input  logic [31:0] x,
  output logic        out_valid,
  output logic [31:0] result,
  output logic        tiny          // a subnormal operand or a flushed result
);

  // ---- seed: round(2048 / (1 + (i + 0.5) / 128)), Q1.11 -------------------
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

  // pipeline payload
  typedef struct packed {
    logic        v;
    logic        s;
    logic [9:0]  re;       // result exponent before the quotient's normalisation, two's complement
    logic [1:0]  special;  // 0 normal, 1 inf, 2 zero, 3 NaN
    logic        tin;      // subnormal operand
    logic        ge;       // A >= M: quotient in [1, 2)
    logic [23:0] m;
    logic [23:0] am;
  } ctl_t;

  ctl_t c0, c1;

  // The payload past stage 1 travels in r3d_dline delay lines (area pass), one per stage that reads it, each
  // exactly the field the register chain c2 .. c10 held there: M at stages 3 and 8 (and 9, a register after 8),
  // A at 7 (its low six bits on to 9 in registers), ge at 8 (and 9), and {v, s, re, special, tin} at 10, taken
  // from c1 after its exponent correction.
  typedef struct packed {
    logic        v;
    logic        s;
    logic [9:0]  re;
    logic [1:0]  special;
    logic        tin;
  } out_t;
  out_t        c1o, c10;
  logic [23:0] m3, m8, m9, am7;
  logic        ge8, ge9;
  logic [5:0]  am8lo, am9lo;
  assign c1o = {c1.v, c1.s, c1.re, c1.special, c1.tin};
  r3d_dline #(.W(24), .D(3))  u_m3  (.clk, .en(1'b1), .d(c0.m), .q(m3));
  r3d_dline #(.W(25), .D(8))  u_m8  (.clk, .en(1'b1), .d({c0.m, c0.ge}), .q({m8, ge8}));
  r3d_dline #(.W(24), .D(7))  u_am7 (.clk, .en(1'b1), .d(c0.am), .q(am7));
  r3d_dline #(.W(15), .D(9))  u_out (.clk, .en(1'b1), .d(c1o), .q(c10));
  always_ff @(posedge clk) begin
    m9 <= m8; ge9 <= ge8;
    am8lo <= am7[5:0]; am9lo <= am8lo;
  end

  // stage 0: unpack, seed
  logic [11:0] t0;
  always_ff @(posedge clk) begin
    logic [7:0] e, ea;
    logic [22:0] f, fa;
    logic xz, xi, xn, az, ai, an, sa;
    e = x[30:23]; f = x[22:0];
    ea = div ? a[30:23] : 8'd127;
    fa = div ? a[22:0] : 23'd0;
    sa = div ? a[31] : 1'b0;
    xz = (e == 8'd0); xi = (e == 8'hff) && (f == 23'd0); xn = (e == 8'hff) && (f != 23'd0);
    az = (ea == 8'd0); ai = (ea == 8'hff) && (fa == 23'd0); an = (ea == 8'hff) && (fa != 23'd0);
    c0.v <= in_valid;
    c0.s <= x[31] ^ sa;
    c0.m <= {1'b1, f};
    c0.am <= {1'b1, fa};
    c0.ge <= (fa >= f);
    // the A < M correction (- 1) is applied in stage 1, off the compare's path (timing, study R104)
    c0.re <= {2'b00, ea} - {2'b00, e} + 10'd127;
    c0.tin <= in_valid && ((xz && f != 23'd0) || (az && fa != 23'd0));
    if (xn || an || (xz && az) || (xi && ai)) c0.special <= 2'd3;
    else if (ai || xz) c0.special <= 2'd1;
    else if (az || xi) c0.special <= 2'd2;
    else c0.special <= 2'd0;
    t0 <= seed(f[22:16]);
  end
  // stage 1: p1 = M * t (Q2.34)
  logic [35:0] p1;
  logic [11:0] t1;
  always_ff @(posedge clk) begin
    c1 <= c0;
    c1.re <= c0.re - {9'd0, !c0.ge};
    t1 <= t0;
    p1 <= c0.m * t0;
  end

  // stage 2: e1s = (2^35 - p1) >> 16 (Q2.18)
  logic [19:0] e1s;
  logic [11:0] t2;
  always_ff @(posedge clk) begin
    logic [35:0] e1;
    t2 <= t1;
    e1 = 36'h8_0000_0000 - p1;
    e1s <= e1[35:16];
  end

  // stage 3: r1 = t * e1s (Q3.29), r1s = r1 >> 9 (Q.20)
  logic [20:0] r1s;
  always_ff @(posedge clk) begin
    logic [31:0] r1;
    r1 = t2 * e1s;
    r1s <= r1[29:9];
  end

  // stage 4: p2 = M * r1s (Q.43)
  logic [44:0] p2;
  logic [20:0] r1s4;
  always_ff @(posedge clk) begin
    r1s4 <= r1s;
    p2 <= m3 * r1s;
  end

  // stage 5: ds = (2^43 - p2) >>> 4, signed, |ds| < 2^24
  logic signed [24:0] ds;
  logic [20:0] r1s5;
  always_ff @(posedge clk) begin
    logic signed [45:0] d;
    r1s5 <= r1s4;
    d = 46'sh800_0000_0000 - $signed({1'b0, p2});
    ds <= d[28:4];
  end

  // stage 6: c = r1s * ds (Q.59), registered out of the DSP
  logic signed [46:0] c6p;
  logic [20:0] r1s6;
  always_ff @(posedge clk) begin
    r1s6 <= r1s5;
    c6p <= $signed({1'b0, r1s5}) * ds;
  end

  // stage 7: r2 = (r1s << 30) + (c >>> 9) (Q.50, ~2^73 / M), r = r2 >> 24
  logic [26:0] r27;
  always_ff @(posedge clk) begin
    logic signed [46:0] csh;
    logic signed [52:0] r2;
    csh = c6p >>> 9;
    r2 = $signed({2'b00, r1s6, 30'd0}) + $signed({{6{csh[46]}}, csh});
    r27 <= r2[50:24];
  end

  // stage 8: P = A * r (~ A 2^49 / M)
  logic [50:0] pq;
  always_ff @(posedge clk) begin
    pq <= am7 * r27;
  end

  // stage 9: Q0 = P >> 26 (A >= M) or >> 25; mq = M * Q0
  logic [48:0] mq;
  logic [24:0] q09;
  always_ff @(posedge clk) begin
    logic [24:0] q0;
    q0 = ge8 ? {1'b0, pq[49:26]} : pq[49:25];
    q09 <= q0;
    mq <= m8 * q0;
  end

  // stage 10: compare M (2 Q0 +- 1) against T = A 2^(s+1), as the sign of
  // d -+ M with d = T - 2 M Q0. Q0 is within [-1, 0] of the rounded quotient,
  // so |d| <= 3 M < 2^26 and d is exact in 30 bits (the 51-bit sums and
  // compares cut to their low 30 bits)
  logic        up10, dn10;
  logic [24:0] q10;
  always_ff @(posedge clk) begin
    logic [29:0] two_mq, tt, d, dlo, dhi;
    q10 <= q09;
    two_mq = {mq[28:0], 1'b0};
    tt = ge9 ? {am9lo, 24'd0} : {am9lo[4:0], 25'd0};
    d = tt - two_mq;
    dlo = d - {6'd0, m9};          // T - M (2 Q0 + 1) >= 0: round up
    dhi = d + {6'd0, m9};          // T - M (2 Q0 - 1) < 0: round down
    up10 <= !dlo[29];
    dn10 <= dhi[29];
  end

  // stage 11: pick Q0 - 1, Q0 or Q0 + 1, pack
  always_ff @(posedge clk) begin
    logic [24:0] q;
    logic [9:0]  re;
    if (up10) q = q10 + 25'd1;
    else if (dn10) q = q10 - 25'd1;
    else q = q10;
    re = c10.re;
    if (q[24]) begin q = q >> 1; re = re + 10'd1; end           // rounded up to 2^24
    out_valid <= c10.v;
    tiny <= c10.v && (c10.tin || (c10.special == 2'd0 && (re[9] || re == 10'd0)));
    case (c10.special)
      2'd1: result <= {c10.s, 8'hff, 23'd0};
      2'd2: result <= {c10.s, 31'd0};
      2'd3: result <= 32'h7fc0_0000;
      default:
        if (re[9] || re == 10'd0) result <= {c10.s, 31'd0};      // below 2^-126: flushed
        else if (re >= 10'd255) result <= {c10.s, 8'hff, 23'd0};
        else result <= {c10.s, re[7:0], q[22:0]};
    endcase
  end

endmodule
