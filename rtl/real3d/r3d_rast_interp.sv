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
// Real3D rasteriser: the shading lane's interpolants, one pixel a clock (D-28).
// Bit for bit r3d_rast_rtl.cpp rtl_tri::eval with depth on 1/w: the shade
// (Gouraud, R51: one shade a vertex, its plane's integer part), and u, v
// through the perspective reciprocal of the 1/w plane.
//
// A plane is v2 = 2c + dx (2x + 1) + dy (2y + 1) at exponent e, exact in 42
// bits (|c| < 2^38, |dx|, |dy| < 2^29). Each gradient splits at bit 18, and
// each half pair is one DSP block summing two 18 x 18 products:
// lo = dxl (2x+1) + dyl (2y+1), hi = dxh (2x+1) + dyh (2y+1), v2 = 2c +
// (hi << 18) + lo. The 1/w value o2 comes with the pixel from the Z walk.
//
//   shade   trunc(v2s 2^(es - 41)) toward zero, saturated to int32
//   u, v    m = the top 24 bits of |o2|, n its leading one, r = round(2^47/m);
//           q = (u2 r) 2^-(n + 24 - eu + eo), the sign applied to the
//           magnitude, truncated toward zero, the low 32 bits
//
// Out with each pixel for fog: r, n, eo and o2 > 0. The per-pixel values wait
// for the reciprocal in an MLAB delay line, not in registers. Every stage
// moves on en; the pipe holds when the output is not taken.
//
// MIP = 1 (study D-33, R67): the pixel's mip level, nearest, out as o_lv. The
// LOD is -log2 B - 2 log2(1/w) with log2(1/w) read off the pixel's Z word,
// whose exponent and top six mantissa bits are the reciprocal's own leading
// one n and normalised m (exponent n + eo + 86, the Z word's bounds), through
// a 64-entry table: L = {exponent, T[m6]}. With the triangle's i_lk = 97664 -
// {E(B), T[B's m6]} (0 when B is not positive and finite: level 0) and
// i_lvm its highest level, lv = clamp((i_lk - 2 L) >> 8, 0, i_lvm): render_d25
// lod = 1, mip = 1, the +128 of its rounding folded into i_lk. Formed off the
// S2 registers into the delay line, four bits wide.

/* verilator lint_off UNUSEDSIGNAL */
module r3d_rast_interp #(
  parameter int TW = 8,
  parameter bit MIP = 1'b0
) (
  input  logic               clk,
  input  logic               rst,

  input  logic               i_valid,
  output logic               i_ready,
  input  logic      [TW-1:0] i_tag,
  input  logic        [8:0]  i_x, i_y,
  input  logic signed [41:0] i_o2,        // the 1/w plane's v2 at the pixel
  input  logic signed [8:0]  i_eo,
  input  logic               i_tex,
  // the triangle's planes: shade, u/w, v/w
  input  logic signed [39:0] i_sc, i_sdx, i_sdy,
  input  logic signed [8:0]  i_se,
  input  logic signed [39:0] i_uc, i_udx, i_udy,
  input  logic signed [8:0]  i_ue,
  input  logic signed [39:0] i_vc, i_vdx, i_vdy,
  input  logic signed [8:0]  i_ve,
  // MIP: the triangle's LOD constant and highest level (r3d_rast_intake's fifth header beat)
  input  logic        [16:0] i_lk,
  input  logic        [3:0]  i_lvm,

  output logic               o_valid,
  input  logic               o_ready,
  output logic      [TW-1:0] o_tag,
  output logic        [8:0]  o_x, o_y,
  output logic               o_tex,
  output logic        [31:0] o_u, o_v,
  output logic        [31:0] o_shade,
  output logic        [24:0] o_r,
  output logic        [5:0]  o_n,
  output logic signed [8:0]  o_eo,
  output logic               o_pos,       // o2 > 0
  output logic        [3:0]  o_lv         // MIP: the mip level (0 with MIP = 0)
);

  wire en = !o_valid || o_ready;
  assign i_ready = en;

  // ---- S1: products, the leading one of |o2| --------------------------------------
  logic               v1;
  logic      [TW-1:0] tag1;
  logic        [8:0]  x1, y1;
  logic               tex1;
  logic        [28:0] los, lou, lov;   // the low halves' sums
  logic signed [23:0] his, hiu, hiv;   // the high halves' sums
  logic signed [39:0] sc1, uc1, vc1;
  logic signed [8:0]  se1, ue1, ve1, eo1;
  logic        [40:0] a1;      // |o2|
  logic               neg1, pos1;
  logic        [5:0]  n1;
  logic        [16:0] lk1;
  logic        [3:0]  lvm1;

  function automatic logic [5:0] lead41(input logic [40:0] a);
    logic [5:0] n;
    n = 6'd0;
    for (int i = 0; i < 41; i++) if (a[i]) n = 6'(i);
    return n;
  endfunction

  function automatic logic [28:0] plo(input logic [39:0] dx, input logic [39:0] dy, input logic [8:0] x, input logic [8:0] y);
    logic [17:0] a, b;
    logic [9:0]  kx, ky;
    a = dx[17:0]; b = dy[17:0]; kx = {x, 1'b1}; ky = {y, 1'b1};
    plo = 29'(a * kx) + 29'(b * ky);
  endfunction
  function automatic logic signed [23:0] phi(input logic [39:0] dx, input logic [39:0] dy, input logic [8:0] x, input logic [8:0] y);
    logic signed [12:0] a, b;
    logic signed [10:0] kx, ky;
    a = dx[30:18]; b = dy[30:18]; kx = {1'b0, x, 1'b1}; ky = {1'b0, y, 1'b1};
    phi = 24'(a * kx) + 24'(b * ky);
  endfunction

  always_ff @(posedge clk) begin
    if (rst) v1 <= 1'b0;
    else if (en) v1 <= i_valid;
    if (en) begin
      tag1 <= i_tag; x1 <= i_x; y1 <= i_y; tex1 <= i_tex;
      los <= plo(i_sdx, i_sdy, i_x, i_y); his <= phi(i_sdx, i_sdy, i_x, i_y);
      lou <= plo(i_udx, i_udy, i_x, i_y); hiu <= phi(i_udx, i_udy, i_x, i_y);
      lov <= plo(i_vdx, i_vdy, i_x, i_y); hiv <= phi(i_vdx, i_vdy, i_x, i_y);
      sc1 <= i_sc; uc1 <= i_uc; vc1 <= i_vc;
      lk1 <= i_lk; lvm1 <= i_lvm;
      se1 <= i_se; ue1 <= i_ue; ve1 <= i_ve; eo1 <= i_eo;
      a1   <= i_o2[41] ? 41'(-i_o2) : i_o2[40:0];
      neg1 <= i_o2[41];
      pos1 <= !i_o2[41] && (i_o2 != 42'sd0);
      n1   <= lead41(i_o2[41] ? 41'(-i_o2) : i_o2[40:0]);
    end
  end

  // ---- S2: plane values, the reciprocal's mantissa --------------------------------
  logic               v2;
  logic      [TW-1:0] tag2;
  logic        [8:0]  x2, y2;
  logic               tex2;
  logic signed [41:0] s2, u2, w2;   // shade, u/w, v/w
  logic signed [8:0]  se2;
  logic signed [9:0]  shu2, shv2;   // n + 24 - e + eo
  logic signed [8:0]  eo2;
  logic        [5:0]  n2;
  logic               neg2, pos2;
  logic        [23:0] m2;
  logic signed [9:0]  ex2;          // MIP: the Z word's exponent, n + eo + 86
  logic        [16:0] lk2;
  logic        [3:0]  lvm2;

  always_ff @(posedge clk) begin
    if (rst) v2 <= 1'b0;
    else if (en) v2 <= v1;
    if (en) begin
      logic [81:0] sh;
      ex2 <= 10'(n1) + {eo1[8], eo1} + 10'sd86;
      lk2 <= lk1; lvm2 <= lvm1;
      tag2 <= tag1; x2 <= x1; y2 <= y1; tex2 <= tex1;
      s2 <= {sc1[39], sc1, 1'b0} + {his, 18'd0} + {13'd0, los};
      u2 <= {uc1[39], uc1, 1'b0} + {hiu, 18'd0} + {13'd0, lou};
      w2 <= {vc1[39], vc1, 1'b0} + {hiv, 18'd0} + {13'd0, lov};
      se2 <= se1; eo2 <= eo1; n2 <= n1; neg2 <= neg1; pos2 <= pos1;
      shu2 <= 10'(n1) + 10'sd24 - {ue1[8], ue1} + {eo1[8], eo1};
      shv2 <= 10'(n1) + 10'sd24 - {ve1[8], ve1} + {eo1[8], eo1};
      // the leading one to bit 81, the top 24 bits
      sh = {a1, 41'd0} << (6'd40 - n1);
      m2 <= sh[81:58];
    end
  end

  // ---- S3 .. S12: the reciprocal; the pixel waits in the delay line ----------------
  logic [24:0] r12;
  r3d_rast_recip u_recip (.clk(clk), .en(en), .m(m2), .r(r12));

  // ---- MIP: the level, from the S2 registers into the line ----------------------------
  // 256 log2(1 + (i + 1/2) / 64), rounded (r3d_rast_rtl.cpp LOG2T)
  function automatic logic [7:0] log2t(input logic [5:0] i);
    case (i)
      6'd0: log2t = 8'd3;    6'd1: log2t = 8'd9;    6'd2: log2t = 8'd14;   6'd3: log2t = 8'd20;
      6'd4: log2t = 8'd25;   6'd5: log2t = 8'd30;   6'd6: log2t = 8'd36;   6'd7: log2t = 8'd41;
      6'd8: log2t = 8'd46;   6'd9: log2t = 8'd51;   6'd10: log2t = 8'd56;  6'd11: log2t = 8'd61;
      6'd12: log2t = 8'd66;  6'd13: log2t = 8'd71;  6'd14: log2t = 8'd75;  6'd15: log2t = 8'd80;
      6'd16: log2t = 8'd85;  6'd17: log2t = 8'd89;  6'd18: log2t = 8'd94;  6'd19: log2t = 8'd98;
      6'd20: log2t = 8'd103; 6'd21: log2t = 8'd107; 6'd22: log2t = 8'd111; 6'd23: log2t = 8'd116;
      6'd24: log2t = 8'd120; 6'd25: log2t = 8'd124; 6'd26: log2t = 8'd128; 6'd27: log2t = 8'd132;
      6'd28: log2t = 8'd136; 6'd29: log2t = 8'd140; 6'd30: log2t = 8'd144; 6'd31: log2t = 8'd148;
      6'd32: log2t = 8'd152; 6'd33: log2t = 8'd155; 6'd34: log2t = 8'd159; 6'd35: log2t = 8'd163;
      6'd36: log2t = 8'd167; 6'd37: log2t = 8'd170; 6'd38: log2t = 8'd174; 6'd39: log2t = 8'd178;
      6'd40: log2t = 8'd181; 6'd41: log2t = 8'd185; 6'd42: log2t = 8'd188; 6'd43: log2t = 8'd192;
      6'd44: log2t = 8'd195; 6'd45: log2t = 8'd198; 6'd46: log2t = 8'd202; 6'd47: log2t = 8'd205;
      6'd48: log2t = 8'd208; 6'd49: log2t = 8'd212; 6'd50: log2t = 8'd215; 6'd51: log2t = 8'd218;
      6'd52: log2t = 8'd221; 6'd53: log2t = 8'd224; 6'd54: log2t = 8'd228; 6'd55: log2t = 8'd231;
      6'd56: log2t = 8'd234; 6'd57: log2t = 8'd237; 6'd58: log2t = 8'd240; 6'd59: log2t = 8'd243;
      6'd60: log2t = 8'd246; 6'd61: log2t = 8'd249; 6'd62: log2t = 8'd252; default: log2t = 8'd255;
    endcase
  endfunction
  logic [3:0] lv2;
  always_comb begin
    logic [7:0]  ze;
    logic [5:0]  zm;
    logic signed [17:0] d;
    // the Z word's fields: 0 for 1/w <= 0 or an exponent <= 0, all ones at >= 255
    if (!pos2 || ex2 <= 10'sd0) begin ze = 8'd0; zm = 6'd0; end
    else if (ex2 >= 10'sd255) begin ze = 8'd255; zm = 6'd63; end
    else begin ze = ex2[7:0]; zm = m2[22:17]; end
    d = $signed({1'b0, lk2}) - $signed({1'b0, ze, log2t(zm), 1'b0});
    if (!MIP || d[17]) lv2 = 4'd0;
    else if (d[16:8] >= {5'd0, lvm2}) lv2 = lvm2;
    else lv2 = d[11:8];
  end

  localparam int DB = TW + 9 + 9 + 1 + 42 + 42 + 42 + 9 + 10 + 10 + 9 + 6 + 1 + 1 + 1;
  localparam int DW = DB + (MIP ? 4 : 0);
  localparam int LAG = 11;
  (* ramstyle = "MLAB, no_rw_check" *) logic [DW-1:0] dl [16];
  logic [3:0] dwp;
  logic [10:0] vsr;   // valid bits along the line
  logic [DW-1:0] dl_d;
  generate if (MIP) begin : g_dlm
    assign dl_d = {tag2, x2, y2, tex2, s2, u2, w2, se2, shu2, shv2, eo2, n2, neg2, pos2, v2, lv2};
  end else begin : g_dl
    assign dl_d = {tag2, x2, y2, tex2, s2, u2, w2, se2, shu2, shv2, eo2, n2, neg2, pos2, v2};
  end endgenerate
  always_ff @(posedge clk) begin
    if (en) dl[dwp] <= dl_d;
  end
  always_ff @(posedge clk) begin
    if (rst) begin dwp <= 4'd0; vsr <= '0; end
    else if (en) begin dwp <= dwp + 4'd1; vsr <= {vsr[9:0], v2}; end
  end
  wire [DW-1:0] dq = dl[4'(dwp - 4'(LAG))];

  // ---- S13: the line's output and r, registered --------------------------------------
  logic               v13;
  logic      [TW-1:0] tag13;
  logic        [8:0]  x13, y13;
  logic               tex13;
  logic signed [41:0] s13, u13, w13;
  logic signed [8:0]  se13, eo13;
  logic signed [9:0]  shu13, shv13;
  logic        [5:0]  n13;
  logic               neg13, pos13;
  logic        [24:0] r13;
  logic               dv13;
  logic        [3:0]  lv13, lv14, lv15;
  always_ff @(posedge clk) begin
    if (rst) v13 <= 1'b0;
    else if (en) v13 <= vsr[10];
    if (en) begin
      {tag13, x13, y13, tex13, s13, u13, w13, se13, shu13, shv13, eo13, n13, neg13, pos13, dv13} <= dq[DW-1 -: DB];
      lv13 <= MIP ? dq[3:0] : 4'd0;
      r13 <= r12;
    end
  end

  // ---- S14: products, the shade's leading one ------------------------------------------
  logic               v14;
  logic      [TW-1:0] tag14;
  logic        [8:0]  x14, y14;
  logic               tex14;
  logic        [65:0] pu14, pv14;
  logic               su14, sv14;          // sign of the quotient
  logic               zu14;                // o2 = 0: u = v = 0
  logic signed [9:0]  shu14, shv14;
  logic        [40:0] sa14;                // |shade plane|
  logic               ss14, sz14;
  logic        [5:0]  sn14;
  logic signed [9:0]  ssh14;               // es - 41
  logic signed [8:0]  eo14;
  logic        [5:0]  n14;
  logic               pos14;
  logic        [24:0] r14;
  always_ff @(posedge clk) begin
    if (rst) v14 <= 1'b0;
    else if (en) v14 <= v13;
    if (en) begin
      logic [40:0] au, aw;
      tag14 <= tag13; x14 <= x13; y14 <= y13; tex14 <= tex13;
      au = u13[41] ? 41'(-u13) : u13[40:0];
      aw = w13[41] ? 41'(-w13) : w13[40:0];
      pu14 <= 66'(au) * 66'(r13);
      pv14 <= 66'(aw) * 66'(r13);
      su14 <= u13[41] ^ neg13;
      sv14 <= w13[41] ^ neg13;
      zu14 <= !pos13 && !neg13;
      shu14 <= shu13; shv14 <= shv13;
      sa14 <= s13[41] ? 41'(-s13) : s13[40:0];
      ss14 <= s13[41];
      sz14 <= (s13 == 42'sd0);
      sn14 <= lead41(s13[41] ? 41'(-s13) : s13[40:0]);
      ssh14 <= {se13[8], se13} - 10'sd41;
      eo14 <= eo13; n14 <= n13; pos14 <= pos13; r14 <= r13; lv14 <= lv13;
    end
  end

  // ---- S15: the shifts ---------------------------------------------------------------------
  // u: the magnitude's bits [sh + 31 : sh], from 2^-32 (sh = -32) to 2^98 (sh = 98)
  function automatic logic [31:0] qsh(input logic [65:0] p, input logic signed [9:0] sh);
    logic [130:0] x;
    logic [7:0]   t;
    if (sh < -10'sd32 || sh > 10'sd98) return 32'd0;
    t = 8'(sh + 10'sd32);
    x = {33'd0, p, 32'd0} >> t;
    return x[31:0];
  endfunction

  logic               v15;
  logic      [TW-1:0] tag15;
  logic        [8:0]  x15, y15;
  logic               tex15;
  logic        [31:0] qu15, qv15, qs15;
  logic               su15, sv15, ss15, sat15;
  logic signed [8:0]  eo15;
  logic        [5:0]  n15;
  logic               pos15;
  logic        [24:0] r15;
  always_ff @(posedge clk) begin
    if (rst) v15 <= 1'b0;
    else if (en) v15 <= v14;
    if (en) begin
      logic signed [10:0] top;
      logic [72:0] xs;
      logic [6:0]  k;
      tag15 <= tag14; x15 <= x14; y15 <= y14; tex15 <= tex14;
      qu15 <= zu14 ? 32'd0 : qsh(pu14, shu14);
      qv15 <= zu14 ? 32'd0 : qsh(pv14, shv14);
      su15 <= su14 && !zu14; sv15 <= sv14 && !zu14;
      // shade: |s| 2^ssh; saturate when its leading one lands at bit 31 or above
      top = {5'd0, sn14} + {ssh14[9], ssh14};
      sat15 <= !sz14 && (top >= 11'sd31);
      ss15 <= ss14;
      if (ssh14 <= -10'sd42 || sz14) qs15 <= 32'd0;
      else begin
        // ssh in (-42, 31): |s| 2^ssh = ({|s|, 32 zeros} >> (32 - ssh))
        k = 7'(10'sd32 - ssh14);
        xs = {sa14, 32'd0} >> k;
        qs15 <= {1'b0, xs[30:0]};
      end
      eo15 <= eo14; n15 <= n14; pos15 <= pos14; r15 <= r14; lv15 <= lv14;
    end
  end

  // ---- out: signs and saturation ----------------------------------------------------------
  always_ff @(posedge clk) begin
    if (rst) o_valid <= 1'b0;
    else if (en) o_valid <= v15;
    if (en) begin
      o_tag <= tag15; o_x <= x15; o_y <= y15; o_tex <= tex15;
      o_u <= su15 ? 32'd0 - qu15 : qu15;
      o_v <= sv15 ? 32'd0 - qv15 : qv15;
      if (sat15) o_shade <= ss15 ? 32'h8000_0000 : 32'h7fff_ffff;
      else o_shade <= ss15 ? 32'd0 - qs15 : qs15;
      o_r <= r15; o_n <= n15; o_eo <= eo15; o_pos <= pos15; o_lv <= MIP ? lv15 : 4'd0;
    end
  end

endmodule
