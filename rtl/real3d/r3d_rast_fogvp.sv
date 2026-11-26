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
// Real3D rasteriser: a viewport's fog record from the geometry's viewport
// record (R62: {0, pass}, node words 0x20, 0x22, 0x23, 0x25, kz), bit for bit
// r3d_rast_rtl.cpp make_fog, once a viewport, so everything is serial:
//
//   fc  per channel p = colour x ambient, (p + (p >> 8) + 128) >> 8: one
//       shift-add multiplier, 8 cycles a channel
//   S   (start16 x 16448 + 2048) >> 12 = shifts and one add; clamped to
//       [-4096, 1024], where lin = clamp(S + P, 0, 1024) with P in [0, 4096]
//       cannot tell it from the full value (14 bits)
//   K   |density| / kz, density 0 unless below 99999; 0 unless kz > 0 and
//       finite, and 0 when the quotient is not finite. A restoring divide,
//       25 steps, round to nearest even. Subnormal operands and results are
//       flushed to zero: fog's P is 0 for any K that is 0 or subnormal
//       (r3d_rast_fogp), so the only difference from the model is a subnormal
//       density or kz with a normal quotient, which no frame has.
//
// One record at a time: i_ready falls from the last word until the result
// (about 28 cycles).

module r3d_rast_fogvp (
  input  logic               clk,
  input  logic               rst,
  input  logic               i_valid,
  output logic               i_ready,
  input  logic        [31:0] i_data,
  input  logic               i_last,
  output logic               o_valid,      // a pulse: the record's result
  output logic        [31:0] o_k,
  output logic signed [13:0] o_s,
  output logic        [23:0] o_fc
);

  logic [2:0]  k;
  logic [23:0] col;
  logic [31:0] dens, n25, kz;
  logic        busy;
  logic [4:0]  cnt;
  logic [24:0] r;
  logic [24:0] q;
  logic [23:0] mb;
  logic signed [9:0] e;
  logic        kzero;
  logic [15:0] acc;
  logic [23:0] fc;
  logic        k_up;                  // (r3d140) the round-up bit, K rounded down, q + 1: cnt 24 to 25
  logic [31:0] k_0;
  logic [24:0] k_s1;

  assign i_ready = !busy;

  // the word count's two decodes the record's last word looks at, kept as registers beside k (c7_s1:
  // k -> the select of d and z -> the mantissa compare -> r, -0.93 ns)
  logic k3, k5;
  // the divide's compare as the subtract's borrow, a carry chain shared with the step's subtract (the
  // compare was a LUT tree in front of the guard's round-up and o_k: 108.6 MHz standalone)
  wire [25:0] rsub = {1'b0, r} - {2'b0, mb};
  wire        rge  = !rsub[25];

  // the start term
  wire signed [15:0] st16 = n25[15:0];
  wire signed [31:0] sfull = (32'(st16) <<< 14) + (32'(st16) <<< 6) + 32'sd2048;
  wire signed [31:0] s12 = sfull >>> 12;
  wire signed [13:0] sclp = (s12 < -32'sd4096) ? -14'sd4096 : (s12 > 32'sd1024) ? 14'sd1024 : 14'(s12);
  // (r3d140) the clamped start term as a register, every clock: n25 loads only with the record's word 4, while the
  // unit is not busy, and o_s takes it at cnt 25, many clocks on (n25 -> the add and the clamps -> o_s was -0.65 /
  // -1.10 ns standalone at 140 MHz)
  logic signed [13:0] sclp_q;
  always_ff @(posedge clk) sclp_q <= sclp;

  // the multiplier's operand: channel cnt[4:3] of the colour, bit cnt[2:0] of the ambient
  // (r3d140: the step's operand registered a step ahead, mt_q, from cnt + 1 (cnt counts every busy cycle) and from 0
  // as the record starts: cnt's selects and the shift were in front of the add into acc and af, u_fv cnt -> af
  // -2.58 ns in the 140 MHz full fit)
  wire [7:0]  amb = n25[23:16];
  function automatic logic [15:0] mterm(input logic [4:0] c, input logic [23:0] cl, input logic [7:0] am);
    logic [7:0] h;
    h = (c[4:3] == 2'd0) ? cl[23:16] : (c[4:3] == 2'd1) ? cl[15:8] : cl[7:0];
    mterm = am[c[2:0]] ? (16'(h) << c[2:0]) : 16'd0;
  endfunction
  logic [15:0] mt_q;
  wire [15:0] acc_n = acc + mt_q;
`ifdef VERILATOR
  always_ff @(posedge clk) if (!rst && busy && cnt < 5'd24 && mt_q != mterm(cnt, col, amb)) $fatal(1, "fogvp: the step's operand a step ahead");
`endif
  // a channel's product is rounded the cycle after its last step (the round's add after the
  // step's was a -40C path); the last channel's goes straight to o_fc
  logic [15:0] af;
  logic [1:0]  af_c;
  logic        af_v;
  wire [16:0] pr  = 17'(af) + 17'(af >> 8) + 17'd128;

  always_ff @(posedge clk) begin
    if (rst) begin k <= '0; k3 <= 1'b0; k5 <= 1'b0; busy <= 1'b0; o_valid <= 1'b0; af_v <= 1'b0; end
    else begin
      o_valid <= 1'b0;
      if (!busy) begin
        if (i_valid) begin
          case (k)
            3'd2: col <= i_data[23:0];
            3'd3: dens <= i_data;
            3'd4: n25 <= i_data;
            3'd5: kz <= i_data;
            default: ;
          endcase
          k <= k + 3'd1; k3 <= k == 3'd2; k5 <= k == 3'd4;
          if (i_last) begin
            logic [31:0] d, z;
            logic [23:0] ma;
            /* verilator lint_off UNUSEDSIGNAL */
            logic [23:0] mlt;               // (its borrow, bit 23)
            /* verilator lint_on UNUSEDSIGNAL */
            logic        dz, zbad;
            k <= '0; k3 <= 1'b0; k5 <= 1'b0;
            d = k3 ? i_data : dens;
            z = k5 ? i_data : kz;
            // |density| below 99999 (0x47C34F80), not zero or subnormal; kz > 0, normal
            dz = !({1'b0, d[30:0]} < 32'h47C3_4F80) || d[30:23] == 8'd0;
            zbad = z[31] || z[30:23] == 8'd0 || z[30:23] == 8'hff;
            kzero <= dz || zbad;
            ma = {1'b1, d[22:0]};
            mb <= {1'b1, z[22:0]};
            // the mantissas' compare as a subtract's borrow, a carry chain (a LUT tree from the geometry's word
            // through the select into r and e: 110 MHz merged fit, out_data -> e -0.93 ns)
            mlt = {1'b0, d[22:0]} - {1'b0, z[22:0]};
            if (mlt[23]) begin r <= {ma, 1'b0}; e <= 10'(d[30:23]) - 10'(z[30:23]) + 10'sd126; end
            else begin r <= {1'b0, ma}; e <= 10'(d[30:23]) - 10'(z[30:23]) + 10'sd127; end
`ifdef VERILATOR
            if (mlt[23] != (ma < {1'b1, z[22:0]})) $fatal(1, "fogvp: the record's compare as a borrow");
`endif
            q <= '0; cnt <= '0; acc <= '0;
            mt_q <= mterm(5'd0, (k == 3'd2) ? i_data[23:0] : col, (k == 3'd4) ? i_data[23:16] : n25[23:16]);
            busy <= 1'b1;
          end
        end
      end else begin
        // a quotient bit, and a multiplier step
        if (rge) begin q <= {q[23:0], 1'b1}; r <= rsub[24:0] << 1; end
        else begin q <= {q[23:0], 1'b0}; r <= r << 1; end
        af_v <= 1'b0;
        if (af_v)
          case (af_c)
            2'd0: fc[23:16] <= pr[15:8];
            2'd1: fc[15:8]  <= pr[15:8];
            default: fc[7:0] <= pr[15:8];
          endcase
        if (cnt < 5'd24) begin
          acc <= acc_n;
          if (cnt[2:0] == 3'd7) begin
            acc <= '0;
            af <= acc_n; af_c <= cnt[4:3]; af_v <= 1'b1;
          end
        end
        cnt <= cnt + 5'd1;
        mt_q <= mterm(cnt + 5'd1, col, amb);
        if (cnt == 5'd24) begin
          // q holds 24 bits; this cycle's bit is the last (the guard). K is formed for both
          // roundings side by side and chosen by the round-up bit last (study R109): the guard's
          // compare, the increment and the exponent's range test were in series into o_k.
          // (r3d140: over two cycles, the round-down K, the round-up bit and the incremented significand
          // registered here and the round-up K formed from them in the next: q -> o_k was -0.95 ns at 140 MHz)
          logic        g, st;
          g  = rge;
          st = g ? (r != {1'b0, mb}) : (r != 25'd0);
          k_up <= g && (st || q[0]);
          k_0  <= (kzero || e >= 10'sd255 || e <= 10'sd0) ? 32'd0 : {1'b0, e[7:0], q[22:0]};
          k_s1 <= {1'b0, q[23:0]} + 25'd1;
        end
        if (cnt == 5'd25) begin
          logic [24:0] sg1;
          logic signed [9:0] ee1;
          logic [31:0] k1;
          sg1 = k_s1;
          ee1 = e;
          if (sg1[24]) begin sg1 = {2'b01, 23'd0}; ee1 = e + 10'sd1; end
          k1 = (kzero || ee1 >= 10'sd255 || ee1 <= 10'sd0) ? 32'd0 : {1'b0, ee1[7:0], sg1[22:0]};
          o_k <= k_up ? k1 : k_0;
          o_s <= sclp_q;
          o_fc <= fc;                         // the last channel was rounded into fc at cnt 24
          o_valid <= 1'b1;
          busy <= 1'b0;
        end
      end
    end
  end

`ifdef VERILATOR
  always_ff @(posedge clk) if (!rst && busy && cnt == 5'd25 && sclp_q != sclp) $fatal(1, "fogvp: the start term registered");
  always_ff @(posedge clk) if (!rst && ((k3 != (k == 3'd3)) || (k5 != (k == 3'd5)) || (rge != (r >= {1'b0, mb}))))
    $fatal(1, "fogvp: k3 / k5 / the borrow");
`endif

endmodule
