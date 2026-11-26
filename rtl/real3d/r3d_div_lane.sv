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
// One unsigned restoring divider lane, two quotient bits a cycle, for the
// rasteriser's per-triangle divisions (edge slopes, and the set-up's FP32
// divides through r3d_rast_div's callers).
//
// The caller loads the dividend bits still to bring down, MSB first at [53],
// an even count of them (nb2 pairs), and the remainder those above them have
// already left (r0 < d). The lane then gives q = the quotient of those pairs
// and the final remainder. There is no normaliser: the caller skips a fixed
// number of leading dividend bits when the divisor is known to be large
// enough, which needs only a 2:1 select.

module r3d_div_lane #(
  // R4 (r3d140): the two bits of a cycle as one radix-4 step, 4r + b1 b0 against d, 2d and 3d side by side (3d formed
  // as the divisor is taken), the digit the largest that fits: the same quotient and remainder as the two restoring
  // steps in series, which were the lane's path (r -> r, -0.65 ns standalone and -1.67 in the 140 MHz full fit)
  parameter bit R4 = 1'b0   // (r3d140: off again for area: t17 did not fit, 4,194 of 4,191 LABs; +96 ALM over the radix-2 form)
) (
  input  logic        clk,
  input  logic        rst,
  input  logic        start,      // load; ignored while busy
  input  logic        hold,       // q is still to be read: it holds while idle (else q and r follow the load)
  input  logic [53:0] n,          // dividend bits, next at [53]
  input  logic [31:0] r0,         // initial remainder, < d
  input  logic [31:0] d,          // sampled at start
  input  logic [4:0]  nb2,        // quotient bit pairs, 0..27
  output logic        busy,
  output logic [53:0] q,
  output logic [31:0] r
);

  logic [53:0] n_r;
  logic [31:0] d_r;
  logic [33:0] d3_r;           // R4: 3 d
  logic [4:0]  k_r;

  // one restoring step: {quotient bit, remainder}
  function automatic logic [32:0] step(input logic [31:0] rr, input logic b, input logic [31:0] dd);
    logic [32:0] s;
    s = {rr, b} - {1'b0, dd};
    if (!s[32]) step = {1'b1, s[31:0]};
    else        step = {1'b0, rr[30:0], b};
  endfunction

  assign busy = k_r != 5'd0;

  // q and r take the load's values in every idle cycle where q is not still to be read (hold), so at a start
  // they hold them as before; only k_r waits on start. A start never comes with hold (the caller takes a
  // division only once the last is read). start ran into q's 54 enables a lane through the front's t_go, the
  // rasteriser front's worst path at 120 MHz (tclose3).
  // (r3d140: the reset only on k_r; q and r in a block of their own, so the reset is in none of their 86 enables:
  // rst_rast -> q / r, 119 + 62 endpoints at -1.1 to -1.7 ns in the 140 MHz full fit. busy is k_r's, so q and r
  // move as before outside the reset; in it they follow the load, which nothing reads.)
  always_ff @(posedge clk) begin
    if (rst) k_r <= 5'd0;
    else if (!busy) begin
      if (start) k_r <= nb2;
    end else k_r <= k_r - 5'd1;
  end
  always_ff @(posedge clk) begin
    if (!busy) begin
      if (!hold) begin
        r <= r0;
        q <= 54'd0;
      end
    end else if (R4) begin
      logic [33:0] x;
      logic [34:0] s1, s2, s3;
      x  = {r, n_r[53], n_r[52]};                       // 4 r + the pair, < 4 d (r < d)
      s1 = {1'b0, x} - {3'b000, d_r};
      s2 = {1'b0, x} - {2'b00, d_r, 1'b0};
      s3 = {1'b0, x} - {1'b0, d3_r};
      if (!s3[34])      begin r <= s3[31:0]; q <= {q[51:0], 2'd3}; end
      else if (!s2[34]) begin r <= s2[31:0]; q <= {q[51:0], 2'd2}; end
      else if (!s1[34]) begin r <= s1[31:0]; q <= {q[51:0], 2'd1}; end
      else              begin r <= x[31:0];  q <= {q[51:0], 2'd0}; end
`ifdef VERILATOR
      begin
        logic [32:0] t1, t2;
        t1 = step(r, n_r[53], d_r);
        t2 = step(t1[31:0], n_r[52], d_r);
        if (!s3[34] ? (s3[31:0] != t2[31:0] || {t1[32], t2[32]} != 2'd3) :
            !s2[34] ? (s2[31:0] != t2[31:0] || {t1[32], t2[32]} != 2'd2) :
            !s1[34] ? (s1[31:0] != t2[31:0] || {t1[32], t2[32]} != 2'd1) :
                      (x[31:0] != t2[31:0] || {t1[32], t2[32]} != 2'd0))
          $fatal(1, "div_lane: the radix-4 step against the two restoring steps");
      end
`endif
    end else begin
      logic [32:0] t1, t2;
      t1 = step(r, n_r[53], d_r);
      t2 = step(t1[31:0], n_r[52], d_r);
      r   <= t2[31:0];
      q   <= {q[51:0], t1[32], t2[32]};
    end
  end

  // the dividend and the divisor follow the inputs while the lane is idle, so they hold the start's when it
  // starts; only q, r and k_r wait on start (it fanned out to 140 enables a lane, the edge's state through the
  // front's go: 110 MHz merged fits, w_act / m_busy -> q / n_r -0.97 ns)
  always_ff @(posedge clk)
    if (!busy) begin n_r <= n; d_r <= d; d3_r <= {2'b00, d} + {1'b0, d, 1'b0}; end
    else n_r <= {n_r[51:0], 2'b00};

`ifdef VERILATOR
  always_ff @(posedge clk) if (!rst && start && !busy && hold) $fatal(1, "div_lane: a start while q is held");
`endif

  // a 33-bit {r, b} is kept in 32 bits: r < d < 2^32 and the step's result is
  // either r'' < d or the shifted r, whose top bit is lost only when 2 r + b
  // >= 2^32 > d, which the subtract then takes. So the else branch never loses a
  // set bit: when s is negative, {r, b} < d < 2^32 and r[31] is 0.

endmodule
