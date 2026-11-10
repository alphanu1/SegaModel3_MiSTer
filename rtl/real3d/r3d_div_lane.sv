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

module r3d_div_lane (
  input  logic        clk,
  input  logic        rst,
  input  logic        start,      // load; ignored while busy
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
  logic [4:0]  k_r;

  // one restoring step: {quotient bit, remainder}
  function automatic logic [32:0] step(input logic [31:0] rr, input logic b, input logic [31:0] dd);
    logic [32:0] s;
    s = {rr, b} - {1'b0, dd};
    if (!s[32]) step = {1'b1, s[31:0]};
    else        step = {1'b0, rr[30:0], b};
  endfunction

  assign busy = k_r != 5'd0;

  always_ff @(posedge clk) begin
    if (rst) begin
      k_r <= 5'd0;
    end else if (start && !busy) begin
      r   <= r0;
      q   <= 54'd0;
      k_r <= nb2;
    end else if (busy) begin
      logic [32:0] t1, t2;
      t1 = step(r, n_r[53], d_r);
      t2 = step(t1[31:0], n_r[52], d_r);
      r   <= t2[31:0];
      q   <= {q[51:0], t1[32], t2[32]};
      k_r <= k_r - 5'd1;
    end
  end

  // the dividend and the divisor follow the inputs while the lane is idle, so they hold the start's when it
  // starts; only q, r and k_r wait on start (it fanned out to 140 enables a lane, the edge's state through the
  // front's go: 110 MHz merged fits, w_act / m_busy -> q / n_r -0.97 ns)
  always_ff @(posedge clk)
    if (!busy) begin n_r <= n; d_r <= d; end
    else n_r <= {n_r[51:0], 2'b00};

  // a 33-bit {r, b} is kept in 32 bits: r < d < 2^32 and the step's result is
  // either r'' < d or the shifted r, whose top bit is lost only when 2 r + b
  // >= 2^32 > d, which the subtract then takes. So the else branch never loses a
  // set bit: when s is negative, {r, b} < d < 2^32 and r[31] is 0.

endmodule
