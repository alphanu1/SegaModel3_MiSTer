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
// FP32 square root, round to nearest even (sqrtf), for the cheap frustum
// set-up. Integer square root of N = M * 2^27 (even exponent) or M * 2^28
// (odd), two radicand bits a cycle, 26 cycles, not pipelined: the set-up needs
// four a viewport. root = floor(sqrt(N)) has 26 bits: 24 of mantissa, a guard
// bit and one more; the remainder is the sticky. A subnormal input is taken as
// zero; a negative non-zero input gives NaN, +inf gives +inf.

module r3d_fsqrt (
  input  logic        clk,
  input  logic        rst,
  input  logic        in_valid,
  input  logic [31:0] x,
  output logic        busy,
  output logic        out_valid,
  output logic [31:0] result
);
  logic [51:0] rad;
  logic [29:0] rem;
  logic [25:0] root;
  logic [4:0]  n;
  logic [7:0]  re;
  logic        run;

  assign busy = run;

  always_ff @(posedge clk) begin
    out_valid <= 1'b0;
    if (rst) begin
      run <= 1'b0;
    end else if (!run) begin
      if (in_valid) begin
        logic [8:0] e9;
        e9 = {1'b0, x[30:23]};
        if (x[30:23] == 8'hff) begin
          out_valid <= 1'b1;
          result <= (x[22:0] == 23'd0 && !x[31]) ? 32'h7f80_0000 : 32'h7fc0_0000;
        end else if (x[30:23] == 8'd0) begin
          out_valid <= 1'b1;
          result <= {x[31], 31'd0};
        end else if (x[31]) begin
          out_valid <= 1'b1;
          result <= 32'h7fc0_0000;
        end else begin
          if (x[23]) begin                          // biased exponent odd: unbiased even
            rad <= {1'b0, 1'b1, x[22:0], 27'd0};    // M << 27
            re  <= 8'((e9 + 9'd127) >> 1);
          end else begin
            rad <= {1'b1, x[22:0], 28'd0};          // M << 28
            re  <= 8'((e9 + 9'd126) >> 1);
          end
          rem <= '0;
          root <= '0;
          n <= '0;
          run <= 1'b1;
        end
      end
    end else begin
      logic [29:0] r2, trial;
      logic [25:0] nroot;
      logic [29:0] nrem;
      r2 = {rem[27:0], rad[51:50]};
      trial = {2'b00, root, 2'b01};
      if (r2 >= trial) begin nrem = r2 - trial; nroot = {root[24:0], 1'b1}; end
      else begin nrem = r2; nroot = {root[24:0], 1'b0}; end
      rad <= rad << 2;
      rem <= nrem;
      root <= nroot;
      n <= n + 5'd1;
      if (n == 5'd25) begin
        logic [24:0] mant;
        logic        up;
        up = nroot[1] && (nroot[0] || nrem != 30'd0 || nroot[2]);
        mant = {1'b0, nroot[25:2]} + {24'd0, up};
        out_valid <= 1'b1;
        if (mant[24]) result <= {1'b0, re + 8'd1, 23'd0};
        else result <= {1'b0, re, mant[22:0]};
        run <= 1'b0;
      end
    end
  end
endmodule
