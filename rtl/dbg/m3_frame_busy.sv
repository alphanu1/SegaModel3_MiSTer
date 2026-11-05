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
// The CPU's busy time per frame (study R104), for the overlay: from the
// vblank that released a frame's work to the CPU's next retirement inside
// one of the game's wait loops. The work starts as the CPU leaves a loop
// (retires its fall-through, the window's end + 4); its age since the last
// vblank start is taken there, and the count goes on to the loop's next
// retirement, across any vblank between, so a frame that needed 1.01
// vblanks reads 1.01. (Counting from every vblank instead read about 0 for
// each vblank the CPU was already waiting at: the first form, on the bench.)
//
// Counted in units of 256 clocks. Latched once a second: the shortest and
// longest frame, and a histogram in units of a vblank (P, the video frame
// at CLK_HZ: 656 x 424 pixels at 16 MHz): < 0.9, 0.9-1.0, 1.0-1.1,
// 1.1-1.25, 1.25-1.5, >= 1.5, and the frames counted.
//
// The wait loops (tools/mame-census/README.md): VF3 1E928-1E930,
// 1E97C-1E984, 1E9DC-1E9E4, 1E9FC-1EA04; Daytona 2 4C754-4C760,
// 4C73C-4C744, 4C6CC-4C6D4 (with step2: the game configuration's step).

module m3_frame_busy #(
  parameter int CLK_HZ = 100_000_000
) (
  input  logic        clk,
  input  logic        rst,
  input  logic        vblank_start,   // a pulse at each frame's vblank
  input  logic        step2,          // Daytona 2's windows, else VF3's
  input  logic        rt_valid,       // the core's retirement port
  input  logic [31:0] rt_pc,
  output logic [31:0] o_minmax,       // {min, max}, units of 256 clocks
  output logic [31:0] o_bins_lo,      // {< 0.9, 0.9-1.0, 1.0-1.1, 1.1-1.25} frames
  output logic [31:0] o_bins_hi,      // {1.25-1.5, >= 1.5, 0, frames}
  output logic [31:0] o_last          // the last frame's busy time, units of 256 clocks
);

  // one video frame in units of 256 clocks, and the bins' edges
  // (int arithmetic: (CLK_HZ / 16000) x 278144 stays below 2^31 up to 123 MHz)
  localparam int P256 = ((CLK_HZ / 16000) * 278144 / 1000) / 256;   // 6790 at 100 MHz
  localparam logic [15:0] E09 = 16'(P256 * 9 / 10);
  localparam logic [15:0] E10 = 16'(P256);
  localparam logic [15:0] E11 = 16'(P256 * 11 / 10);
  localparam logic [15:0] E12 = 16'(P256 * 5 / 4);
  localparam logic [15:0] E15 = 16'(P256 * 3 / 2);

  // the windows: rt_pc[31:20] zero, then 16 bits of the block and a word mask
  /* verilator lint_off UNUSEDSIGNAL */
  function automatic logic win(input logic [31:0] pc, input logic s2);
    logic [15:0] b;
    logic [1:0]  w;
    b = pc[19:4]; w = pc[3:2];
    if (pc[31:20] != 12'd0) win = 1'b0;
    else if (!s2)
      win = (b == 16'h1E92 && w[1]) || (b == 16'h1E93 && w == 2'd0) ||
            (b == 16'h1E97 && w == 2'd3) || (b == 16'h1E98 && !w[1]) ||
            (b == 16'h1E9D && w == 2'd3) || (b == 16'h1E9E && !w[1]) ||
            (b == 16'h1E9F && w == 2'd3) || (b == 16'h1EA0 && !w[1]);
    else
      win = (b == 16'h4C75 && w != 2'd0) || (b == 16'h4C76 && w == 2'd0) ||
            (b == 16'h4C73 && w == 2'd3) || (b == 16'h4C74 && !w[1]) ||
            (b == 16'h4C6C && w == 2'd3) || (b == 16'h4C6D && !w[1]);
  endfunction
  /* verilator lint_on UNUSEDSIGNAL */

  // the loops' fall-throughs
  function automatic logic xit(input logic [31:0] pc, input logic s2);
    if (!s2) xit = pc == 32'h1E934 || pc == 32'h1E988 || pc == 32'h1E9E8 || pc == 32'h1EA08;
    else     xit = pc == 32'h4C764 || pc == 32'h4C748 || pc == 32'h4C6D8;
  endfunction

  logic        hit, ext;               // a retirement in a window / at a loop's exit, registered
  always_ff @(posedge clk) begin
    hit <= !rst && rt_valid && win(rt_pc, step2);
    ext <= !rst && rt_valid && xit(rt_pc, step2);
  end
  logic [7:0]  vpre;
  logic [15:0] vage;                   // since the last vblank start, units of 256 clocks

  logic        run;                    // counting this frame
  logic [7:0]  pre;
  logic [15:0] busy;                   // units of 256 clocks (saturates)
  logic [15:0] mn, mx;
  logic [7:0]  bn [6];
  logic [7:0]  nf;
  logic [26:0] sec;
  logic        tick;
  always_ff @(posedge clk) begin
    tick <= (sec == 27'(CLK_HZ - 1));
    sec  <= tick ? 27'd0 : sec + 27'd1;
    if (rst) begin
      run <= 1'b0; pre <= 8'd0; busy <= 16'd0; sec <= 27'd0; tick <= 1'b0; vpre <= 8'd0; vage <= 16'hFFFF;
      mn <= 16'hFFFF; mx <= 16'd0; nf <= 8'd0;
      for (int k = 0; k < 6; k++) bn[k] <= 8'd0;
    end else begin
      vpre <= vblank_start ? 8'd0 : vpre + 8'd1;
      if (vblank_start) vage <= 16'd0;
      else if (vpre == 8'hFF && vage != 16'hFFFF) vage <= vage + 16'd1;
      if (run) begin
        pre <= pre + 8'd1;
        if (pre == 8'hFF && busy != 16'hFFFF) busy <= busy + 16'd1;
      end
      if (run && hit) begin                       // the frame's work is done
`ifdef M3_FBUSY_LOG
        $display("fbusy %0d (x256 clocks, %0d.%02d vblanks)", busy, busy / E10, (busy % E10) * 100 / E10);
`endif
        run <= 1'b0;
        o_last <= {16'd0, busy};
        if (busy < mn) mn <= busy;
        if (busy > mx) mx <= busy;
        nf <= nf + 8'd1;
        if (busy < E09)      bn[0] <= bn[0] + 8'd1;
        else if (busy < E10) bn[1] <= bn[1] + 8'd1;
        else if (busy < E11) bn[2] <= bn[2] + 8'd1;
        else if (busy < E12) bn[3] <= bn[3] + 8'd1;
        else if (busy < E15) bn[4] <= bn[4] + 8'd1;
        else                 bn[5] <= bn[5] + 8'd1;
      end else if (!run && ext) begin            // the next frame's work starts
        run <= 1'b1; pre <= vpre; busy <= vage;
      end
      if (tick) begin
        o_minmax  <= {mn, mx};
        o_bins_lo <= {bn[0], bn[1], bn[2], bn[3]};
        o_bins_hi <= {bn[4], bn[5], 8'd0, nf};
        mn <= 16'hFFFF; mx <= 16'd0; nf <= 8'd0;
        for (int k = 0; k < 6; k++) bn[k] <= 8'd0;
      end
    end
  end

endmodule
