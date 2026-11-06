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
// vblank that released a frame's work to the CPU's last entry into a wait
// loop before it leaves one with a newer vblank behind it. A frame starts as
// the CPU leaves a loop (retires its fall-through, the window's end + 4) with
// a vblank since the last frame began; the age since that vblank is taken
// there. A loop left with no new vblank (VF3 waits in more than one loop a
// frame) is part of the frame. The count goes on across vblanks, so a frame that
// needed 1.01 vblanks reads 1.01. (Earlier forms, on the bench: counting from
// every vblank read about 0 for each vblank the CPU was already waiting at,
// and ending at every loop entry split a frame at its first loop.)
//
// Counted in units of 256 clocks. Latched once a second: the shortest and
// longest frame, and a histogram in units of a vblank (P, the video frame
// at CLK_HZ: 656 x 424 pixels at 16 MHz): < 0.9, 0.9-1.0, 1.0-1.1,
// 1.1-1.25, 1.25-1.5, >= 1.5, and the frames counted.
//
// Only the vblank polls end a frame (study R104, from the code): VF3
// 1E928-1E930 and 1E97C-1E984 poll r13+0x8034, which the vblank handler
// (0x500 -> 0x1ADC, the tile generator's interrupt, acknowledged at F1180010)
// counts up; Daytona 2 4C6CC-4C6D4 polls 0x100030 and clears it, the same
// pattern. The other loops of tools/mame-census/README.md are waits inside a
// frame: VF3 1E9DC-1E9E4 (mfdec), 1E9FC-1EA04 (r13+0x8037, the decrementer
// handler's count), 2518-251C (a count-down); Daytona 2 4C73C-4C744 (mfdec),
// 4C754-4C760 (0x100033, as VF3's 0x8037). With step2 (the game
// configuration's step) Daytona 2's.

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
            (b == 16'h1E97 && w == 2'd3) || (b == 16'h1E98 && !w[1]);
    else
      win = (b == 16'h4C6C && w == 2'd3) || (b == 16'h4C6D && !w[1]);
  endfunction
  /* verilator lint_on UNUSEDSIGNAL */

  // the loops' fall-throughs
  function automatic logic xit(input logic [31:0] pc, input logic s2);
    if (!s2) xit = pc == 32'h1E934 || pc == 32'h1E988;
    else     xit = pc == 32'h4C6D8;
  endfunction

  logic        hit, ext;               // a retirement in a window / at a loop's exit, registered
  always_ff @(posedge clk) begin
    hit <= !rst && rt_valid && win(rt_pc, step2);
    ext <= !rst && rt_valid && xit(rt_pc, step2);
  end
  logic [7:0]  vpre;
  logic [15:0] vage;                   // since the last vblank start, units of 256 clocks

  logic        run;                    // counting this frame
  logic        inw, vbs;               // in a loop (entered since the last exit); a vblank since the frame began
  logic [15:0] cand;                   // the frame's time at that entry
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
      inw <= 1'b0; vbs <= 1'b0; cand <= 16'd0;
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
      if (vblank_start) vbs <= 1'b1;
      if (hit && !inw) begin                      // into a loop: the frame may end here
        inw <= 1'b1; cand <= busy;
      end
      if (ext) begin                              // out of a loop
        inw <= 1'b0;
        if (!run || vbs || vblank_start) begin
          // a vblank since the frame began: it ended at the last loop entry, and the next one starts now
          run <= 1'b1; pre <= vpre; busy <= vage; vbs <= 1'b0;
          if (run) begin
`ifdef M3_FBUSY_LOG
            $display("fbusy %0d (x256 clocks, %0d.%02d vblanks)", cand, cand / E10, (cand % E10) * 100 / E10);
`endif
            o_last <= {16'd0, cand};
            if (cand < mn) mn <= cand;
            if (cand > mx) mx <= cand;
            nf <= nf + 8'd1;
            if (cand < E09)      bn[0] <= bn[0] + 8'd1;
            else if (cand < E10) bn[1] <= bn[1] + 8'd1;
            else if (cand < E11) bn[2] <= bn[2] + 8'd1;
            else if (cand < E12) bn[3] <= bn[3] + 8'd1;
            else if (cand < E15) bn[4] <= bn[4] + 8'd1;
            else                 bn[5] <= bn[5] + 8'd1;
          end
        end
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
