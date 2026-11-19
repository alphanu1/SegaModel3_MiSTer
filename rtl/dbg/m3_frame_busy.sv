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
// 4C754-4C760 (0x100033, as VF3's 0x8037). Sega Rally 2 (srally2, from its
// code): the frame function (787EC) ends in 7887C-78894, which polls 0x4C2188
// while it is above 0; the vblank handler (IRQ 0x02 at 78A70 -> 786F8) counts it
// down at 78740-78748, and the frame reloads it from 0x4C2184 (78898-788A8). The
// delay function at 788F4 waits N vblanks in 78928-78930 on the same word (a
// wait between frames, at scene changes). game: 0 VF3's windows (Step 1.x and
// every other set), 1 Daytona 2's (Step 2.x), 2 Sega Rally 2's.
//
// Daytona 2's game loop is paced by the decrementer, not by the vblank poll (cpuspeed, 2026-11-17: the board's
// race and the bench's attract counted no frame with the poll alone). The vblank handler (1D74) reloads DEC from
// 0x10000C; the decrementer handler (1C44) counts 0x100033 up. The pacing call (4C6FC) waits in 4C73C-4C744 while
// DEC is negative (expired, not yet reloaded by the next vblank), then in 4C754-4C760 until 0x100033 moves, and the
// loop starts at the fall-through, 4C764. So a loop's deadline is the decrementer event a vblank after the one
// that released it: its frame runs from the exit at 4C764 (counted from 0 there, not from the vblank) to the
// first entry into either wait. The vblank poll 4C6CC-4C6D4 (exit 4C6D8, counted from the vblank) is kept for the
// code that uses it (the boot).
//
// A second figure per frame: the part of it the CPU spent in the game's Real3D DMA waits, where the core's 3D side
// (the DMA engine, the Real3D's host port) can hold it, counted while the last retirement was in one of them:
// VF3 61DC4-61DD0 (the 53C810 poll, R171); Daytona 2 2B64-2B7C (C200000C bit 7, the DMA engine busy) and 52A4-52C4
// (0x700100, the DMA's completion); Sega Rally 2 its DMA calls whole, 6F468-6F4DC and 6F50C-6F558 (C200000C bit 7
// before and after each start; every Real3D word it sends, the 88000000 flush included, goes through them). Latched
// with the rest once a second: {min, max} of the DMA wait, the longest frame's {DMA wait, the rest}, and the DMA
// wait summed over the second.

module m3_frame_busy #(
  parameter int CLK_HZ = 100_000_000
) (
  input  logic        clk,
  input  logic        rst,
  input  logic        vblank_start,   // a pulse at each frame's vblank
  input  logic [1:0]  game,           // the windows: 0 VF3's, 1 Daytona 2's, 2 Sega Rally 2's
  input  logic        rt_valid,       // the core's retirement port
  input  logic [31:0] rt_pc,
  output logic [31:0] o_minmax,       // {min, max}, units of 256 clocks
  output logic [31:0] o_bins_lo,      // {< 0.9, 0.9-1.0, 1.0-1.1, 1.1-1.25} frames
  output logic [31:0] o_bins_hi,      // {1.25-1.5, >= 1.5, game, frames}
  output logic [31:0] o_last,         // the last frame's busy time, units of 256 clocks
  output logic [31:0] o3_mm,          // {min, max} of the DMA wait, units of 256 clocks
  output logic [31:0] o3_long,        // the longest frame's {DMA wait, the rest}
  output logic [31:0] o3_sum          // the DMA wait over the second
);

  // one video frame in units of 256 clocks, and the bins' edges
  // (x * 278144 / 1000 taken as x * 278 + x * 144 / 1000, exact since x * 278 is whole: x * 278144 itself
  // passes 2^31 above 123 MHz)
  localparam int P256 = ((CLK_HZ / 16000) * 278 + (CLK_HZ / 16000) * 144 / 1000) / 256;   // 6790 at 100 MHz
  localparam logic [15:0] E09 = 16'(P256 * 9 / 10);
  localparam logic [15:0] E10 = 16'(P256);
  localparam logic [15:0] E11 = 16'(P256 * 11 / 10);
  localparam logic [15:0] E12 = 16'(P256 * 5 / 4);
  localparam logic [15:0] E15 = 16'(P256 * 3 / 2);

  // the windows: rt_pc[31:20] zero, then 16 bits of the block and a word mask
  /* verilator lint_off UNUSEDSIGNAL */
  function automatic logic win(input logic [31:0] pc, input logic [1:0] g);
    logic [15:0] b;
    logic [1:0]  w;
    b = pc[19:4]; w = pc[3:2];
    if (pc[31:20] != 12'd0) win = 1'b0;
    else if (g == 2'd0)
      win = (b == 16'h1E92 && w[1]) || (b == 16'h1E93 && w == 2'd0) ||
            (b == 16'h1E97 && w == 2'd3) || (b == 16'h1E98 && !w[1]);
    else if (g == 2'd1)                   // 4C6CC-4C6D4, 4C73C-4C744, 4C754-4C760
      win = (b == 16'h4C6C && w == 2'd3) || (b == 16'h4C6D && !w[1]) ||
            (b == 16'h4C73 && w == 2'd3) || (b == 16'h4C74 && !w[1]) ||
            (b == 16'h4C75 && w != 2'd0) || (b == 16'h4C76 && w == 2'd0);
    else                                  // 7888C-78894, 78928-78930
      win = (b == 16'h7888 && w == 2'd3) || (b == 16'h7889 && !w[1]) ||
            (b == 16'h7892 && w[1]) || (b == 16'h7893 && w == 2'd0);
  endfunction
  /* verilator lint_on UNUSEDSIGNAL */

  // the Real3D DMA waits
  function automatic logic dw(input logic [31:0] pc, input logic [1:0] g);
    if (pc[31:20] != 12'd0)  dw = 1'b0;
    else if (g == 2'd0)      dw = pc[19:2] >= 18'h18771 && pc[19:2] <= 18'h18774;
    else if (g == 2'd1)      dw = (pc[19:2] >= 18'hAD9 && pc[19:2] <= 18'hADF) ||
                                  (pc[19:2] >= 18'h14A9 && pc[19:2] <= 18'h14B1);
    else                     dw = (pc[19:2] >= 18'h1BD1A && pc[19:2] <= 18'h1BD37) ||
                                  (pc[19:2] >= 18'h1BD43 && pc[19:2] <= 18'h1BD56);
  endfunction

  // the loops' fall-throughs
  function automatic logic xit(input logic [31:0] pc, input logic [1:0] g);
    if (g == 2'd0)      xit = pc == 32'h1E934 || pc == 32'h1E988;
    else if (g == 2'd1) xit = pc == 32'h4C6D8 || pc == 32'h4C764;
    else                xit = pc == 32'h78898 || pc == 32'h78934;
  endfunction

  logic        in3;                    // the last retirement was in a DMA wait
  logic [15:0] d3, cand3;              // this frame's DMA wait so far; at the frame's end
  logic [15:0] m3n, m3x, l3d, l3c;
  logic [31:0] s3;
  logic        hit, ext, exd;          // a retirement in a window / at a loop's exit / at the decrementer wait's exit
  always_ff @(posedge clk) begin
    hit <= !rst && rt_valid && win(rt_pc, game);
    ext <= !rst && rt_valid && xit(rt_pc, game);
    exd <= !rst && rt_valid && game == 2'd1 && rt_pc == 32'h4C764;
    if (rst) in3 <= 1'b0; else if (rt_valid) in3 <= dw(rt_pc, game);
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
      d3 <= 16'd0; cand3 <= 16'd0; m3n <= 16'hFFFF; m3x <= 16'd0; l3d <= 16'd0; l3c <= 16'd0; s3 <= 32'd0;
    end else begin
      vpre <= vblank_start ? 8'd0 : vpre + 8'd1;
      if (vblank_start) vage <= 16'd0;
      else if (vpre == 8'hFF && vage != 16'hFFFF) vage <= vage + 16'd1;
      if (run) begin
        pre <= pre + 8'd1;
        if (pre == 8'hFF && busy != 16'hFFFF) busy <= busy + 16'd1;
        if (pre == 8'hFF && in3 && d3 != 16'hFFFF) d3 <= d3 + 16'd1;
      end
      if (vblank_start) vbs <= 1'b1;
      if (hit && !inw) begin                      // into a loop: the frame may end here
        inw <= 1'b1; cand <= busy; cand3 <= d3;
      end
      if (ext) begin                              // out of a loop
        inw <= 1'b0;
        if (!run || vbs || vblank_start) begin
          // a vblank since the frame began: it ended at the last loop entry, and the next one starts now
          // (from the vblank, or for a decrementer-paced loop from its exit)
          run <= 1'b1; pre <= exd ? 8'd0 : vpre; busy <= exd ? 16'd0 : vage; vbs <= 1'b0; d3 <= 16'd0;
          if (run) begin
`ifdef M3_FBUSY_LOG
            $display("fbusy %0d (x256 clocks, %0d.%02d vblanks) dma %0d", cand, cand / E10, (cand % E10) * 100 / E10, cand3);
`endif
            o_last <= {16'd0, cand};
            if (cand < mn) mn <= cand;
            if (cand > mx) begin mx <= cand; l3d <= cand3; l3c <= cand - cand3; end
            if (cand3 < m3n) m3n <= cand3;
            if (cand3 > m3x) m3x <= cand3;
            s3 <= s3 + {16'd0, cand3};
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
        o_bins_hi <= {bn[4], bn[5], 6'd0, game, nf};   // (the windows in use in the third byte)
        mn <= 16'hFFFF; mx <= 16'd0; nf <= 8'd0;
        for (int k = 0; k < 6; k++) bn[k] <= 8'd0;
        o3_mm <= {m3n, m3x}; o3_long <= {l3d, l3c}; o3_sum <= s3;
        m3n <= 16'hFFFF; m3x <= 16'd0; l3d <= 16'd0; l3c <= 16'd0; s3 <= 32'd0;
      end
    end
  end

endmodule
