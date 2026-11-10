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
// Real3D command and status (D-37 stage B): the CPU's and the 53C810's
// accesses to the Real3D's windows, the status word, the ping-pong flip and
// the frame sequence.
//
// Host port (from m3_bus): h_go strobes one access while h_rdy; writes are
// posted (one held here until its target takes it), a read answers on h_rv the
// cycle after. Write data is the little-endian value (the bus word
// byte-reversed, as Supermodel's handlers pass it); read data is in the CPU's
// byte order.
//
//   84000000-3F  status (reads). Word 0 is Supermodel's 0xFDFFFFFF with bit 25
//                the ping-pong bit (FFFFFFFD / FFFFFFFF on the bus), words
//                5-8 its line-of-sight floats (0 in its trace), the rest
//                FFFFFFFF. VF3 reads all nine words through the SDK's
//                little-endian copy (0x63F68 / 0x64230). TOT = 1 reports the
//                Stat_Pckt fields instead: gp_done, rend_done and tot_clks,
//                the last frame's drawing in 33.33 MHz ticks (study 5.3, the
//                section 10 risk row); off by default, as no game has been
//                seen to need it.
//   88000000     command port: the frame's data is complete (Supermodel's
//                Flush): the texture words so far are flushed for upload
//   8C/8E/98     culling and polygon RAM: r3d_mem
//   90, 94       VROM port, texture FIFO: r3d_texup
//   9C           buffer configuration: kept in m3_bus with the JTAG (this
//                design's double buffer needs no update-buffer sizes)
//
// The ping-pong bit flips at the start of the line in tile register 08 (when
// it is below 512), as stage A's m3_bus did. Supermodel's DrawFrame needs the
// command port and a write of tile register 0C before the flip; both clear at
// every flip. A frame is drawn when both came since the last flip and the
// Real3D can take it (the geometry idle, r3d_mem's banks in step, a store bank
// free, no upload running); otherwise it is dropped and the last image stays.
//
// A draw: at the flip, r3d_mem swaps its banks and the geometry starts on
// bank R (busy until its records are all out), and the rasteriser begins a
// frame. Flushed textures are uploaded once no earlier frame is still to be
// drawn (they would read the old sheets), and the rasteriser's frame ends only
// when the geometry and the upload are both done, so its replay sees them.
// Layer buffers: four, a frame drawing into a free one; a finished frame is
// shown from the next vertical blank. The Mercury's block-culling bit (its
// JTAG mode word, bit 2) is taken at the draw and hides that frame's 3D.

`timescale 1ns/1ps

module r3d_cmd #(
  parameter bit TOT = 1'b0
) (
  input  logic          clk,
  input  logic          rst,

  // host
  input  logic          h_go,
  output logic          h_rdy,
  input  logic          h_we,
  input  logic [31:0]   h_addr,
  input  logic [31:0]   h_wdata,
  output logic          h_rv,
  output logic [31:0]   h_rdata,

  // tile generator registers (m3_bus's tg_reg_*, little-endian values)
  input  logic          tg_reg_we,
  input  logic [5:0]    tg_reg_addr,
  input  logic [31:0]   tg_reg_wdata,

  // video: a pulse at each line start, and the line (384 starts the blanking)
  input  logic          ln_start,
  input  logic [9:0]    ln,

  // JTAG mode words (m3_jtag)
  input  logic          mw_we,
  input  logic [3:0]    mw_dev,
  input  logic [31:0]   mw_val,

  // r3d_mem
  output logic          m_valid,
  input  logic          m_ready,
  output logic [1:0]    m_reg,
  output logic [19:0]   m_addr,
  output logic [31:0]   m_data,
  output logic          m_swap,
  input  logic          m_can_swap,
  output logic          m_rbusy,

  // r3d_texup
  output logic          t_valid,
  input  logic          t_ready,
  output logic          t_vrom,
  output logic [31:0]   t_data,
  output logic          t_flush,
  input  logic          t_pend,
  output logic          t_mark,
  input  logic          t_mpend,
  output logic          t_go,
  input  logic          t_busy,

  // geometry: started at the draw; g_busy until its last record is taken
  output logic          g_start,
  input  logic          g_busy,

  // rasteriser
  input  logic          f_ready,
  output logic          f_begin,
  output logic          f_end,
  output logic [1:0]    f_buf,          // the layer buffer, with f_end
  input  logic          r_done,         // a frame's layer is complete (in f_end order)
  input  logic          r_busy,         // the rasteriser is replaying (TOT's rend_done)

  // compositor
  output logic [1:0]    show_buf,
  output logic          show_3d,

  output logic          pingpong,
  output logic          flip,
  output logic [15:0]   n_draw,         // frames drawn (begun)
  output logic [15:0]   n_drop          // frames submitted but dropped
);

  // ===========================================================================
  // the held access
  // ===========================================================================
  logic        b_v;
  logic [7:0]  b_hi;                    // address bits 31:24
  logic [21:2] b_a;
  logic [31:0] b_d;
  logic        sw_wait;                   // a draw waits for the writes before the flip
`ifdef M3_DMAFAST
  // (study R144; on in Model3.qsf and the system bench) the held access as a FIFO of PFN posted writes: a write is
  // taken whenever there is room, so the bus and the DMA move on while the windows drain; a read does not wait for
  // them (below), the swap waits for them all, as with the one held access
`ifdef M3_R3B
  // (with r3d_mem's 8E bursts the windows drain near the bus's rate: 32 posted writes, in MLAB; without them the
  // windows drain at about a third of it and 8,192 are a bench model, not buildable)
  localparam int PFN = 32;
`else
  localparam int PFN = 8192;
`endif
  localparam int PFW = $clog2(PFN);
  // {address bits 31:24, 21:2, data}: one 60-bit word, three MLABs (three arrays took four: 8 bits used one)
  (* ramstyle = "MLAB, no_rw_check" *) logic [59:0] pf [0:PFN-1];
  logic [PFW-1:0] pf_w, pf_r;
  logic [PFW:0]   pf_n, pf_max /* verilator public_flat_rd */;
  wire         pf_pop;
  assign b_v = pf_n != '0;
  assign {b_hi, b_a, b_d} = pf[pf_r];
`ifdef M3_DMAFAST_RW
  assign h_rdy = !sw_wait && (h_we ? pf_n < (PFW+1)'(PFN) : pf_n == '0);
`else
  // (a read is the status word or FFFFFFFF, which no posted write changes: it does not wait for the drain; with
  // the wait, the DMA device's register read held the bus for the whole drain, and the CPU's fills behind it)
  // (h_rdy does not look at h_we: m3_bus forms its strobe from h_rdy, so that was a loop)
  assign h_rdy = !sw_wait && pf_n < (PFW+1)'(PFN);
`endif
`else
  assign h_rdy = !b_v && !sw_wait;
`endif

  wire  b_mem = (b_hi == 8'h8C) || (b_hi == 8'h8E) || (b_hi == 8'h98);
  wire  b_tex = (b_hi == 8'h90) || (b_hi == 8'h94);
  wire  b_cmd = (b_hi == 8'h88);
  assign m_valid = b_v && b_mem;
  assign m_reg   = (b_hi == 8'h8C) ? 2'd0 : (b_hi == 8'h8E) ? 2'd1 : 2'd2;
  assign m_addr  = (b_hi == 8'h8E) ? {2'd0, b_a[19:2]} : b_a[21:2];
  assign m_data  = b_d;
  assign t_valid = b_v && b_tex;
  assign t_vrom  = (b_hi == 8'h90);
  assign t_data  = b_d;
  assign t_flush = b_v && b_cmd && t_ready;

  // ===========================================================================
  // the flip, the draw signals, the mode word
  // ===========================================================================
  logic [8:0]  fl_line;
  logic        fl_on;
  logic        cmd_seen, tg_seen;
  logic        merc_bc;                   // Mercury mode word bit 2
  always_ff @(posedge clk) begin
    if (rst) begin fl_line <= 9'd0; fl_on <= 1'b1; merc_bc <= 1'b0; end
    else begin
      if (tg_reg_we && tg_reg_addr == 6'h02) begin fl_line <= tg_reg_wdata[8:0]; fl_on <= (tg_reg_wdata[31:9] == 23'd0); end
      if (mw_we && mw_dev == 4'd1) merc_bc <= mw_val[2];
    end
  end
  assign flip = ln_start && ln == {1'b0, fl_line} && fl_on && ln < 10'd384;
  wire   vbl  = ln_start && ln == 10'd384;

  // ===========================================================================
  // the sequence
  // ===========================================================================
  logic        g_run;                     // the geometry of the frame begun is running
  logic        g_seen;                    // ... and has raised g_busy
  logic        x_need, x_run;             // this frame's upload: wanted, running
  logic        e_wait;                    // a begun frame waits for its f_end
  logic        blk;                       // this frame's block culling
  logic [1:0]  inf_n;                     // frames ended, not yet done
  logic [1:0]  inf_q [2];                 // their buffers, oldest first
  logic [1:0]  pend_i;
  logic        pend_v, show_v;
  logic [3:0]  hide;                      // block culling, by buffer
  logic [22:0] tot, tot_l;                // 33.33 MHz ticks of the frame drawing, and the last
  logic [1:0]  t3;

  // status word 0 as the Real3D holds it: {spare 4, gp_done, dp_done,
  // ping_pong, update_done, rend_done, tot_clks 23}
  wire  [31:0] st0 = TOT ? {4'hF, !g_run, 1'b1, pingpong, 1'b1, !r_busy, tot_l}
                         : {6'h3F, pingpong, 25'h1FF_FFFF};
  function automatic logic [31:0] bswap(input logic [31:0] v);
    bswap = {v[7:0], v[15:8], v[23:16], v[31:24]};
  endfunction

  wire  [3:0]  used = (show_v ? (4'd1 << show_buf) : 4'd0) | (pend_v ? (4'd1 << pend_i) : 4'd0) |
                      ((inf_n != 2'd0) ? (4'd1 << inf_q[0]) : 4'd0) | ((inf_n == 2'd2) ? (4'd1 << inf_q[1]) : 4'd0);
  wire  [1:0]  free_i = !used[0] ? 2'd0 : !used[1] ? 2'd1 : !used[2] ? 2'd2 : 2'd3;
  wire         can_start = !g_run && !e_wait && f_ready;
  wire         end_now = e_wait && g_seen && !g_busy && !x_need && !x_run && !(&used);

`ifdef M3_DMAFAST
  // the FIFO: a write pushed as the host's strobe is taken; the head popped as its target takes it
  wire pf_push = h_go && h_rdy && h_we;
  assign pf_pop = b_v && (b_mem ? m_ready : (b_tex || b_cmd) ? t_ready : 1'b1);
  always_ff @(posedge clk)
    if (rst) begin pf_w <= '0; pf_r <= '0; pf_n <= '0; pf_max <= '0; end
    else begin
`ifdef M3_DMAFAST_FAULT
      // (a planted fault: one word in 8,192 through the FIFO has its low bit flipped; the comparison must catch it)
      if (pf_push) begin pf[pf_w] <= {h_addr[31:24], h_addr[21:2], h_wdata ^ {31'd0, pf_w == PFW'(PFN - 1)}}; pf_w <= pf_w + 1'b1; end
`else
      if (pf_push) begin pf[pf_w] <= {h_addr[31:24], h_addr[21:2], h_wdata}; pf_w <= pf_w + 1'b1; end
`endif
      if (pf_pop) pf_r <= pf_r + 1'b1;
      pf_n <= pf_n + (PFW+1)'(pf_push) - (PFW+1)'(pf_pop);
      if (pf_n > pf_max) pf_max <= pf_n;
    end
`endif
  always_ff @(posedge clk) begin
    h_rv <= 1'b0; m_swap <= 1'b0; g_start <= 1'b0; f_begin <= 1'b0; f_end <= 1'b0; t_go <= 1'b0; t_mark <= 1'b0;
    if (rst) begin
`ifndef M3_DMAFAST
      b_v <= 1'b0;
`endif
      pingpong <= 1'b0; cmd_seen <= 1'b0; tg_seen <= 1'b0;
      g_run <= 1'b0; g_seen <= 1'b0; m_rbusy <= 1'b0; sw_wait <= 1'b0; x_need <= 1'b0; x_run <= 1'b0; e_wait <= 1'b0;
      inf_n <= '0; pend_v <= 1'b0; show_v <= 1'b0; show_buf <= 2'd0; hide <= '0;
      n_draw <= '0; n_drop <= '0; tot <= '0; tot_l <= 23'h7F_FFFF; t3 <= '0;
    end else begin
      // ---- the host
      if (h_go && h_rdy) begin
        if (h_we) begin
`ifndef M3_DMAFAST
          b_v <= 1'b1; b_hi <= h_addr[31:24]; b_a <= h_addr[21:2]; b_d <= h_wdata;
`endif
        end else begin
          h_rv <= 1'b1;
          if (h_addr[5:2] == 4'd0) h_rdata <= bswap(st0);
          else if (h_addr[5:2] >= 4'd5 && h_addr[5:2] <= 4'd8) h_rdata <= 32'd0;
          else h_rdata <= 32'hFFFF_FFFF;
        end
      end
`ifdef M3_DMAFAST
      if (b_v && b_cmd && t_ready) cmd_seen <= 1'b1;
`else
      if (b_v) begin
        if (b_mem) begin if (m_ready) b_v <= 1'b0; end
        else if (b_tex) begin if (t_ready) b_v <= 1'b0; end
        else if (b_cmd) begin if (t_ready) begin b_v <= 1'b0; cmd_seen <= 1'b1; end end
        else b_v <= 1'b0;
      end
`endif
      if (tg_reg_we && tg_reg_addr == 6'h03) tg_seen <= 1'b1;

      // ---- the flip: a draw begins, or the frame is dropped
      if (flip) begin
        pingpong <= !pingpong;
        cmd_seen <= 1'b0; tg_seen <= 1'b0;
        if (cmd_seen && tg_seen) begin
          if (can_start) sw_wait <= 1'b1;
          else n_drop <= n_drop + 16'd1;
        end
      end
      // the swap: once the write held at the flip has gone in and r3d_mem's
      // banks are in step (the host waits meanwhile, so nothing after the flip
      // reaches the frame)
      if (sw_wait && !b_v && m_can_swap) begin
        sw_wait <= 1'b0;
        m_swap <= 1'b1; m_rbusy <= 1'b1;
        g_start <= 1'b1; g_run <= 1'b1; g_seen <= 1'b0;
        f_begin <= 1'b1; e_wait <= 1'b1;
        x_need <= 1'b1; t_mark <= 1'b1; blk <= merc_bc;
        n_draw <= n_draw + 16'd1;
        tot <= '0;
      end

      // ---- the geometry: busy rises, then falls with its last record taken
      if (g_run) begin
        if (g_busy) g_seen <= 1'b1;
        else if (g_seen) begin g_run <= 1'b0; m_rbusy <= 1'b0; end
      end

      // ---- uploads. A frame's: what was flushed by its flip (the mark), once
      // nothing before it is left to draw (it would read the old sheets).
      // Between frames, whatever has been flushed, so the ring never fills.
      // An upload always ends at the mark, so one between frames marks first.
      if (!x_run && !t_busy && inf_n == 2'd0 && !t_mark) begin
        if (x_need) begin
          x_need <= 1'b0;
          if (t_mpend) begin t_go <= 1'b1; x_run <= 1'b1; end
        end else if (!e_wait && !sw_wait && t_pend) begin
          t_mark <= 1'b1; x_need <= 1'b1;
        end
      end
      if (x_run && !t_go && !t_busy) x_run <= 1'b0;

      // ---- the frame's end: geometry and upload done, a buffer free
      if (end_now) begin
        f_end <= 1'b1; f_buf <= free_i; e_wait <= 1'b0;
        for (int i = 0; i < 4; i++) if (free_i == 2'(i)) hide[i] <= blk;
      end
      // the queue of frames ended: push at inf_n, pop the head when one is done
      // (at most one is in flight when another ends: the store has two banks)
      if (r_done) begin
        pend_i <= inf_q[0]; pend_v <= 1'b1;
        inf_q[0] <= (end_now && inf_n == 2'd1) ? free_i : inf_q[1];
      end else if (end_now) begin
        if (inf_n == 2'd0) inf_q[0] <= free_i;
        else inf_q[1] <= free_i;
      end
      inf_n <= inf_n + {1'b0, end_now} - {1'b0, r_done};
      if (vbl && pend_v && !(r_done)) begin show_buf <= pend_i; show_v <= 1'b1; pend_v <= 1'b0; end

      // ---- TOT: the frame's drawing in 33.33 MHz ticks (every third clock)
      t3 <= (t3 == 2'd2) ? 2'd0 : t3 + 2'd1;
      if (t3 == 2'd2 && tot != 23'h7F_FFFF) tot <= tot + 23'd1;
      if (r_done) tot_l <= tot;
    end
  end
  assign show_3d = show_v && !hide[show_buf];

endmodule
