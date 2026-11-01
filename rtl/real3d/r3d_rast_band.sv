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
// Real3D rasteriser: the colour bands, the blend and the band write (block 7,
// D-28, D-29). Shaded pixels come one a clock in band order, each with its
// colour after fog and, when it blends, MAME's factor f. Two band buffers of
// 8 rows x 512 {drawn, RGB888}, each in two banks by x parity: one is drawn
// while the other is written out to the 3D layer and cleared for the band two
// ahead.
//
//   P0  the pixel; its word read (the bank of x[0], address {row, x[8:1]})
//   P1  the colour under it, d; u = c - d + 256 per channel
//   P2  u f: red and green in one product, {u_r, 9'd0, u_g} x f, blue alone
//   P3  d + (u f >> 8) - f = (c f + d (256 - f)) >> 8 exactly, or c
//   W   the word written
// A blending pixel waits in P0 while P1, P2, P3 or W holds a write to its
// word, so no result is forwarded. A plain write never waits: writes stay in
// order.
//
// Scroll fog (render_d25's rule): a band is cleared to the fog colour inside
// the rectangle and to 0 outside, which is what a blend reads; a pixel still
// undrawn at the flush shows the fog colour at the scroll-fog alpha, given as
// sf_out (a constant of the frame, so the flush has no products).
//
// The flush: a row is one burst of 248 beats into the layer, two pixels a
// beat in m2_fb_read's form ({7'd0, drawn, RGB} a pixel), from an 8-entry
// queue read ahead of the port (the M2.5 viewer's m3v_raster flush, which is
// bit-exact on the board). The memory port is m2_ddr3's request contract,
// which m2_sdram's ports share (D-20): moving the layer to the second SDRAM is
// that controller's port, not a change here.
//
// eob ends a band: it waits for the pixels in flight and for the last flush,
// then starts this band's flush and switches buffers.

/* verilator lint_off UNUSEDSIGNAL */
module r3d_rast_band #(
  parameter logic [24:0] FB_STRIDE = 25'd256      // 64-bit words a row
) (
  input  logic        clk,
  input  logic        rst,

  // the frame: f_start clears both buffers for bands 0 and 1
  input  logic        f_start,
  input  logic [24:0] fb_base,      // 64-bit word address of the layer being drawn
  input  logic [8:0]  sf_x0, sf_x1, // scroll fog rectangle [x0, x1) x [y0, y1); empty when off
  input  logic [8:0]  sf_y0, sf_y1,
  input  logic [23:0] sf_col,       // its colour, what a blend reads
  input  logic [23:0] sf_out,       // what an undrawn pixel shows
  output logic        busy,         // a clear or a flush runs, or pixels are in flight

  // pixels, one a clock; eob (no pixel) ends band p_band
  input  logic        p_valid,
  output logic        p_ready,
  input  logic        p_eob,
  input  logic [5:0]  p_band,
  input  logic [8:0]  p_x,
  input  logic [2:0]  p_row,
  input  logic [23:0] p_rgb,
  input  logic        p_bl,
  input  logic [7:0]  p_f,

  // the layer: write bursts (m2_ddr3's contract)
  output logic        m_req,
  output logic [24:0] m_addr,
  output logic [7:0]  m_blen,
  output logic [63:0] m_din,
  input  logic        m_wacc,
  input  logic        m_ack
);

  // ---- the buffers: [buffer][bank], 2048 x 25 each (5 M10K) ---------------------------------
  logic        bwe [2][2];
  logic [10:0] bwa [2][2], bra [2];
  logic [24:0] bwd [2][2], brq [2][2];
  genvar gi, gk;
  generate
    for (gi = 0; gi < 2; gi++) begin : g_buf
      for (gk = 0; gk < 2; gk++) begin : g_bank
        r3d_ram #(.W(25), .A(11), .SYNC(1'b1), .BLOCK(1'b1)) u_m (
          .clk, .we(bwe[gi][gk]), .wa(bwa[gi][gk]), .wd(bwd[gi][gk]), .ra(bra[gi]), .rq(brq[gi][gk]));
      end
    end
  endgenerate

  // ---- shading -----------------------------------------------------------------------------
  logic        cur;                    // the buffer being drawn
  logic        p0_v, p0_eob, p0_bl;
  logic [5:0]  p0_band;
  logic [8:0]  p0_x;
  logic [2:0]  p0_row;
  logic [23:0] p0_c;
  logic [7:0]  p0_f;
  logic        p1_v, p1_bl, p1_k;  logic [10:0] p1_a; logic [23:0] p1_c; logic [7:0] p1_f;
  logic        p2_v, p2_bl, p2_k;  logic [10:0] p2_a; logic [23:0] p2_c, p2_d; logic [7:0] p2_f; logic [8:0] p2_ur, p2_ug, p2_ub;
  logic        p3_v, p3_bl, p3_k;  logic [10:0] p3_a; logic [23:0] p3_c, p3_d; logic [7:0] p3_f;
  logic [34:0] p3_rg;  logic [16:0] p3_b;
  logic        w_v, w_k;           logic [10:0] w_a;  logic [23:0] w_d;

  wire [10:0] p0_a = {p0_row, p0_x[8:1]};
  // a blend waits for every write in flight to its word
  wire haz = p0_bl && ((p1_v && p1_a == p0_a && p1_k == p0_x[0]) || (p2_v && p2_a == p0_a && p2_k == p0_x[0]) ||
                       (p3_v && p3_a == p0_a && p3_k == p0_x[0]) || (w_v && w_a == p0_a && w_k == p0_x[0]));
  wire inflight = p1_v || p2_v || p3_v || w_v;
  wire haz_v /* verilator public_flat_rd */ = haz && p0_v;   // for the bench's count

  typedef enum logic [2:0] { C_IDLE, C_CLR, C_ROW, C_PRE, C_BURST, C_WAIT } cst_t;
  cst_t cst;
  wire eob_go = p0_v && p0_eob && !inflight && cst == C_IDLE;
  logic cl_own;   // the frame clear's last write still landing
  wire px_go  = p0_v && !p0_eob && !haz && cst != C_CLR && !cl_own;
  wire p0_go  = px_go || eob_go;
  assign p_ready = !p0_v || p0_go;

  // the flush engine's state
  logic        fbuf;                   // the buffer being written out
  logic [5:0]  fband;
  logic [2:0]  crow, cwait;
  logic [10:0] cclr;
  logic [7:0]  rd_k;
  logic        rd_v, cm_rd;
  logic [10:0] rd_a1, cm_ra;
  (* ramstyle = "MLAB, no_rw_check" *) logic [49:0] fq [8];   // never written over an entry still to go
  logic [2:0]  fq_wp, fq_rp;
  logic [3:0]  ahead;
  logic        cm_cw;
  logic [10:0] cm_wa;
  logic [24:0] cm_wd [2][2];           // clear words: [buffer][bank]

  // the clear word of pixel (x, y): the fog colour inside the rectangle
  function automatic logic [24:0] clr(input logic [8:0] x, input logic [9:0] y);
    clr = (x >= sf_x0 && x < sf_x1 && y >= {1'b0, sf_y0} && y < {1'b0, sf_y1}) ? {1'b0, sf_col} : 25'd0;
  endfunction

  // shading pipe
  logic [23:0] d1;
  always_comb d1 = p1_k ? brq[cur][1][23:0] : brq[cur][0][23:0];

  always_ff @(posedge clk) begin
    if (rst) begin
      p0_v <= 1'b0; p1_v <= 1'b0; p2_v <= 1'b0; p3_v <= 1'b0; w_v <= 1'b0;
    end else begin
      if (p_valid && p_ready) begin
        p0_v <= 1'b1; p0_eob <= p_eob; p0_band <= p_band; p0_x <= p_x; p0_row <= p_row; p0_c <= p_rgb;
        p0_bl <= p_bl && !p_eob; p0_f <= p_f;
      end else if (p0_go) p0_v <= 1'b0;
      // P1
      p1_v <= px_go; p1_bl <= p0_bl; p1_k <= p0_x[0]; p1_a <= p0_a; p1_c <= p0_c; p1_f <= p0_f;
      // P2
      p2_v <= p1_v; p2_bl <= p1_bl; p2_k <= p1_k; p2_a <= p1_a; p2_c <= p1_c; p2_d <= d1; p2_f <= p1_f;
      p2_ur <= 9'({1'b0, p1_c[23:16]} - {1'b0, d1[23:16]} + 9'd256);
      p2_ug <= 9'({1'b0, p1_c[15:8]}  - {1'b0, d1[15:8]}  + 9'd256);
      p2_ub <= 9'({1'b0, p1_c[7:0]}   - {1'b0, d1[7:0]}   + 9'd256);
      // P3
      p3_v <= p2_v; p3_bl <= p2_bl; p3_k <= p2_k; p3_a <= p2_a; p3_c <= p2_c; p3_d <= p2_d; p3_f <= p2_f;
      p3_rg <= 35'({p2_ur, 9'd0, p2_ug}) * 35'(p2_f);
      p3_b  <= 17'(p2_ub) * 17'(p2_f);
      // W
      w_v <= p3_v; w_k <= p3_k; w_a <= p3_a;
      if (p3_bl) begin
        w_d[23:16] <= 8'({1'b0, p3_d[23:16]} + 9'(p3_rg[34:26]) - {1'b0, p3_f});
        w_d[15:8]  <= 8'({1'b0, p3_d[15:8]}  + 9'(p3_rg[16:8])  - {1'b0, p3_f});
        w_d[7:0]   <= 8'({1'b0, p3_d[7:0]}   + 9'(p3_b[16:8])   - {1'b0, p3_f});
      end else w_d <= p3_c;
    end
  end

  // ---- the flush: clear, or write out a band then clear its buffer for the band two ahead ---
  assign m_blen = 8'd248;
  wire [8:0] fy = {fband, crow};
  wire [9:0] cy = {1'b0, fband, rd_a1[10:8]} + 10'd16;   // the row of the band two ahead
  wire rd_go = (cst == C_PRE || cst == C_BURST) && rd_k != 8'd248 && ahead < 4'd8;
  // a word as the layer holds it: undrawn and not black is the scroll fog at its alpha
  function automatic logic [31:0] shown(input logic [24:0] w);
    if (w[24]) shown = {7'd0, 1'b1, w[23:0]};
    else if (w[23:0] != 24'd0) shown = {7'd0, 1'b1, sf_out};
    else shown = 32'd0;
  endfunction
  assign m_din = {7'd0, fq[fq_rp][49:25], 7'd0, fq[fq_rp][24:0]};
  logic [31:0] sh0, sh1;
  always_comb begin sh0 = shown(brq[fbuf][0]); sh1 = shown(brq[fbuf][1]); end

  always_ff @(posedge clk) begin
    if (rst) begin
      cst <= C_IDLE; m_req <= 1'b0; cm_cw <= 1'b0; cm_rd <= 1'b0; rd_v <= 1'b0;
      fq_wp <= '0; fq_rp <= '0; ahead <= '0; cur <= 1'b0; fbuf <= 1'b0;
    end else begin
      cm_cw <= 1'b0; cm_rd <= 1'b0;
      rd_v <= cm_rd;
      case (cst)
        C_IDLE: begin
          if (f_start) begin cclr <= '0; cur <= 1'b0; cst <= C_CLR; end
          else if (eob_go) begin fbuf <= cur; fband <= p0_band; cur <= ~cur; crow <= 3'd0; cst <= C_ROW; end
        end
        C_CLR: begin                          // both buffers: band 0 into 0, band 1 into 1
          cm_cw <= 1'b1; cm_wa <= cclr;
          cm_wd[0][0] <= clr({cclr[7:0], 1'b0}, {7'd0, cclr[10:8]});
          cm_wd[0][1] <= clr({cclr[7:0], 1'b1}, {7'd0, cclr[10:8]});
          cm_wd[1][0] <= clr({cclr[7:0], 1'b0}, {7'd1, cclr[10:8]});
          cm_wd[1][1] <= clr({cclr[7:0], 1'b1}, {7'd1, cclr[10:8]});
          cclr <= cclr + 11'd1;
          if (cclr == 11'd2047) begin cwait <= 3'd4; cst <= C_WAIT; end
        end
        C_ROW: begin
          rd_k <= 8'd0; fq_wp <= '0; fq_rp <= '0; ahead <= '0;
          m_addr <= fb_base + {16'd0, fy} * FB_STRIDE;
          cst <= C_PRE;
        end
        // fill the queue before the burst: the port takes a beat a cycle
        C_PRE: if (ahead == 4'd8 && fq_wp == 3'd0 && rd_v == 1'b0 && !cm_rd && rd_k == 8'd8) begin
          m_req <= 1'b1; cst <= C_BURST;
        end
        C_BURST: if (m_ack) begin
          m_req <= 1'b0;
          if (crow == 3'd7) begin cwait <= 3'd4; cst <= C_WAIT; end
          else begin crow <= crow + 3'd1; cst <= C_ROW; end
        end
        C_WAIT: begin                         // the last clears land
          cwait <= cwait - 3'd1;
          if (cwait == 3'd0) cst <= C_IDLE;
        end
        default: cst <= C_IDLE;
      endcase
      if (rd_go) begin cm_rd <= 1'b1; cm_ra <= {crow, rd_k}; rd_k <= rd_k + 8'd1; end
      if (cm_rd) rd_a1 <= cm_ra;
      // a word read back: into the queue as the layer holds it, and cleared for the band two ahead
      if (rd_v) begin
        fq[fq_wp] <= {sh1[24:0], sh0[24:0]};
        fq_wp <= fq_wp + 3'd1;
        cm_cw <= 1'b1; cm_wa <= rd_a1;
        cm_wd[fbuf][0] <= clr({rd_a1[7:0], 1'b0}, cy);
        cm_wd[fbuf][1] <= clr({rd_a1[7:0], 1'b1}, cy);
      end
      if (m_wacc) fq_rp <= fq_rp + 3'd1;
      if (cst != C_ROW) ahead <= ahead + {3'd0, rd_go} - {3'd0, m_wacc};
    end
  end

  // ---- the ports: the drawn buffer is shading's; the other, and both in a clear, the flush's ---
  always_ff @(posedge clk) if (rst) cl_own <= 1'b0; else cl_own <= (cst == C_CLR);
  always_comb begin
    for (int i = 0; i < 2; i++) begin
      logic fl;
      fl = (cst == C_CLR) || cl_own || (1'(i) == fbuf && 1'(i) != cur);
      bra[i] = fl ? cm_ra : p0_a;
      for (int k = 0; k < 2; k++) begin
        if (fl) begin bwe[i][k] = cm_cw; bwa[i][k] = cm_wa; bwd[i][k] = cm_wd[i][k]; end
        else begin bwe[i][k] = w_v && w_k == 1'(k) && 1'(i) == cur; bwa[i][k] = w_a; bwd[i][k] = {1'b1, w_d}; end
      end
    end
  end

  assign busy = (cst != C_IDLE) || p0_v || inflight || cm_cw || rd_v;

endmodule
