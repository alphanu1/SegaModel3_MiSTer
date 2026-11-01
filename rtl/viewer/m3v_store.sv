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
// M2.5 viewer, TEMPORARY: the polygon store in DDR3 (study D-20), without
// bins, until the rasteriser's store client lands. Every band replays every
// stored triangle, which is slow (tenths of a second a frame) and fine for a
// still frame.
//
// In: the geometry's word stream with SETUP_OUT (r3d_geo emitter header,
// study R76): a set-up record a triangle, header words 0..HW-1 (word 0 with
// n = 3; HW = 5 with the LOD word, LOD_WORD = 1), then x1 y1 x2 y2 x3 y3 (y
// sorted), det, and pdx, pdy, ps per parameter: HW + 19 words textured, HW +
// 13 not. A viewport record (n = 0, 6 words) is dropped here: the viewer has
// no fog yet.
//
// Write: each set-up record goes to one of eight lists, {pass, alpha buffer}
// = {w0[1:0], w0[2]}, in arrival order: list l, record i at LIST_BASE + l *
// 2^19 + 16 i (64-bit words, 128 bytes a record, up to 32 K records a list).
// The stored record is a word 0 of the store's own, then the set-up record:
//   word 0  {9'd0, nw[4:0], ymax[8:0], ymin[8:0]}: the rows the triangle can
//           cover lie in [ymin, ymax], floor(y1) and floor(y3) clamped to
//           0..511 (conservative; the walker is exact); nw the set-up
//           record's length in words (23 / 17, 24 / 18 with the LOD word)
//   1..nw   the set-up record as the geometry sent it
// (Until R76 m3v_setup built this record, word 0 included.)
//
// Read, per band: render_d25's draw order. For pass 0..3 with any triangle:
// wait for the rasteriser to drain, a new Z generation, the opaque list
// first to last, then the alpha list last to first (MAME's alpha buffer is
// drawn backwards). A record whose rows [ymin, ymax] miss the band is
// skipped; the rest go to the rasteriser as words 1..nw.

module m3v_store #(
  parameter logic [24:0] LIST_BASE = 25'h080_0000,
  parameter bit          LOD       = 1'b1          // the geometry's LOD_WORD: a 5-word header
) (
  input  logic        clk,
  input  logic        rst,

  input  logic        clear,        // a new frame: the lists empty
  // write side
  input  logic        w_valid,
  output logic        w_ready,
  input  logic [31:0] w_data,
  input  logic        w_last,
  output logic        w_busy,

  // read side: one band
  input  logic        band_go,
  input  logic [5:0]  band,
  output logic        band_busy,
  input  logic        rs_idle,      // the rasteriser has drained
  output logic        new_gen,
  output logic        r_valid,
  input  logic        r_ready,
  output logic [31:0] r_data,
  output logic        r_last,

  // DDR3 (m2_ddr3 contract)
  output logic        m_req,
  output logic        m_we,
  output logic [24:0] m_addr,
  output logic [7:0]  m_blen,
  output logic [63:0] m_din,
  input  logic        m_wacc,
  input  logic        m_rvalid,
  input  logic        m_ack,
  input  logic [63:0] m_dout,

  output logic [7:0][15:0] cnt,     // records a list
  output logic [15:0] n_tris,       // set-up records taken (the overlay's triangle count)
  output logic        overflow
);

  // the set-up record's y1 and y3 words (header words 0..HW-1, then x1 y1 x2 y2 x3 y3)
  localparam logic [4:0] HW = LOD ? 5'd5 : 5'd4;
  localparam logic [4:0] W_Y1 = HW + 5'd1, W_Y3 = HW + 5'd5;

  // floor(f) clamped to 0..511 (m3v_setup's, R64)
  function automatic logic [8:0] ffloor(input logic [31:0] f);
    logic [31:0] m;
    logic [4:0]  s;
    if (f[31] || f[30:23] < 8'd127) ffloor = 9'd0;
    else if (f[30:23] >= 8'd136) ffloor = 9'd511;
    else begin
      s = 5'(8'd150 - f[30:23]);           // 23 - (E - 127)
      m = {8'd0, 1'b1, f[22:0]} >> s;
      ffloor = m[8:0];
    end
  endfunction

  // the record buffer: 16 beats of two words
  logic [31:0] rb [32];
  logic [4:0]  rw;                  // words written / read
  logic [3:0]  beat;

  // ---- write ------------------------------------------------------------------
  typedef enum logic [1:0] { W_IN, W_REQ, W_ACK } wst_t;
  wst_t wst;
  logic [2:0]  wl;
  logic [4:0]  wn;                  // words in the stored record (store word 0 included)
  logic        wvp;                 // the record is a viewport record (n = 0): dropped
  logic [8:0]  ymin, ymax;
  logic        mw_req;
  logic [24:0] mw_addr;
  logic [7:0]  mw_blen;
  logic        rd_active;
  assign w_ready = (wst == W_IN) && !rd_active;
  assign w_busy  = (wst != W_IN);

  // ---- read -------------------------------------------------------------------
  typedef enum logic [3:0] { R_IDLE, R_PASS, R_DRAIN, R_GEN, R_NEXT, R_REQ, R_WAIT, R_SEND, R_DONE } rst_t;
  rst_t rs;
  logic [1:0]  pass;
  logic        alist;               // 0 the opaque list, 1 the alpha list
  logic [15:0] ri;                  // record in the list
  logic [15:0] rleft;
  logic [4:0]  rk, rn;
  logic        mr_req;
  logic [24:0] mr_addr;
  logic [3:0]  rbeat;

  assign rd_active = (rs != R_IDLE);
  assign band_busy = rd_active;

  wire [2:0] rl = {pass, alist};
  wire [8:0] b0 = {band, 3'd0}, b1 = {band, 3'b111};

  // DDR3 port: write or read, never both
  assign m_req  = mw_req || mr_req;
  assign m_we   = mw_req;
  assign m_addr = mw_req ? mw_addr : mr_addr;
  assign m_blen = mw_req ? mw_blen : 8'd13;
  assign m_din  = {rb[{beat, 1'b1}], rb[{beat, 1'b0}]};

  always_ff @(posedge clk) begin
    if (rst) begin
      wst <= W_IN; rw <= '0; mw_req <= 1'b0; overflow <= 1'b0; n_tris <= '0;
      for (int i = 0; i < 8; i++) cnt[i] <= '0;
      rs <= R_IDLE; mr_req <= 1'b0; new_gen <= 1'b0; r_valid <= 1'b0;
    end else begin
      new_gen <= 1'b0;
      if (clear) begin
        for (int i = 0; i < 8; i++) cnt[i] <= '0;
        overflow <= 1'b0; n_tris <= '0;
      end
      // ---- write ----
      case (wst)
        // record word rw into rb[rw + 1]; word 0 (rb[0]) is built at the end
        W_IN: if (w_valid && w_ready) begin
          rb[rw + 5'd1] <= w_data;
          rw <= rw + 5'd1;
          if (rw == 5'd0) begin wl <= {w_data[1:0], w_data[2]}; wvp <= w_data[31:27] == 5'd0; end
          if (rw == W_Y1) ymin <= ffloor(w_data);
          if (rw == W_Y3) ymax <= ffloor(w_data);
          if (w_last) begin
            rw <= '0;
            // a viewport record is not stored (no fog in the viewer yet)
            if (!(rw == 5'd0 ? w_data[31:27] == 5'd0 : wvp)) begin wn <= rw + 5'd2; wst <= W_REQ; end
          end
        end
        W_REQ: begin
          if (cnt[wl] == 16'hffff || cnt[wl][15]) begin overflow <= 1'b1; wst <= W_IN; end
          else begin
            rb[0]   <= {9'd0, 5'(wn - 5'd1), ymax, ymin};
            mw_req  <= 1'b1;
            mw_addr <= LIST_BASE + {3'd0, wl, 19'd0} + {5'd0, cnt[wl], 4'd0};
            mw_blen <= 8'((wn + 5'd1) >> 1);
            beat    <= 4'd0;
            wst     <= W_ACK;
          end
        end
        W_ACK: begin
          if (m_wacc) beat <= beat + 4'd1;
          if (m_ack) begin mw_req <= 1'b0; cnt[wl] <= cnt[wl] + 16'd1; n_tris <= n_tris + 16'd1; wst <= W_IN; end
        end
        default: wst <= W_IN;
      endcase

      // ---- read ----
      if (r_valid && r_ready) r_valid <= 1'b0;
      case (rs)
        R_IDLE: if (band_go && wst == W_IN) begin pass <= 2'd0; rs <= R_PASS; end
        R_PASS: begin
          if (cnt[{pass, 1'b0}] == 16'd0 && cnt[{pass, 1'b1}] == 16'd0) begin
            if (pass == 2'd3) rs <= R_DONE; else pass <= pass + 2'd1;
          end else rs <= R_DRAIN;
        end
        R_DRAIN: if (rs_idle) begin new_gen <= 1'b1; rs <= R_GEN; end
        R_GEN: begin
          alist <= 1'b0; ri <= 16'd0; rleft <= cnt[{pass, 1'b0}];
          rs <= R_NEXT;
        end
        R_NEXT: begin
          if (rleft == 16'd0) begin
            if (!alist) begin
              alist <= 1'b1; rleft <= cnt[{pass, 1'b1}]; ri <= cnt[{pass, 1'b1}] - 16'd1;
            end else if (pass == 2'd3) rs <= R_DONE;
            else begin pass <= pass + 2'd1; rs <= R_PASS; end
          end else begin
            mr_req  <= 1'b1;
            mr_addr <= LIST_BASE + {3'd0, rl, 19'd0} + {5'd0, ri, 4'd0};
            rbeat   <= 4'd0;
            rs      <= R_WAIT;
          end
        end
        R_WAIT: begin
          if (m_rvalid) begin
            rb[{rbeat, 1'b0}] <= m_dout[31:0];
            rb[{rbeat, 1'b1}] <= m_dout[63:32];
            rbeat <= rbeat + 4'd1;
          end
          if (m_ack) begin
            mr_req <= 1'b0;
            rleft <= rleft - 16'd1;
            ri <= alist ? ri - 16'd1 : ri + 16'd1;
            rk <= 5'd1;
            rs <= R_SEND;
          end
        end
        R_SEND: begin
          // word 0: {nw, ymax, ymin}; skip a record that misses the band
          rn <= rb[0][22:18];
          if (rk == 5'd1 && (rb[0][17:9] < b0 || rb[0][8:0] > b1)) rs <= R_NEXT;
          else if (!r_valid || r_ready) begin
            r_valid <= 1'b1;
            r_data  <= rb[rk];
            r_last  <= (rk == rb[0][22:18]);
            rk <= rk + 5'd1;
            if (rk == rb[0][22:18]) rs <= R_NEXT;
          end
        end
        R_DONE: if (!r_valid) rs <= R_IDLE;
        default: rs <= R_IDLE;
      endcase
    end
  end

endmodule
