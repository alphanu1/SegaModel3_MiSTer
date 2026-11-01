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
// Real3D memory windows (study D-6, D-37 stage B): the CPU's and the 53C810's
// writes to culling RAM (8C, 8E) and polygon RAM (98), into the memories the
// geometry reads.
//
//   8C  low culling RAM, one copy in SDRAM (word form: the Real3D's 32-bit
//       value, little-endian, at byte SDR_8C + 4a)
//   8E  high culling RAM, two copies in SDRAM (SDR_8E0, SDR_8E1)
//   98  polygon RAM, two copies in DDR3 (64-bit words DDR_P0, DDR_P1; word a
//       in lane a[0] of beat a / 2)
//
// Why two copies (sim/system/notes.md, Stage B). The geometry of frame N must
// read the memories as they were at frame N's ping-pong flip (Supermodel: the
// writes after the flip wait in its update buffers until the next vertical
// blank, and it draws at the end of the frame). VF3 rewrites the same 19,348
// polygon RAM words in every vertical blank and its 8E words before every
// flip, while the geometry needs up to 77% of a frame (R79), so one copy
// cannot be read for that long. A true ping-pong (writes to one copy, swapped
// at the flip) is wrong: polygon RAM's models are written once.
//
// So both copies are written. Bank R is the geometry's; W is the other. A
// write always goes to W at once; it goes to R at once too unless R is busy
// (r_busy: the geometry is reading it) or earlier writes to R are still
// deferred. A deferred write leaves only its address: it extends the open run
// (consecutive words of one window) or starts a new one, and closed runs wait
// in a FIFO. Once R is free, each run is copied from W to R; a copy reads W,
// which always holds the newest data, so the order of the copies does not
// matter, and a write that lands during a copy defers again. Anything outside
// the pending runs is equal in W and R, so a copy may round out to whole
// beats. VF3 makes at most 37 polygon RAM runs and 6 8E runs a frame (trace,
// frames 1-1500). swap exchanges the banks, only when nothing is deferred,
// so the new W equals the new R.
//
// Writes go first; copies run between them, a burst at a time. With the run
// FIFO full a waiting write cannot be taken, so the copies go first then.
//
// Ports: SDRAM m2_sdram's p_ contract (16-bit writes; 64-bit reads, p_long
// 0); DDR3 m2_ddr3's contract with byte enables (be live with din).

`timescale 1ns/1ps

module r3d_mem #(
  parameter int          AW      = 26,
  parameter logic [26:0] SDR_8C  = 27'h6D0_0000,
  parameter logic [26:0] SDR_8E0 = 27'h710_0000,
  parameter logic [26:0] SDR_8E1 = 27'h720_0000,
  parameter logic [24:0] DDR_P0  = 25'h10_0000,
  parameter logic [24:0] DDR_P1  = 25'h18_0000,
  parameter int          LD      = 8,              // log2 closed runs held (M10K)
  parameter int          CB      = 4               // log2 beats a polygon RAM copy burst
) (
  input  logic          clk,
  input  logic          rst,

  // writes: w_reg 0 8C, 1 8E, 2 98; w_addr the word in the window
  input  logic          w_valid,
  output logic          w_ready,
  input  logic [1:0]    w_reg,
  input  logic [19:0]   w_addr,
  input  logic [31:0]   w_data,

  input  logic          swap,
  input  logic          r_busy,
  output logic          bank,            // R
  output logic          can_swap,

  // SDRAM
  output logic          s_req,
  output logic          s_we,
  output logic [AW:1]   s_addr,
  output logic [15:0]   s_din,
  output logic [1:0]    s_be,
  input  logic          s_ack,
  input  logic [63:0]   s_dout,

  // DDR3
  output logic          d_req,
  output logic          d_we,
  output logic [24:0]   d_addr,
  output logic [7:0]    d_blen,
  output logic [63:0]   d_din,
  output logic [7:0]    d_be,
  input  logic          d_wacc,
  input  logic          d_rvalid,
  input  logic          d_ack,
  input  logic [63:0]   d_dout,

  output logic [15:0]   n_runs,          // runs deferred, all time
  output logic [15:0]   n_copy           // copy transfers, all time
);

  localparam int RL = 19;                // run length bits

  // ===========================================================================
  // the sequence
  // ===========================================================================
  typedef enum logic [3:0] {
    S_IDLE, S_SW, S_DW, S_RUN, S_POP, S_CP, S_SR, S_SW2, S_DR, S_DST
  } st_t;
  st_t          st;
  logic [1:0]   cw;                       // the write's window
  logic [20:0]  ca;                       // its word; the copy's position
  logic [31:0]  cd;                       // its data
  logic         to_r;                     // the second pass: bank R
  logic         dfr;                      // the write defers its R copy
  logic         sh;                       // SDRAM half (0 low, 1 high)
  logic [1:0]   sk;                       // copy: the half of the pair's four
  logic [63:0]  sq;                       // copy: the SDRAM pair read
  logic         cwin;                     // copy: 0 8E, 1 98
  logic [20:0]  cend;                     // copy: last word
  logic [CB:0]  cn, ci;                   // copy: beats this burst, beats moved
  logic         gap;                      // the SDRAM request edge

  // ===========================================================================
  // the run FIFO {window (0 8E, 1 98), start, length}, and the open run
  // ===========================================================================
  (* ramstyle = "M10K, no_rw_check" *) logic [RL+20:0] rf [1 << LD];
  logic [LD:0]    rf_wp, rf_rp;
  logic [RL+20:0] rf_q;
  wire            rf_empty = (rf_wp == rf_rp);
  wire            rf_full  = (rf_wp - rf_rp) == (LD+1)'(1 << LD);
  logic           o_v, o_win;
  logic [19:0]    o_st;
  logic [RL-1:0]  o_n;
  wire            o_ext  = o_v && o_win == cw[1] && ca[19:0] == o_st + 20'(o_n) && o_n != {RL{1'b1}};
  wire            rf_push = (st == S_RUN) && !o_ext && o_v;
  always_ff @(posedge clk) begin
    if (rf_push) rf[rf_wp[LD-1:0]] <= {o_win, o_st, o_n};
    rf_q <= rf[rf_rp[LD-1:0]];
  end

  // the copy buffer: a polygon RAM burst
  (* ramstyle = "M10K, no_rw_check" *) logic [63:0] cbuf [1 << CB];
  logic [63:0]  cb_q;
  wire  [CB-1:0] cb_ra = CB'(ci) + CB'(d_wacc);
  always_ff @(posedge clk) begin
    if (st == S_DR && d_rvalid) cbuf[CB'(ci)] <= d_dout;
    cb_q <= cbuf[cb_ra];
  end

  wire          wbank   = !bank;
  wire          bk      = to_r ? bank : wbank;
  wire [26:0]   sbase   = (cw == 2'd0) ? SDR_8C : bk ? SDR_8E1 : SDR_8E0;
  wire [24:0]   pbase   = bk ? DDR_P1 : DDR_P0;
  wire          pending = o_v || !rf_empty;
  wire          copy_go = !r_busy && pending;

  assign w_ready  = (st == S_IDLE) && !swap && !rf_full;
  assign can_swap = (st == S_IDLE) && !pending;
  assign d_din    = (st == S_DST) ? cb_q : {cd, cd};

  always_ff @(posedge clk) begin
    gap <= 1'b0;
    if (rst) begin
      st <= S_IDLE; bank <= 1'b0; s_req <= 1'b0; d_req <= 1'b0;
      rf_wp <= '0; rf_rp <= '0; o_v <= 1'b0; n_runs <= '0; n_copy <= '0;
    end else begin
      if (swap && st == S_IDLE) bank <= !bank;
      case (st)
        S_IDLE: begin
          to_r <= 1'b0; sh <= 1'b0;
          if (w_valid && w_ready) begin
            cw <= w_reg; ca <= {1'b0, w_addr}; cd <= w_data;
            dfr <= r_busy || pending;
            st <= (w_reg == 2'd2) ? S_DW : S_SW;
          end else if (copy_go && !swap && (!w_valid || rf_full)) begin   // a full FIFO must drain, or the bus waits for ever
            st <= S_POP;
          end
        end

        // ---- one 32-bit word to SDRAM, its two halves
        S_SW: begin
          if (!s_req && !gap && !s_ack) begin
            s_req <= 1'b1; s_we <= 1'b1;
            s_addr <= AW'(sbase[26:1]) + AW'({ca[19:0], sh});
            s_din <= sh ? cd[31:16] : cd[15:0]; s_be <= 2'b11;
          end
          if (s_ack) begin
            s_req <= 1'b0; gap <= 1'b1;
            sh <= !sh;
            if (sh) begin
              if (cw == 2'd0 || to_r) st <= S_IDLE;
              else if (dfr) st <= S_RUN;
              else to_r <= 1'b1;
            end
          end
        end

        // ---- one 32-bit word to DDR3: one beat, its lane
        S_DW: begin
          if (!d_req) begin
            d_req <= 1'b1; d_we <= 1'b1; d_blen <= 8'd1;
            d_addr <= pbase + 25'(ca[19:1]);
            d_be <= ca[0] ? 8'hF0 : 8'h0F;
          end
          if (d_ack) begin
            d_req <= 1'b0;
            if (to_r) st <= S_IDLE;
            else if (dfr) st <= S_RUN;
            else to_r <= 1'b1;
          end
        end

        // ---- a deferred write: extend the open run, or close it and open one
        S_RUN: begin
          if (o_ext) o_n <= o_n + 1'b1;
          else begin
            if (o_v) rf_wp <= rf_wp + 1'b1;
            o_v <= 1'b1; o_win <= cw[1]; o_st <= ca[19:0]; o_n <= RL'(1);
            n_runs <= n_runs + 16'd1;
          end
          st <= S_IDLE;
        end

        // ---- copies, W to R. Take a run: a closed one first (rf_q has been
        // valid since its last push or pop: two cycles at least), else the open one
        S_POP: begin
          logic          win;
          logic [19:0]   s0;
          logic [RL-1:0] n0;
          if (!rf_empty) begin
            {win, s0, n0} = rf_q; rf_rp <= rf_rp + 1'b1;
          end else begin
            win = o_win; s0 = o_st; n0 = o_n; o_v <= 1'b0;
          end
          cwin <= win;
          cend <= {1'b0, s0} + 21'(n0) - 21'd1;
          ca <= {1'b0, s0[19:1], 1'b0};
          st <= S_CP;
        end
        // the next piece of the run, or the run is done
        S_CP: begin
          if (ca > cend) st <= S_IDLE;
          else begin
            n_copy <= n_copy + 16'd1;
            if (cwin) begin
              // polygon RAM: up to 2^CB beats, not past the run's last beat
              logic [19:0] left;
              left = cend[20:1] - ca[20:1];
              cn <= (left >= 20'((1 << CB) - 1)) ? (CB+1)'(1 << CB) : (CB+1)'(left) + 1'b1;
              ci <= '0;
              st <= S_DR;
            end else begin
              sk <= 2'd0;
              st <= S_SR;
            end
          end
        end
        // 8E: read the pair {ca, ca + 1} from W, then write its four halves to R
        S_SR: begin
          if (!s_req && !gap && !s_ack) begin
            s_req <= 1'b1; s_we <= 1'b0;
            s_addr <= AW'((wbank ? SDR_8E1 : SDR_8E0) >> 1) + AW'({ca[19:0], 1'b0});
          end
          if (s_ack) begin
            s_req <= 1'b0; gap <= 1'b1;
            sq <= s_dout;
            st <= S_SW2;
          end
        end
        S_SW2: begin
          if (!s_req && !gap && !s_ack) begin
            s_req <= 1'b1; s_we <= 1'b1;
            s_addr <= AW'((bank ? SDR_8E1 : SDR_8E0) >> 1) + AW'({ca[19:1], sk});
            s_be <= 2'b11;
            case (sk)
              2'd0: s_din <= sq[15:0];
              2'd1: s_din <= sq[31:16];
              2'd2: s_din <= sq[47:32];
              default: s_din <= sq[63:48];
            endcase
          end
          if (s_ack) begin
            s_req <= 1'b0; gap <= 1'b1;
            sk <= sk + 2'd1;
            if (sk == 2'd3) begin ca <= ca + 21'd2; st <= S_CP; end
          end
        end
        // polygon RAM: read the burst from W, then write it to R
        S_DR: begin
          if (!d_req) begin
            d_req <= 1'b1; d_we <= 1'b0; d_blen <= 8'(cn);
            d_addr <= (wbank ? DDR_P1 : DDR_P0) + 25'(ca[19:1]);
          end
          if (d_rvalid) ci <= ci + 1'b1;
          if (d_ack) begin
            d_req <= 1'b0; ci <= '0;
            st <= S_DST;
          end
        end
        S_DST: begin
          if (!d_req) begin
            d_req <= 1'b1; d_we <= 1'b1; d_blen <= 8'(cn); d_be <= 8'hFF;
            d_addr <= (bank ? DDR_P1 : DDR_P0) + 25'(ca[19:1]);
          end
          if (d_wacc) ci <= ci + 1'b1;
          if (d_ack) begin
            d_req <= 1'b0;
            ca <= ca + {15'd0, cn, 1'b0};
            st <= S_CP;
          end
        end
        default: st <= S_IDLE;
      endcase
    end
  end

endmodule
