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
// A Real3D geometry read port (r3d_geo: req held to ack, 1-8 32-bit words,
// then the words in order, one a beat) on an m2_sdram port with eight-word
// reads (p_long). The SDRAM is read in aligned four-word blocks (eight 16-bit
// words, never across a row), the first half on p_lo, the second on p_ack;
// a 32-bit word is {SDRAM word 2k + 1, SDRAM word 2k}, little-endian as the
// loader writes it. One request at a time: the next is acknowledged when
// this one's words are all out.
//
// The parent maps a block: blk_sdr is the SDRAM word address of geometry word
// blk_addr (a multiple of 4), blk_zero says the block is outside every region
// and reads as zeros (the model's rule for a bad address).
//
// Area (study R97): as m3v_ddr_rd, each word of a request (at most eight, the
// queue empty when one is taken) has its own slot by its address, [2:0]:
// even words in one 4 x 32 MLAB, odd words in another, and a zero flag per
// slot for words from a zero block. The range tests are on the block's
// offset from the request's first word, not on 24-bit addresses. A request
// past 0xFFFFFF no longer wraps.

module m3v_sdr_rd #(
  parameter int AW = 26
) (
  input  logic          clk,
  input  logic          rst,

  input  logic          g_req,
  input  logic [23:0]   g_addr,
  input  logic [2:0]    g_len,
  output logic          g_ack,
  output logic          g_rvalid,
  output logic [31:0]   g_rdata,

  output logic [23:0]   blk_addr,
  input  logic [AW:1]   blk_sdr,
  input  logic          blk_zero,

  output logic          p_req,
  output logic [AW:1]   p_addr,
  input  logic          p_lo,
  input  logic          p_ack,
  input  logic [63:0]   p_dout,

  output logic [15:0]   n_req          // requests served
);

  typedef enum logic [1:0] { A_IDLE, A_BLK, A_WAIT, A_DRAIN } st_t;
  st_t st;
  logic [21:0] blk;                   // the block, word address [23:2]
  logic [4:0]  o;                     // signed: the block's first word less the request's
  logic [3:0]  n;                     // words asked for, 1-8
  logic [1:0]  zk;                    // zero block: word being sent
  logic [2:0]  rp;                    // the next word out, [2:0]
  logic [3:0]  q_n;
  logic [7:0]  zf;                    // the slot holds a zero block's word

  (* ramstyle = "MLAB, no_rw_check" *) logic [31:0] qe [4];   // even words
  (* ramstyle = "MLAB, no_rw_check" *) logic [31:0] qo [4];   // odd words

  assign blk_addr = {blk, 2'b00};
  // word j of the block is in the request
  function automatic logic inr(input logic [4:0] oo, input logic [1:0] j, input logic [3:0] nn);
    logic [4:0] x;
    x = oo + {3'd0, j};
    inr = !x[4] && (x[3:0] < nn);
  endfunction
  wire in0 = inr(o, 2'd0, n), in1 = inr(o, 2'd1, n), in2 = inr(o, 2'd2, n), in3 = inr(o, 2'd3, n);
  wire [4:0] o4 = o + 5'd4;
  wire last = !o4[4] && !(o4[3:0] < n);    // the next block is past the request

  wire hi = p_ack;                         // the second half (p_ack wins if both)
  logic push_a, push_b, pz;
  always_comb begin
    push_a = 1'b0; push_b = 1'b0; pz = 1'b0;
    if (st == A_WAIT && p_lo)  begin push_a = in0; push_b = in1; end
    if (st == A_WAIT && p_ack) begin push_a = in2; push_b = in3; end
    if (st == A_BLK && blk_zero) begin
      pz = (zk == 2'd0 && in0) || (zk == 2'd1 && in1) || (zk == 2'd2 && in2) || (zk == 2'd3 && in3);
      push_a = pz;
    end
  end
  wire [31:0] rd_q = rp[0] ? qo[rp[2:1]] : qe[rp[2:1]];

  always_ff @(posedge clk) begin
    if (st == A_WAIT && push_a) qe[{blk[0], hi}] <= p_dout[31:0];
    if (st == A_WAIT && push_b) qo[{blk[0], hi}] <= p_dout[63:32];
  end

  always_ff @(posedge clk) begin
    if (rst) begin
      st <= A_IDLE; g_ack <= 1'b0; p_req <= 1'b0; g_rvalid <= 1'b0;
      q_n <= '0; n_req <= '0;
    end else begin
      g_ack <= 1'b0;
      // the queue
      g_rvalid <= 1'b0;
      if (q_n != 4'd0) begin g_rvalid <= 1'b1; g_rdata <= zf[rp] ? 32'd0 : rd_q; rp <= rp + 3'd1; end
      if (pz) zf[{blk[0], zk}] <= 1'b1;
      if (st == A_WAIT && push_a) zf[{blk[0], hi, 1'b0}] <= 1'b0;
      if (st == A_WAIT && push_b) zf[{blk[0], hi, 1'b1}] <= 1'b0;
      q_n <= q_n + {3'd0, push_a} + {3'd0, push_b} - {3'd0, q_n != 4'd0};

      case (st)
        A_IDLE: if (g_req && !g_ack) begin
          g_ack <= 1'b1;
          n <= {1'b0, g_len} + 4'd1;
          o <= 5'd0 - {3'd0, g_addr[1:0]};
          blk <= g_addr[23:2];
          rp <= g_addr[2:0];
          zk <= 2'd0;
          n_req <= n_req + 16'd1;
          st <= A_BLK;
        end
        A_BLK: if (blk_zero) begin
          zk <= zk + 2'd1;
          if (zk == 2'd3) begin
            blk <= blk + 22'd1; o <= o4;
            if (last) st <= A_DRAIN;
          end
        end else begin
          p_req <= 1'b1; p_addr <= blk_sdr;
          st <= A_WAIT;
        end
        A_WAIT: if (p_ack) begin
          p_req <= 1'b0;
          blk <= blk + 22'd1; o <= o4;
          st <= last ? A_DRAIN : A_BLK;
        end
        // the last words leave the queue before the next request is taken
        A_DRAIN: if (q_n == 4'd0 && !push_a && !push_b) st <= A_IDLE;
        default: st <= A_IDLE;
      endcase
    end
  end

endmodule
