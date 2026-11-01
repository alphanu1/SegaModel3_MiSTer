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
  logic [23:0] a0, a1;                // the request's words [a0, a1)
  logic [23:0] blk;
  logic [2:0]  zk;                    // zero block: word being sent

  // the word queue
  logic [31:0] q [8];
  logic [2:0]  q_wp, q_rp;
  logic [3:0]  q_n;

  assign blk_addr = blk;
  wire in0 = (blk >= a0) && (blk < a1);
  wire in1 = (blk + 24'd1 >= a0) && (blk + 24'd1 < a1);
  wire in2 = (blk + 24'd2 >= a0) && (blk + 24'd2 < a1);
  wire in3 = (blk + 24'd3 >= a0) && (blk + 24'd3 < a1);
  wire [23:0] blk_n = blk + 24'd4;

  logic       push_a, push_b;
  logic [31:0] pd_a, pd_b;

  always_comb begin
    push_a = 1'b0; push_b = 1'b0; pd_a = p_dout[31:0]; pd_b = p_dout[63:32];
    if (st == A_WAIT && p_lo)  begin push_a = in0; push_b = in1; end
    if (st == A_WAIT && p_ack) begin push_a = in2; push_b = in3; end
    if (st == A_BLK && blk_zero) begin
      push_a = (zk == 3'd0 && in0) || (zk == 3'd1 && in1) || (zk == 3'd2 && in2) || (zk == 3'd3 && in3);
      pd_a = 32'd0;
    end
  end

  always_ff @(posedge clk) begin
    if (rst) begin
      st <= A_IDLE; g_ack <= 1'b0; p_req <= 1'b0; g_rvalid <= 1'b0;
      q_wp <= '0; q_rp <= '0; q_n <= '0; n_req <= '0;
    end else begin
      g_ack <= 1'b0;
      // the queue
      g_rvalid <= 1'b0;
      if (q_n != 4'd0) begin g_rvalid <= 1'b1; g_rdata <= q[q_rp]; q_rp <= q_rp + 3'd1; end
      if (push_a && push_b) begin
        q[q_wp] <= pd_a; q[q_wp + 3'd1] <= pd_b; q_wp <= q_wp + 3'd2;
      end else if (push_a) begin
        q[q_wp] <= pd_a; q_wp <= q_wp + 3'd1;
      end else if (push_b) begin
        q[q_wp] <= pd_b; q_wp <= q_wp + 3'd1;
      end
      q_n <= q_n + {3'd0, push_a} + {3'd0, push_b} - {3'd0, q_n != 4'd0};

      case (st)
        A_IDLE: if (g_req && !g_ack) begin
          g_ack <= 1'b1;
          a0 <= g_addr; a1 <= g_addr + {21'd0, g_len} + 24'd1;
          blk <= {g_addr[23:2], 2'b00};
          zk <= 3'd0;
          n_req <= n_req + 16'd1;
          st <= A_BLK;
        end
        A_BLK: if (blk_zero) begin
          zk <= zk + 3'd1;
          if (zk == 3'd3) begin
            zk <= 3'd0; blk <= blk_n;
            if (blk_n >= a1) st <= A_DRAIN;
          end
        end else begin
          p_req <= 1'b1; p_addr <= blk_sdr;
          st <= A_WAIT;
        end
        A_WAIT: if (p_ack) begin
          p_req <= 1'b0;
          blk <= blk_n;
          st <= (blk_n >= a1) ? A_DRAIN : A_BLK;
        end
        // the last words leave the queue before the next request is taken
        A_DRAIN: if (q_n == 4'd0 && !push_a && !push_b) st <= A_IDLE;
        default: st <= A_IDLE;
      endcase
    end
  end

endmodule
