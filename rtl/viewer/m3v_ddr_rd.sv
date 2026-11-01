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
// The geometry's polygon RAM port (r3d_geo: req held to ack, 1-8 32-bit
// words, then the words in order) on DDR3 (study D-6), through the m2_ddr3
// contract: one burst a request, the 64-bit words covering it, a 32-bit word
// in each half (the low half the even word, little-endian as the loader
// writes it). Words past the 4 MiB window read as zeros.

module m3v_ddr_rd (
  input  logic        clk,
  input  logic        rst,

  input  logic [24:0] base,          // 64-bit word address of polygon RAM word 0

  input  logic        g_req,
  input  logic [23:0] g_addr,
  input  logic [2:0]  g_len,
  output logic        g_ack,
  output logic        g_rvalid,
  output logic [31:0] g_rdata,

  output logic        m_req,
  output logic [24:0] m_addr,
  output logic [7:0]  m_blen,
  input  logic        m_rvalid,
  input  logic        m_ack,
  input  logic [63:0] m_dout,

  output logic [15:0] n_req
);

  typedef enum logic [1:0] { A_IDLE, A_WAIT, A_ZERO, A_DRAIN } st_t;
  st_t st;
  logic [23:0] a0, a1, w;           // request [a0, a1); w the word of the next half
  logic [31:0] q [16];
  logic [3:0]  q_wp, q_rp;
  logic [4:0]  q_n;

  wire in0 = (w >= a0) && (w < a1);
  wire in1 = (w + 24'd1 >= a0) && (w + 24'd1 < a1);
  logic push_a, push_b;
  always_comb begin
    push_a = (st == A_WAIT) && m_rvalid && in0;
    push_b = (st == A_WAIT) && m_rvalid && in1;
    if (st == A_ZERO) begin push_a = 1'b1; push_b = 1'b0; end
  end
  wire [31:0] pd_a = (st == A_ZERO) ? 32'd0 : m_dout[31:0];
  wire [31:0] pd_b = m_dout[63:32];

  always_ff @(posedge clk) begin
    if (rst) begin
      st <= A_IDLE; g_ack <= 1'b0; m_req <= 1'b0; g_rvalid <= 1'b0;
      q_wp <= '0; q_rp <= '0; q_n <= '0; n_req <= '0;
    end else begin
      g_ack <= 1'b0;
      g_rvalid <= 1'b0;
      if (q_n != 5'd0) begin g_rvalid <= 1'b1; g_rdata <= q[q_rp]; q_rp <= q_rp + 4'd1; end
      if (push_a && push_b) begin
        q[q_wp] <= pd_a; q[q_wp + 4'd1] <= pd_b; q_wp <= q_wp + 4'd2;
      end else if (push_a) begin
        q[q_wp] <= pd_a; q_wp <= q_wp + 4'd1;
      end else if (push_b) begin
        q[q_wp] <= pd_b; q_wp <= q_wp + 4'd1;
      end
      q_n <= q_n + {4'd0, push_a} + {4'd0, push_b} - {4'd0, q_n != 5'd0};

      case (st)
        A_IDLE: if (g_req && !g_ack) begin
          logic [23:0] e;
          e = g_addr + {21'd0, g_len};
          g_ack <= 1'b1;
          a0 <= g_addr; a1 <= e + 24'd1;
          w  <= {g_addr[23:1], 1'b0};
          n_req <= n_req + 16'd1;
          if (g_addr >= 24'h10_0000) begin w <= g_addr; st <= A_ZERO; end
          else begin
            m_req  <= 1'b1;
            m_addr <= base + 25'(g_addr[23:1]);
            m_blen <= 8'(e[23:1] - g_addr[23:1]) + 8'd1;
            st <= A_WAIT;
          end
        end
        A_WAIT: begin
          if (m_rvalid) w <= w + 24'd2;
          if (m_ack) begin m_req <= 1'b0; st <= A_DRAIN; end
        end
        A_ZERO: begin
          w <= w + 24'd1;
          if (w + 24'd1 >= a1) st <= A_DRAIN;
        end
        A_DRAIN: if (q_n == 5'd0 && !push_a && !push_b) st <= A_IDLE;
        default: st <= A_IDLE;
      endcase
    end
  end

endmodule
