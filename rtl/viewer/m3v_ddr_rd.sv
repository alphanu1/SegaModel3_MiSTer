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
//
// Area (study R97): a request is at most eight words and the queue is empty
// when one is taken, so each word has its own slot by its address, [2:0]:
// even words in one 4 x 32 MLAB, odd words in another, one write port each.
// The range tests are on the word's offset from the request's first word, a
// few bits, not on 24-bit addresses. A request past 0xFFFFFF no longer wraps
// (the old compare gave fewer words than asked there).

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
  logic [4:0]  o;                   // signed: the next half's word less the first word
  logic [3:0]  n;                   // words asked for, 1-8
  logic [2:0]  wa;                  // the next even half's word, [2:0]
  logic [2:0]  rp;                  // the next word out, [2:0]
  logic        zero;                // this request reads zeros
  logic [3:0]  q_n;

  (* ramstyle = "MLAB, no_rw_check" *) logic [31:0] qe [4];   // even words
  (* ramstyle = "MLAB, no_rw_check" *) logic [31:0] qo [4];   // odd words

  wire [4:0] o1  = o + 5'd1;
  wire in0 = !o[4]  && (o[3:0]  < n);
  wire in1 = !o1[4] && (o1[3:0] < n);
  wire push_a = ((st == A_WAIT) && m_rvalid && in0) || (st == A_ZERO);
  wire push_b = (st == A_WAIT) && m_rvalid && in1;
  wire [31:0] rd_q = rp[0] ? qo[rp[2:1]] : qe[rp[2:1]];

  always_ff @(posedge clk) begin
    if (push_a && st == A_WAIT) qe[wa[2:1]] <= m_dout[31:0];
    if (push_b) qo[wa[2:1]] <= m_dout[63:32];
  end

  always_ff @(posedge clk) begin
    if (rst) begin
      st <= A_IDLE; g_ack <= 1'b0; m_req <= 1'b0; g_rvalid <= 1'b0;
      q_n <= '0; n_req <= '0;
    end else begin
      g_ack <= 1'b0;
      g_rvalid <= 1'b0;
      if (q_n != 4'd0) begin g_rvalid <= 1'b1; g_rdata <= zero ? 32'd0 : rd_q; rp <= rp + 3'd1; end
      q_n <= q_n + {3'd0, push_a} + {3'd0, push_b} - {3'd0, q_n != 4'd0};

      case (st)
        A_IDLE: if (g_req && !g_ack) begin
          logic [23:0] e;
          e = g_addr + {21'd0, g_len};
          g_ack <= 1'b1;
          n <= {1'b0, g_len} + 4'd1;
          o <= {5{g_addr[0]}};            // -1 when the first word is odd
          wa <= {g_addr[2:1], 1'b0};
          rp <= g_addr[2:0];
          n_req <= n_req + 16'd1;
          if (g_addr >= 24'h10_0000) begin o <= 5'd0; zero <= 1'b1; st <= A_ZERO; end
          else begin
            zero <= 1'b0;
            m_req  <= 1'b1;
            m_addr <= base + 25'(g_addr[23:1]);
            m_blen <= 8'(e[23:1] - g_addr[23:1]) + 8'd1;
            st <= A_WAIT;
          end
        end
        A_WAIT: begin
          // frozen once past the end, so extra beats can never wrap it
          if (m_rvalid && (o[4] || o[3:0] < n)) begin o <= o + 5'd2; wa <= wa + 3'd2; end
          if (m_ack) begin m_req <= 1'b0; st <= A_DRAIN; end
        end
        A_ZERO: begin
          o <= o1;
          if (!(o1[3:0] < n)) st <= A_DRAIN;
        end
        A_DRAIN: if (q_n == 4'd0 && !push_a && !push_b) st <= A_IDLE;
        default: st <= A_IDLE;
      endcase
    end
  end

endmodule
