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
// The geometry's VROM read port (r3d_geo: req held to ack, 1-8 32-bit words,
// then the words in order) on an m2_sdram port, for stage A's MRA layout
// (sim/system/notes.md): VROM word i = {g[19:0], b[1:0], h} is the
// little-endian word at SDRAM byte SDR_VROM + {b, g, h, 00}. Only the pair
// {h = 0, h = 1} is contiguous, so each SDRAM read (four 16-bit words, p_long
// 0) gives two VROM words; the pair read last is kept for the next word.
// Geometry addresses below 0x800000 are not VROM and read as zero (the
// viewer's rule, m3v_core); bit 23 set, bits 22:0 are the VROM word.
// One request at a time, as m3v_sdr_rd.

`timescale 1ns/1ps

module r3d_vrom_rd #(
  parameter int          AW       = 26,
  parameter logic [26:0] SDR_VROM = 27'h400_0000
) (
  input  logic          clk,
  input  logic          rst,
  input  logic          g_req,
  input  logic [23:0]   g_addr,
  input  logic [2:0]    g_len,
  output logic          g_ack,
  output logic          g_rvalid,
  output logic [31:0]   g_rdata,

  output logic          p_req,
  output logic [AW:1]   p_addr,
  input  logic          p_ack,
  input  logic [63:0]   p_dout,
  output logic [15:0]   n_req          // SDRAM reads, all time
);

  typedef enum logic [1:0] { V_IDLE, V_WORD, V_WAIT } st_t;
  st_t         st;
  logic [23:0] a;                     // the next word
  logic [3:0]  left;                  // words left
  logic        q_v;                   // the pair in q is {qa, h}
  logic [21:0] qa;
  logic [63:0] q;
  logic        gap;
  wire  [21:0] pa = {a[22:3], a[2:1]};

  always_ff @(posedge clk) begin
    g_ack <= 1'b0; g_rvalid <= 1'b0; gap <= 1'b0;
    if (rst) begin
      st <= V_IDLE; p_req <= 1'b0; q_v <= 1'b0; n_req <= '0;
    end else begin
      case (st)
        V_IDLE: if (g_req && !g_ack) begin
          g_ack <= 1'b1;
          a <= g_addr; left <= {1'b0, g_len} + 4'd1;
          st <= V_WORD;
        end
        V_WORD: begin
          if (left == 4'd0) st <= V_IDLE;
          else if (!a[23]) begin
            g_rvalid <= 1'b1; g_rdata <= 32'd0;
            a <= a + 24'd1; left <= left - 4'd1;
          end else if (q_v && qa == pa) begin
            g_rvalid <= 1'b1; g_rdata <= a[0] ? q[63:32] : q[31:0];
            a <= a + 24'd1; left <= left - 4'd1;
          end else if (!gap) begin
            p_req <= 1'b1;
            p_addr <= AW'(SDR_VROM[26:1]) + AW'({a[2:1], a[22:3], 2'b00});
            st <= V_WAIT;
          end
        end
        V_WAIT: if (p_ack) begin
          p_req <= 1'b0; gap <= 1'b1;
          q <= p_dout; qa <= pa; q_v <= 1'b1;
          n_req <= n_req + 16'd1;
          st <= V_WORD;
        end
        default: st <= V_IDLE;
      endcase
    end
  end

endmodule
