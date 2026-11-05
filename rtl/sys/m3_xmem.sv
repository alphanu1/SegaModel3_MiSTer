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
// VROM in the extra memory (study R100: the MRA's index 1; DDR3 now, the
// second SDRAM later). The image keeps the MRA's VROM layout (sim/system/
// notes.md): the sixteen ROMs in four blocks of four, so VROM word i =
// {g, b, h} is half h of 64-bit beat b * BLK + g of the region, BLK 2 M beats
// for a 64 MiB VROM (vrom64, Daytona 2) and 1 M for 32 MiB (VF3).
//
// Two clients, one at a time, on one pipelined read port (m3_ddr3p's ports P:
// req held to ack, the beats later and in order on the shared dout):
//   g_*  the geometry's VROM port, as r3d_vrom_rd's (req held to g_ack, 1-8
//        words, then the words in order). A request's pairs, at most five
//        beats, are all issued at once, so it costs one latency and not one
//        a pair. With !vrom64, words below 0x800000 read zero (r3d_vrom_rd's
//        rule); with vrom64 all 24 bits are VROM (both emulators: a model at
//        0x100000 or above is VROM[addr]).
//   t_*  r3d_texup's VROM port, m2_sdram's p_ contract (one 64-bit read at
//        an SDRAM word address in the VROM image's SDRAM layout; t_ack a cycle
//        with the data): the beat is the same offset from the region's base.
// The back end is a beat address only (b_addr from the region's base, XBASE
// added by the caller), so the second SDRAM is a different caller, not a
// change here.

module m3_xmem #(
  parameter int          AW       = 26,
  parameter logic [26:0] SDR_VROM = 27'h400_0000   // where r3d_texup believes VROM is
) (
  input  logic          clk,
  input  logic          rst,
  input  logic          vrom64,

  input  logic          g_req,
  input  logic [23:0]   g_addr,
  input  logic [2:0]    g_len,
  output logic          g_ack,
  output logic          g_rvalid,
  output logic [31:0]   g_rdata,

  input  logic          t_req,
  input  logic [AW:1]   t_addr,
  output logic          t_ack,
  output logic [63:0]   t_dout,

  output logic          b_req,
  output logic [23:0]   b_addr,      // beat in the region
  input  logic          b_ack,
  input  logic          b_rvalid,
  input  logic [63:0]   b_dout
);

  typedef enum logic [1:0] { S_IDLE, S_GEO, S_TEX } st_t;
  st_t st;

  // ---- the geometry's request
  logic [23:0] a0;                    // first word
  logic [3:0]  nw;                    // words
  logic [2:0]  nb, ib, rb;            // beats: needed, issued, received
  logic [3:0]  ow;                    // words given
  logic [22:0] pc;                    // the next pair to issue
  logic        zero;                  // VF3 below 0x800000: zeros, no reads
  (* ramstyle = "MLAB, no_rw_check" *) logic [63:0] bq [8];

  wire [22:0] p0  = a0[23:1];
  wire [23:0] wa  = a0 + 24'(ow);                       // the word to give next
  wire [2:0]  wb  = 3'(wa[23:1] - p0);                  // its beat
  wire        have = zero || (wb < rb);
  wire [63:0] q   = bq[wb];

  // a pair's beat: {g, b} -> b * BLK + g
  function automatic logic [23:0] beat(input logic [22:0] p, input logic v64);
    beat = v64 ? {1'b0, p[1:0], p[22:2]} : {2'b00, p[1:0], p[21:2]};
  endfunction

  logic t_pend;

  always_ff @(posedge clk) begin
    g_ack <= 1'b0; g_rvalid <= 1'b0; t_ack <= 1'b0;
    if (b_rvalid && st == S_GEO) begin bq[rb] <= b_dout; rb <= rb + 3'd1; end
    if (rst) begin
      st <= S_IDLE; b_req <= 1'b0; t_pend <= 1'b0;
    end else begin
      case (st)
        S_IDLE: begin
          if (g_req && !g_ack) begin
            g_ack <= 1'b1;
            a0 <= g_addr; nw <= {1'b0, g_len} + 4'd1; ow <= 4'd0;
            nb <= 3'((({1'b0, g_addr} + 25'(g_len)) >> 1) - 25'(g_addr[23:1])) + 3'd1;
            ib <= 3'd0; rb <= 3'd0; pc <= g_addr[23:1];
            zero <= !vrom64 && !g_addr[23];
            st <= S_GEO;
          end else if (t_req && !t_ack && !t_pend) begin
            b_req <= 1'b1; b_addr <= 24'(t_addr - AW'(SDR_VROM[26:1])) >> 2;
            t_pend <= 1'b1;
            st <= S_TEX;
          end
        end
        S_GEO: begin
          // issue the pairs
          if (b_req && b_ack) b_req <= 1'b0;
          if (!zero && (!b_req || b_ack) && ib != nb) begin
            b_req <= 1'b1; b_addr <= beat(pc, vrom64);
            pc <= pc + 23'd1; ib <= ib + 3'd1;
          end
          // give the words, one a clock
          if (ow != nw && have) begin
            g_rvalid <= 1'b1;
            g_rdata <= zero ? 32'd0 : (wa[0] ? q[63:32] : q[31:0]);
            ow <= ow + 4'd1;
          end
          if (ow == nw && (zero || rb == nb)) st <= S_IDLE;
        end
        S_TEX: begin
          if (b_req && b_ack) b_req <= 1'b0;
          if (b_rvalid) begin t_ack <= 1'b1; t_dout <= b_dout; t_pend <= 1'b0; st <= S_IDLE; end
        end
        default: st <= S_IDLE;
      endcase
      if (!t_req) t_pend <= 1'b0;
    end
  end

endmodule
