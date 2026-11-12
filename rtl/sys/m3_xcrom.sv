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
// Banked CROM in the extra memory (study R170). Spikeout's banked CROM is
// 112 MiB (CROM0-2 32 MiB each, CROM3 16 MiB); with the 8 MiB fixed CROM, the
// backup RAM and the state (15.1 MiB) it is 135 MiB, past the 128 MiB SDRAM.
// So a bank map entry can name a slot in the extra memory's CROM region
// instead (configuration byte bit 6), and m3_bus sends such a bank's reads out
// on its SDRAM port with s_x set; m3_system routes them here.
//
// The bus's SDRAM-target contract (m3_sdr_port), reads only (a write to CROM
// never reaches the port: m3_bus X_ROM_W):
//   MC_RD       one 32-bit word, the one at s_sa (word form, as in SDRAM:
//               the even word in the beat's low half);
//   MC_LINE_RD  eight words, critical word (s_sa) first, wrapping in the line.
// s_ack in the clock after the request is taken (the bus drops s_req on it);
// the words follow, never in s_ack's clock.
//
// One DDR3 burst a transaction on an m3_ddr_arb port (the loader's, idle
// while the game runs): a line's four beats from its word 0, or the one beat
// of a single read. The words go out a clock each as their beat is in, from
// two 4 x 32 MLABs (even and odd words, m3v_ddr_rd's form). The region is
// 16 MiB from BASE (two 8 MiB slots: s_sa[23] is the slot); higher slot bits
// wrap, so a stray slot can never reach the sound bridge's block above it.

module m3_xcrom
  import ppc_cache_pkg::*;
#(
  parameter logic [24:0] BASE = 25'h1C0_0000    // the region's first beat (byte 0x3E000000 from the cores' 0x30000000)
) (
  input  logic        clk,
  input  logic        rst,

  input  logic        s_req,
  input  logic [1:0]  s_cmd,
  input  logic [23:2] s_sa,
  output logic        s_ack,
  output logic        s_rvalid,
  output logic [31:0] s_rdata,      // zero but in s_rvalid's clocks

  output logic        m_req,
  output logic [24:0] m_addr,
  output logic [7:0]  m_blen,
  input  logic        m_rvalid,
  input  logic        m_ack,
  input  logic [63:0] m_dout
);

  (* ramstyle = "MLAB, no_rw_check" *) logic [31:0] qe [4];   // even words, by beat
  (* ramstyle = "MLAB, no_rw_check" *) logic [31:0] qo [4];   // odd words

  logic        line;            // a line read (else one word)
  logic [2:0]  nb;              // beats in, 0-4
  logic [1:0]  wb;              // the next beat's slot
  logic [2:0]  rp;              // the next word out
  logic [3:0]  left;            // words still to go out
  wire  [1:0]  rb    = line ? rp[2:1] : 2'd0;
  wire         avail = {1'b0, rb} < nb;
  wire  [31:0] rd_q  = rp[0] ? qo[rb] : qe[rb];
  wire         busy  = m_req || (left != 4'd0);

  always_ff @(posedge clk) if (m_req && m_rvalid) begin
    qe[wb] <= m_dout[31:0];
    qo[wb] <= m_dout[63:32];
  end

  always_ff @(posedge clk) begin
    s_ack <= 1'b0;
    s_rvalid <= 1'b0;
    s_rdata <= 32'd0;
    if (rst) begin
      m_req <= 1'b0; left <= 4'd0; nb <= 3'd0; wb <= 2'd0;
    end else begin
      if (s_req && !s_ack && !busy) begin
        s_ack  <= 1'b1;
        line   <= (s_cmd == MC_LINE_RD);
        rp     <= (s_cmd == MC_LINE_RD) ? s_sa[4:2] : {2'b00, s_sa[2]};
        left   <= (s_cmd == MC_LINE_RD) ? 4'd8 : 4'd1;
        nb     <= 3'd0; wb <= 2'd0;
        m_req  <= 1'b1;
        m_addr <= BASE + 25'((s_cmd == MC_LINE_RD) ? {s_sa[23:5], 2'b00} : s_sa[23:3]);
        m_blen <= (s_cmd == MC_LINE_RD) ? 8'd4 : 8'd1;
      end
      if (m_req && m_rvalid) begin nb <= nb + 3'd1; wb <= wb + 2'd1; end
      if (m_req && m_ack) m_req <= 1'b0;
      if (left != 4'd0 && avail) begin
        s_rvalid <= 1'b1; s_rdata <= rd_q;
        rp <= rp + 3'd1; left <= left - 4'd1;
      end
    end
  end

`ifdef VERILATOR
  always_ff @(posedge clk) if (!rst && s_req && !s_ack && !busy && s_cmd[0]) $fatal(1, "m3_xcrom: a write");
`endif

endmodule
