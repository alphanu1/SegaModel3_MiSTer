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
// The system bus's SDRAM target: one transaction of the CPU caches' protocol
// (ppc_cache_pkg.sv) at a time, on one m2_sdram port (16-bit words, a request
// per rising edge, 4-word bursts, or 8 with p_long).
//
// Byte order in SDRAM (study 7.3, sim/system/notes.md). The SDRAM is a byte
// image, word w = bytes 2w (low) and 2w + 1 (high), which is how the MRA's
// ioctl stream lands. Two forms:
//   - word form (s_swap = 0): RAM, CROM, backup RAM. The 32-bit value the
//     PowerPC sees at byte address A is the little-endian word at SDRAM A:
//     SDRAM word A/2 is its bits 15:0, A/2 + 1 its bits 31:16. This is
//     Supermodel's host layout (ram[addr ^ 3]) and MAME's ROM_LOAD64_WORD_SWAP
//     files as they come, and it is the order the Real3D reads (VROM, culling
//     RAM: the 32-bit value as stored);
//   - byte form (s_swap = 1): the tile generator's VRAM, a straight image of
//     the bus (CPU byte A at SDRAM byte A), which is what m3_tilegen reads.
//
// Transactions (s_sa is the SDRAM byte address of the access, or of the
// critical word for a line fill):
//   MC_RD       one 4-word burst at the 8-byte-aligned address, half chosen
//   MC_WR       one two-column write burst (m2_sdram WB); acknowledged at
//               once, written after (posted, in order)
//   MC_LINE_RD  two 8-word bursts, the first from the critical word's pair
//               (s_sa[4:3]), the second from four words on, each wrapping in
//               the line (m2_sdram's WB-port wrap, study R104); beats go out
//               critical word first, wrapping, as each arrives
//   MC_LINE_WR  eight beats taken one per s_wnext, a beat every four clocks,
//               into the line buffer, then two eight-column write bursts: the
//               first once beats 0-3 are in, the second after the last beat.
//               The bus is free after the eighth beat; the bursts are posted
// s_zero answers reads with zeros without touching SDRAM (an absent CROM
// bank). One transaction at a time; a new one is taken only when the last
// is done, so reads see earlier writes: a posted write is drained before the
// next transaction of either master, and nothing passes it.
//
// Writes are m2_sdram write bursts (its WB parameter must name this port):
// column 0 goes with the request on p_din / p_be, and the port reads column
// wb_col's word and lanes from wb_din / wb_be as it goes. Before,
// every 16-bit word was a request of its own: a store two, a cast-out
// sixteen (sim/system/notes.md, "posted writes").

module m3_sdr_port
  import ppc_cache_pkg::*;
#(
  parameter int AW = 26
) (
  input  logic          clk,
  input  logic          rst,

  input  logic          s_req,
  input  logic [1:0]    s_cmd,
  input  logic [AW:0]   s_sa,
  input  logic          s_swap,
  input  logic          s_zero,
  input  logic [3:0]    s_be,
  input  logic [31:0]   s_wdata,
  output logic          s_ack,
  output logic          s_wnext,
  output logic          s_rvalid,
  output logic [31:0]   s_rdata,
  output logic          s_idle,

  output logic          p_req,
  output logic          p_we,
  output logic [AW:1]   p_addr,
  output logic [15:0]   p_din,
  output logic [1:0]    p_be,
  output logic          p_long,
  input  logic [63:0]   p_dout,
  input  logic          p_lo,
  input  logic          p_ack,
  input  logic [3:0]    wb_col,
  output logic [15:0]   wb_din,
  output logic [1:0]    wb_be
);

  function automatic logic [31:0] bswap(input logic [31:0] v);
    bswap = {v[7:0], v[15:8], v[23:16], v[31:24]};
  endfunction

  typedef enum logic [2:0] { T_IDLE, T_RD, T_LR, T_WR, T_LW, T_ZERO } st_t;
  st_t          st;
  logic [1:0]   cmd;
  logic [AW:0]  sa;
  logic         swap;
  logic [3:0]   be;
  logic [31:0]  wd;

  // ---------------------------------------------------------------- the port
  // p_req rises once per transaction and falls with its acknowledge; `gap`
  // keeps it low for a cycle before the next.
  logic         gap;
  logic         half;            // T_LR: the burst in flight is the second half
  logic [2:0]   wbeat;           // T_LW: beats taken
  logic         wfull;           // T_LW: all eight beats taken
  logic [1:0]   wph;             // T_LW: the beat clock (one s_wnext in four)
  logic [7:0]   bv;              // T_LR / T_ZERO: beats arrived
  logic [7:0][31:0] bd;
  logic [2:0]   ek;              // beats sent
  logic [2:0]   crit;

  // A write column's 16-bit word and lanes ({lanes, word}): the low word
  // (A/2) of a 32-bit beat, or with hi the high (A/2 + 1).
  function automatic logic [17:0] col16(input logic [31:0] w, input logic [3:0] b, input logic sw, input logic hi);
    if (!sw) col16 = hi ? {b[3:2], w[31:16]} : {b[1:0], w[15:0]};
    // byte form: CPU byte k (bits 31-8k) at SDRAM byte A + k
    else     col16 = hi ? {b[0], b[1], w[7:0], w[15:8]} : {b[2], b[3], w[23:16], w[31:24]};
  endfunction
  // Column 0 goes with the request (p_din / p_be); column wb_col (1 on) is
  // read while the column before it is written. A line's columns are beat
  // {half, col[2:1]}, a single write's are wd.
  wire          lw = (st == T_LW);
  assign {p_be, p_din}   = col16(lw ? bd[{half, 2'b00}] : wd, lw ? 4'hF : be, swap, 1'b0);
  assign {wb_be, wb_din} = col16(lw ? bd[{half, wb_col[2:1]}] : wd, lw ? 4'hF : be, swap, wb_col[0]);

  wire [2:0] enext = crit + ek;    // the beat to send next
  // T_LR: the second half has arrived (no more requests)
  wire       half_done = half && bv[3'({crit[2:1], 1'b0} + 3'd7)];
  // the line word of a burst's beat j (0-3): the bursts start at the critical pair and its +4, wrapping
  wire [2:0] lw0 = {crit[2:1], 1'b0} + {half, 2'b00};

  always_ff @(posedge clk) begin
    s_ack <= 1'b0; s_wnext <= 1'b0; s_rvalid <= 1'b0;
    if (rst) begin
      st <= T_IDLE; p_req <= 1'b0; gap <= 1'b0;
    end else begin
      gap <= 1'b0;
      case (st)
        T_IDLE: if (s_req && !s_ack) begin
          cmd <= s_cmd; sa <= s_sa; swap <= s_swap; be <= s_be; wd <= s_wdata;
          crit <= (s_cmd == MC_LINE_RD) ? s_sa[4:2] : 3'd0;
          bv <= '0; ek <= '0; half <= 1'b0; wbeat <= '0; wfull <= 1'b0; wph <= '0;
          s_ack <= 1'b1;
          if (s_zero && !s_cmd[0]) begin
            st <= T_ZERO; bv <= (s_cmd == MC_RD) ? 8'h01 : 8'hFF; bd <= '0;
          end else case (s_cmd)
            MC_RD: begin
              st <= T_RD; p_req <= !gap; p_we <= 1'b0; p_long <= 1'b0;
              p_addr <= {s_sa[AW:3], 2'b00};
            end
            MC_LINE_RD: begin
              st <= T_LR; p_req <= !gap; p_we <= 1'b0; p_long <= 1'b1;
              p_addr <= {s_sa[AW:5], s_sa[4:3], 2'b00};   // R104: from the critical word's pair, wrapping in the line
            end
            MC_WR: st <= T_WR;
            default: st <= T_LW;     // MC_LINE_WR
          endcase
        end

        T_ZERO: begin
          // zeros: as many beats as the command wants
          if (bv[enext]) begin
            s_rvalid <= 1'b1; s_rdata <= 32'd0; ek <= ek + 3'd1;
            if ((cmd == MC_RD) || ek == 3'd7) st <= T_IDLE;
          end
        end

        T_RD: begin
          if (!p_req && !gap && !p_ack) p_req <= 1'b1;
          if (p_ack) begin
            p_req <= 1'b0; gap <= 1'b1;
            s_rvalid <= 1'b1;
            s_rdata <= swap ? bswap(sa[2] ? p_dout[63:32] : p_dout[31:0]) : (sa[2] ? p_dout[63:32] : p_dout[31:0]);
            st <= T_IDLE;
          end
        end

        T_LR: begin
          if (!p_req && !gap && !p_ack && !half_done) p_req <= 1'b1;
          // arrivals: words lw0 .. lw0 + 3 of the line (R104: from the critical pair, wrapping)
          if (p_lo) begin
            bd[lw0]         <= swap ? bswap(p_dout[31:0])  : p_dout[31:0];
            bd[lw0 + 3'd1]  <= swap ? bswap(p_dout[63:32]) : p_dout[63:32];
            bv[lw0] <= 1'b1; bv[lw0 + 3'd1] <= 1'b1;
          end
          if (p_ack) begin
            bd[lw0 + 3'd2]  <= swap ? bswap(p_dout[31:0])  : p_dout[31:0];
            bd[lw0 + 3'd3]  <= swap ? bswap(p_dout[63:32]) : p_dout[63:32];
            bv[lw0 + 3'd2] <= 1'b1; bv[lw0 + 3'd3] <= 1'b1;
            p_req <= 1'b0; gap <= 1'b1;
            if (!half) begin
              half <= 1'b1;
              p_addr <= {sa[AW:5], !sa[4], sa[3], 2'b00};
            end
          end
          // out, critical word first
          if (bv[enext]) begin
            s_rvalid <= 1'b1; s_rdata <= bd[enext]; ek <= ek + 3'd1;
            if (ek == 3'd7) st <= T_IDLE;
          end
        end

        T_WR: begin
          // one two-column burst: the low word (A/2), then the high (A/2 + 1)
          if (!p_req && !gap && !p_ack && !half) begin
            p_req <= 1'b1; p_we <= 1'b1; p_long <= 1'b0; p_addr <= {sa[AW:2], 1'b0}; half <= 1'b1;
          end
          if (p_ack) begin p_req <= 1'b0; gap <= 1'b1; st <= T_IDLE; end
        end

        T_LW: begin
          // the beats, one s_wnext every four clocks: the bus passes it on a
          // clock later and the master shows the next beat the clock after
          // that, as for the bus's own line writes (B_BEAT, three clocks)
          wph <= wph + 2'd1;
          if (wph == 2'd0 && !wfull) s_wnext <= 1'b1;
          if (s_wnext) begin
            bd[wbeat] <= s_wdata; wbeat <= wbeat + 3'd1;
            if (wbeat == 3'd7) wfull <= 1'b1;
          end
          // the halves: the first once beats 0-3 are in, the second once all are
          if (!p_req && !gap && !p_ack && (half ? wfull : (wbeat[2] || wfull))) begin
            p_req <= 1'b1; p_we <= 1'b1; p_long <= 1'b1; p_addr <= {sa[AW:5], half, 3'b000};
          end
          if (p_ack) begin
            p_req <= 1'b0; gap <= 1'b1;
            if (half) st <= T_IDLE;
            half <= 1'b1;
          end
        end

        default: st <= T_IDLE;
      endcase
    end
  end

  assign s_idle = (st == T_IDLE);

endmodule
