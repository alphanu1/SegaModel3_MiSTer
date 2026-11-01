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
//   MC_WR       one or two single-word writes (the halves with lanes set);
//               acknowledged at once, written after (posted, in order)
//   MC_LINE_RD  two 8-word bursts, the half with the critical word first;
//               beats go out critical word first, wrapping, as each arrives
//   MC_LINE_WR  eight beats taken one per s_wnext, sixteen single writes
// s_zero answers reads with zeros without touching SDRAM (an absent CROM
// bank). One transaction at a time; a new one is taken only when the last
// is done, so reads see earlier writes.
//
// Cost (sim/system/notes.md): m2_sdram writes one 16-bit word a request, so
// a cast-out is 16 requests. A burst write path is stage B work.

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
  input  logic          p_ack
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
  logic         wsel;            // T_WR / T_LW: the high 16-bit word next
  logic [2:0]   wbeat;           // T_LW: the beat
  logic         wbeat_v;         // T_LW: wd holds beat wbeat
  logic [7:0]   bv;              // T_LR / T_ZERO: beats arrived
  logic [7:0][31:0] bd;
  logic [2:0]   ek;              // beats sent
  logic [2:0]   crit;

  // the 16-bit word and lanes of wd for this write (wsel: the high word)
  logic [15:0]  w16;
  logic [1:0]   w_be;
  always_comb begin
    if (!swap) begin
      w16  = wsel ? wd[31:16] : wd[15:0];
      w_be = wsel ? be[3:2]   : be[1:0];
    end else begin
      // byte form: CPU byte k (bits 31-8k) at SDRAM byte A + k
      w16  = wsel ? {wd[7:0], wd[15:8]} : {wd[23:16], wd[31:24]};
      w_be = wsel ? {be[0], be[1]}      : {be[2], be[3]};
    end
  end

  wire [2:0] enext = crit + ek;    // the beat to send next
  // T_LR: the second half has arrived (no more requests)
  wire       half_done = half && bv[{!crit[2], 2'd3}];

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
          bv <= '0; ek <= '0; half <= 1'b0; wsel <= 1'b0; wbeat <= '0; wbeat_v <= 1'b0;
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
              p_addr <= {s_sa[AW:5], s_sa[4], 3'b000};
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
          // arrivals: the half in flight is crit[2] ^ half
          if (p_lo) begin
            bd[{crit[2] ^ half, 2'd0}] <= swap ? bswap(p_dout[31:0])  : p_dout[31:0];
            bd[{crit[2] ^ half, 2'd1}] <= swap ? bswap(p_dout[63:32]) : p_dout[63:32];
            bv[{crit[2] ^ half, 2'd0}] <= 1'b1; bv[{crit[2] ^ half, 2'd1}] <= 1'b1;
          end
          if (p_ack) begin
            bd[{crit[2] ^ half, 2'd2}] <= swap ? bswap(p_dout[31:0])  : p_dout[31:0];
            bd[{crit[2] ^ half, 2'd3}] <= swap ? bswap(p_dout[63:32]) : p_dout[63:32];
            bv[{crit[2] ^ half, 2'd2}] <= 1'b1; bv[{crit[2] ^ half, 2'd3}] <= 1'b1;
            p_req <= 1'b0; gap <= 1'b1;
            if (!half) begin
              half <= 1'b1;
              p_addr <= {sa[AW:5], !crit[2], 3'b000};
            end
          end
          // out, critical word first
          if (bv[enext]) begin
            s_rvalid <= 1'b1; s_rdata <= bd[enext]; ek <= ek + 3'd1;
            if (ek == 3'd7) st <= T_IDLE;
          end
        end

        T_WR: begin
          // the halves with lanes: low word (A/2) then high word (A/2 + 1)
          if (!p_req && !gap && !p_ack) begin
            if (w_be == 2'b00) begin
              if (wsel) st <= T_IDLE;
              wsel <= 1'b1;
            end else begin
              p_req <= 1'b1; p_we <= 1'b1; p_long <= 1'b0;
              p_addr <= {sa[AW:2], wsel}; p_din <= w16; p_be <= w_be;
            end
          end
          if (p_ack) begin
            p_req <= 1'b0; gap <= 1'b1;
            if (wsel) st <= T_IDLE;
            wsel <= 1'b1;
          end
        end

        T_LW: begin
          // take beat wbeat, then write its two halves
          if (!wbeat_v) begin
            if (!s_wnext) begin s_wnext <= 1'b1; end
            else begin wd <= s_wdata; wbeat_v <= 1'b1; be <= 4'hF; wsel <= 1'b0; end
          end else begin
            if (!p_req && !gap && !p_ack) begin
              p_req <= 1'b1; p_we <= 1'b1; p_long <= 1'b0;
              p_addr <= {sa[AW:5], wbeat, wsel}; p_din <= w16; p_be <= w_be;
            end
            if (p_ack) begin
              p_req <= 1'b0; gap <= 1'b1;
              if (wsel) begin
                wbeat_v <= 1'b0; wbeat <= wbeat + 3'd1;
                if (wbeat == 3'd7) st <= T_IDLE;
              end
              wsel <= !wsel;
            end
          end
        end

        default: st <= T_IDLE;
      endcase
    end
  end

  assign s_idle = (st == T_IDLE);

endmodule
