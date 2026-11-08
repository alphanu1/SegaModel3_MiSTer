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
// Two clients on one pipelined read port (m3_ddr3p's ports P: req held to
// ack, the beats later and in order on the shared dout):
//   g_*  the geometry's VROM port, as r3d_vrom_rd's (req held to g_ack, 1-8
//        words, then the words in order). A request's pairs, at most five
//        beats, are issued back to back, so it costs one latency and not one
//        a pair. With !vrom64, words below 0x800000 read zero (r3d_vrom_rd's
//        rule); with vrom64 all 24 bits are VROM (both emulators: a model at
//        0x100000 or above is VROM[addr]).
//   t_*  r3d_texup's VROM port, m2_sdram's p_ contract (one 64-bit read at
//        an SDRAM word address in the VROM image's SDRAM layout; t_ack a cycle
//        with the data): the beat is the same offset from the region's base.
// The back end is a beat address only (b_addr from the region's base, XBASE
// added by the caller), so the second SDRAM is a different caller, not a
// change here.
//
// PIPE (study R127): the geometry's requests overlap. Up to GQ requests are
// taken (g_ack) before the first one's words are back: their beats are
// issued in request order as the port takes them, come back in order into a
// 32-beat ring, and each request's words are given in order from the ring as
// its beats arrive. r3d_mstream keeps up to four 8-word bursts asked for, so
// with GQ = 4 the stream's whole prefetch is in DDR3 at once instead of one
// burst at a time. A texture read waits until the geometry has no beat in
// flight, and holds back the geometry's next beat while it waits, so every
// beat's owner is known from the order alone. PIPE = 0 is the one-at-a-time
// form as before.

module m3_xmem #(
  parameter int          AW       = 26,
  parameter logic [26:0] SDR_VROM = 27'h400_0000,  // where r3d_texup believes VROM is
  parameter bit          PIPE     = 1'b1,          // R127: geometry requests in flight together
  parameter int          GQ       = 4,             // PIPE: geometry requests taken and not yet given, at most (2 or 4)
  parameter bit          XLIN     = 1'b1           // R127: VROM in the extra memory in pair order (m3_system VROM_LIN)
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
  output logic [7:0]    b_blen,      // beats (XLIN: a request's pairs in one burst)
  input  logic          b_ack,
  input  logic          b_rvalid,
  input  logic [63:0]   b_dout
);

  // a pair's beat: {g, b} -> b * BLK + g; XLIN (R127): pair p is beat p, so a request's pairs are one burst
  function automatic logic [23:0] beat(input logic [22:0] p, input logic v64);
    if (XLIN) beat = v64 ? {1'b0, p} : {2'b00, p[21:0]};
    else      beat = v64 ? {1'b0, p[1:0], p[22:2]} : {2'b00, p[1:0], p[21:2]};
  endfunction

  // a texture read's beat: its SDRAM word offset / 4. (Before R127 the offset was cut to 24 bits before the
  // shift, which drops bit 24: with vrom64 every texture-port beat in VROM blocks 2 and 3 read blocks 0 and 1.)
  wire [AW-1:0] t_off  = t_addr - AW'(SDR_VROM[26:1]);
  // XLIN: the SDRAM layout's {b, g} is pair {g, b}
  wire [23:0]   t_beat = !XLIN ? t_off[25:2] : vrom64 ? {1'b0, t_off[22:2], t_off[24:23]} : {2'b00, t_off[21:2], t_off[23:22]};

  generate if (!PIPE) begin : g_one
  // ===========================================================================
  // one request at a time (as before R127)
  // ===========================================================================
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

  logic t_pend;
  assign b_blen = 8'd1;

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
            b_req <= 1'b1; b_addr <= t_beat;
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

  end else begin : g_pipe
  // ===========================================================================
  // R127: geometry requests in flight together
  // ===========================================================================
  localparam int QB = $clog2(GQ);
  localparam int RB = (GQ > 4) ? 6 : 5;   // the ring: 2^RB beats, at least GQ x 5
  // the requests taken: in order, issued at xi, given at xo, written at xw
  logic [QB:0]   xw, xi, xo;
  logic [23:0]   ra [GQ];             // first word
  logic [2:0]    rl [GQ];             // words - 1
  logic [2:0]    rn [GQ];             // beats - 1
  logic          rz [GQ];             // VF3 below 0x800000: zeros, no reads
  logic [RB-1:0] rbs [GQ];            // its first beat's place in the ring
  // the ring: beats allocated at fa (as requests are taken), issued (ni), received at fr
  logic [RB-1:0] fa, fr;
  logic [5:0]    ni;                  // beats issued and not yet received
  (* ramstyle = "MLAB, no_rw_check" *) logic [63:0] ring [1 << RB];
  // the issuer: the request at xi, its next pair and beats left
  logic          i_on;
  logic [22:0]   i_pc;
  logic [2:0]    i_left;              // beats left - 1
  // the giver: the request at xo, kept in registers (h_*) so the word's ring address does not go through the
  // queue's select (the standalone fit's worst path, xo into g_rdata); the words given
  logic          h_v, h_a, h_z;
  logic [2:0]    h_l;
  logic [RB-1:0] h_bs;
  logic [2:0]    ow;
  // the texture read in flight
  logic          t_out;

  wire [QB-1:0] wi = xw[QB-1:0], ii = xi[QB-1:0], oi = xo[QB-1:0];
  wire          q_full  = (xw[QB] != xo[QB]) && (wi == oi);
  wire          q_issue = xi != xw;                     // a request not yet issued
  wire          q_give  = h_v;                          // a request not yet given (its beats are in order from h_bs)
  // the giver's word: its beat in the request is (first word's half + words given) / 2
  wire [3:0]    o_wl  = {3'd0, h_a} + {1'b0, ow};
  wire [2:0]    o_wb  = o_wl[3:1];
  wire [RB-1:0] o_ix  = h_bs + RB'(o_wb);
  wire [RB-1:0] o_got = fr - h_bs;                      // beats received from the request's first (0 .. 5 GQ)
  wire          o_have = h_z || (RB'(o_wb) < o_got);
  wire [QB:0]   xo1   = xo + 1'b1;
  wire [QB-1:0] oi1   = xo1[QB-1:0];
  wire          g_take = g_req && !g_ack && !q_full;
  wire [63:0]   o_q   = ring[o_ix];
  // a texture read waits for the geometry's beats to be back
  wire          g_hold = t_req && !t_ack && !t_out;

  wire [3:0]    g_s   = {3'd0, g_addr[0]} + {1'b0, g_len};
  wire [2:0]    g_nb1 = g_s[3:1];                       // beats - 1

  always_ff @(posedge clk) begin
    g_ack <= 1'b0; g_rvalid <= 1'b0; t_ack <= 1'b0;
    if (b_rvalid && !t_out) begin ring[fr] <= b_dout; fr <= fr + 1'b1; end
    if (rst) begin
      xw <= '0; xi <= '0; xo <= '0; fa <= '0; fr <= '0; ni <= '0;
      i_on <= 1'b0; ow <= '0; b_req <= 1'b0; t_out <= 1'b0; h_v <= 1'b0;
    end else begin
      logic issued;
      issued = 1'b0;
      // ---- take a request
      if (g_take) begin
        g_ack <= 1'b1;
        ra[wi] <= g_addr; rl[wi] <= g_len; rn[wi] <= g_nb1;
        rz[wi] <= !vrom64 && !g_addr[23];
        rbs[wi] <= fa;
        if (vrom64 || g_addr[23]) fa <= fa + RB'(g_nb1) + 1'b1;
        xw <= xw + 1'b1;
        // the issuer idle with nothing queued: it takes the request now, a clock sooner
        if (!i_on && !q_issue && (vrom64 || g_addr[23])) begin i_on <= 1'b1; i_pc <= g_addr[23:1]; i_left <= g_nb1; end
      end
      // ---- issue: the request at xi, a beat each time the port takes one
      if (b_req && b_ack) b_req <= 1'b0;
      if (!t_out && !g_hold && (!b_req || b_ack)) begin
        if (i_on && XLIN) begin
          // the request's pairs in one burst
          b_req <= 1'b1; b_addr <= beat(i_pc, vrom64); b_blen <= {5'd0, i_left} + 8'd1; issued = 1'b1;
          i_on <= 1'b0; xi <= xi + 1'b1;
        end else if (i_on) begin
          b_req <= 1'b1; b_addr <= beat(i_pc, vrom64); b_blen <= 8'd1; issued = 1'b1;
          i_pc <= i_pc + 23'd1; i_left <= i_left - 3'd1;
          if (i_left == 3'd0) begin i_on <= 1'b0; xi <= xi + 1'b1; end
        end
      end
      if (!i_on && q_issue && !(issued)) begin
        if (rz[ii]) xi <= xi + 1'b1;                    // zeros: nothing to read
        else begin i_on <= 1'b1; i_pc <= ra[ii][23:1]; i_left <= rn[ii]; end
      end
      ni <= ni + (!issued ? 6'd0 : XLIN ? {3'd0, i_left} + 6'd1 : 6'd1) - 6'(b_rvalid && !t_out);
      // ---- give: the request at xo, a word a clock as its beats arrive
      if (q_give && o_have) begin
        g_rvalid <= 1'b1;
        g_rdata <= h_z ? 32'd0 : (o_wl[0] ? o_q[63:32] : o_q[31:0]);
        ow <= ow + 3'd1;
        if (ow == h_l) begin
          ow <= '0; xo <= xo1;
          // the next head: queued, or taken this clock, or none
          if (xo1 != xw) begin h_a <= ra[oi1][0]; h_l <= rl[oi1]; h_z <= rz[oi1]; h_bs <= rbs[oi1]; end
          else if (g_take) begin h_a <= g_addr[0]; h_l <= g_len; h_z <= !vrom64 && !g_addr[23]; h_bs <= fa; end
          else h_v <= 1'b0;
        end
      end else if (!h_v && g_take) begin
        h_v <= 1'b1; h_a <= g_addr[0]; h_l <= g_len; h_z <= !vrom64 && !g_addr[23]; h_bs <= fa;
      end
      // ---- the texture read: alone on the port
      if (g_hold && !b_req && ni == 6'd0) begin
        b_req <= 1'b1; b_addr <= t_beat; b_blen <= 8'd1;
        t_out <= 1'b1;
      end
      if (t_out && b_rvalid) begin t_ack <= 1'b1; t_dout <= b_dout; t_out <= 1'b0; end
    end
  end

`ifdef VERILATOR
  // the head registers are the queue's entry at xo
  always_ff @(posedge clk) if (!rst && h_v != (xo != xw)) $fatal(1, "m3_xmem: head valid %0d with %0d requests", h_v, xw - xo);
  always_ff @(posedge clk) if (!rst && h_v && (h_a != ra[oi][0] || h_l != rl[oi] || h_z != rz[oi] || h_bs != rbs[oi]))
    $fatal(1, "m3_xmem: head registers differ from the queue's entry");
  // the ring holds every request's beats: at most GQ x 5 beats are allocated
  always_ff @(posedge clk) if (!rst) begin
    if (RB'(fa - fr) > RB'(5 * GQ)) $fatal(1, "m3_xmem: %0d beats allocated past those received", fa - fr);
    if (ni > 6'(5 * GQ)) $fatal(1, "m3_xmem: %0d beats in flight", ni);
  end
`endif

  end endgenerate

endmodule
