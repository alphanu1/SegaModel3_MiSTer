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
// The DDR3 master with reads in flight (study D-33). One f2sdram port, two
// kinds of client:
//
//   port L   m2_ddr3's consumer contract exactly (D-20): req held to ack, one
//            transaction at a time, reads and write bursts, rvalid a beat,
//            wacc / wnext a write beat, ack at the end. Model 2's
//            m2_ddr3 rules all hold: the address and burst count are latched
//            for the burst (its R362), a write's byte enables are the
//            source's beat by beat (R661), and no request is taken in the
//            cycle its own ack is out (R660).
//   ports P  pipelined reads: p_req with p_addr / p_blen held until p_ack;
//            p_ack is the read taken into the master's command queue, not the
//            data. The data comes back later, in order, one p_rvalid a beat on
//            the shared dout. Several reads may be in flight; the port never
//            waits for a read's data before taking the next. With p_we a P
//            request is a write burst instead: its beats go out on p_din, one
//            a p_wacc (taken this cycle; the next beat's word the cycle after),
//            as port L's do.
//
// A read of either kind is tracked in a 2^QB-entry queue from the cycle the
// bridge takes it (DDRAM_BUSY low) to its last beat. Avalon returns read data
// in command order, so the queue's head names the owner of every beat; a
// write between reads changes nothing. The command itself sits in a 2-entry
// queue whose head drives the pins from registers, so p_ack depends on no
// pin and the port can still take a command every cycle.
//
// PIPE = 0 takes a read only when no read is in flight or queued: one read at
// a time, as m2_ddr3 behaves, with the same ports (D-33's switch).
//
// Debug: for every read, the cycles from the first cycle its command is on
// the pins to its first beat (m2_ddr3's dbg_lat for a one-beat read: Model 2
// R351's 13 typical, 50 worst), with the port that owns it; the reads the
// bridge holds now, and the most it has held. A client sees two cycles more
// than the pin figure: the command register and dout's.

`timescale 1ns/1ps

module m3_ddr3p #(
  parameter bit PIPE = 1'b1,
  parameter int NP   = 1,          // pipelined read ports
  parameter int QB   = 5,          // log2 reads tracked in flight
  parameter bit RQREG = 1'b0,      // the read queue in registers, not an MLAB
  parameter int MAXQ = 1 << QB     // reads queued or in flight at most (1 .. 2^QB)
) (
  input  logic        clk,
  input  logic        rst_n,
  input  logic [28:0] base,        // 64-bit word address of addr 0 (Model 2 R654)

  // ---- port L: m2_ddr3's contract
  input  logic        req,
  input  logic        we,
  input  logic [24:0] addr,
  input  logic [7:0]  blen,
  input  logic [63:0] din,
  input  logic [7:0]  be,
  output logic        wnext,
  output logic        wacc,
  output logic        rvalid,
  output logic        ack,

  // ---- ports P: pipelined reads
  input  logic [NP-1:0]        p_req,
  input  logic [NP-1:0][24:0]  p_addr,
  input  logic [NP-1:0][7:0]   p_blen,
  input  logic [NP-1:0]        p_we,
  input  logic [NP-1:0][63:0]  p_din,
  output logic [NP-1:0]        p_ack,
  output logic [NP-1:0]        p_rvalid,
  output logic [NP-1:0]        p_wacc,

  output logic [63:0] dout,        // every read's beats, L's and P's

  // ---- the framework's DDRAM port
  output logic        DDRAM_CLK,
  input  logic        DDRAM_BUSY,
  output logic [7:0]  DDRAM_BURSTCNT,
  output logic [28:0] DDRAM_ADDR,
  output logic [63:0] DDRAM_DIN,
  output logic [7:0]  DDRAM_BE,
  output logic        DDRAM_WE,
  output logic        DDRAM_RD,
  input  logic [63:0] DDRAM_DOUT,
  input  logic        DDRAM_DOUT_READY,

  // ---- debug
  output logic        dbg_lat_v,   // a read's first beat this cycle (registered)
  output logic [15:0] dbg_lat,     // its latency, pins, cycles
  output logic [3:0]  dbg_lat_src, // 0 port L, 1 + k port P k
  output logic [15:0] dbg_lat_max,
  output logic [QB:0] dbg_acc,     // reads the bridge has taken and not finished
  output logic [QB:0] dbg_acc_max,
  output logic        dbg_take,    // a read taken by the bridge this cycle (combinational)
  output logic [3:0]  dbg_take_src,
  output logic        dbg_done,    // a read's last beat this cycle (combinational)
  output logic [3:0]  dbg_done_src,
  output logic        dbg_idle,    // nothing queued, nothing in flight, port L free
  output logic [7:0]  dbg_st,      // {slot 1, slot 0, slot 0 a write, L busy, reads queued or in flight [3:0]}
  output logic [31:0] dbg_rdy,     // DDRAM_DOUT_READY pulses seen, all of them
  output logic [31:0] dbg_rq       // {head owner 4, head beats 8, beats received 8, write ptr 6, read ptr 6}
);

  localparam int SB = 4;           // source field: 0 L, 1 + k P k
  localparam int Q  = 1 << QB;

  assign DDRAM_CLK = clk;

  // ===========================================================================
  // the command queue: two entries, the head on the pins
  // ===========================================================================
  logic          c_v  [2];
  logic          c_wr [2];
  logic [28:0]   c_a  [2];
  logic [7:0]    c_n  [2];
  logic [7:0]    c_be [2];
  logic [SB-1:0] c_s  [2];
  logic [15:0]   c_t;              // when the head first went on the pins
  logic [7:0]    wbeats;           // write beats still to give, the head's

  assign DDRAM_RD       = c_v[0] && !c_wr[0];
  assign DDRAM_WE       = c_v[0] &&  c_wr[0];
  assign DDRAM_ADDR     = c_a[0];
  assign DDRAM_BURSTCNT = c_n[0];
  logic [63:0] pdin;
  always_comb begin
    pdin = din;                                         // port L's live beat, or a P port's
    for (int k = 0; k < NP; k++) if (c_s[0] == SB'(k + 1)) pdin = p_din[k];
  end
  assign DDRAM_DIN      = pdin;
  assign DDRAM_BE       = c_wr[0] ? be : c_be[0];       // R661
  wire   wtake          = DDRAM_WE && !DDRAM_BUSY;      // a write beat taken
  assign wacc           = wtake && c_s[0] == '0;        // R660
  always_comb for (int k = 0; k < NP; k++) p_wacc[k] = wtake && c_s[0] == SB'(k + 1);

  wire take    = c_v[0] && !DDRAM_BUSY;                 // the pins' command, or a write beat, is taken
  wire rd_take = take && !c_wr[0];
  wire head_go = rd_take || (wtake && wbeats == 8'd1);  // the head leaves

  // ===========================================================================
  // reads in flight: the queue from the bridge's take to the last beat
  // ===========================================================================
  // The queue is read synchronously: rh is the head, read a cycle ahead (the
  // entry that will be the head next cycle), with the write forwarded when it
  // lands on that entry in the same cycle. (The board, fit_s12: an
  // asynchronous read of the 64-deep MLAB built by Quartus 17 gave the head as
  // zero, so every beat went to port L; the bench could not see it.)
  logic [QB:0]   rq_wp, rq_rp;
  logic [7:0]    rbeat;            // beats of the head read received
  logic [SB+8+16-1:0] rh;
  wire  [SB-1:0] rh_s = rh[SB+24-1 -: SB];
  wire  [7:0]    rh_n = rh[23:16];
  wire  [15:0]   rh_t = rh[15:0];
  wire           r_last = DDRAM_DOUT_READY && (rbeat + 8'd1 == rh_n);
  // RQREG: the two candidates (the head staying, or the next one) come from
  // registers only, and the port's late signals (the beat's valid, BUSY
  // through rd_take) only pick between them at the end. Reading the queue at
  // rp_next missed 125 MHz by 0.39 ns from DOUT_READY through the 64:1 read
  // mux (fit_s24). The MLAB form keeps the single read at rp_next.
  wire  [QB:0]   rp_next = rq_rp + (QB+1)'(r_last);
  wire  [QB:0]   rp_1 = rq_rp + (QB+1)'(1);
  wire  [SB+8+16-1:0] rq_new = {c_s[0], c_n[0], c_t};
  generate if (RQREG) begin : g_rqr
    (* ramstyle = "logic" *) logic [SB+8+16-1:0] rq [Q];
    wire fw0 = rq_wp[QB-1:0] == rq_rp[QB-1:0];
    wire fw1 = rq_wp[QB-1:0] == rp_1[QB-1:0];
    wire [SB+8+16-1:0] rq0 = rq[rq_rp[QB-1:0]];
    wire [SB+8+16-1:0] rq1 = rq[rp_1[QB-1:0]];
    always_ff @(posedge clk) if (rd_take) rq[rq_wp[QB-1:0]] <= rq_new;
    always_ff @(posedge clk)
      rh <= r_last ? ((rd_take && fw1) ? rq_new : rq1) : ((rd_take && fw0) ? rq_new : rq0);
  end else begin : g_rqm
    (* ramstyle = "MLAB, no_rw_check" *) logic [SB+8+16-1:0] rq [Q];
    always_ff @(posedge clk) if (rd_take) rq[rq_wp[QB-1:0]] <= rq_new;
    always_ff @(posedge clk)
      rh <= (rd_take && rq_wp[QB-1:0] == rp_next[QB-1:0]) ? rq_new : rq[rp_next[QB-1:0]];
  end endgenerate
  assign dbg_rq = {rh_s, rh_n, rbeat, 6'(rq_wp), 6'(rq_rp)};
  assign dbg_take = rd_take;
  assign dbg_take_src = c_s[0];
  assign dbg_done = r_last;
  assign dbg_done_src = rh_s;
  assign dbg_idle = !c_v[0] && !c_v[1] && (inflt == '0) && !l_busy;
  assign dbg_st   = {c_v[1], c_v[0], c_wr[0], l_busy, (inflt > (QB+2)'(15)) ? 4'hF : 4'(inflt)};
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) dbg_rdy <= '0;
    else if (DDRAM_DOUT_READY) dbg_rdy <= dbg_rdy + 32'd1;
  end

  // reads queued or in flight: the cap on taking another
  logic [QB+1:0] inflt;
  wire  [1:0]    n_rdq = {1'b0, c_v[0] && !c_wr[0]} + {1'b0, c_v[1] && !c_wr[1]};
  wire           rd_room = PIPE ? (inflt < (QB+2)'(MAXQ)) : (inflt == '0);

  // ===========================================================================
  // taking a command: port L first (one at a time, held to ack), then P
  // ===========================================================================
  logic          l_busy;           // L's transaction taken and not yet acknowledged
  wire           q_room = !c_v[1] || head_go;          // a slot will be free at the edge
  wire           l_take = req && !l_busy && !ack && q_room && (we || rd_room);
  logic [NP-1:0] p_win;
  logic [$clog2(NP+1)-1:0] p_last;
  always_comb begin
    p_win = '0;
    if (!l_take && !(req && !l_busy && !ack) && q_room) begin
      // round robin from the port after the last taken
      for (int j = NP; j >= 1; j--)
        if (p_req[(int'(p_last) + j) % NP] && (p_we[(int'(p_last) + j) % NP] || rd_room)) begin
          p_win = '0; p_win[(int'(p_last) + j) % NP] = 1'b1;
        end
    end
  end
  assign p_ack = p_win;
  wire           p_take = |p_win;

  // the new command
  logic          n_wr;
  logic [24:0]   n_a;
  logic [7:0]    n_n, n_be;
  logic [SB-1:0] n_s;
  always_comb begin
    n_wr = 1'b0; n_a = addr; n_n = (blen == 8'd0) ? 8'd1 : blen; n_be = be; n_s = '0;
    if (l_take) begin n_wr = we; end
    else begin
      n_be = 8'hff;
      for (int k = 0; k < NP; k++) if (p_win[k]) begin
        n_a = p_addr[k]; n_n = (p_blen[k] == 8'd0) ? 8'd1 : p_blen[k]; n_s = SB'(k + 1); n_wr = p_we[k];
      end
    end
  end
  wire           n_v = l_take || p_take;

  logic [15:0]   now;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      c_v[0] <= 1'b0; c_v[1] <= 1'b0; c_wr[0] <= 1'b0; c_wr[1] <= 1'b0;
      c_a[0] <= '0; c_a[1] <= '0; c_n[0] <= 8'd1; c_n[1] <= 8'd1; c_be[0] <= 8'hff; c_be[1] <= 8'hff;
      c_s[0] <= '0; c_s[1] <= '0; c_t <= '0; wbeats <= 8'd1;
      rq_wp <= '0; rq_rp <= '0; rbeat <= '0; inflt <= '0;
      l_busy <= 1'b0; ack <= 1'b0; wnext <= 1'b0; rvalid <= 1'b0; p_rvalid <= '0; dout <= '0;
      p_last <= '0; now <= '0;
      dbg_lat_v <= 1'b0; dbg_lat <= '0; dbg_lat_src <= '0; dbg_lat_max <= '0;
      dbg_acc <= '0; dbg_acc_max <= '0;
    end else begin
      now <= now + 16'd1;
      ack <= 1'b0; wnext <= 1'b0; rvalid <= 1'b0; p_rvalid <= '0; dbg_lat_v <= 1'b0;

      // ---- the queue: the head leaves, the second moves up, the new one goes in
      if (wtake) begin
        if (c_s[0] == '0) wnext <= 1'b1;
        if (wbeats != 8'd1) wbeats <= wbeats - 8'd1;
        else if (c_s[0] == '0) ack <= 1'b1;            // L's write done (m2_ddr3: with the last wnext)
        // a write's latency: its first beat taken
        if (wbeats == c_n[0]) begin
          dbg_lat_v <= 1'b1; dbg_lat <= now - c_t; dbg_lat_src <= c_s[0];
        end
      end
      begin
        logic          v0, v1;
        logic          w0;
        logic [28:0]   a0;
        logic [7:0]    nn0, be0;
        logic [SB-1:0] s0;
        logic          load0;
        v0 = c_v[0]; v1 = c_v[1];
        w0 = c_wr[0]; a0 = c_a[0]; nn0 = c_n[0]; be0 = c_be[0]; s0 = c_s[0];
        load0 = 1'b0;
        if (head_go) begin
          v0 = v1; w0 = c_wr[1]; a0 = c_a[1]; nn0 = c_n[1]; be0 = c_be[1]; s0 = c_s[1];
          v1 = 1'b0;
          load0 = c_v[1];
        end
        if (n_v) begin
          if (!v0) begin
            v0 = 1'b1; w0 = n_wr; a0 = base + 29'(n_a); nn0 = n_n; be0 = n_be; s0 = n_s; load0 = 1'b1;
          end else begin
            v1 = 1'b1;
            c_wr[1] <= n_wr; c_a[1] <= base + 29'(n_a); c_n[1] <= n_n; c_be[1] <= n_be; c_s[1] <= n_s;
          end
        end
        c_v[0] <= v0; c_v[1] <= v1;
        c_wr[0] <= w0; c_a[0] <= a0; c_n[0] <= nn0; c_be[0] <= be0; c_s[0] <= s0;
        if (load0) begin c_t <= now + 16'd1; wbeats <= nn0; end
      end
      if (l_take) l_busy <= 1'b1;
      if (ack) l_busy <= 1'b0;
      if (p_take) for (int k = 0; k < NP; k++) if (p_win[k]) p_last <= ($clog2(NP+1))'(k);

      // ---- reads in flight
      if (rd_take) rq_wp <= rq_wp + 1'b1;
      inflt <= inflt + (QB+2)'(n_v && !n_wr) - (QB+2)'(r_last);
      begin
        logic [QB:0] acc;
        acc = dbg_acc + (QB+1)'(rd_take) - (QB+1)'(r_last);
        dbg_acc <= acc;
        if (acc > dbg_acc_max) dbg_acc_max <= acc;
      end

      // ---- read data: the head's owner
      if (DDRAM_DOUT_READY) begin
        dout <= DDRAM_DOUT;
        if (rh_s == '0) rvalid <= 1'b1;
        for (int k = 0; k < NP; k++) if (rh_s == SB'(k + 1)) p_rvalid[k] <= 1'b1;
        if (rbeat == 8'd0) begin
          logic [15:0] l;
          l = now - rh_t;
          dbg_lat_v <= 1'b1; dbg_lat <= l; dbg_lat_src <= rh_s;
          if (l > dbg_lat_max) dbg_lat_max <= l;
        end
        if (r_last) begin
          rbeat <= '0; rq_rp <= rq_rp + 1'b1;
          if (rh_s == '0) ack <= 1'b1;                 // L's read done, with its last rvalid
        end else rbeat <= rbeat + 8'd1;
      end
    end
  end

endmodule
