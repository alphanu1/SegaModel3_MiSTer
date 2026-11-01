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
// Model word stream for the Real3D polygon decoder. A model below word
// 0x100000 is in polygon RAM, above it in VROM (MAME's draw_model). From the
// model's first word the stream prefetches 8-word bursts, up to three in
// flight, into a 32-word FIFO, so memory latency (12-20 cycles) overlaps the
// decode. Words past the end of the region read as zero, as the reference
// model does. "stop" abandons a model: bursts in flight are drained and
// discarded before the next start, and so is a burst asked for and not yet
// acknowledged, which stop cannot withdraw (a request holds until its ack).
// ready_start waits for it: a slow memory can acknowledge it after the last
// acknowledged burst has drained, and taken by a new model it would put the
// old model's words at the head of the FIFO and step next past the new
// model's first eight (m2-notes section 15).
//
// The colour table (polygon RAM 0x400 + index) shares the polygon RAM port:
// a colour read waits until no stream burst is in flight there.

module r3d_mstream (
  input  logic        clk,
  input  logic        rst,

  input  logic        start,
  input  logic [23:0] addr,
  input  logic        stop,
  output logic        ready_start,     // idle: a new model may start

  output logic        w_valid,
  output logic [31:0] w_data,
  input  logic        w_pop,

  input  logic        c_req,           // colour read
  input  logic [19:0] c_addr,
  output logic        c_valid,
  output logic [31:0] c_data,

  output logic        pram_req,
  output logic [23:0] pram_addr,
  output logic [2:0]  pram_len,
  input  logic        pram_ack,
  input  logic        pram_rvalid,
  input  logic [31:0] pram_rdata,

  output logic        vrom_req,
  output logic [23:0] vrom_addr,
  output logic [2:0]  vrom_len,
  input  logic        vrom_ack,
  input  logic        vrom_rvalid,
  input  logic [31:0] vrom_rdata
);
  localparam int FD = 32;               // one MLAB deep: three bursts in flight cover 12-20 cycles

  logic        active, draining, in_vrom;
  logic [24:0] next;             // next word to request (index in the region)
  // the region ends at 2^20 (polygon RAM) or 2^24 (VROM) words: next is past
  // it when those bits overflow, and within eight words of it when the bits
  // between are all ones (then 2^k - next = 8 - next[2:0])
  logic        past_end, near_end;
  assign past_end = in_vrom ? next[24] : (next[24:20] != 5'd0);
  assign near_end = in_vrom ? &next[23:3] : &next[19:3];
  logic [2:0]  outst;            // bursts in flight
  logic [6:0]  count;            // FIFO occupancy
  logic [4:0]  wp, rp;
  (* ramstyle = "MLAB, no_rw_check" *) logic [31:0] fifo [0:FD-1];
  // valid words of each burst in flight (8, or fewer at the region end)
  logic [3:0]  bvalid [0:3];
  logic [1:0]  bw, br;
  logic [2:0]  beat;
  logic        creq_pending, c_issued;

  logic        rv;
  logic [31:0] rd;
  assign rv = in_vrom ? vrom_rvalid : pram_rvalid;
  assign rd = in_vrom ? vrom_rdata : pram_rdata;

  assign ready_start = !active && !draining && outst == 3'd0 && !pram_req && !vrom_req;
  assign w_valid = active && count != 7'd0;
  assign w_data = fifo[rp];

  // space for one more burst beyond those in flight
  logic room;
  assign room = ({1'b0, count} + {1'b0, outst, 3'b000} + 8'd8) <= 8'(FD);

  always_ff @(posedge clk) begin
    c_valid <= 1'b0;
    if (rst) begin
      active <= 1'b0; draining <= 1'b0; outst <= '0; count <= '0;
      wp <= '0; rp <= '0; bw <= '0; br <= '0; beat <= '0;
      pram_req <= 1'b0; vrom_req <= 1'b0;
      creq_pending <= 1'b0; c_issued <= 1'b0;
    end else begin
      logic push, pop;
      logic [31:0] pd;
      push = 1'b0; pd = 32'd0;
      pop = w_pop && w_valid;

      if (c_req) creq_pending <= 1'b1;

      // ---- start / stop
      if (start && ready_start) begin
        active <= 1'b1;
        in_vrom <= (addr >= 24'h10_0000);
        next <= {1'b0, addr};
        count <= '0; wp <= '0; rp <= '0;
        pop = 1'b0;
      end
      if (stop && active) begin
        active <= 1'b0;
        draining <= 1'b1;
        pop = 1'b0;
      end

      // ---- issue a burst, or synthesise zeros past the region end
      if (active && !stop && !pram_req && !vrom_req && room && outst != 3'd4 && !(creq_pending && !in_vrom)) begin
        if (past_end) begin
          // zeros: one word a cycle straight into the FIFO
          if (count != 7'(FD)) begin push = 1'b1; pd = 32'd0; end
        end else begin
          if (in_vrom) begin vrom_req <= 1'b1; vrom_addr <= next[23:0]; vrom_len <= 3'd7; end
          else begin pram_req <= 1'b1; pram_addr <= next[23:0]; pram_len <= 3'd7; end
          bvalid[bw] <= near_end ? 4'(4'd8 - {1'b0, next[2:0]}) : 4'd8;
        end
      end
      if (pram_req && pram_ack) begin
        pram_req <= 1'b0;
        if (c_issued) ; else begin bw <= bw + 2'd1; outst <= outst + 3'd1; next <= next + 25'd8; end
      end
      if (vrom_req && vrom_ack) begin
        vrom_req <= 1'b0;
        bw <= bw + 2'd1; outst <= outst + 3'd1; next <= next + 25'd8;
      end

      // ---- colour read: polygon RAM, when no stream burst is in flight there
      if (creq_pending && !c_issued && !pram_req && (in_vrom || outst == 3'd0 || !active)) begin
        pram_req <= 1'b1; pram_addr <= {4'd0, c_addr}; pram_len <= 3'd0; c_issued <= 1'b1;
      end
      if (c_issued && !pram_req && pram_rvalid && (in_vrom || outst == 3'd0 || !active)) begin
        c_valid <= 1'b1; c_data <= pram_rdata; c_issued <= 1'b0; creq_pending <= 1'b0;
      end

      // ---- returning beats of stream bursts
      if (rv && outst != 3'd0 && !(c_issued && !in_vrom)) begin
        if (!draining && active && {1'b0, beat} < bvalid[br]) begin
          push = 1'b1; pd = rd;
        end else if (!draining && active) begin
          push = 1'b1; pd = 32'd0;                        // past the region end
        end
        beat <= beat + 3'd1;
        if (beat == 3'd7) begin
          br <= br + 2'd1;
          outst <= outst - 3'd1 + ((pram_req && pram_ack && !c_issued) || (vrom_req && vrom_ack) ? 3'd1 : 3'd0);
          if (draining && outst == 3'd1) draining <= 1'b0;
        end
      end

      if (push) begin fifo[wp] <= pd; wp <= wp + 5'd1; end
      if (pop) rp <= rp + 5'd1;
      if (!(start && ready_start)) count <= count + {6'd0, push} - {6'd0, pop};
      if (draining && outst == 3'd0) draining <= 1'b0;
    end
  end
endmodule
