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
// M2.5 viewer: the DDR3 probe (study D-33, D-34). On the board it measures
// what texture-cache fills will see through m3_ddr3p's pipelined read port,
// in three cases:
//
//   (a) idle: the viewer not drawing and the layer's scan-out paused (so_off,
//       when the OSD asks for it: the screen shows the overlay on black)
//   (b) the viewer not drawing, the layer scanned out
//   (c) the viewer drawing: layer writes, scan-out, polygon RAM, the store
//
// Tests, one at a time, in a loop:
//
//   latency   2048 one-beat reads, one at a time, at random words of an
//             8 MiB region: each read's cycles from its command on the pins
//             to its first beat (m3_ddr3p's debug; Model 2 R351's measure
//             and set-up, one word, one read at a time). Minimum, median,
//             99th percentile, maximum of the batch.
//   in flight 32 one-beat reads back to back: the most the bridge took before
//             the first beat came back; 32 eight-beat reads back to back: the
//             most reads the bridge held at once (32 is the master's cap)
//   bandwidth 4-beat reads (a texture-cache line, 32 B) at random lines of the
//             region, at most N = 1, 4, 8 or 16 of the probe's own in flight,
//             for 800,000 cycles: beats in that window, which at 100 MHz is
//             MB/s x 1000
//
// A test belongs to the case at its start and is dropped if the case changes
// before it ends. Every beat is checked against the pattern the probe writes
// over the region first (through port L, m2_ddr3's contract), so order and
// routing are tested at every depth.
//
// The results sit in a 32-word MLAB, binary, and go into decimal digits (a
// time-shared double dabble) after each loop, into a 32-word MLAB the overlay
// reads (rd_idx, the value the cycle after). Index:
//
//   8c + 0..3  case c latency: min, median, p99, max (cycles at the pins)
//   8c + 4..7  case c bandwidth at N = 1, 4, 8, 16 (MB/s x 1000)
//   24         reads taken before the first beat (most seen)
//   25         reads held at once (most seen)
//   26         beats that did not match the pattern
//   27         the slowest first beat of any read, any client (m3_ddr3p)
//   28 + c     latency batches kept, case c
//   31         loops
//
// One sequencer, one read port of each memory and one converter serve all of
// it (study 8.4: time-shared).

module m3v_probe #(
  parameter logic [24:0] RBASE = 25'h120_0000,   // the region, 64-bit words from the DDR3 base
  parameter int          RB    = 20,             // log2 words in it (8 MiB, the texture RAM's size)
  parameter logic [3:0]  SRC   = 4'd1,           // the probe's port in m3_ddr3p's debug
  parameter int          WIN   = 800000,         // bandwidth window, cycles
  parameter int          NLAT  = 2048            // reads a latency batch
) (
  input  logic        clk,
  input  logic        rst,
  input  logic        en,
  input  logic        pause,         // the OSD: pause the scan-out while idle, case (a)
  input  logic        drawing,
  output logic        so_off,        // the layer's scan-out held off

  // port L: the pattern's write bursts
  output logic        w_req,
  output logic [24:0] w_addr,
  output logic [7:0]  w_blen,
  output logic [63:0] w_din,
  input  logic        w_wacc,
  input  logic        w_ack,

  // port P: reads
  output logic        p_req,
  output logic [24:0] p_addr,
  output logic [7:0]  p_blen,
  input  logic        p_ack,
  input  logic        p_rvalid,
  input  logic [63:0] p_dout,

  // m3_ddr3p's debug
  input  logic        lat_v,
  input  logic [15:0] lat,
  input  logic [3:0]  lat_src,
  input  logic [15:0] lat_max,
  input  logic [5:0]  acc,

  // the results in decimal digits
  input  logic [4:0]  rd_idx,
  output logic [31:0] rd_val,
  output logic        busy
);

  function automatic logic [63:0] patt(input logic [RB-1:0] o);
    logic [24:0] a;
    a = 25'(o);
    patt = {7'h5A, a, 7'h3C, ~a};
  endfunction

  assign so_off = en && pause && !drawing;
  wire [1:0] case_now = drawing ? 2'd2 : so_off ? 2'd0 : 2'd1;

  // ===========================================================================
  // memories
  // ===========================================================================
  // the results, binary
  (* ramstyle = "MLAB, no_rw_check" *) logic [23:0] st [32] /* verilator public_flat_rd */;
  logic        st_we;
  logic [4:0]  st_wa, st_ra;
  logic [23:0] st_wd;
  wire  [23:0] st_q = st[st_ra];
  always_ff @(posedge clk) if (st_we) st[st_wa] <= st_wd;

  // the results, decimal, for the overlay
  (* ramstyle = "MLAB, no_rw_check" *) logic [31:0] dsp [32];
  logic        ds_we;
  logic [4:0]  ds_wa;
  logic [31:0] ds_wd;
  always_ff @(posedge clk) begin
    if (ds_we) dsp[ds_wa] <= ds_wd;
    rd_val <= dsp[rd_idx];
  end

  // the latency histogram of a batch, 256 bins (255: 255 and over)
  (* ramstyle = "M10K, no_rw_check" *) logic [11:0] hist [256];
  logic        h_we;
  logic [7:0]  h_wa, h_ra;
  logic [11:0] h_wd, h_q;
  always_ff @(posedge clk) begin
    if (h_we) hist[h_wa] <= h_wd;
    h_q <= hist[h_ra];
  end

  // start offsets of the probe's reads in flight, for the pattern check
  (* ramstyle = "MLAB, no_rw_check" *) logic [RB-1:0] af [32];
  logic [5:0]  af_wp, af_rp;
  logic [7:0]  rbeat;                // beats of the oldest read received
  wire  [RB-1:0] ra_rand;
  wire         took;
  always_ff @(posedge clk) if (took) af[af_wp[4:0]] <= ra_rand;
  wire  [RB-1:0] af_h = af[af_rp[4:0]];

  // ===========================================================================
  // the sequencer
  // ===========================================================================
  typedef enum logic [4:0] {
    S_IDLE, S_CLR, S_WR, S_WRW,
    S_LAT, S_LATW, S_SCAN, S_STL,
    S_INF, S_INFW, S_STI,
    S_BW, S_BWW,
    S_CONV, S_CONVR, S_CONVS, S_CONVW
  } st_t;
  st_t s;

  logic [31:0] lfsr;
  logic [RB:0] wa;                   // pattern: words written
  logic [7:0]  wb;                   // beats of this burst taken
  logic [1:0]  cls;                  // the test's case
  logic        dirty;
  logic [11:0] n, nd;                // a batch's reads issued, done
  logic [15:0] bmin, bmax;
  logic [5:0]  oc;                   // the probe's reads in flight
  logic [5:0]  cap;
  logic [7:0]  blen;
  logic        issue;
  logic [1:0]  hk;                   // histogram update step
  logic [8:0]  sb;                   // scan bin
  logic [11:0] cum;
  logic        f50, f99;
  logic [7:0]  p50, p99;
  logic [5:0]  ntot, ndone;
  logic        inf8, seen1;
  logic [5:0]  accmx, i_first, i_held;
  logic [1:0]  bk;                   // bandwidth test: N = 1, 4, 8, 16
  logic [19:0] wcnt, beats;
  logic [15:0] errs;
  logic [3:0]  uk;                   // results update step
  logic [4:0]  cj;                   // conversion: value
  logic [4:0]  ck;                   // conversion: bit
  logic [55:0] dd;                   // {BCD, binary}

  assign busy   = (s != S_IDLE);
  assign w_blen = 8'd128;
  assign w_addr = RBASE + 25'(wa);
  assign w_din  = patt(RB'(wa) + RB'(wb));
  assign p_blen = blen;

  // a random word, line or 8-word block
  wire   [RB-1:0] bmask = RB'(blen - 8'd1);
  assign ra_rand = lfsr[RB-1:0] & ~bmask;
  assign p_addr  = RBASE + 25'(ra_rand);
  assign p_req   = issue && (oc < cap);
  assign took    = p_req && p_ack;
  wire   r_done  = p_rvalid && (rbeat + 8'd1 == blen);

  function automatic logic [3:0] add3(input logic [3:0] d);
    add3 = (d >= 4'd5) ? d + 4'd3 : d;
  endfunction

  // a results update: entry a, value v
`define M3V_PUT(a, v) begin st_we <= 1'b1; st_wa <= (a); st_wd <= (v); end

  always_ff @(posedge clk) begin
    if (rst) begin
      s <= S_IDLE; lfsr <= 32'hACE1_2468; w_req <= 1'b0; issue <= 1'b0;
      st_we <= 1'b0; ds_we <= 1'b0; h_we <= 1'b0; hk <= '0;
      oc <= '0; af_wp <= '0; af_rp <= '0; rbeat <= '0; errs <= '0;
      cap <= 6'd1; blen <= 8'd1;
    end else begin
      st_we <= 1'b0; ds_we <= 1'b0; h_we <= 1'b0;
      if (took) lfsr <= {lfsr[30:0], 1'b0} ^ (lfsr[31] ? 32'h0400_0007 : 32'd0);

      // ---- reads in flight: the pattern check, in order
      if (took) af_wp <= af_wp + 6'd1;
      if (p_rvalid) begin
        if (p_dout != patt(af_h + RB'(rbeat)) && errs != 16'hffff) errs <= errs + 16'd1;
        if (r_done) begin rbeat <= '0; af_rp <= af_rp + 6'd1; end
        else rbeat <= rbeat + 8'd1;
      end
      oc <= oc + 6'(took) - 6'(r_done);

      case (s)
        S_IDLE: if (en) begin st_wa <= '0; st_we <= 1'b1; st_wd <= '0; s <= S_CLR; end

        // ---- the results zeroed, then the pattern over the region
        S_CLR: begin
          st_we <= 1'b1; st_wa <= st_wa + 5'd1; st_wd <= '0;
          if (st_wa == 5'd31) begin st_we <= 1'b0; wa <= '0; s <= S_WR; end
        end
        S_WR: begin wb <= '0; w_req <= 1'b1; s <= S_WRW; end
        S_WRW: begin
          if (w_wacc) wb <= wb + 8'd1;
          if (w_ack) begin
            w_req <= 1'b0;
            wa <= wa + (RB+1)'(128);
            if (wa + (RB+1)'(128) == (RB+1)'(1 << RB)) s <= S_LAT;
            else s <= S_WR;
          end
        end

        // ---- latency: one read at a time
        S_LAT: if (en) begin
          cls <= case_now; dirty <= 1'b0; n <= '0; nd <= '0; hk <= '0;
          bmin <= 16'hffff; bmax <= '0; blen <= 8'd1; cap <= 6'd1; issue <= 1'b1;
          s <= S_LATW;
        end
        S_LATW: begin
          if (case_now != cls) dirty <= 1'b1;
          if (took) begin
            n <= n + 12'd1;
            if (n == 12'(NLAT - 1)) issue <= 1'b0;
          end
          // the latency comes with the read's first beat; its bin is read now
          // and written two cycles on (reads are several cycles apart)
          hk <= '0;
          if (lat_v && lat_src == SRC) begin
            h_ra <= (lat > 16'd255) ? 8'd255 : lat[7:0];
            if (lat < bmin) bmin <= lat;
            if (lat > bmax) bmax <= lat;
            nd <= nd + 12'd1;
            hk <= 2'd1;
          end else if (hk == 2'd1) hk <= 2'd2;
          else if (hk == 2'd2) begin
            h_we <= 1'b1; h_wa <= h_ra; h_wd <= h_q + 12'd1;
            if (nd == 12'(NLAT)) begin sb <= '0; cum <= '0; f50 <= 1'b0; f99 <= 1'b0; s <= S_SCAN; end
          end
        end
        // the median and p99; the bins cleared behind
        S_SCAN: begin
          logic [11:0] c;
          sb <= sb + 9'd1;
          h_ra <= sb[7:0];
          if (sb >= 9'd2) begin
            c = cum + h_q;
            cum <= c;
            if (!f50 && c >= 12'(NLAT / 2)) begin f50 <= 1'b1; p50 <= 8'(sb - 9'd2); end
            if (!f99 && c >= 12'(NLAT - NLAT / 100)) begin f99 <= 1'b1; p99 <= 8'(sb - 9'd2); end
            h_we <= 1'b1; h_wa <= 8'(sb - 9'd2); h_wd <= '0;
          end
          if (sb == 9'd257) begin uk <= '0; s <= S_STL; end
        end
        // the batch into the results: read an entry, write it the next cycle
        S_STL: begin
          logic [4:0] b;
          b = {cls, 3'd0};
          uk <= uk + 4'd1;
          case (uk)
            4'd0: st_ra <= b;
            4'd1: begin if (!dirty) `M3V_PUT(b, (st_q == '0 || 24'(bmin) < st_q) ? 24'(bmin) : st_q) st_ra <= b + 5'd3; end
            4'd2: begin if (!dirty) `M3V_PUT(b + 5'd1, 24'(p50)) end
            4'd3: begin if (!dirty) `M3V_PUT(b + 5'd2, 24'(p99)) end
            4'd4: begin if (!dirty) `M3V_PUT(b + 5'd3, (24'(bmax) > st_q) ? 24'(bmax) : st_q) st_ra <= 5'd28 + 5'(cls); end
            4'd5: begin if (!dirty) `M3V_PUT(5'd28 + 5'(cls), st_q + 24'd1) end
            default: begin `M3V_PUT(5'd27, 24'(lat_max)) inf8 <= 1'b0; s <= S_INF; end
          endcase
        end

        // ---- in flight: 32 reads back to back
        S_INF: begin
          blen <= inf8 ? 8'd8 : 8'd1; cap <= 6'd32; issue <= 1'b1;
          ntot <= '0; ndone <= '0; seen1 <= 1'b0; accmx <= '0;
          s <= S_INFW;
        end
        S_INFW: begin
          if (took) begin
            ntot <= ntot + 6'd1;
            if (ntot == 6'd31) issue <= 1'b0;
          end
          if (p_rvalid) seen1 <= 1'b1;
          if ((inf8 || (!seen1 && !p_rvalid)) && acc > accmx) accmx <= acc;
          if (r_done) begin
            ndone <= ndone + 6'd1;
            if (ndone == 6'd31) begin
              if (inf8) begin i_held <= accmx; uk <= '0; s <= S_STI; end
              else begin i_first <= accmx; inf8 <= 1'b1; s <= S_INF; end
            end
          end
        end
        S_STI: begin
          uk <= uk + 4'd1;
          case (uk)
            4'd0: st_ra <= 5'd24;
            4'd1: begin `M3V_PUT(5'd24, (24'(i_first) > st_q) ? 24'(i_first) : st_q) st_ra <= 5'd25; end
            4'd2: `M3V_PUT(5'd25, (24'(i_held) > st_q) ? 24'(i_held) : st_q)
            default: begin `M3V_PUT(5'd26, 24'(errs)) bk <= 2'd0; s <= S_BW; end
          endcase
        end

        // ---- bandwidth at N in flight
        S_BW: begin
          cls <= case_now; dirty <= 1'b0;
          blen <= 8'd4; cap <= (bk == 2'd0) ? 6'd1 : (bk == 2'd1) ? 6'd4 : (bk == 2'd2) ? 6'd8 : 6'd16;
          issue <= 1'b1; wcnt <= 20'(WIN - 1); beats <= '0;
          s <= S_BWW;
        end
        S_BWW: begin
          if (case_now != cls) dirty <= 1'b1;
          if (issue) begin
            if (p_rvalid) beats <= beats + 20'd1;
            wcnt <= wcnt - 20'd1;
            if (wcnt == '0) issue <= 1'b0;
          end else if (oc == '0 && !p_rvalid) begin
            if (!dirty) `M3V_PUT({cls, 1'b1, bk}, 24'(beats))
            if (bk == 2'd3) begin st_ra <= 5'd31; uk <= '0; cj <= '0; s <= S_CONV; end
            else begin bk <= bk + 2'd1; s <= S_BW; end
          end
        end

        // ---- the loop count, then the results into decimal digits, a value at a time
        S_CONV: begin
          if (uk == '0) begin uk <= 4'd1; end                          // entry 31 read
          else if (uk == 4'd1) begin `M3V_PUT(5'd31, st_q + 24'd1) uk <= 4'd2; end
          else begin st_ra <= cj; s <= S_CONVR; end
        end
        S_CONVR: begin dd <= {32'd0, st_q}; ck <= 5'd0; s <= S_CONVS; end
        S_CONVS: begin
          logic [31:0] b;
          for (int k = 0; k < 8; k++) b[4*k +: 4] = add3(dd[24 + 4*k +: 4]);
          dd <= {b, dd[23:0]} << 1;
          ck <= ck + 5'd1;
          if (ck == 5'd23) s <= S_CONVW;
        end
        S_CONVW: begin
          ds_we <= 1'b1; ds_wa <= cj; ds_wd <= dd[55:24];
          cj <= cj + 5'd1;
          if (cj == 5'd31) s <= S_LAT;
          else begin st_ra <= cj + 5'd1; s <= S_CONVR; end
        end
        default: s <= S_IDLE;
      endcase
    end
  end

endmodule
`undef M3V_PUT
