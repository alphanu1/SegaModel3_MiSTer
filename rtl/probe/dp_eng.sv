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
// The DDR3 probe's engine, on the DDR3 clock (the variable PLL: 100, 125, 85
// or 62.5 MHz): m3_ddr3p, the background clients, the traffic generator and the
// measurement. dp_core, on the fixed clock, prints what it reports.
//
// After its reset it waits for the master to be idle (nothing owed from before
// the reset), writes a pattern over an 8 MiB region in 32-beat bursts, then
// runs windows of WIN cycles (one second at the clock in use):
//
//   automatic  17 cases (dp_core's table): 4-beat random reads at N = 1, 2,
//              4, 8, 16 in flight, with no background, with the layer (a
//              sequential 32-beat writer and reader at 22 MB/s each) and with
//              the layer and the store (and a sequential reader at 100 MB/s);
//              sequential 4-beat reads at 16; random 8-beat reads at 32
//   manual     one setting, window after window: read or write, random or
//              sequential, 1-128 beats, 1-32 in flight, the background
//
// Per window: bytes moved by the probe, latency from the command's first cycle
// on the pins to its first beat (a read) or to its first beat taken (a write)
// as min, median (512-bin histogram) and max, the most of the probe's reads
// the bridge held at once, and the background's bytes. Every read beat is
// checked against the pattern; writes write the pattern.
//
// Diagnostics (status bits, quasi-static): s_skip skips the pattern (reads
// whatever is there; errors are not counted); s_lpath sends the probe's reads
// through port L, one at a time on m2_ddr3's contract, instead of the pipelined
// port; s_init1 writes the pattern a beat at a time.
//
// Crossings to dp_core: a window's results stay put from r_tgl's toggle until
// r_ack matches it; the progress counters are copied into d_* when d_req
// toggles, and d_ack follows. The settings are quasi-static (a change resets
// this engine), synchronised here.

`ifdef PROBE_M2
`define DP_M2 1'b1
`else
`define DP_M2 1'b0
`endif
`ifdef PROBE_MAXQ
`define DP_MAXQ `PROBE_MAXQ
`else
`define DP_MAXQ 1
`endif
module dp_eng #(
  // M2 = 1 (macro PROBE_M2): Model 2's m2_ddr3 itself, unchanged, in place of
  // m3_ddr3p behind port L; the probe's reads then go through port L one at a
  // time, and latency is measured at the client (request to first beat)
  parameter bit          M2       = `DP_M2,
  // the most reads the master keeps in flight (m3_ddr3p's MAXQ; macro
  // PROBE_MAXQ, 1 by default while it is proved on the board a step at a time):
  // the sweep skips the cases above it
  parameter int          MAXQ     = M2 ? 1 : `DP_MAXQ,
  parameter logic [28:0] DDR_BASE = 29'h0600_0000,   // byte 0x30000000 (Model 2 R654)
  parameter int          RB       = 20,              // the region: 2^RB words (8 MiB)
  parameter int          BGB      = 32,              // background and pattern bursts, beats
  parameter int          NCASE    = 17,
  parameter int          WIN0     = 100_000_000,     // window cycles at clock setting 0..3
  parameter int          WIN1     = 125_000_000,
  parameter int          WIN2     = 85_000_000,
  parameter int          WIN3     = 62_500_000,
  parameter int          PL0 = 1164, PL1 = 1455, PL2 = 989, PL3 = 727,   // a 256-byte burst at 22 MB/s
  parameter int          PS0 = 256,  PS1 = 320,  PS2 = 218, PS3 = 160    // ... at 100 MB/s
) (
  input  logic        clk,
  input  logic        mem_rst_n,     // the clock locked: the master's only reset
  input  logic        rst_req,       // the engine held in reset (from dp_core, any clock)

  // settings (quasi-static, from dp_core)
  input  logic        s_manual, s_wr, s_seq,
  input  logic [2:0]  s_burst, s_infl,
  input  logic [1:0]  s_bg, s_clk,
  input  logic        s_skip, s_lpath, s_init1,

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

  output logic        m_idle_o,      // the master idle (to dp_core, synchronised there)

  // a window's results
  output logic        r_tgl,
  input  logic        r_ack,
  output logic [4:0]  r_k,
  output logic [5:0]  r_n,
  output logic [1:0]  r_bg,
  output logic [7:0]  r_ln,
  output logic        r_seq, r_wr,
  output logic [31:0] r_bytes, r_bgb,
  output logic [15:0] r_lmin, r_lmax,
  output logic [8:0]  r_lmed,
  output logic [5:0]  r_ac,
  output logic [15:0] r_pass, r_err,

  // progress, copied on request
  input  logic        d_req,
  output logic        d_ack,
  output logic [31:0] d_iw, d_rd, d_bt, d_busy,
  output logic [3:0]  d_st,
  output logic [6:0]  d_acc, d_oc,
  output logic [31:0] d_rdy, d_lrv,
  output logic [7:0]  d_mst,
  output logic [31:0] d_rq
);

  // ---- synchronisers -----------------------------------------------------------------
  (* altera_attribute = "-name SYNCHRONIZER_IDENTIFICATION FORCED_IF_ASYNCHRONOUS" *) logic [2:0] rr_s;
  always_ff @(posedge clk) rr_s <= {rr_s[1:0], rst_req};
  wire rst_n = mem_rst_n && !rr_s[2];
  logic [16:0] set_s1, set_s2;
  always_ff @(posedge clk) begin set_s1 <= {s_skip, s_lpath, s_init1, s_manual, s_wr, s_seq, s_burst, s_infl, s_bg, s_clk, 1'b0}; set_s2 <= set_s1; end
  wire        skip = set_s2[16], lpath = set_s2[15] || M2, init1 = set_s2[14];
  wire        manual = set_s2[13], wr_m = set_s2[12], seq_m = set_s2[11];
  wire [2:0]  burst_c = set_s2[10:8], infl_c = set_s2[7:5];
  wire [1:0]  bg_m = set_s2[4:3], clk_c = set_s2[2:1];
  logic [2:0] ra_s, dq_s;
  always_ff @(posedge clk) begin ra_s <= {ra_s[1:0], r_ack}; dq_s <= {dq_s[1:0], d_req}; end

  function automatic logic [63:0] patt(input logic [RB-1:0] o);
    logic [24:0] a;
    a = 25'(o);
    patt = {7'h5A, a, 7'h3C, ~a};
  endfunction

  // ===========================================================================
  // the master: port L behind three background clients, P port 0 the probe
  // ===========================================================================
  // port L's clients: 0 the writer, 1 the layer reader, 2 the store reader,
  // 3 the probe's reads when s_lpath
  logic [3:0]       b_req, b_we, b_wacc, b_rvalid, b_ack;
  logic [3:0][24:0] b_addr;
  logic [3:0][7:0]  b_blen;
  logic [3:0][63:0] b_din;
  logic        m_req, m_we, m_wacc, m_rvalid, m_ack, m_wnext;
  logic [24:0] m_addr;
  logic [7:0]  m_blen, m_be;
  logic [63:0] m_din, dout, p_din;
  logic        p_req, p_ack, p_rvalid, p_wacc, p_we;
  logic [24:0] p_addr;
  logic [7:0]  p_blen;
  logic        lat_v, take, done, m_idle;
  logic [15:0] lat;
  logic [3:0]  lat_src, take_src, done_src;
  logic [6:0]  m_acc;
  logic [7:0]  m_st;
  logic [31:0] m_rdy;

  m3v_ddr_arb #(.NP(4)) u_arb (
    .clk, .rst(!rst_n),
    .req(b_req), .we(b_we), .addr(b_addr), .blen(b_blen), .din(b_din),
    .wacc(b_wacc), .rvalid(b_rvalid), .ack(b_ack),
    .m_req, .m_we, .m_addr, .m_blen, .m_din, .m_be,
    .m_wacc, .m_rvalid, .m_ack);

  logic [31:0] m_rq;
  generate if (M2) begin : g_m2
    m2_ddr3 u_ddr3 (
      .clk, .rst_n(mem_rst_n), .base(DDR_BASE),
      .req(m_req), .we(m_we), .addr(m_addr), .blen(m_blen), .din(m_din), .be(m_be),
      .wnext(m_wnext), .wacc(m_wacc), .rvalid(m_rvalid), .ack(m_ack), .dout,
      .DDRAM_CLK, .DDRAM_BUSY, .DDRAM_BURSTCNT, .DDRAM_ADDR, .DDRAM_DIN, .DDRAM_BE,
      .DDRAM_WE, .DDRAM_RD, .DDRAM_DOUT, .DDRAM_DOUT_READY,
      .dbg_lat_last(), .dbg_lat_max(), .dbg_inflight_max(), .dbg_stuck_wr(), .dbg_acks());
    assign p_ack = 1'b0; assign p_rvalid = 1'b0; assign p_wacc = 1'b0;
    assign lat_v = 1'b0; assign lat = '0; assign lat_src = '0;
    assign take = 1'b0; assign take_src = '0; assign done = 1'b0; assign done_src = '0;
    assign m_acc = '0; assign m_st = {3'd0, m_req, 4'd0}; assign m_rq = '0;
    assign m_idle = !m_req;
    always_ff @(posedge clk or negedge mem_rst_n)
      if (!mem_rst_n) m_rdy <= '0; else if (DDRAM_DOUT_READY) m_rdy <= m_rdy + 32'd1;
  end else begin : g_m3
    wire [5:0] acc;
    assign m_acc = {1'b0, acc};
    m3_ddr3p #(.PIPE(1'b1), .NP(1), .QB(5), .RQREG(1'b1), .MAXQ(MAXQ)) u_ddr3 (
      .clk, .rst_n(mem_rst_n), .base(DDR_BASE),
      .req(m_req), .we(m_we), .addr(m_addr), .blen(m_blen), .din(m_din), .be(m_be),
      .wnext(m_wnext), .wacc(m_wacc), .rvalid(m_rvalid), .ack(m_ack),
      .p_req, .p_addr, .p_blen, .p_we, .p_din, .p_ack, .p_rvalid, .p_wacc, .dout,
      .DDRAM_CLK, .DDRAM_BUSY, .DDRAM_BURSTCNT, .DDRAM_ADDR, .DDRAM_DIN, .DDRAM_BE,
      .DDRAM_WE, .DDRAM_RD, .DDRAM_DOUT, .DDRAM_DOUT_READY,
      .dbg_lat_v(lat_v), .dbg_lat(lat), .dbg_lat_src(lat_src), .dbg_lat_max(),
      .dbg_acc(acc), .dbg_acc_max(),
      .dbg_take(take), .dbg_take_src(take_src), .dbg_done(done), .dbg_done_src(done_src),
      .dbg_idle(m_idle), .dbg_st(m_st), .dbg_rdy(m_rdy), .dbg_rq(m_rq));
  end endgenerate
  assign m_idle_o = m_idle;

  // the latency histogram of a window: 512 bins (511: 511 and over)
  (* ramstyle = "M10K, no_rw_check" *) logic [31:0] hist [512];
  logic        h_we;
  logic [8:0]  h_wa, h_ra;
  logic [31:0] h_wd, h_q;
  always_ff @(posedge clk) begin
    if (h_we) hist[h_wa] <= h_wd;
    h_q <= hist[h_ra];
  end

  // start offsets of the probe's bursts in flight: the pattern a read is
  // checked against, or a write writes
  (* ramstyle = "MLAB, no_rw_check" *) logic [RB-1:0] af [64];
  logic [6:0]    af_wp, af_rp;
  logic [7:0]    rbeat;
  logic [RB-1:0] ra;
  wire           took;
  always_ff @(posedge clk) if (took) af[af_wp[5:0]] <= ra;
  // read synchronously, a cycle ahead, the write forwarded (as m3_ddr3p's queue)
  logic [RB-1:0] af_h;
  wire           af_pop;
  wire  [6:0]    ap_next = af_rp + 7'(af_pop);
  always_ff @(posedge clk) af_h <= (took && af_wp[5:0] == ap_next[5:0]) ? ra : af[ap_next[5:0]];

  // live: the master idle once since the reset (nothing owed from before)
  logic live;
  always_ff @(posedge clk) begin
    if (!rst_n) live <= 1'b0;
    else if (m_idle) live <= 1'b1;
  end
  wire pv = (lpath ? b_rvalid[3] : p_rvalid) && live;   // a read beat of the probe's own
  wire pw = p_wacc && live;            // a write beat of the probe's taken

  // ===========================================================================
  // the background clients (arbiter ports 0 writer, 1 layer reader, 2 store
  // reader), a BGB-beat burst a period; port 0 writes the pattern first
  // ===========================================================================
  localparam int BB = $clog2(BGB);
  logic        bg_on, st_on, init_wr, init_done;
  logic [RB:0] iw;
  logic [7:0]  ib;
  logic [2:0]  cr0, cr1, cr2;
  logic [11:0] pc_l;
  logic [9:0]  pc_s;
  logic [17-BB:0] wa_bg;
  logic [RB-BB-1:0] ra1, ra2;
  logic        bg_cnt, bg_clr;
  logic [31:0] bg_beats;
  wire  [11:0] p_l = (clk_c == 2'd1) ? 12'(PL1) : (clk_c == 2'd2) ? 12'(PL2) : (clk_c == 2'd3) ? 12'(PL3) : 12'(PL0);
  wire  [9:0]  p_s = (clk_c == 2'd1) ? 10'(PS1) : (clk_c == 2'd2) ? 10'(PS2) : (clk_c == 2'd3) ? 10'(PS3) : 10'(PS0);

  assign b_we = 4'b0001;
  assign b_blen[0] = (init_wr && init1) ? 8'd1 : 8'(BGB);
  assign b_blen[1] = 8'(BGB);
  assign b_blen[2] = 8'(BGB);
  assign b_addr[0] = init_wr ? 25'(iw) : 25'h20_0000 + 25'({wa_bg, BB'(0)});   // the writer's region: 1 MiB at word 0x200000
  assign b_addr[1] = 25'({ra1, BB'(0)});
  assign b_addr[2] = 25'({ra2, BB'(0)});
  assign b_din[0]  = init_wr ? patt(RB'(iw) + RB'(ib)) : 64'h0123_4567_89AB_CDEF;
  assign b_din[1]  = '0;
  assign b_din[2]  = '0;

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      b_req[2:0] <= '0; cr0 <= '0; cr1 <= '0; cr2 <= '0; pc_l <= '0; pc_s <= '0;
      iw <= '0; ib <= '0; init_wr <= 1'b1; init_done <= 1'b0;
      wa_bg <= '0; ra1 <= '0; ra2 <= '0; bg_beats <= '0;
    end else begin
      pc_l <= (pc_l >= p_l - 12'd1) ? '0 : pc_l + 12'd1;
      pc_s <= (pc_s >= p_s - 10'd1) ? '0 : pc_s + 10'd1;
      begin
        logic [2:0] n0, n1, n2;
        n0 = cr0 + 3'((pc_l == '0) && cr0 != 3'd4) - 3'(b_req[0] && b_ack[0] && !init_wr && cr0 != '0);
        n1 = cr1 + 3'((pc_l == '0) && cr1 != 3'd4) - 3'(b_req[1] && b_ack[1] && cr1 != '0);
        n2 = cr2 + 3'((pc_s == '0) && cr2 != 3'd4) - 3'(b_req[2] && b_ack[2] && cr2 != '0);
        cr0 <= bg_on ? n0 : '0;
        cr1 <= bg_on ? n1 : '0;
        cr2 <= st_on ? n2 : '0;
      end
      if (b_req[0] && b_wacc[0]) ib <= ib + 8'd1;
      if (b_req[0] && b_ack[0]) begin
        b_req[0] <= 1'b0; ib <= '0;
        if (init_wr) begin
          iw <= iw + (init1 ? (RB+1)'(1) : (RB+1)'(BGB));
          if (iw + (init1 ? (RB+1)'(1) : (RB+1)'(BGB)) == (RB+1)'(1 << RB)) begin init_wr <= 1'b0; init_done <= 1'b1; end
        end else wa_bg <= wa_bg + 1'b1;
      end else if (!b_req[0] && !b_ack[0] && live && ((init_wr && !skip) || (bg_on && cr0 != '0))) b_req[0] <= 1'b1;
      if (skip && init_wr && live) begin init_wr <= 1'b0; init_done <= 1'b1; end
      if (b_req[1] && b_ack[1]) begin b_req[1] <= 1'b0; ra1 <= ra1 + 1'b1; end
      else if (!b_req[1] && !b_ack[1] && live && bg_on && cr1 != '0) b_req[1] <= 1'b1;
      if (b_req[2] && b_ack[2]) begin b_req[2] <= 1'b0; ra2 <= ra2 - 1'b1; end
      else if (!b_req[2] && !b_ack[2] && live && st_on && cr2 != '0) b_req[2] <= 1'b1;
      if (bg_clr) bg_beats <= '0;
      else if (bg_cnt) bg_beats <= bg_beats + 32'(b_wacc[0] && !init_wr) + 32'(b_rvalid[1]) + 32'(b_rvalid[2]);
    end
  end

  // ===========================================================================
  // progress counters
  // ===========================================================================
  logic [31:0] n_rd, n_bt, n_busy, n_lrv;
  always_ff @(posedge clk) begin
    if (!rst_n) begin n_rd <= '0; n_bt <= '0; n_busy <= '0; n_lrv <= '0; end
    else begin
      if (took) n_rd <= n_rd + 32'd1;
      if (m_rvalid) n_lrv <= n_lrv + 32'd1;
      if (pv || pw) n_bt <= n_bt + 32'd1;
      if (DDRAM_BUSY) n_busy <= n_busy + 32'd1;
    end
  end

  // ===========================================================================
  // the sequencer
  // ===========================================================================
  typedef enum logic [3:0] { S_LIVE, S_INIT, S_CASE, S_RUN, S_DRAIN, S_SCAN, S_RES, S_RESW } st_t;
  st_t s;

  logic [4:0]  k;
  logic [5:0]  cap;
  logic [7:0]  blen;
  logic        seq, wr;
  logic        issue;
  logic [6:0]  oc;                     // the probe's bursts in flight (issued, not done)
  logic [5:0]  held, held_mx;
  logic [31:0] lfsr;
  logic [RB-1:0] sq;
  logic [26:0] wcnt;
  logic [31:0] beats, nlat, cum, half;
  logic        ck_v;
  logic [63:0] ck_got, ck_want;
  logic [15:0] lmin, lmax;
  logic [8:0]  lmed;
  logic        fmed;
  logic [1:0]  hk;
  logic [9:0]  sb;
  logic [15:0] passes, errors;

  assign p_blen = blen;
  assign p_we   = wr;
  wire  [RB-1:0] bmask = RB'(blen - 8'd1);
  assign ra     = seq ? sq : (lfsr[RB-1:0] & ~bmask);
  assign p_addr = 25'(ra);
  // the probe's request: the pipelined port, or port L (held to its ack, one
  // at a time, never raised in the ack's cycle)
  wire   want   = issue && live && (oc < 7'(lpath ? 7'd1 : 7'(cap)));
  assign p_req  = want && !lpath;
  assign p_din  = patt(af_h + RB'(rbeat));
  logic  lp_req;
  logic [RB-1:0] lp_a;                 // held from the take to the ack (m2_ddr3's contract)
  assign b_req[3]  = lp_req;
  assign b_addr[3] = 25'(lp_a);
  assign b_blen[3] = blen;
  assign b_din[3]  = '0;
  assign took   = lpath ? (want && !lp_req && !b_ack[3]) : (p_req && p_ack);
  always_ff @(posedge clk) begin
    if (!rst_n) lp_req <= 1'b0;
    else if (b_ack[3]) lp_req <= 1'b0;
    else if (lpath && took) begin lp_req <= 1'b1; lp_a <= ra; end
  end
  wire   r_done = (pv || pw) && (rbeat + 8'd1 == blen);
  assign af_pop = r_done;
  // latency: the master's (pins to first beat) on the pipelined port; on port L,
  // measured here, from the request to its first beat (the arbiter and the
  // master included)
  logic [15:0] l_cnt;
  logic        l_first;
  always_ff @(posedge clk) begin
    if (took) begin l_cnt <= 16'd1; l_first <= 1'b1; end
    else begin
      if (l_cnt != 16'hffff) l_cnt <= l_cnt + 16'd1;
      if (pv) l_first <= 1'b0;
    end
  end
  wire        lat_l = lpath && pv && l_first && live;
  wire [15:0] lat_e = lpath ? l_cnt : lat;
  wire   lat_mine = lpath ? lat_l : (lat_v && lat_src == 4'd1 && live);

  function automatic logic [5:0] ncap(input logic [4:0] kk);
    case (kk)
      5'd0, 5'd5, 5'd10:  ncap = 6'd1;
      5'd1, 5'd6, 5'd11:  ncap = 6'd2;
      5'd2, 5'd7, 5'd12:  ncap = 6'd4;
      5'd3, 5'd8, 5'd13:  ncap = 6'd8;
      5'd16:              ncap = 6'd32;
      default:            ncap = 6'd16;
    endcase
  endfunction
  wire [26:0] win = (clk_c == 2'd1) ? 27'(WIN1) : (clk_c == 2'd2) ? 27'(WIN2) : (clk_c == 2'd3) ? 27'(WIN3) : 27'(WIN0);

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      s <= S_LIVE; h_we <= 1'b0; issue <= 1'b0; bg_on <= 1'b0; st_on <= 1'b0; bg_cnt <= 1'b0; bg_clr <= 1'b0;
      oc <= '0; held <= '0; af_wp <= '0; af_rp <= '0; rbeat <= '0; lfsr <= 32'hACE1_2468; sq <= '0;
      passes <= '0; errors <= '0; k <= '0; hk <= '0; blen <= 8'd4; cap <= 6'd1; seq <= 1'b0; wr <= 1'b0; ck_v <= 1'b0;
      r_tgl <= 1'b0; d_ack <= 1'b0;
    end else begin
      h_we <= 1'b0; bg_clr <= 1'b0;
      if (took) begin
        lfsr <= {lfsr[30:0], 1'b0} ^ (lfsr[31] ? 32'h0400_0007 : 32'd0);
        sq <= sq + RB'(blen);
        af_wp <= af_wp + 7'd1;
      end
      // the pattern check, in order (its operands registered, the compare a
      // cycle on); a write's beats counted the same way
      ck_v <= pv && !skip; ck_got <= dout; ck_want <= patt(af_h + RB'(rbeat));
      if (ck_v && ck_got != ck_want && errors != 16'hffff) errors <= errors + 16'd1;
      if (pv || pw) begin
        if (r_done) begin rbeat <= '0; af_rp <= af_rp + 7'd1; end
        else rbeat <= rbeat + 8'd1;
      end
      oc <= oc + 7'(took) - 7'(r_done);
      // reads held by the bridge, and the most (taken from the registered count)
      held <= held + 6'(take && take_src == 4'd1 && live) - 6'(done && done_src == 4'd1 && live);
      if (issue && held > held_mx) held_mx <= held;
      // the histogram: the bin read now, written two cycles on
      hk <= '0;
      if (lat_mine) begin
        h_ra <= (lat_e > 16'd511) ? 9'd511 : lat_e[8:0];
        if (lat_e < lmin) lmin <= lat_e;
        if (lat_e > lmax) lmax <= lat_e;
        nlat <= nlat + 32'd1;
        hk <= 2'd1;
      end else if (hk == 2'd1) hk <= 2'd2;
      else if (hk == 2'd2) begin h_we <= 1'b1; h_wa <= h_ra; h_wd <= h_q + 32'd1; end

      // progress, copied on request
      if (dq_s[2] != d_ack) begin
        d_ack <= dq_s[2];
        d_iw <= 32'(iw); d_rd <= n_rd; d_bt <= n_bt; d_busy <= n_busy; d_st <= 4'(s); d_acc <= m_acc; d_oc <= oc;
        d_rdy <= m_rdy; d_lrv <= n_lrv; d_mst <= m_st; d_rq <= m_rq;
      end

      case (s)
        S_LIVE: if (live) s <= S_INIT;
        S_INIT: if (init_done) begin k <= '0; s <= S_CASE; end

        // ---- a window (a sweep case asking more reads in flight than the master
        // takes is skipped)
        S_CASE: if (!manual && ncap(k) > 6'(MAXQ)) begin
          if (k == 5'(NCASE - 1)) begin k <= '0; passes <= passes + 16'd1; end
          else k <= k + 5'd1;
        end else begin
          if (manual) begin
            cap   <= 6'(1) << ((infl_c > 3'd5) ? 3'd5 : infl_c);
            blen  <= 8'(1) << burst_c;
            seq   <= seq_m;
            wr    <= wr_m;
            bg_on <= bg_m != 2'd0;
            st_on <= bg_m[1];
          end else begin
            case (k)
              5'd0, 5'd5, 5'd10:  cap <= 6'd1;
              5'd1, 5'd6, 5'd11:  cap <= 6'd2;
              5'd2, 5'd7, 5'd12:  cap <= 6'd4;
              5'd3, 5'd8, 5'd13:  cap <= 6'd8;
              5'd16:              cap <= 6'd32;
              default:            cap <= 6'd16;
            endcase
            blen  <= (k == 5'd16) ? 8'd8 : 8'd4;
            seq   <= (k == 5'd15);
            wr    <= 1'b0;
            bg_on <= (k >= 5'd5 && k < 5'd15);
            st_on <= (k >= 5'd10 && k < 5'd15);
          end
          bg_clr <= 1'b1;
          wcnt <= win - 27'd1; beats <= '0; nlat <= '0; lmin <= 16'hffff; lmax <= '0; held_mx <= '0;
          issue <= 1'b1; bg_cnt <= 1'b1;
          s <= S_RUN;
        end
        S_RUN: begin
          wcnt <= wcnt - 27'd1;
          if (pv || pw) beats <= beats + 32'd1;
          if (wcnt == '0) begin issue <= 1'b0; bg_cnt <= 1'b0; bg_on <= 1'b0; st_on <= 1'b0; s <= S_DRAIN; end
        end
        S_DRAIN: if (oc == '0 && !pv && !pw && hk == '0 && !lat_mine && b_req == '0 && !ck_v) begin
          sb <= '0; cum <= '0; fmed <= 1'b0; half <= (nlat + 32'd1) >> 1; s <= S_SCAN;
        end
        // the median: bin sb - 2 added this cycle, the sum compared a cycle on
        // (the bin then sb - 3); the bins cleared behind
        S_SCAN: begin
          sb <= sb + 10'd1;
          h_ra <= sb[8:0];
          if (sb >= 10'd2 && sb <= 10'd513) begin
            cum <= cum + h_q;
            h_we <= 1'b1; h_wa <= 9'(sb - 10'd2); h_wd <= '0;
          end
          if (sb >= 10'd3 && !fmed && cum >= half && nlat != '0) begin fmed <= 1'b1; lmed <= 9'(sb - 10'd3); end
          if (sb == 10'd514) s <= S_RES;
        end
        // the results out; on when dp_core has them
        S_RES: begin
          r_k <= k; r_n <= cap; r_ln <= blen; r_seq <= seq; r_wr <= wr;
          r_bg <= manual ? bg_m : (k >= 5'd10 && k < 5'd15) ? 2'd2 : (k >= 5'd5 && k < 5'd15) ? 2'd1 : 2'd0;
          r_bytes <= {beats[28:0], 3'd0}; r_bgb <= {bg_beats[28:0], 3'd0};
          r_lmin <= (nlat == '0) ? 16'd0 : lmin; r_lmax <= lmax; r_lmed <= lmed; r_ac <= held_mx;
          r_pass <= (manual || k == 5'(NCASE - 1)) ? passes + 16'd1 : passes; r_err <= errors;
          if (manual || k == 5'(NCASE - 1)) begin passes <= passes + 16'd1; k <= '0; end
          else k <= k + 5'd1;
          r_tgl <= !r_tgl;
          s <= S_RESW;
        end
        S_RESW: if (ra_s[2] == r_tgl) s <= S_CASE;
        default: s <= S_LIVE;
      endcase
    end
  end

endmodule
`undef DP_M2
`undef DP_MAXQ
