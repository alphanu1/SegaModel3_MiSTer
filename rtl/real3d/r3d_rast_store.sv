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
// Real3D rasteriser: the polygon store (block 8, study D-20, R46). The
// geometry's set-up records are binned by 8-row band as they arrive, kept in
// DDR3, and each band's records are read back alone, in render_d25's draw
// order: per pass, the opaque list forward, then the alpha list backward
// (MAME's reverse buffer order).
//
// Records. Each goes once into its list's region, list l = {pass, alpha}
// (8 lists), packed: the polygon number and det are dropped (the rasteriser
// reads neither), header word 3 and word 1's texture position share one word
// with the viewport's number. Textured 21 words (22 with MIP), 11 beats;
// untextured 15 (16), 8 beats. Opaque lists grow up from the region's base,
// alpha lists down from its top, so a run of consecutive alpha records read
// as one burst comes out newest first: the reverse order with no reordering.
//
// Bins. For each (band, list), 384 of them, the record's runs: a run is
// consecutive records of the list, contiguous in DDR3, up to RUNMAX beats,
// one read burst on replay. A bin's open run (the newest) lives on chip with
// its count of closed runs; a closed run goes to the bin's run list in DDR3,
// one 32-bit entry {start beat, beats}. So the chip holds 384 x {closed
// runs, open run} a bank (38 bits, M10K), and nothing grows with the frame.
// Census (r3d_store_census, 400 + 400 frames, 8-row bands): p99 18,043 /
// 16,434 (record, band) pairs a frame, 5,710 / 4,096 runs at 64 beats, at
// most 2,442 records in one bin.
//
// A record's bands are floor(y) of its first and last sorted vertex (FP32),
// conservative against the rows the front walks: a band the record misses
// costs the front a triangle with no rows, never a lost one.
//
// Two banks: frame N+1 is binned while frame N is replayed. A bank is free
// until b_begin, binned until b_end, full until p_go, played until
// p_release (the frame's layer is finished: the viewport table indexed by
// the bank is still read until then). b_begin sweeps the bank's 384 entries
// to empty (384 cycles), so nothing is cleared in reset.
//
// Replay. Per band, per pass with any record: a NEWGEN marker (a new Z
// generation), the opaque bin's runs (DDR3 list, then the open run), the alpha
// bin's (open run, then the list from its end), each run one burst into a
// 2^FB-beat FIFO (a burst is issued only with room for all of it: rvalid
// cannot be refused). After the band's passes a BANDEND marker. The FIFO's
// reader rebuilds each record as r3d_rast_intake takes it (SETUP4: words
// 0-3, [B], 6 vertex words, det = 0, the planes). Word 3 comes back as
// {0, word 3 [7] at [14] (texture inversion, study R135), viewport [13:8],
// new generation [7], word 3 [6:0]}: bit 7 (and o_new,
// on the first word) marks a pass's first record. At BANDEND the reader stops
// (o_eob) until o_next.
//
// DDR3 layout, 64-bit words from base (study R136):
//   records  {k, pass[1:0], beat[LB:0]} with PAIR: a pass's opaque list grows
//            up from its region's base and its alpha list down from the top,
//            so either may use what the other leaves (2^(LB+1) beats a pass);
//            {k, list[2:0], beat[LB-1:0]} without (2^LB beats a list). Both
//            2^(LB+3) words a bank, 2^(LB+4) for the two.
//   runs     RUNO + {k, bin[8:0], beat[BB-1:0]}, bin = band 8 + list, two
//            runs a beat (entry n in lane n[0]); 2^(BB+10) words for the two banks.
//            RUNO defaults to the end of the records; the core puts the runs
//            in DDR3's top 64 MiB (m3_system).
// Defaults (LB 19, PAIR, BB 12): 8 MiB a pass (1,048,575 beats, 95,325
// textured records), 8,190 runs a bin, 64 MiB of records and 32 MiB of runs.
// Study R136: at LB 17 without PAIR (1 MiB a list, 131,072 beats) Daytona 2's
// opaque list of pass 0 overflowed in 39 of 400 attract frames (Supermodel's
// rows, as the geometry sends them; up to 158,644 beats) and the newest
// records, the player's car, were dropped; Scud Race needs up to 283,335 in
// one list and 2,570 runs in one bin. Nothing bounds a frame's records (the
// core's geometry has no limit, and the Pro-1000's rated 750,000 polygons a
// second is passed by Scud Race's attract), so the sizes are three times the
// heaviest of 5,143 frames of five games: 311,849 beats in a pass, 2,570 runs
// in a bin. A run-list entry carries a 25-bit start beat. A record or a pair
// still refused for room is dropped (the newest, as MAME's buffers and
// Supermodel's guess at the hardware's overload) and counted: n_ovff, frames
// with an overflow, for the overlay.
//
// Memory port: m2_ddr3's request contract, which m2_sdram's ports share
// (D-20, D-34): req held to ack, a write beat per wacc (din and be live),
// a read beat per rvalid, one transaction at a time. Moving the store to the
// second SDRAM is that controller's port.

/* verilator lint_off UNUSEDSIGNAL */
module r3d_rast_store #(
  parameter bit MIP    = 1'b0,    // a fifth header word (the LOD word B)
  parameter int LB     = 19,      // log2 beats a list's share of the records (study R136)
  parameter bit PAIR   = 1'b1,    // study R136: a pass's two lists share one region of 2^(LB+1) beats
  parameter logic [24:0] RUNO = 25'(1 << (LB + 4)),   // the run lists' offset from base (words)
  parameter int BB     = 12,      // log2 beats a bin's run list (study R136: 12, 8,190 runs; was 11)
  parameter int RUNMAX = 64,      // beats a run (a read burst) at most
  parameter int FB     = 8        // log2 beats of the replay FIFO
) (
  input  logic        clk,
  input  logic        rst,
  input  logic [24:0] base,

  // ---- binning
  output logic        b_ready,     // a bank is free: b_begin may be given
  input  logic        b_begin,
  input  logic        b_end,       // the frame's last record has been given
  output logic        b_bank,      // the bank being binned
  output logic        b_busy,      // binning (b_begin to the last record stored after b_end)
  input  logic        w_valid,
  output logic        w_ready,
  input  logic [31:0] w_data,
  input  logic        w_last,
  input  logic [5:0]  w_vp,        // the record's viewport (its number in the frame)
  input  logic        w_vpv,       // a viewport has been given (else no fog)

  // ---- replay
  output logic        p_ready,     // a bank is full and the replay is idle
  input  logic        p_go,
  output logic        p_bank,      // the bank being replayed
  input  logic        p_release,   // its frame is finished: the bank is free
  output logic        o_valid,
  input  logic        o_ready,
  output logic [31:0] o_data,
  output logic        o_last,
  output logic        o_new,       // first word of a (band, pass)'s first record
  output logic [5:0]  o_band,      // the band of the records coming out
  output logic        o_eob,       // band o_band's last record has gone; waiting for o_next
  input  logic        o_next,
  output logic        p_busy,      // the replay runs (not yet at the frame's last BANDEND)

  // ---- DDR3 (m2_ddr3's contract)
  output logic        m_req,
  output logic        m_we,
  output logic [24:0] m_addr,
  output logic [7:0]  m_blen,
  output logic [63:0] m_din,
  output logic [7:0]  m_be,
  input  logic        m_wacc,
  input  logic        m_rvalid,
  input  logic        m_ack,
  input  logic [63:0] m_dout,

  // ---- counts (the frame being binned)
  output logic [15:0] n_rec,       // records stored
  output logic [17:0] n_pair,      // (record, band) pairs
  output logic [15:0] n_run,       // runs closed to DDR3
  output logic [15:0] n_drop,      // records dropped: no band, faces away, or no room
  output logic [2:0]  ovf,         // {a bin's run list full, a list region full, the bin table}
  output logic [15:0] n_ovff       // study R136: frames binned with an overflow since reset (saturating)
);

  localparam int H   = MIP ? 5 : 4;               // header words
  localparam int RB  = PAIR ? LB + 1 : LB;         // a record's start beat in its region
  // study R136: the entry kept at 40 bits (the table's four M10K) at RB 20 and BB 12: the closed-run count
  // is 0 .. 2^(BB+1) - 2 (one bit narrower; a run list holds 8,190 runs, and the replay's count of items,
  // the closed runs and the open one, still fits), and the open run's valid bit is its beats being nonzero
  localparam int NRB = BB + 1;                     // closed-run count
  localparam int EW  = NRB + RB + 7;               // table entry {nr, ost, onb}
  localparam int FW  = 1 << FB;

  // ===========================================================================
  // bank states
  // ===========================================================================
  localparam logic [1:0] B_FREE = 2'd0, B_BIN = 2'd1, B_FULL = 2'd2, B_PLAY = 2'd3;
  logic [1:0] bst [2];
  logic       wbk, pbk;                            // the next bank to bin, to play

  // the replay's state, declared here for the binning side's use
  typedef enum logic [3:0] { P_IDLE, P_TO, P_TA, P_TB, P_DEC, P_BIN, P_CK, P_CKW, P_RE, P_RQ, P_RW, P_NEXT, P_DONE } pst_t;
  pst_t ps;
  logic pb;                                        // the bank being replayed
  wire  p_take = (ps == P_IDLE) && p_go && bst[pbk] == B_FULL;
  wire  p_rel  = p_release && ps == P_DONE;
  wire  p_trd  = (ps == P_TO) || (ps == P_TA);     // the replay reads the table this cycle

  // addresses in a bank
  function automatic logic [24:0] rec_a(input logic bk, input logic [2:0] l, input logic [RB-1:0] b);
    rec_a = PAIR ? 25'({bk, l[2:1], b}) : 25'({bk, l, b});
  endfunction
  function automatic logic [24:0] run_a(input logic bk, input logic [8:0] bin, input logic [BB-1:0] b);
    run_a = RUNO + 25'({bk, bin, b});
  endfunction

  // ===========================================================================
  // the bin table: 2 x 384 entries, M10K; one read port shared (replay first),
  // the write port the binning side's. Four M10K (1024 x 10 each): three blocks
  // of 256 x 40 would hold it (max_depth = 256), but their 3:1 read mux sat in
  // the table's read-to-address path and failed 10 ns (-0.34 ns at 100C)
  // ===========================================================================
  (* ramstyle = "M10K, no_rw_check" *) logic [EW-1:0] tab [768];
  logic [9:0]    t_ra, t_wa;
  logic          t_we;
  logic [EW-1:0] t_wd, t_q;
  always_ff @(posedge clk) begin
    if (t_we) tab[t_wa] <= t_wd;
    t_q <= tab[t_ra];
  end
  function automatic logic [9:0] tidx(input logic bk, input logic [8:0] bin);
    tidx = bk ? 10'({1'b0, bin} + 10'd384) : {1'b0, bin};
  endfunction
  function automatic logic [NRB-1:0] e_nr(input logic [EW-1:0] e);  e_nr = e[EW-1 -: NRB];   endfunction
  function automatic logic           e_ov(input logic [EW-1:0] e);  e_ov = e[6:0] != 7'd0;    endfunction
  function automatic logic [RB-1:0]  e_st(input logic [EW-1:0] e);  e_st = e[RB+6:7];        endfunction
  function automatic logic [6:0]     e_nb(input logic [EW-1:0] e);  e_nb = e[6:0];           endfunction

  // ===========================================================================
  // the port: two owners, the binning side (W) and the replay (P)
  // ===========================================================================
  logic        wq_req, pq_req;
  logic        wq_we;
  logic [24:0] wq_addr, pq_addr;
  logic [7:0]  wq_blen, pq_blen, wq_be;
  logic [63:0] wq_din;
  logic [1:0]  own;                                // 0 none, 1 W, 2 P
  logic        last_w;                             // W had the last grant
  assign m_req  = (own == 2'd1) ? wq_req : (own == 2'd2) ? pq_req : 1'b0;
  assign m_we   = (own == 2'd1) ? wq_we : 1'b0;
  assign m_addr = base + ((own == 2'd1) ? wq_addr : pq_addr);   // one adder for every address
  assign m_blen = (own == 2'd1) ? wq_blen : pq_blen;
  assign m_din  = wq_din;
  assign m_be   = wq_be;
  wire   w_wacc = m_wacc && own == 2'd1;
  wire   w_ack  = m_ack  && own == 2'd1;
  wire   p_rv   = m_rvalid && own == 2'd2;
  wire   p_ack  = m_ack  && own == 2'd2;
  always_ff @(posedge clk) begin
    if (rst) begin own <= 2'd0; last_w <= 1'b0; end
    else if (own == 2'd0) begin
      if (wq_req && (!pq_req || !last_w)) begin own <= 2'd1; last_w <= 1'b1; end
      else if (pq_req) begin own <= 2'd2; last_w <= 1'b0; end
    end else if (m_ack) own <= 2'd0;
  end

  // ===========================================================================
  // binning
  // ===========================================================================
  // floor(y) of an FP32 y as a row: {not a number, below 0, 512 or more, row}
  function automatic logic [11:0] frow(input logic [31:0] f);
    logic [7:0] e;
    logic [8:0] r;
    logic       nan, neg, big;
    e = f[30:23];
    nan = (e == 8'hff) && (f[22:0] != 23'd0);
    neg = f[31] && (f[30:0] != 31'd0);
    big = !f[31] && (e >= 8'd136);                 // 2^9 and up, and +inf
    r = 9'd0;
    if (e >= 8'd127 && e < 8'd136) r = {1'b1, f[22:15]} >> (4'd8 - 4'(e - 8'd127));
    frow = {nan, neg, big, r};
  endfunction

  typedef enum logic [3:0] { W_IDLE, W_SWEEP, W_REC, W_ADDR, W_WR, W_BRD, W_BQ, W_RUN } wst_t;
  wst_t ws;
  logic          wb;                                // the bank being binned
  logic          end_pend;
  logic [8:0]    sw;                                // sweep pointer
  logic [4:0]    wk;                                // incoming word
  logic [4:0]    wj;                                // its stored index
  logic [10:0]   w1q;                               // word 1's texture position
  logic [2:0]    wl;                                // the record's list
  logic          wbad;                              // malformed, not a number, faces away
  logic [11:0]   fy0, fy2;
  // each list's next beat (opaque) or start (alpha), in an MLAB read registered: the list wl is set at a
  // record's first word and read in W_ADDR at least H + 6 words later, and the last write to it was the
  // record before's W_ADDR, so the registered word is the live one (checked below). It was eight
  // registers and an 8-way select in front of the fit test and the address.
  // Study R136: RB bits a pointer (one MLAB wide at RB 20). An opaque list keeps its next beat O, an alpha
  // list its start S modulo 2^RB (0 when empty: the region's top). A record fits when O + beats < S (S = 0
  // read as 2^RB): the region's last beat is never used, which is what keeps O below 2^RB. The sweep
  // clears every pointer to 0.
  (* ramstyle = "M10K, no_rw_check" *) logic [RB-1:0] nx [8];    // M10K (area pass): read registered, as nx_q
  logic [RB-1:0] nx_q;
  // (PAIR) a copy of nx read at the pass's other list, the bound the record must not cross
  (* ramstyle = "M10K, no_rw_check" *) logic [RB-1:0] nxp [8];   // M10K (area pass): read registered, as nxp_q
  logic [RB-1:0] nxp_q;
  logic          nx_we;
  logic [2:0]    nx_wa;
  logic [RB-1:0] nx_wd;
  logic [RB-1:0] wa;                                // the record's start beat
  logic [3:0]    wnb;                               // its beats
  logic [5:0]    bc, bhi;                           // band in hand, last band
  logic [3:0]    wbt;                               // beat being written
  logic [31:0]   wrun;                              // the run being closed
  logic          wrun_hi;                           // its lane

  // the record buffer, two words a beat by parity (MLAB, read a beat ahead)
  // in M10K, old data on a read of the word written (study R98: -40 ALM, +2 M10K)
  (* ramstyle = "M10K" *) logic [31:0] rbe [16];
  (* ramstyle = "M10K" *) logic [31:0] rbo [16];
  logic [31:0] rqe, rqo;
  wire  [3:0]  rb_ra = (ws != W_WR) ? 4'd0 : w_wacc ? wbt + 4'd1 : wbt;
  always_ff @(posedge clk) begin rqe <= rbe[rb_ra]; rqo <= rbo[rb_ra]; end

  // the word as stored: word 3 becomes {0, word 1 [30:20], 0, word 3 [7], viewport, 0, word 3 [6:0]}
  wire [31:0] w3p = {1'b0, w1q, 5'd0, w_data[7], w_vp, 1'b0, w_data[6:5], w_vpv ? w_data[4:0] : 5'd0};
  wire [31:0] wsd = (wk == 5'd3) ? w3p : w_data;
  wire        wkeep = (wk != 5'd1) && (wk != 5'(H + 6));
  wire [11:0] frq = frow(w_data);
  // index stored: 0, -, 2, 1, [3], k - 1 for the vertices, -, k - 2 for the planes
  always_comb begin
    if (wk == 5'd3) wj = 5'd1;
    else if (wk < 5'd3) wj = wk;
    else if (wk < 5'(H + 6)) wj = wk - 5'd1;
    else wj = wk - 5'd2;
  end

  // the record's bands, its address, the run update
  wire [5:0]  blo_w  = fy0[10] ? 6'd0 : 6'(fy0[8:3]);
  wire [5:0]  bhi_w  = (fy2[9] || fy2[8:0] >= 9'd384) ? 6'd47 : 6'(fy2[8:3]);
  wire        nob    = fy0[9] || fy0[8:0] >= 9'd384 || fy2[10] || fy0[11] || fy2[11] || (blo_w > bhi_w);
  // nob and the bands registered: fy0 and fy2 are written at words H + 1 and H + 5 and the record's
  // last word is H + 6 or later, so in W_ADDR they have held a cycle and the registered copies equal
  // the live ones (checked below). fy2 through bhi_w and nob into the lists' enables was the 100 MHz
  // system build's worst path, -0.96 ns (study R104).
  logic       nob_q;
  logic [5:0] blo_q, bhi_q;
  always_ff @(posedge clk) begin nob_q <= nob; blo_q <= blo_w; bhi_q <= bhi_w; end
`ifdef VERILATOR
  always_ff @(posedge clk)
    if (!rst && ws == W_ADDR && {nob_q, blo_q, bhi_q} != {nob, blo_w, bhi_w}) $fatal(1, "store: nob registered");
`endif
  wire        walpha = wl[0];
  // the region's free space is [O, S): O the opaque list's next beat, S the alpha list's start; a list
  // without PAIR is bounded by its own region's end (opaque) or base (alpha)
  wire [RB-1:0] nxl  = nx_q;
  wire [RB-1:0] nlo  = walpha ? (PAIR ? nxp_q : '0) : nxl;
  wire [RB-1:0] nhs  = walpha ? nxl : (PAIR ? nxp_q : '0);
  wire [RB:0]   nhi  = {nhs == '0, nhs};
  wire        wfit   = (RB+1)'(nlo) + (RB+1)'(wnb) < nhi;
  wire [RB-1:0] wa_n = walpha ? nxl - RB'(wnb) : nxl;
  // W_ADDR's fit test, start beat and the list's next pointer, formed a cycle early as registers: W_ADDR is
  // entered only from the record's last word, where wnb is set from wk (wk >> 1), and lasts a cycle; nx is
  // written only in W_ADDR and the sweep, so nx_q and wl are the same in both cycles (checked below). The
  // pointer read through the add and compare into W_ADDR's ~80 enables and nx's write port was 114.8 MHz
  // standalone (dpram -> wq_blen), and dpram_llo1 -> nx / wa in c7_s1 (-0.49 ns).
  wire [3:0]    wnb_k  = 4'(wk >> 1);
  logic         wfit_q;
  logic [RB-1:0] wa_q, wnx_q;
  always_ff @(posedge clk) begin
    wfit_q <= (RB+1)'(nlo) + (RB+1)'(wnb_k) < nhi;
    wa_q   <= walpha ? nxl - RB'(wnb_k) : nxl;
    wnx_q  <= nxl + RB'(wnb_k);
  end
`ifdef VERILATOR
  always_ff @(posedge clk)
    if (!rst && ws == W_ADDR && {wfit_q, wa_q, wnx_q} != {wfit, wa_n, nxl + RB'(wnb)}) $fatal(1, "store: the fit a cycle early");
`endif
  wire [8:0]  wbin   = {bc, wl};
  // the entry read for (bc, wl): extend its open run, or close it and open one
  wire [NRB-1:0] q_nr = e_nr(t_q);
  wire           q_ov = e_ov(t_q);
  wire [RB-1:0]  q_st = e_st(t_q);
  wire [6:0]     q_nb = e_nb(t_q);
  // q_ext without an adder on the table's output (study R109): q_nb + wnb <= RUNMAX against a registered
  // RUNMAX - wnb; wa + wnb == q_st against a registered wa + wnb; q_st + q_nb == wa by the carry-save test
  // (a + b == c iff a ^ b ^ c is the carries (a & b) | ((a | b) & ~c), one place up). wnb is set at a
  // record's last word and wa in W_ADDR, at least a cycle before W_BQ, so the registers hold them.
  logic [6:0]    rmw;
  logic [RB-1:0] wae;
  always_ff @(posedge clk) begin rmw <= 7'(RUNMAX) - 7'(wnb); wae <= wa + RB'(wnb); end
  wire  [RB-1:0] q_nbx = RB'(q_nb);
  wire  [RB-1:0] q_cs  = (q_st & q_nbx) | ((q_st | q_nbx) & ~wa);
  wire           q_ext = q_ov && (q_nb <= rmw) &&
                         (walpha ? (wae == q_st) : ((q_st ^ q_nbx ^ wa) == {q_cs[RB-2:0], 1'b0}));
`ifdef VERILATOR
  always_ff @(posedge clk)
    if (!rst && ws == W_BQ && q_ext != (q_ov && (7'(q_nb) + 7'(wnb) <= 7'(RUNMAX)) &&
                                        (walpha ? (wa + RB'(wnb) == q_st) : (q_st + RB'(q_nb) == wa))))
      $fatal(1, "store: q_ext");
`endif
  // W_BQ's table write a cycle later (study R109): the new entry's adds (the run's beats, the closed-run
  // count) are formed from the registered entry, not from the table's output. The next read of that
  // entry is the next record's, many cycles later; the next band's (W_BRD) is another entry.
  logic          bq_v, bq_ext, bq_cl;
  logic [9:0]    bq_wa;
  logic [EW-1:0] bq_e;
`ifdef VERILATOR
  logic [EW-1:0] bq_chk;
  always_ff @(posedge clk) if (!rst && t_we && $past(bq_v) && t_wd != bq_chk) $fatal(1, "store: the entry a cycle on");
`endif
  wire           q_full = q_nr == NRB'((1 << (BB + 1)) - 2);

  // nx's port: the sweep's first eight cycles set every list's start (odd lists, alpha, at the top);
  // W_ADDR writes the record's list
  always_comb begin
    nx_we = 1'b0; nx_wa = wl; nx_wd = walpha ? wa_q : wnx_q;
    if (ws == W_SWEEP && sw < 9'd8) begin
      nx_we = 1'b1; nx_wa = sw[2:0]; nx_wd = '0;
    end else if (ws == W_ADDR && !nob_q && wfit_q) nx_we = 1'b1;
  end
  always_ff @(posedge clk) begin
    if (nx_we) begin nx[nx_wa] <= nx_wd; nxp[nx_wa] <= nx_wd; end
    nx_q <= nx[wl];
    nxp_q <= nxp[wl ^ 3'd1];
  end
`ifdef VERILATOR
  logic [RB-1:0] nx_chk [8];
  always_ff @(posedge clk) begin
    if (!rst && ws == W_IDLE && b_begin && bst[wbk] == B_FREE)
      for (int l = 0; l < 8; l++) nx_chk[l] <= '0;
    if (!rst && nx_we && ws == W_ADDR) nx_chk[nx_wa] <= nx_wd;
    if (!rst && ws == W_ADDR && nx_q != nx_chk[wl]) $fatal(1, "store: nx registered");
    if (!rst && ws == W_ADDR && PAIR && nxp_q != nx_chk[wl ^ 3'd1]) $fatal(1, "store: nxp registered");
    // a pass's opaque list never reaches its alpha list's start (O < S, S = 0 the top)
    if (!rst && ws == W_ADDR && PAIR && nx_chk[{wl[2:1], 1'b1}] != '0 && nx_chk[{wl[2:1], 1'b0}] >= nx_chk[{wl[2:1], 1'b1}])
      $fatal(1, "store: lists crossed");
  end
`endif

  always_ff @(posedge clk) begin
    if (rst) begin
      ws <= W_IDLE; end_pend <= 1'b0; wq_req <= 1'b0; t_we <= 1'b0; bq_v <= 1'b0;
      bst[0] <= B_FREE; bst[1] <= B_FREE; wbk <= 1'b0;
      n_rec <= '0; n_pair <= '0; n_run <= '0; n_drop <= '0; ovf <= '0; n_ovff <= '0;
    end else begin
      t_we <= 1'b0; bq_v <= 1'b0;
      if (b_end) end_pend <= 1'b1;
      case (ws)
        W_IDLE: if (b_begin && bst[wbk] == B_FREE) begin
          wb <= wbk; wbk <= !wbk; bst[wbk] <= B_BIN; sw <= '0; end_pend <= 1'b0;
          n_rec <= '0; n_pair <= '0; n_run <= '0; n_drop <= '0; ovf <= '0;
          ws <= W_SWEEP;
        end
        W_SWEEP: begin
          t_we <= 1'b1; t_wa <= tidx(wb, sw); t_wd <= '0;
          sw <= sw + 9'd1;
          if (sw == 9'd383) begin wk <= '0; wbad <= 1'b0; ws <= W_REC; end
        end
        W_REC: begin
          if (w_valid) begin
            if (wkeep) begin
              if (wj[0]) rbo[wj[4:1]] <= wsd; else rbe[wj[4:1]] <= wsd;
            end
            if (wk == 5'd0) wl <= {w_data[1:0], w_data[2]};
            if (wk == 5'd1) w1q <= w_data[30:20];
            if (wk == 5'd3 && w_data[6]) wbad <= 1'b1;            // faces away (FACE_DROP = 0)
            if (wk == 5'(H + 1)) fy0 <= frq;
            if (wk == 5'(H + 5)) fy2 <= frq;
            wk <= wk + 5'd1;
            if (w_last) begin
              wk <= '0; wbad <= 1'b0;
              // stored words wk - 1 (wk the last word's index), beats half that, rounded up
              wnb <= 4'(wk >> 1);
              if (wbad || (w_data[6] && wk == 5'd3) || wk < 5'(H + 6)) n_drop <= n_drop + 16'd1;
              else ws <= W_ADDR;
            end
          end else if (wk == 5'd0 && end_pend) begin
            bst[wb] <= B_FULL; end_pend <= 1'b0; ws <= W_IDLE;
          end
        end
        W_ADDR: begin
          if (nob_q) begin n_drop <= n_drop + 16'd1; ws <= W_REC; end
          else if (!wfit_q) begin
            n_drop <= n_drop + 16'd1; ovf[1] <= 1'b1; ws <= W_REC;
            if (ovf == 3'd0 && n_ovff != 16'hffff) n_ovff <= n_ovff + 16'd1;   // the frame's first overflow
          end
          else begin
            wa <= wa_q;
            bc <= blo_q; bhi <= bhi_q;
            wq_req <= 1'b1; wq_we <= 1'b1; wq_be <= 8'hff;
            wq_addr <= 25'(rec_a(wb, wl, wa_q));
            wq_blen <= 8'(wnb);
            wbt <= '0;
            ws <= W_WR;
            n_rec <= n_rec + 16'd1;
          end
        end
        W_WR: begin
          if (w_wacc) wbt <= wbt + 4'd1;
          if (w_ack) begin wq_req <= 1'b0; ws <= W_BRD; end
        end
        W_BRD: if (!p_trd) ws <= W_BQ;                         // the read issued (t_ra below)
        W_BQ: begin
          // t_q: the entry for (bc, wl)
          logic [EW-1:0] ne;
          logic          close, keep;
          close = q_ov && !q_ext;
          keep  = !(close && q_full);
          ne = t_q;
          if (!keep) begin
            ovf[2] <= 1'b1;
            if (ovf == 3'd0 && n_ovff != 16'hffff) n_ovff <= n_ovff + 16'd1;
          end
          else n_pair <= n_pair + 18'd1;
          bq_v <= keep; bq_wa <= tidx(wb, wbin); bq_e <= t_q; bq_ext <= q_ext; bq_cl <= close;
`ifdef VERILATOR
          if (keep) begin
            if (q_ext) ne = {q_nr, walpha ? wa : q_st, 7'(q_nb) + 7'(wnb)};
            else       ne = {close ? q_nr + NRB'(1) : q_nr, wa, 7'(wnb)};
          end
          bq_chk <= ne;
`endif
          // the old open run's write, its fields loaded whether or not it goes: the port is idle in W_BQ (W_WR
          // and W_RUN end on their ack) and reads them only with wq_req (the close test from the table's
          // word was in front of 30 enables: the table's M10K to wq_addr, 116.7 MHz standalone)
          wrun <= {(25-RB)'(0), q_st, q_nb};
          wq_we <= 1'b1; wq_be <= q_nr[0] ? 8'hf0 : 8'h0f;
          wq_addr <= 25'(run_a(wb, wbin, q_nr[BB:1]));
          wq_blen <= 8'd1;
          if (close && keep) begin
            // the old open run to the bin's run list, entry q_nr
            wq_req <= 1'b1;
            n_run <= n_run + 16'd1;
            ws <= W_RUN;
          end else if (bc == bhi) ws <= W_REC;
          else begin bc <= bc + 6'd1; ws <= W_BRD; end
        end
        W_RUN: if (w_ack) begin
          wq_req <= 1'b0;
          if (bc == bhi) ws <= W_REC;
          else begin bc <= bc + 6'd1; ws <= W_BRD; end
        end
        default: ws <= W_IDLE;
      endcase
      // W_BQ's entry, written a cycle on
      if (bq_v) begin
        t_we <= 1'b1; t_wa <= bq_wa;
        if (bq_ext) t_wd <= {e_nr(bq_e), walpha ? wa : e_st(bq_e), 7'(e_nb(bq_e)) + 7'(wnb)};
        else        t_wd <= {bq_cl ? e_nr(bq_e) + NRB'(1) : e_nr(bq_e), wa, 7'(wnb)};
      end
      // the replay's bank states
      if (p_take) bst[pbk] <= B_PLAY;
      if (p_rel) bst[pb] <= B_FREE;
    end
  end
  always_comb wq_din = (ws == W_RUN) ? {wrun, wrun} : {rqo, rqe};
  assign w_ready = (ws == W_REC);
  assign b_ready = (ws == W_IDLE) && bst[wbk] == B_FREE && !b_begin;
  assign b_bank  = wb;
  assign b_busy  = (ws != W_IDLE);

  // ===========================================================================
  // replay
  // ===========================================================================
  logic [5:0]    pband;
  logic [1:0]    pp;                                // pass
  logic          pal;                               // 0 the opaque bin, 1 the alpha bin
  logic [EW-1:0] eo, ea;
  logic [NRB-1:0] pn;                               // items of the bin left (runs, the open one included)
  logic [NRB-1:0] pr;                               // the DDR3 run in hand
  logic [NRB-1:0] pnm1, pnm1h;                      // the bin's nr - 1 and (nr - 1) / 2 + 1 (P_BIN)
  logic          popen;                             // the item in hand is the open run
  logic          ck_v;                              // the run buffer holds the chunk of pr
  logic [NRB-1:0] ck_c;                             // that chunk (pr >> 5)
  logic [RB-1:0] rst_b;                             // the run to read: start, beats
  logic [6:0]    rnb;
  // the bin entry in hand (pal ? ea : eo) as a register, loaded wherever eo, ea or pal is (c7_s1: pal, fanout
  // 87, through the select and the run compare into P_RE's enables, -0.81 ns)
  logic [EW-1:0] ecur;
`ifdef VERILATOR
  always_ff @(posedge clk) if (!rst && ps != P_IDLE && ps != P_TO && ps != P_TA && ecur != (pal ? ea : eo)) $fatal(1, "store: ecur registered");
`endif
  wire [8:0]     pbin = {pband, pp, pal};
  assign p_ready = (ps == P_IDLE) && bst[pbk] == B_FULL;
  assign p_bank  = pb;
  assign p_busy  = (ps != P_IDLE) && (ps != P_DONE);
  assign t_ra    = (ps == P_TO) ? tidx(pb, {pband, pp, 1'b0}) : (ps == P_TA) ? tidx(pb, {pband, pp, 1'b1}) : tidx(wb, wbin);

  // the run buffer: 32 runs, two a beat by parity (MLAB)
  // in M10K, old data on a read of the word written (study R98: -40 ALM, +2 M10K)
  (* ramstyle = "M10K" *) logic [31:0] rre [16];
  (* ramstyle = "M10K" *) logic [31:0] rro [16];
  logic [3:0]  rr_wa;
  logic [31:0] rr_q;
  // each bank read into its own output register and the parity select after them (area pass, Model 2 R985):
  // a select in front of the register made Quartus read a clock early and forward the writes (Warning 276020,
  // 72 flip-flops of pass-through in merge4b_s3). rr_q is every clock the word the register held.
  logic [31:0] rre_q, rro_q;
  logic        rr_odd;
  always_ff @(posedge clk) begin
    if (p_rv && ps == P_CKW) begin rre[rr_wa] <= m_dout[31:0]; rro[rr_wa] <= m_dout[63:32]; end
    rre_q  <= rre[pr[4:1]];
    rro_q  <= rro[pr[4:1]];
    rr_odd <= pr[0];
  end
  assign rr_q = rr_odd ? rro_q : rre_q;

  // the FIFO: beats, {marker, word} by word parity; markers fill both halves
  // marker word: [31] 0 NEWGEN 1 BANDEND, [5:0] band
  (* ramstyle = "M10K, no_rw_check" *) logic [32:0] fe [FW];
  (* ramstyle = "M10K, no_rw_check" *) logic [32:0] fo [FW];
  logic [FB:0]   fwp;                               // beats written
  logic [FB+1:0] g_hw;                              // the reader's record head (word)
  logic          f_we;
  logic [32:0]   f_wde, f_wdo;
  always_ff @(posedge clk) if (f_we) begin fe[fwp[FB-1:0]] <= f_wde; fo[fwp[FB-1:0]] <= f_wdo; end
  wire [FB:0]  f_used = fwp - g_hw[FB+1:1];
  wire [FB:0]  f_free = (FB+1)'(FW) - f_used;
  wire         mk_room = f_free >= (FB+1)'(2);
  logic        mk_we;                               // the replay writes a marker
  logic [32:0] mk_wd;
  always_comb begin
    f_we = mk_we || (p_rv && ps == P_RW);
    f_wde = mk_we ? mk_wd : {1'b0, m_dout[31:0]};
    f_wdo = mk_we ? mk_wd : {1'b0, m_dout[63:32]};
  end

  always_ff @(posedge clk) begin
    if (rst) begin
      ps <= P_IDLE; pq_req <= 1'b0; pbk <= 1'b0; fwp <= '0; mk_we <= 1'b0; ck_v <= 1'b0;
    end else begin
      mk_we <= 1'b0;
      if (mk_we) fwp <= fwp + (FB+1)'(1);
      if (p_rv && ps == P_RW) fwp <= fwp + (FB+1)'(1);
      case (ps)
        P_IDLE: if (p_take) begin pb <= pbk; pbk <= !pbk; pband <= '0; pp <= '0; ps <= P_TO; end
        P_TO: ps <= P_TA;
        P_TA: begin eo <= t_q; ecur <= pal ? ea : t_q; ps <= P_TB; end
        P_TB: begin ea <= t_q; ecur <= pal ? t_q : eo; ps <= P_DEC; end
        P_DEC: begin
          if (!e_ov(eo) && !e_ov(ea)) ps <= P_NEXT;           // no run: no record (a bin is never only closed runs)
          else if (mk_room && !mk_we) begin
            mk_we <= 1'b1; mk_wd <= {1'b1, 1'b0, 25'd0, pband};
            pal <= !e_ov(eo); ecur <= !e_ov(eo) ? ea : eo;
            ps <= P_BIN;
          end
        end
        P_BIN: begin
          // the bin in hand: its items, and the first
          pn <= e_nr(ecur) + NRB'(1);
          popen <= pal;                                       // backward: the open run first
          pr <= pal ? e_nr(ecur) - NRB'(1) : '0;
          pnm1 <= e_nr(ecur) - NRB'(1);
          pnm1h <= NRB'((e_nr(ecur) - NRB'(1)) >> 1) + NRB'(1);
          ck_v <= 1'b0;
          ps <= P_RE;
        end
        P_RE: begin
          // the item in hand: the open run, or DDR3 run pr (its chunk loaded first)
          if (pn == '0) begin
            if (!pal && e_ov(ea)) begin pal <= 1'b1; ecur <= ea; ps <= P_BIN; end
            else ps <= P_NEXT;
          end else if (pal ? popen : (pr == e_nr(ecur))) begin rst_b <= e_st(ecur); rnb <= e_nb(ecur); ps <= P_RQ; end
          else if (ck_v && ck_c == (pr >> 5)) ps <= P_CK;    // rr_q valid next cycle
          else begin
            // load the chunk: forward pr .. the chunk's end or the last run; backward its start .. pr
            // (the beats from registers, side by side, study R109: nr - 1, the compare, the select, the
            // subtract and the add were in series; backward the chunk's beats are pr[4:1] + 1, forward
            // 16 - pr[4:1] to the chunk's end or (nr - 1) / 2 + 1 - pr / 2 to the last run)
            logic [NRB-1:0] cs;
            logic [7:0]     bl;
            cs = pal ? {pr[NRB-1:5], 5'd0} : pr;
            if (pal) bl = 8'(pr[4:1]) + 8'd1;
            else if ({pr[NRB-1:5], 5'd31} < pnm1) bl = 8'd16 - 8'(pr[4:1]);
            else bl = 8'(pnm1h - pr[NRB-1:1]);
`ifdef VERILATOR
            begin
              logic [NRB-1:0] ce;
              ce = pal ? pr : ((({pr[NRB-1:5], 5'd31}) < e_nr(ecur) - NRB'(1)) ? {pr[NRB-1:5], 5'd31} : e_nr(ecur) - NRB'(1));
              if (bl != 8'(ce[NRB-1:1] - cs[NRB-1:1]) + 8'd1 || pnm1 != e_nr(ecur) - NRB'(1)) $fatal(1, "store: the chunk's beats");
            end
`endif
            pq_req <= 1'b1;
            pq_addr <= 25'(run_a(pb, pbin, cs[BB:1]));
            pq_blen <= bl;
            rr_wa <= cs[4:1];
            ck_c <= pr >> 5;
            ps <= P_CKW;
          end
        end
        P_CKW: begin
          if (p_rv) rr_wa <= rr_wa + 4'd1;
          if (p_ack) begin pq_req <= 1'b0; ck_v <= 1'b1; ps <= P_RE; end
        end
        P_CK: begin rst_b <= rr_q[7 +: RB]; rnb <= rr_q[6:0]; ps <= P_RQ; end
        P_RQ: if (f_free > (FB+1)'(rnb) && !mk_we) begin
          pq_req <= 1'b1;
          pq_addr <= 25'(rec_a(pb, {pp, pal}, rst_b));
          pq_blen <= 8'(rnb);
          ps <= P_RW;
        end
        P_RW: if (p_ack) begin
          pq_req <= 1'b0;
          pn <= pn - NRB'(1);
          if (pal) begin if (popen) popen <= 1'b0; else pr <= pr - NRB'(1); end
          else pr <= pr + NRB'(1);
          ps <= P_RE;
        end
        P_NEXT: begin
          if (pp != 2'd3) begin pp <= pp + 2'd1; ps <= P_TO; end
          else if (mk_room && !mk_we) begin
            mk_we <= 1'b1; mk_wd <= {1'b1, 1'b1, 25'd0, pband};
            pp <= 2'd0;
            if (pband == 6'd47) ps <= P_DONE;
            else begin pband <= pband + 6'd1; ps <= P_TO; end
          end
        end
        P_DONE: if (p_rel) ps <= P_IDLE;
        default: ps <= P_IDLE;
      endcase
    end
  end

  // ===========================================================================
  // the FIFO's reader: records rebuilt for the intake
  // ===========================================================================
  // ops, one a cycle, all through one stage Y (the FIFO's registered read):
  // HEAD (word 0, or a marker), S1 (word 1 latched), C1 / C3 (words 1 and 3
  // from it), RD (a word out), DET (0)
  localparam logic [2:0] O_HEAD = 3'd0, O_S1 = 3'd1, O_C1 = 3'd2, O_C3 = 3'd3, O_RD = 3'd4, O_DET = 3'd5;
  typedef enum logic [2:0] { G_HEAD, G_S1, G_DEC, G_RUN, G_EOB } gst_t;
  gst_t gs;
  logic [FB+1:0] g_ra;                              // the word an op reads
  logic [4:0]    gq;                                // the record's word going out (0 .. nw - 1)
  logic [4:0]    g_nw;                              // words out for this record
  logic [3:0]    g_nb;                              // its beats in the FIFO
  logic          y_v, y_last, y_newm;
  logic [2:0]    y_op;
  logic          y_par;
  logic [32:0]   y_e, y_o;
  logic [31:0]   s1;
  logic          new_pend, rec_new;
  logic          mk_seen, mk_end;                   // HEAD found a marker; a BANDEND
  logic          g_tex;
  // the output FIFO, four entries
  (* ramstyle = "logic" *) logic [33:0] oq [4];   // registers: an MLAB's read here was the worst path
  logic [1:0]    oq_wp, oq_rp;
  logic [2:0]    oq_n;
  wire  [32:0]   y_w = y_par ? y_o : y_e;
  wire           emit = y_v && (y_op == O_HEAD ? !y_w[32] : (y_op != O_S1));
  wire           o_pop = o_valid && o_ready;
  wire           room = (oq_n + 3'(y_v)) < 3'd4;    // a new op's word has a place
  // a word is in the FIFO when it is behind the write pointer
  function automatic logic avail(input logic [FB+1:0] a, input logic [FB:0] wp);
    logic [FB+1:0] d;
    d = {wp, 1'b0} - a;
    avail = (d != '0) && (d <= (FB+2)'(2 * FW));
  endfunction
  // the op for word q of the record (q >= 1), and the word it reads. They are
  // registered for the word after gq as gq moves (g_op, g_rj, g_lq), so the issue
  // test starts at registers (the decode and the add were the -40C path of a fit)
  function automatic logic [7:0] g_dec(input logic [4:0] q);   // {op, j}
    logic [2:0] op;
    logic [4:0] j;
    j = 5'd0; op = O_RD;
    if (q == 5'd1) op = O_C1;
    else if (q == 5'd2) j = 5'd2;
    else if (q == 5'd3) op = O_C3;
    else if (q < 5'(H + 6)) j = q - 5'd1;
    else if (q == 5'(H + 6)) op = O_DET;
    else j = q - 5'd2;
    g_dec = {op, j};
  endfunction
  logic [2:0]    g_op;                              // = op of gq
  logic [FB+1:0] g_rj;                              // = g_hw + j of gq
  logic          g_lq;                              // = gq == g_nw - 1
  wire g_rd_ok = (g_op != O_RD) || avail(g_rj, fwp);
  wire [7:0] g_d1 = g_dec(5'd1), g_dn = g_dec(gq + 5'd1);
  logic          g_iss;                             // an op issued this cycle
  logic [2:0]    g_iop;
  logic          g_ilast;
  always_comb begin
    g_iss = 1'b0; g_iop = O_RD; g_ra = g_hw; g_ilast = 1'b0;
    case (gs)
      G_HEAD: if (room && avail(g_hw, fwp)) begin g_iss = 1'b1; g_iop = O_HEAD; end
      G_S1:   begin g_iss = 1'b1; g_iop = O_S1; g_ra = g_hw + (FB+2)'(1); end
      G_RUN:  if (room && g_rd_ok) begin
        g_iss = 1'b1; g_iop = g_op; g_ra = g_rj; g_ilast = g_lq;
      end
      default: ;
    endcase
  end
  always_ff @(posedge clk) begin
    y_e <= fe[g_ra[FB:1]]; y_o <= fo[g_ra[FB:1]];
  end
  always_ff @(posedge clk) begin
    if (rst) begin
      gs <= G_HEAD; g_hw <= '0; y_v <= 1'b0; new_pend <= 1'b0; mk_seen <= 1'b0; mk_end <= 1'b0;
      oq_wp <= '0; oq_rp <= '0; oq_n <= '0; o_band <= '0;
    end else begin
      y_v <= g_iss; y_op <= g_iop; y_par <= g_ra[0]; y_last <= g_ilast;
      y_newm <= (g_iop == O_HEAD) && new_pend;
      // Y: the marker test, latches, the word out
      if (y_v && y_op == O_HEAD) begin
        mk_seen <= y_w[32];
        mk_end <= y_w[31];
        if (y_w[32]) begin
          o_band <= y_w[5:0];
          if (!y_w[31]) new_pend <= 1'b1;
        end else begin
          g_tex <= y_w[4];
          rec_new <= new_pend;
          new_pend <= 1'b0;
        end
      end
      if (y_v && y_op == O_S1) s1 <= y_w[31:0];
      if (emit) begin
        logic [31:0] d;
        case (y_op)
          O_C1:  d = {1'b0, s1[30:20], 20'd0};
          O_C3:  d = {16'd0, 1'b0, s1[14], s1[13:8], rec_new, s1[6:0]};
          O_DET: d = 32'd0;
          default: d = y_w[31:0];
        endcase
        oq[oq_wp] <= {y_newm, y_last, d};
        oq_wp <= oq_wp + 2'd1;
      end
      if (o_pop) oq_rp <= oq_rp + 2'd1;
      oq_n <= oq_n + 3'(emit) - 3'(o_pop);
      case (gs)
        G_HEAD: if (g_iss) gs <= G_S1;
        G_S1: gs <= G_DEC;
        G_DEC: begin
          if (mk_seen) begin
            if (mk_end) gs <= G_EOB;
            else begin g_hw <= g_hw + (FB+2)'(2); gs <= G_HEAD; end
          end else begin
            g_nw <= g_tex ? 5'(H + 19) : 5'(H + 13);
            g_nb <= g_tex ? 4'd11 : 4'd8;
            gq <= 5'd1;
            g_op <= g_d1[7:5]; g_rj <= g_hw + (FB+2)'(g_d1[4:0]);
            g_lq <= 1'b0;                          // nw >= 13
            gs <= G_RUN;
          end
        end
        G_RUN: if (g_iss) begin
          gq <= gq + 5'd1;
          g_op <= g_dn[7:5]; g_rj <= g_hw + (FB+2)'(g_dn[4:0]);
          g_lq <= (gq + 5'd1 == g_nw - 5'd1);
          if (g_ilast) begin g_hw <= g_hw + (FB+2)'({g_nb, 1'b0}); gs <= G_HEAD; end
        end
        G_EOB: if (o_next) begin g_hw <= g_hw + (FB+2)'(2); mk_seen <= 1'b0; gs <= G_HEAD; end
        default: gs <= G_HEAD;
      endcase
    end
  end
  wire [33:0] oq_h = oq[oq_rp];
  assign o_valid = oq_n != '0;
  assign o_data  = oq_h[31:0];
  assign o_last  = oq_h[32];
  assign o_new   = oq_h[33];
  assign o_eob   = (gs == G_EOB) && !y_v && oq_n == '0;

endmodule
