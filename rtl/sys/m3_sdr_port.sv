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
//               With L16 (study R107) the line is one sixteen-column read from the critical
//               pair, wrapping (m2_sdram L16): four beats a group, the groups on p_lo three
//               times and the last with p_ack
//               With RA (study R107) the port keeps two stream buffers of a line each. A line read
//               (a fill, or a hit below) arms a buffer for the line after it: the older buffer after a
//               fill, the same buffer after a hit. An armed buffer is read ahead, as one sixteen-column
//               read from the line's word 0, whenever the port is otherwise idle. A line read of a
//               buffered line is answered from the buffer, a beat a clock, critical word first, and a
//               line read of the line being read ahead takes its beats as they arrive. Only word-form
//               reads below RA_TOP (the tile RAM: CROM, the fixed CROM, the backup RAM, work RAM, which
//               only this port writes while the game runs) are read ahead. A write through the port
//               drops a buffered or armed copy of its line; ra_clr drops both.
//               With PW (study R107) the eight beats go into a write buffer of their own and the
//               port is free once the last is in; its two bursts are posted and go when the port
//               has nothing else to do. A read of another line goes first; a write, or a read of
//               that line, waits for them (so nothing is reordered against the same line, and
//               writes stay in order). The bursts may be split by a read between the two.
//   MC_LINE_WR  eight beats taken one per s_wnext, a beat every four clocks (every clock with LW1),
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
  parameter int AW = 26,
  parameter bit L16 = 1'b0,        // the line fill as one sixteen-column read (m2_sdram's L16 for this port)
  parameter bit LW1 = 1'b0,        // a cast-out's beats one a clock, not one in four (study R107)
  parameter bit PAIR = 1'b0,       // with L16: the line read from the critical word, a beat on each p_lo (m2_sdram PAIR)
  parameter bit RA = 1'b0,         // with L16 and PAIR: the next line read ahead into a two-line buffer (study R107)
  parameter logic [26:0] RA_TOP = 27'h6C0_0000,  // RA: lines below this SDRAM byte address only (m3_system: SDR_VRAM)
  parameter bit RA_CONF = 1'b0,    // RA: a fill arms a buffer only when it is of the line after the last fill's (a stream seen)
  parameter bit PW = 1'b0,         // a line write's bursts posted from a buffer of their own; reads of other lines go first (study R107)
  parameter bit FWD = 1'b0,        // with L16 and PAIR: each beat to the bus in the clock it arrives, not from the buffer a clock later (R112)
  parameter bit FREQ = 1'b0,       // a read's request to m2_sdram in the clock the port takes it, not a clock later (R112)
  parameter int PW_DLY = 0         // PW: a posted line write waits this many clocks before its first burst unless something needs it (R112)
) (
  input  logic          clk,
  input  logic          rst,
  input  logic          ra_clr,        // RA: the read-ahead buffer emptied (the game's reset; downloads write SDRAM past the port)

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

  typedef enum logic [3:0] { T_IDLE, T_RD, T_LR, T_WR, T_LW, T_ZERO, T_HIT, T_RA, T_PW } st_t;
  st_t          st;
  logic [1:0]   cmd;
  logic [AW:0]  sa;
  logic         swap;
  logic [3:0]   be;
  logic [31:0]  wd;

  // ---------------------------------------------------------------- the port
  // pr_req rises once per transaction and falls with its acknowledge; `gap`
  // keeps it low for a cycle before the next.
  logic         gap;
  logic         pr_req, pr_we, pr_long;   // the request as registered
  logic [AW:1]  pr_addr;
  logic         half;            // T_LR: the burst in flight is the second half
  logic         wh;              // T_LW: the burst is the second half (its own register: it selects the column data, study R107)
  logic [2:0]   wbeat;           // T_LW: beats taken
  logic         wfull;           // T_LW: all eight beats taken
  logic [1:0]   wph;             // T_LW: the beat clock (one s_wnext in four)
  logic [3:0]   ws;              // T_LW with LW1: s_wnext sent
  logic [1:0]   wq;              // T_LW with LW1: s_wnext one and two clocks back
  logic [7:0]   bv;              // T_LR / T_ZERO: beats arrived
  // The line buffer: even words in one MLAB, odd words in another (a burst's pair writes one of each, a
  // single beat one), read asynchronously at one address (study R110: it was 256 flip-flops and two 8:1
  // selects). A line write's words 0 and 4, which go with its bursts' requests, are also kept in registers.
  (* ramstyle = "MLAB, no_rw_check" *) logic [31:0] bd_e [4];
  (* ramstyle = "MLAB, no_rw_check" *) logic [31:0] bd_o [4];
  logic [31:0]  lr0, lr4;
  logic [2:0]   ek;              // beats sent
  logic [2:0]   crit;
  logic [1:0]   grp;             // T_LR with L16: the four-word groups arrived

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
  // PW: the posted line write's (T_PW) from its own buffer (an MLAB, one beat written a clock), its half
  // pw_h and form pw_sw
  (* ramstyle = "MLAB, no_rw_check" *) logic [31:0] wbuf [8];
  logic             pw_v, pw_h, pw_sw;
  logic [AW:5]      pw_ln;
  // (with PW only T_PW writes a line, so its buffer, half and form are chosen by the parameter, not by a
  // mux: the column data path, wb_col through the 8:1 into m2_sdram's din_r, is as long as without PW)
  // lw: a line write is the port's (T_LW, or T_PW), as a register of its own set and cleared with those
  // states (study R107: the state decode in front of the column data was PW's worst path)
  logic         lw;
  wire          lwh   = PW ? pw_h : wh;
  // the beat to send next, crit + ek, kept in a register beside ek (the sum in front of the buffer's read
  // address was the block's worst path)
  logic [2:0]   enext;
  // the line buffer's one read: without PW the write column's beat in T_LW, else the beat to send
  wire  [2:0]   bra   = (!PW && lw) ? {wh, wb_col[2:1]} : enext;
  wire  [31:0]  bq    = bra[0] ? bd_o[bra[2:1]] : bd_e[bra[2:1]];
  wire  [31:0]  lw_w0 = lwh ? lr4 : lr0;
  wire  [31:0]  lw_wk = PW ? wbuf[{pw_h, wb_col[2:1]}] : bq;
  wire          lw_sw = swap;            // (T_PW sets swap to the posted line's form as it begins)
  assign {p_be, p_din}   = col16(lw ? lw_w0 : wd, lw ? 4'hF : be, lw_sw, 1'b0);
  assign {wb_be, wb_din} = col16(lw ? lw_wk : wd, lw ? 4'hF : be, lw_sw, wb_col[0]);
  // PW: a request that must wait for the posted write (a write, or a read of its line). The line compare
  // is registered (pw_cmp: s_sa is held with s_req), so a read that comes up while a write is posted
  // waits a clock for it (pw_wait) rather than putting a 22-bit compare in front of the accept
  logic         pw_cmp, s_req_q;
  // a line write posted at this edge: pw_cmp, formed now, compares with the old pw_ln, so a read up now
  // waits a clock more (R112: with m3_bus's FAST a read can be up in the clock the last beat is taken)
  wire          pw_post = PW && (st == T_LW) && (LW1 ? wq[1] : s_wnext) && wbeat == 3'd7;
  always_ff @(posedge clk) begin
    pw_cmp  <= s_sa[AW:5] == pw_ln;
    s_req_q <= s_req && !s_ack && !pw_post;
  end
  // PW_DLY (R112): the posted write's first burst waits PW_DLY clocks after the post, so that a read coming
  // soon after (the D-cache's next fill) need not wait behind a burst; it goes at once when a request needs
  // it (a write, or a read of its line, compared), and the second burst follows the first as before
  logic [5:0]   pw_age;
  always_ff @(posedge clk)
    if (pw_post) pw_age <= '0;
    else if (pw_v && pw_age != 6'h3F) pw_age <= pw_age + 6'd1;
  wire          pw_need = s_req && !s_ack && (s_cmd[0] || (s_req_q && pw_cmp));
  wire          pw_due  = (PW_DLY == 0) || pw_h || pw_need || (pw_age >= 6'(PW_DLY));
  wire          pw_blk  = PW && pw_v && (s_cmd[0] || !s_req_q || pw_cmp);
  wire          pw_wait = PW && pw_v && s_req && !s_ack && !s_cmd[0] && !s_req_q;

  // T_LR: the second half has arrived (no more requests)
  wire       half_done = half && bv[3'({crit[2:1], 1'b0} + 3'd7)];
  // the line word of a burst's beat j (0-3): the bursts start at the critical pair and its +4, wrapping
  wire [2:0] lw0 = {crit[2:1], 1'b0} + {half, 2'b00};
  wire [2:0] lwg = {crit[2:1], 1'b0} + {grp, 1'b0};   // L16: the group's first line word
  logic [2:0] pk;                // L16 with PAIR: beats arrived

  // ---------------------------------------------------------- read-ahead (RA)
  // two line buffers in one MLAB-sized array, {buffer, beat}; for each, rl the line (SDRAM byte
  // address bits AW:5), rv valid (read), ra armed (to be read ahead); rlru the buffer a fill arms
  // next; ra_b the buffer being read ahead; ra_m: a line read took the read-ahead in flight
  (* ramstyle = "MLAB, no_rw_check" *) logic [31:0] rab [0:15];
  logic [1:0]       rv, ra;
  logic [1:0][AW:5] rl;
  logic             rlru, ra_b, hb, ra_m;
  logic [AW:5]      lf_nx;          // RA_CONF: the line after the last line fill's
  logic [AW:5]      sa_nx;          // RA: the line after this transaction's, formed as it is taken (timing)
  wire  [1:0]       lm   = {rl[1] == s_sa[AW:5], rl[0] == s_sa[AW:5]};   // the request's line is a buffer's
  wire  [1:0]       rhit = RA ? (rv & lm) : 2'b00;
  wire              ra_wr = (st == T_RA) && (p_lo || p_ack);   // a read-ahead beat
  wire              rd_ok = !s_zero && !s_swap;                 // a request the buffers may answer
  // T_RA: a line read of the line being read ahead is taken now
  wire              ra_take = RA && (st == T_RA) && !ra_m && s_req && !s_ack && s_cmd == MC_LINE_RD && rd_ok && s_sa[AW:5] == rl[ra_b];
  always_ff @(posedge clk) if (RA && ra_wr) rab[{ra_b, pk}] <= p_dout[31:0];
  // the arming's compares, registered (R112: RA_CONF's sa == lf_nx into the arming was the spike's worst
  // path). sa, sa_nx, lf_nx and rl are set as a transaction is taken or as it ends, and a fill or a hit
  // arms at its eighth beat, at least eight clocks after it was taken, so each is the compare of the values
  // that stand when it is used (checked below)
  logic             ar_ok_q;                       // a line that may be read ahead (form, region, a stream seen)
  logic [1:0]       ar_nx_q;                       // a buffer already has the line after this one (with rv / ra)
  always_ff @(posedge clk) begin
    ar_ok_q <= !swap && sa[AW:5] < RA_TOP[AW:5] && (!RA_CONF || sa[AW:5] == lf_nx);
    ar_nx_q <= {rl[1] == sa_nx, rl[0] == sa_nx};
  end

  // the line buffer's writes: a pair (words a, a + 1, a even: one each side) or one word (the side a[0])
  wire  [31:0]  pd_lo = swap ? bswap(p_dout[31:0])  : p_dout[31:0];
  wire  [31:0]  pd_hi = swap ? bswap(p_dout[63:32]) : p_dout[63:32];
  wire          lw_beat = (st == T_LW) && (LW1 ? wq[1] : s_wnext);   // a cast-out beat in s_wdata
  logic         bw_pair, bw_one;
  logic [2:0]   bw_a;
  logic [31:0]  bw_lo;
  always_comb begin
    bw_pair = 1'b0; bw_one = 1'b0; bw_a = wbeat; bw_lo = pd_lo;
    if (RA && ra_wr) begin                                     // a read-ahead beat, word form
      bw_one = 1'b1; bw_a = pk; bw_lo = p_dout[31:0];
    end else if (st == T_LR && L16 && PAIR) begin              // a beat a group, from the critical word
      bw_one = p_lo || p_ack; bw_a = crit + pk;
    end else if (st == T_LR && L16) begin                      // a pair a group
      bw_pair = p_lo || p_ack; bw_a = lwg;
    end else if (st == T_LR) begin                             // a burst's two pairs
      bw_pair = p_lo || p_ack; bw_a = p_ack ? lw0 + 3'd2 : lw0;
    end else if (!PW && lw_beat) begin                         // a cast-out beat (with PW it goes to wbuf)
      bw_one = 1'b1; bw_lo = s_wdata;
    end
  end
  always_ff @(posedge clk) begin
    if (bw_pair || (bw_one && !bw_a[0])) bd_e[bw_a[2:1]] <= bw_lo;
    if (bw_pair || (bw_one &&  bw_a[0])) bd_o[bw_a[2:1]] <= bw_pair ? pd_hi : bw_lo;
    if (lw_beat && wbeat == 3'd0) lr0 <= s_wdata;
    if (lw_beat && wbeat == 3'd4) lr4 <= s_wdata;
    if (PW && lw_beat) wbuf[wbeat] <= s_wdata;
  end

  // FREQ (R112): a read the port takes in T_IDLE has its request up to m2_sdram in the same clock (the
  // registered pr_req follows from the next), so m2_sdram's mailbox latches it a clock sooner. Only where
  // the registered form would raise it at once (no gap) and only for a read that goes to SDRAM (not a
  // zero read, not a read-ahead buffer's line). The address is the one T_IDLE registers.
  wire          acc_rd  = (st == T_IDLE) && s_req && !s_ack && !pw_blk && !s_cmd[0] && !s_zero &&
                          !(RA && s_cmd == MC_LINE_RD && rd_ok && |rhit);
  wire          p_fast  = FREQ && acc_rd && !gap;
  wire [AW:1]   fa_rd   = (s_cmd == MC_LINE_RD) ? ((L16 && PAIR) ? {s_sa[AW:5], s_sa[4:2], 1'b0} : {s_sa[AW:5], s_sa[4:3], 2'b00})
                                                : {s_sa[AW:3], 2'b00};
  // (in T_IDLE the registered request is never up, and m2_sdram takes the address only as a request rises,
  // so the fields are chosen by the state alone: the accept is not in front of them)
  wire          fsel    = FREQ && (st == T_IDLE);
  assign p_req  = pr_req || p_fast;
  assign p_addr = fsel ? fa_rd : pr_addr;
  assign p_we   = fsel ? 1'b0 : pr_we;
  assign p_long = fsel ? (s_cmd == MC_LINE_RD) : pr_long;
  // FWD (R112): with L16 and PAIR the beats arrive in the order they are sent (the critical word, then each
  // one after it), one at most a clock, and the bus takes a beat every clock, so each goes to the bus in the
  // clock it arrives: then no arrived beat is ever left unsent (pk == ek at every arrival, checked below) and
  // the line buffer is not read in T_LR. (A first form also required pk == ek in front of the send; the
  // compare and its fall-back to the buffer put the arrival, the beat count and the buffer's read address
  // in one path, the spike's worst.)
  wire          fwd     = FWD && L16 && PAIR && (st == T_LR) && (p_lo || p_ack);
  wire          lr_bv   = !(FWD && L16 && PAIR) && bv[enext];   // T_LR: a beat sent from the line buffer

  always_ff @(posedge clk) begin
    s_ack <= 1'b0; s_wnext <= 1'b0; s_rvalid <= 1'b0;
    if (rst) begin
      st <= T_IDLE; pr_req <= 1'b0; gap <= 1'b0; rv <= '0; ra <= '0; rlru <= 1'b0; pw_v <= 1'b0; lw <= 1'b0;
    end else begin
      if (RA && ra_clr) begin rv <= '0; ra <= '0; end
      gap <= 1'b0;
      case (st)
        T_IDLE: if (RA && s_req && !s_ack && !pw_blk && s_cmd == MC_LINE_RD && rd_ok && |rhit) begin
          // a buffered line: from the buffer, a beat a clock (R107)
          cmd <= s_cmd; sa <= s_sa; sa_nx <= s_sa[AW:5] + 1'b1; swap <= s_swap; crit <= s_sa[4:2]; ek <= '0;
          enext <= s_sa[4:2];
          s_ack <= 1'b1; hb <= rhit[1]; rlru <= !rhit[1]; st <= T_HIT;
        end else if (s_req && !s_ack && !pw_blk) begin
          cmd <= s_cmd; sa <= s_sa; sa_nx <= s_sa[AW:5] + 1'b1; swap <= s_swap; be <= s_be; wd <= s_wdata;
          // a write drops a copy of its line, read or armed; a line read of an armed line reads it
          // itself (its end arms the line after, as for any fill)
          if (RA && (s_cmd[0] || s_cmd == MC_LINE_RD)) begin
            if (lm[0]) begin rv[0] <= 1'b0; ra[0] <= 1'b0; end
            if (lm[1]) begin rv[1] <= 1'b0; ra[1] <= 1'b0; end
          end
          crit <= (s_cmd == MC_LINE_RD) ? s_sa[4:2] : 3'd0;
          enext <= (s_cmd == MC_LINE_RD) ? s_sa[4:2] : 3'd0;
          bv <= '0; ek <= '0; half <= 1'b0; wh <= 1'b0; wbeat <= '0; wfull <= 1'b0; wph <= '0; grp <= '0; ws <= '0; wq <= '0; pk <= '0;
          s_ack <= 1'b1;
          if (s_zero && !s_cmd[0]) begin
            st <= T_ZERO; bv <= (s_cmd == MC_RD) ? 8'h01 : 8'hFF;
          end else case (s_cmd)
            MC_RD: begin
              st <= T_RD; pr_req <= !gap; pr_we <= 1'b0; pr_long <= 1'b0;
              pr_addr <= {s_sa[AW:3], 2'b00};
            end
            MC_LINE_RD: begin
              st <= T_LR; pr_req <= !gap; pr_we <= 1'b0; pr_long <= 1'b1;
              pr_addr <= (L16 && PAIR) ? {s_sa[AW:5], s_sa[4:2], 1'b0}    // R107: from the critical word itself
                                      : {s_sa[AW:5], s_sa[4:3], 2'b00};  // R104: from the critical word's pair, wrapping in the line
            end
            MC_WR: st <= T_WR;
            default: begin st <= T_LW; lw <= 1'b1; end     // MC_LINE_WR
          endcase
        end else if (pw_wait) begin
          // a read just up, its line compared next clock
        end else if (PW && pw_v && !gap && pw_due) begin
          st <= T_PW; swap <= pw_sw; lw <= 1'b1;   // the posted line write's next burst, in its own form
        end else if (RA && |ra && !gap && !s_req && !pw_v) begin
          // the port is idle: a read-ahead, the armed buffer a fill or hit armed last first, as a
          // sixteen-column read from the line's word 0
          ra_b <= (ra[0] && ra[1]) ? !rlru : ra[1];
          ra[(ra[0] && ra[1]) ? !rlru : ra[1]] <= 1'b0;
          st <= T_RA; pk <= '0; bv <= '0; ra_m <= 1'b0; ek <= '0; enext <= crit;
          pr_req <= 1'b1; pr_we <= 1'b0; pr_long <= 1'b1;
          pr_addr <= {rl[(ra[0] && ra[1]) ? !rlru : ra[1]], 4'b0000};
        end

        T_HIT: begin
          s_rvalid <= 1'b1; s_rdata <= rab[{hb, enext}]; ek <= ek + 3'd1; enext <= enext + 3'd1;
          if (ek == 3'd7) begin
            st <= T_IDLE;
            // and the line after it, into the same buffer, unless the other holds it or is armed for it
            if (!(ar_nx_q[!hb] && (rv[!hb] || ra[!hb]))) begin
              ra[hb] <= 1'b1; rv[hb] <= 1'b0; rl[hb] <= sa_nx;
            end
          end
        end

        T_PW: begin
          // one burst, the half pw_h; back to idle after it, so a waiting read may go between the two
          if (!pr_req && !gap && !p_ack) begin
            pr_req <= 1'b1; pr_we <= 1'b1; pr_long <= 1'b1; pr_addr <= {pw_ln, pw_h, 3'b000};
          end
          if (p_ack) begin
            pr_req <= 1'b0; gap <= 1'b1; st <= T_IDLE; lw <= 1'b0;
            if (pw_h) pw_v <= 1'b0;
            pw_h <= 1'b1;
          end
        end

        T_RA: begin
          // the beats, into the buffer and (for a line read that takes this one) the line buffer
          if (ra_wr) begin pk <= pk + 3'd1; bv[pk] <= 1'b1; end
          if (ra_take) begin
            cmd <= s_cmd; sa <= s_sa; sa_nx <= s_sa[AW:5] + 1'b1; swap <= s_swap; crit <= s_sa[4:2]; ek <= '0;
            enext <= s_sa[4:2];
            s_ack <= 1'b1; ra_m <= 1'b1; rlru <= !ra_b;
          end
          if (ra_m && bv[enext]) begin
            s_rvalid <= 1'b1; s_rdata <= bq; ek <= ek + 3'd1; enext <= enext + 3'd1;
          end
          if (p_ack) begin
            pr_req <= 1'b0; gap <= 1'b1; rv[ra_b] <= 1'b1;
            // a line read that took it goes on from the buffer, from the beat after the one sent now
            // (taken in this very clock: from the first beat)
            if (ra_m || ra_take) begin st <= T_HIT; hb <= ra_b; if (ra_m && bv[enext]) begin ek <= ek + 3'd1; enext <= enext + 3'd1; end end
            else st <= T_IDLE;
          end
        end

        T_ZERO: begin
          // zeros: as many beats as the command wants
          if (bv[enext]) begin
            s_rvalid <= 1'b1; s_rdata <= 32'd0; ek <= ek + 3'd1; enext <= enext + 3'd1;
            if ((cmd == MC_RD) || ek == 3'd7) st <= T_IDLE;
          end
        end

        T_RD: begin
          if (!pr_req && !gap && !p_ack) pr_req <= 1'b1;
          if (p_ack) begin
            pr_req <= 1'b0; gap <= 1'b1;
            s_rvalid <= 1'b1;
            s_rdata <= swap ? bswap(sa[2] ? p_dout[63:32] : p_dout[31:0]) : (sa[2] ? p_dout[63:32] : p_dout[31:0]);
            st <= T_IDLE;
          end
        end

        T_LR: if (L16) begin
          // one request: four groups of two beats, from the critical pair, wrapping (R107)
          if (!pr_req && !gap && !p_ack && !half) pr_req <= 1'b1;
          if ((p_lo || p_ack) && PAIR) begin
            bv[crit + pk] <= 1'b1;
            pk <= pk + 3'd1;
          end else if (p_lo || p_ack) begin
            bv[lwg] <= 1'b1; bv[lwg + 3'd1] <= 1'b1;
            grp <= grp + 2'd1;
          end
          if (p_ack) begin pr_req <= 1'b0; gap <= 1'b1; half <= 1'b1; end
          if (lr_bv || fwd) begin
            s_rvalid <= 1'b1; s_rdata <= fwd ? pd_lo : bq; ek <= ek + 3'd1; enext <= enext + 3'd1;
            if (ek == 3'd7) begin
              st <= T_IDLE;
              // R107: read the next line ahead, into the older buffer, unless one holds it
              if (RA) lf_nx <= sa_nx;
              if (RA && ar_ok_q && !(ar_nx_q[0] && (rv[0] || ra[0])) && !(ar_nx_q[1] && (rv[1] || ra[1]))) begin
                ra[rlru] <= 1'b1; rv[rlru] <= 1'b0; rl[rlru] <= sa_nx; rlru <= !rlru;
              end
            end
          end
        end else begin
          if (!pr_req && !gap && !p_ack && !half_done) pr_req <= 1'b1;
          // arrivals: words lw0 .. lw0 + 3 of the line (R104: from the critical pair, wrapping)
          if (p_lo) begin
            bv[lw0] <= 1'b1; bv[lw0 + 3'd1] <= 1'b1;
          end
          if (p_ack) begin
            bv[lw0 + 3'd2] <= 1'b1; bv[lw0 + 3'd3] <= 1'b1;
            pr_req <= 1'b0; gap <= 1'b1;
            if (!half) begin
              half <= 1'b1;
              pr_addr <= {sa[AW:5], !sa[4], sa[3], 2'b00};
            end
          end
          // out, critical word first
          if (bv[enext]) begin
            s_rvalid <= 1'b1; s_rdata <= bq; ek <= ek + 3'd1; enext <= enext + 3'd1;
            if (ek == 3'd7) st <= T_IDLE;
          end
        end

        T_WR: begin
          // one two-column burst: the low word (A/2), then the high (A/2 + 1)
          if (!pr_req && !gap && !p_ack && !half) begin
            pr_req <= 1'b1; pr_we <= 1'b1; pr_long <= 1'b0; pr_addr <= {sa[AW:2], 1'b0}; half <= 1'b1;
          end
          if (p_ack) begin pr_req <= 1'b0; gap <= 1'b1; st <= T_IDLE; end
        end

        T_LW: begin
          // the beats, one s_wnext every four clocks: the bus passes it on a
          // clock later and the master shows the next beat the clock after
          // that, as for the bus's own line writes (B_BEAT, three clocks)
          // LW1 (study R107): eight s_wnext on eight clocks, each beat taken two clocks after its
          // s_wnext. The bus passes s_wnext on as m_wnext a clock later; the master's beat count
          // steps at that edge and it shows the next beat combinationally from it (ppc_dcache's
          // cbm[wb_cnt], m3_scripts' fifo[rp]); the bus registers the beat into s_wdata every
          // clock, the clock after its last m_wnext included. So with s_wnext high from clock t,
          // s_wdata holds beat k at clock t + k + 2.
          if (LW1) begin
            wq <= {wq[0], s_wnext};
            if (!ws[3]) begin s_wnext <= 1'b1; ws <= ws + 4'd1; end
            if (wq[1]) begin
              wbeat <= wbeat + 3'd1;
              if (wbeat == 3'd7) wfull <= 1'b1;
            end
          end else begin
            wph <= wph + 2'd1;
            if (wph == 2'd0 && !wfull) s_wnext <= 1'b1;
            if (s_wnext) begin
              wbeat <= wbeat + 3'd1;
              if (wbeat == 3'd7) wfull <= 1'b1;
            end
          end
          // PW: the beats go into the write buffer instead (above); once the last is in, the bursts are posted
          if (PW && (LW1 ? wq[1] : s_wnext) && wbeat == 3'd7) begin
            st <= T_IDLE; lw <= 1'b0; pw_v <= 1'b1; pw_h <= 1'b0; pw_ln <= sa[AW:5]; pw_sw <= swap;
          end
          // the halves: the first once beats 0-3 are in, the second once all are
          if (!PW && !pr_req && !gap && !p_ack && (wh ? wfull : (wbeat[2] || wfull))) begin
            pr_req <= 1'b1; pr_we <= 1'b1; pr_long <= 1'b1; pr_addr <= {sa[AW:5], wh, 3'b000};
          end
          if (p_ack) begin
            pr_req <= 1'b0; gap <= 1'b1;
            if (wh) begin st <= T_IDLE; lw <= 1'b0; end
            wh <= 1'b1;
          end
        end

        default: st <= T_IDLE;
      endcase
    end
  end

  assign s_idle = ((st == T_IDLE) || (st == T_RA)) && !(PW && pw_v);   // (a read-ahead is not a posted write; PW's is)

`ifdef VERILATOR
  // FREQ: in T_IDLE the registered request is down (the fields are chosen by the state alone, above)
  always_ff @(posedge clk)
    if (!rst && FREQ && st == T_IDLE && pr_req) $fatal(1, "m3_sdr_port: FREQ, the registered request up in T_IDLE");
  // FWD: every beat of a PAIR fill arrives when the ones before it have gone (R112: the pk == ek guard, which
  // falls back to the line buffer, is never needed while the bus takes a beat each clock)
  always_ff @(posedge clk)
    if (!rst && FWD && L16 && PAIR && st == T_LR && (p_lo || p_ack) && pk != ek)
      $fatal(1, "m3_sdr_port: FWD, a beat arrived with an earlier one still to send");
  // the arming's registered compares are the live ones wherever they are used (R112)
  always_ff @(posedge clk)
    if (!rst && RA && (st == T_LR || st == T_HIT) && ek == 3'd7 && (st == T_HIT || lr_bv || fwd) &&
        (ar_ok_q != (!swap && sa[AW:5] < RA_TOP[AW:5] && (!RA_CONF || sa[AW:5] == lf_nx)) ||
         ar_nx_q != {rl[1] == sa_nx, rl[0] == sa_nx}))
      $fatal(1, "m3_sdr_port: the read-ahead's registered compares");
  // enext is crit + ek wherever a beat is sent from it
  always_ff @(posedge clk)
    if (!rst && (st == T_LR || st == T_HIT || st == T_ZERO || (st == T_RA && ra_m)) && enext != 3'(crit + ek))
      $fatal(1, "m3_sdr_port: enext");
`endif


endmodule
