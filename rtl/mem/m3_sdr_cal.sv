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
// sys140: the read-capture calibration of one SDRAM, at power-up. The capture clock (m2_sdram CAPCLK) is stepped
// through every PLL step of its period (NSTEP = 48 at 140 MHz from an 840 MHz VCO: 148.8 ps a step; the PLL's VCO is
// held there by the IP tool's physical parameters, Model3.sv) and at each the SDRAM's reads are checked; the capture
// phase is then set to the centre of the widest passing window (wrap-around aware), held inside the band the timing
// analysis covers (BAND_LO steps below the generated phase to BAND_HI above: Model3.sdc's hold / setup uncertainty on
// the capture clocks' transfers into clk_sys), and the reads are checked once more there (the verify pass). BAND_HI
// is 14: the first SDRAM's dq_r -> cap transfer has 2.22-2.35 ns of setup slack at slow 100 C before the band (t7b
// 2.349, combo7_s2 2.222), so 16 steps (2.381 ns) and 15 (2.232) each failed it in one fit; that is also where, at
// that corner, the capture word starts to arrive a clk_sys clock late, so the window as the reads see it ends there.
//
// It is a master of m3_sdr_port's s_* port (m3_bus's protocol), so the reads are the CPU's: sixteen-column line
// fills at full rate, a beat a clock, after line writes (cast-outs) of the same lines:
//   - NL lines, spread over the banks (byte bits 12:11), rows (16:14) and columns, and with TWO_CHIPS both chips
//     (byte bit 26); with TWO_CHIPS 0 every address keeps BASE's bits 26:24 (the second SDRAM: its controller
//     serves its second chip only, DCHIP1);
//   - four patterns by line: walking ones and zeros, the DQs toggling each column (0000 / FFFF), 5555 / AAAA
//     alternating, and a hash of the line and beat; written once, then read REPS times at each step, every beat
//     compared; a step passes only if every beat of every pass is right. Every word of a line differs from the
//     others and every 16-bit column from its neighbours (the walking bit modulo 16, the toggling patterns' low
//     four bits of each half the column's number), so a read a column or a word early or late cannot compare equal
//     (R198: before t7f the walking bit ran off the word, leaving lines of equal words; no line could pass a slip
//     even then, but the margin was in the hash lines alone).
// The writes do not depend on the capture phase; the phase is moved only with no read outstanding.
//
// The stepper (Model3.sv, on clk_hps) takes the offset `off` (0-47 steps from the generated phase, modulo the period)
// and reports `settled` when the counter is there; after a change the calibration waits 16 clocks and then for
// `settled`, and 512 more before it reads.
//
// Results (static after `done`): the window's width (0-48), its last step, the chosen offset and flags: none (no step
// passes: the phase stays at 0), narrow (width under WMIN), all (every step passes: the test cannot see an edge, so
// the phase stays at 0), clamped (the window went on past the band's top: the late side held there), out (nothing
// of the window left inside the band after the early margin: offset 0, then verified like any choice), verify (the reads at the chosen offset, after the stepper moved there, went wrong: the sweep's offsets
// were not where the stepper put the clock; the phase goes back to 0). After done the offset follows the OSD:
// `osd` 0 is Auto (the chosen offset), 1-23 the manual offsets of MAN (single steps over the band).
// R203 (the board: the early edge the test sees moves 10 steps from boot to boot, cold to warm, and lies up to 12
// steps earlier than where the games fail): NSW sweeps intersected, HOSTILE traffic (each read straight after a
// write elsewhere), and the choice the middle of [first + EDGE_M, min(last, BAND_HI)] (also held above -BAND_LO),
// not the window's centre.
// `done` is held low until all this is over: m3_system keeps the loader (and so the HPS's downloads, through
// ioctl_wait) and the game off this SDRAM until then. EN 0: done at once, offset from the OSD alone.

// area-sys (one engine for both SDRAMs): with NS 2 the one engine calibrates the second SDRAM (port B) and then the
// first (port A), one after the other: 128 ms more before the first download (256 ms in all), and about 200 ALM
// less than two calibrators. Each run is the one-SDRAM run, transaction for transaction and clock for clock from
// its start. PARTNER keeps R203's cross traffic, which two calibrations running together gave: every transaction
// the engine starts on the SDRAM it calibrates also goes to the other SDRAM's port when that port is idle (its
// address forced into range by the caller; its read data ignored, its write data whatever s_wdata shows), so the
// other SDRAM's bus switches while this one's reads come in, as when both ran their own loops. The other SDRAM is
// not yet calibrated (B first: A is still empty) or not yet in use (A second: the game is still held), so nothing
// it holds is overwritten; the caller moves the partner copies' addresses where nothing is (s_pb says which SDRAM
// is under calibration). B is skipped (A alone) when skip_b says the module is not there. The second SDRAM's
// results are kept in registers of their own once its run is over; the first's are the engine's, as before.
// The pass map is a shift register (a step's verdict shifted in at C_NEXT, the scan a rotation of two turns), so
// no step decode and no 48:1 select: the same map, the same scan, the same clocks.

module m3_sdr_cal
  import ppc_cache_pkg::*;
#(
  parameter int          AW        = 26,
  parameter bit          EN        = 1'b1,
  parameter int          NS        = 1,             // SDRAMs: 1 (port A) or 2 (B, then A)
  parameter int          NL        = 64,            // lines (32 bytes each), a power of two up to 64
  parameter int          REPS      = 16,            // read passes at each step (up to 64)
  parameter int          NSW       = 4,             // sweeps, their pass maps intersected (up to 8)
  parameter bit          HOSTILE   = 1'b1,          // every line read straight after a line write elsewhere
  parameter bit          PARTNER   = 1'b1,          // NS 2: the other SDRAM takes the same transactions when idle
  parameter int          EDGE_M    = 12,            // steps of margin taken off the window's early edge
  parameter int          NSTEP     = 48,            // PLL steps in a period
  parameter int          BAND_LO   = 8,             // the analysed band: BAND_LO steps below the generated phase
  parameter int          BAND_HI   = 14,            //   to BAND_HI above (BAND_LO + BAND_HI < NSTEP)
  parameter int          WMIN      = 6,             // a window narrower than this is flagged
  parameter logic [26:0] BASE      = 27'h000_0000,  // port A's lines
  parameter bit          TWO_CHIPS = 1'b1,
  parameter logic [26:0] BASE_B    = 27'h000_0000,  // port B's (NS 2)
  parameter bit          TWO_B     = 1'b0,
  parameter int          SETTLE    = 512            // clocks after the phase is there, before the reads
) (
  input  logic                clk,
  input  logic                rst,           // the memory's reset (PLL lock)
  input  logic [NS-1:0]       start,         // each controller is up (B: m3_sdr2's probe passed, the module there)
  input  logic                skip_b,        // NS 2: B will not start (no module): A alone
  output logic [NS-1:0]       done,
  output logic                busy,          // the engine still owns the ports (some SDRAM not yet done)
  output logic                s_pb,          // NS 2: B is the SDRAM under calibration (A takes the partner copies)

  // m3_sdr_port's s_* (the bus's side, which m3_system / m3_sdr2 switch to this while the engine owns them): the
  // command fields shared, a request each
  output logic [NS-1:0]       s_req,
  output logic [1:0]          s_cmd,
  output logic [AW:0]         s_sa,
  output logic [3:0]          s_be,
  output logic [31:0]         s_wdata,
  output logic [3:0]          s_wbe,
  input  logic [NS-1:0]       s_ack,
  input  logic [NS-1:0]       s_wnext,
  input  logic [NS-1:0]       s_rvalid,
  input  logic [NS-1:0][31:0] s_rdata,
  input  logic [NS-1:0]       s_idle,        // PARTNER: the port has nothing in hand (m3_sdr_port s_idle)

  // each capture clock's offset (to the stepper) and its state
  input  logic [NS-1:0][4:0]  osd,           // 0 Auto, 1-23 MAN[osd]
  output logic [NS-1:0][5:0]  off,
  input  logic [NS-1:0]       settled,

  output logic [NS-1:0][6:0]  c_width,
  output logic [NS-1:0][5:0]  c_last,        // the window's last step (0 .. NSTEP - 1)
  output logic [NS-1:0][5:0]  c_off,
  output logic [NS-1:0][5:0]  c_flags,       // {verify, out, clamped, all, narrow, none}
  output logic [5:0]          c_centre,      // the run's (the bench's)
  output logic [NSTEP-1:0]    pass_map       // the run's: step j in bit j at every step's start and at the end
);
  localparam int LB = $clog2(NL);
  localparam int RB = (REPS > 1) ? $clog2(REPS) : 1;
  // the manual offsets, single steps over the band: OSD 1 is 0, 2-15 are +1 .. +14, 16-23 are -8 .. -1 (as
  // NSTEP - 8 .. NSTEP - 1); 24-31 (not offered) are 0
  function automatic logic [5:0] man(input logic [4:0] o);
    if (o >= 5'd2 && o <= 5'd15) man = {1'b0, o - 5'd1};
    else if (o >= 5'd16 && o <= 5'd23) man = 6'(NSTEP) - {1'b0, 5'd24 - o};
    else man = 6'd0;
  endfunction

  // the SDRAM being calibrated: 0 A, 1 B (always 0 with NS 1)
  logic          ps;
  wire           pi = (NS == 2) ? ps : 1'b0;
  wire           qi = (NS == 2) ? !ps : 1'b0;    // NS 2: the other

  // a line's byte address on SDRAM b
  function automatic logic [AW:0] la(input logic [LB-1:0] i, input logic b);
    logic [26:0] a, ii;
    ii = 27'(i);
    a = (b ? BASE_B : BASE) ^ (ii << 5) ^ (27'(ii[2:1]) << 11) ^ (27'(ii >> 3) << 14);
    if (b ? TWO_B : TWO_CHIPS) a = a ^ (27'(ii[0]) << 26);
    la = a[AW:0];
  endfunction
  // a beat's word
  function automatic logic [31:0] patt(input logic [LB-1:0] i, input logic [2:0] k);
    logic [15:0] w;
    logic [31:0] h;
    logic [7:0]  ik;
    /* verilator lint_off UNUSEDSIGNAL */
    logic [5:0]  sh;                                 // its low four bits: the walking bit
    /* verilator lint_on UNUSEDSIGNAL */
    ik = {2'b00, 6'(i)} ^ {5'd0, k};
    sh = (6'(i) >> 2) + 6'(k);
    w = 16'd1 << sh[3:0];                            // modulo 16: never off the word
    h = {ik, 8'h5A, ~ik, 8'hC3} ^ {8'(i), 8'(i) ^ 8'h96, 5'd0, k, 8'd0};
    // one Feistel-like step (R203, for area: 16 XORs in place of three xorshifts): the high half kept, the low half
    // XORed with it rotated by 5; a bijection, so the words stay unique, and no two neighbouring columns of a line
    // are equal in either column order (tools/patt.py)
    h = {h[31:16], h[15:0] ^ {h[26:16], h[31:27]}};
    case (i[1:0])
      2'd0:    patt = {~w, w};                                  // walking one, walking zero
      // the DQs toggling each column, the low four bits of each half its column's number ({k, half})
      2'd1:    patt = (k[0] ? 32'hFFFF_0000 : 32'h0000_FFFF) ^ {12'd0, k, 1'b1, 12'd0, k, 1'b0};
      2'd2:    patt = (k[0] ? 32'h5555_AAAA : 32'hAAAA_5555) ^ {12'd0, k, 1'b1, 12'd0, k, 1'b0};
      default: patt = h;
    endcase
  endfunction

  typedef enum logic [3:0] { C_IDLE, C_WREQ, C_WBEAT, C_STEP, C_WAIT, C_SET, C_RREQ, C_RBEAT, C_NEXT, C_SCAN, C_CEN, C_PICK, C_DONE, C_END, C_QA } cs_t;
  logic          ver;             // the verify pass: one more step, at the chosen offset
  logic [2:0]    sw;              // the sweep
  logic          xw, xd;          // HOSTILE: the scratch write under way / done before this line's read
  logic [6:0]    run2, best2;     // the scan of the steps that count (inside the band, EDGE_M after the edge)
  logic [5:0]    bend2;
  logic [5:0]    cen;
  // HOSTILE's scratch lines: another bank (byte bit 11) and 4 MiB on (bit 22), so every line read follows a
  // line write of another line's pattern to another bank and row: read-after-write turnaround, both directions of
  // the bus at full rate, and (PARTNER) the other SDRAM's bus switching with this one's
  localparam logic [26:0] XOFF = 27'h040_0800;
  cs_t           cs;
  logic [LB-1:0] li;              // the line
  logic [RB-1:0] rep;             // the pass
  logic [5:0]    st;              // the step
  logic [9:0]    wt;              // the wait
  logic [1:0]    pq;              // s_wnext one and two clocks back
  logic [2:0]    sb, rb;          // write beat shown, read beat taken
  logic          bad;             // a beat wrong at this step
  logic [6:0]    run, best;       // the scan
  logic [5:0]    sp6;             // its position (0 .. NSTEP - 1), and the turn
  logic          sp_t;
  logic [5:0]    bend;            // the best window's last step
  logic          flag_none, flag_narrow, flag_all, flag_clamp, flag_out, flag_ver;
  logic [5:0]    chosen;

  wire  [2:0]    sb_n = (cs == C_WBEAT && pq[1]) ? sb + 3'd1 : sb;
  // one pattern generator for the writes (the beat shown next) and the reads (the beat taken): never both at once
  // (HOSTILE's scratch lines take another line's pattern, li ^ 2: the other half of the pattern set)
  wire  [31:0]   pw = patt(xw ? li ^ LB'(2) : li, (cs == C_RBEAT) ? rb : sb_n);
  wire  [AW:0]   lsa = la(li, pi);
  wire  [AW:0]   xsa = lsa ^ XOFF[AW:0];
  // the choice (R203), made on the pass map by the scan, not by arithmetic: a step counts if it passed, the EDGE_M
  // steps before it passed too (the early edge taken EDGE_M steps in: the power-up test sees it up to that much
  // earlier than the games do, cold and quiet) and it lies inside the band (-BAND_LO .. +BAND_HI: STA's limits,
  // the late one where the slow corner puts the capture word's transfer into clk_sys); the widest run of such steps
  // (best2, ending at bend2) is what is left, and the choice its middle (rounded up), as the old centre.
  // The scan rotates the map, so the step at sp6 is in bit 0.
  wire           inb = (sp6 <= 6'(BAND_HI)) || (sp6 >= 6'(NSTEP - BAND_LO));
  wire           ok2 = pass_map[0] && run >= 7'(EDGE_M) && inb;
  wire  [6:0]    back = (best2 - 7'd1) >> 1;
  wire  [6:0]    cen_r = {1'b0, bend2} + 7'(NSTEP) - back;
  wire  [5:0]    cen_m = (cen_r >= 7'(NSTEP)) ? 6'(cen_r - 7'(NSTEP)) : cen_r[5:0];
  // a run starts (C_IDLE, and B's end with NS 2): A when NS 1; with NS 2 once A is up and B is up or known absent
  wire           go1 = (NS == 1) ? start[0] : (start[0] && (start[NS-1] || skip_b));
  // PARTNER: the other port takes this transaction too, if its controller is up and it has nothing in hand
  wire           pt  = (NS == 2) && PARTNER && start[qi] && !s_req[qi] && s_idle[qi] && !s_ack[qi];
  // the second SDRAM's results, kept once its run is over (the first's are the engine's own)
  logic [6:0]    rb_width;
  logic [5:0]    rb_last, rb_off, rb_flags;

  assign s_be = 4'hF; assign s_wbe = 4'hF;

  always_ff @(posedge clk) begin
    if (rst || !EN) begin
      cs <= C_IDLE; done <= {NS{!EN}}; s_req <= '0; s_cmd <= MC_LINE_RD; s_sa <= '0; s_wdata <= '0; ps <= 1'b0;
      li <= '0; rep <= '0; st <= '0; wt <= '0; pq <= '0; sb <= '0; rb <= '0; bad <= 1'b0;
      run <= '0; best <= '0; sp6 <= '0; sp_t <= 1'b0; bend <= '0; pass_map <= '0;
      flag_none <= 1'b0; flag_narrow <= 1'b0; flag_all <= 1'b0; flag_clamp <= 1'b0; flag_out <= 1'b0; flag_ver <= 1'b0;
      chosen <= '0; ver <= 1'b0; sw <= '0; xw <= 1'b0; xd <= 1'b0; run2 <= '0; best2 <= '0; bend2 <= '0; cen <= '0;
      rb_width <= '0; rb_last <= '0; rb_off <= '0; rb_flags <= '0;
    end else begin
      s_req <= s_req & ~s_ack;
      s_wdata <= pw;                     // beat k in s_wdata at t + k + 2 (m3_bus's timing for LW1)
      pq <= {pq[0], s_wnext[pi]};
      sb <= sb_n;
      case (cs)
        C_IDLE: if (go1) begin cs <= C_WREQ; li <= '0; ps <= (NS == 2) && start[NS-1]; end

        // ---- the writes: each line once, a cast-out of eight beats
        C_WREQ: begin
          s_req[pi] <= 1'b1; if (pt) s_req[qi] <= 1'b1;
          s_cmd <= MC_LINE_WR; s_sa <= lsa; sb <= '0; pq <= '0;
          cs <= C_WBEAT;
        end
        C_WBEAT: if (pq[1] && sb == 3'd7) begin
          // the port took the last beat this clock
          if (xw) begin xw <= 1'b0; xd <= 1'b1; cs <= C_RREQ; end        // HOSTILE: now the line's read
          else if (li == LB'(NL - 1)) begin cs <= C_STEP; st <= '0; sw <= '0; end
          else begin li <= li + 1'd1; cs <= C_WREQ; end
        end

        // ---- a step: the phase there, then REPS passes of every line
        C_STEP: begin wt <= '0; bad <= 1'b0; cs <= C_WAIT; end
        C_WAIT: begin
          wt <= wt + 10'd1;
          if (wt >= 10'd16 && settled[pi]) begin wt <= '0; cs <= C_SET; end
        end
        C_SET: begin
          wt <= wt + 10'd1;
          if (wt == 10'(SETTLE)) begin li <= '0; rep <= '0; cs <= C_RREQ; end
        end
        C_RREQ: begin
          s_req[pi] <= 1'b1; if (pt) s_req[qi] <= 1'b1;
          if (HOSTILE && !xd) begin
            // the scratch line first: another line's pattern, another bank and row
            s_cmd <= MC_LINE_WR; s_sa <= xsa; sb <= '0; pq <= '0; xw <= 1'b1; cs <= C_WBEAT;
          end else begin
            s_cmd <= MC_LINE_RD; s_sa <= lsa; rb <= '0; cs <= C_RBEAT;
          end
        end
        C_RBEAT: if (s_rvalid[pi]) begin
          if (s_rdata[pi] != pw) bad <= 1'b1;
          rb <= rb + 3'd1;
          if (rb == 3'd7) begin
            xd <= 1'b0;
            if (li != LB'(NL - 1)) begin li <= li + 1'd1; cs <= C_RREQ; end
            else if (rep != RB'(REPS - 1)) begin li <= '0; rep <= rep + 1'd1; cs <= C_RREQ; end
            else cs <= C_NEXT;
          end
        end
        C_NEXT: begin
          if (ver) begin
            // the verify pass: wrong reads at the chosen offset, after the stepper took the clock there
            if (bad) begin flag_ver <= 1'b1; chosen <= 6'd0; end
            cs <= C_DONE;
          end else begin
            // NSW sweeps: a step passes only if it passed in every one; its verdict shifted in at the top, so after
            // the step NSTEP - 1 the map is in place again (step j in bit j) and bit 0 is the next step's old one
            pass_map <= {!bad && (sw == 3'd0 || pass_map[0]), pass_map[NSTEP-1:1]};
            if (st != 6'(NSTEP - 1)) begin st <= st + 6'd1; cs <= C_STEP; end
            else if (sw != 3'(NSW - 1)) begin st <= '0; sw <= sw + 3'd1; cs <= C_STEP; end
            else begin cs <= C_SCAN; sp6 <= '0; sp_t <= 1'b0; run <= '0; best <= '0; run2 <= '0; best2 <= '0; end
          end
        end

        // ---- the widest passing run over two turns (wrap-around), width at most NSTEP: the map rotated a step a
        // clock, so the step at sp6 is in bit 0, and after the two turns the map is in place again
        C_SCAN: begin
          pass_map <= {pass_map[0], pass_map[NSTEP-1:1]};
          if (pass_map[0]) begin
            if (run < 7'(NSTEP)) run <= run + 7'd1;
            if (run + 7'd1 > best && run < 7'(NSTEP)) begin best <= run + 7'd1; bend <= sp6; end
          end else run <= '0;
          if (ok2) begin
            run2 <= run2 + 7'd1;
            if (run2 + 7'd1 > best2) begin best2 <= run2 + 7'd1; bend2 <= sp6; end
          end else run2 <= '0;
          if (sp_t && sp6 == 6'(NSTEP - 1)) cs <= C_CEN;
          sp6 <= (sp6 == 6'(NSTEP - 1)) ? 6'd0 : sp6 + 6'd1;
          if (sp6 == 6'(NSTEP - 1)) sp_t <= 1'b1;
        end
        C_CEN: begin cen <= cen_m; cs <= C_PICK; end       // from the scan's final best2 / bend2
        C_PICK: begin
          cs <= C_DONE;
          if (best == 7'd0) begin flag_none <= 1'b1; chosen <= 6'd0; end
          else if (best >= 7'(NSTEP)) begin flag_all <= 1'b1; chosen <= 6'd0; end
          else begin
            flag_narrow <= best < 7'(WMIN);
            // clamped: the window went on past the band's top (the late side held there)
            flag_clamp <= best2 != 7'd0 && bend2 == 6'(BAND_HI) && pass_map[BAND_HI + 1];
            flag_out <= best2 == 7'd0;                            // nothing left after the margin: offset 0
            chosen <= (best2 == 7'd0) ? 6'd0 : cen;
            // then the verify pass at the chosen offset (C_STEP .. C_NEXT once more)
            ver <= 1'b1; st <= (best2 == 7'd0) ? 6'd0 : cen; cs <= C_STEP;
          end
        end
        C_DONE: begin
          done[pi] <= 1'b1;
          cs <= C_END;
          if (NS == 2 && pi) begin
            // B is over: its results kept, then A's run from the start
            rb_width <= best; rb_last <= bend; rb_off <= chosen;
            rb_flags <= {flag_ver, flag_out, flag_clamp, flag_all, flag_narrow, flag_none};
            ps <= 1'b0; cs <= C_QA; li <= '0; ver <= 1'b0; chosen <= '0;
            flag_none <= 1'b0; flag_narrow <= 1'b0; flag_all <= 1'b0; flag_clamp <= 1'b0; flag_out <= 1'b0; flag_ver <= 1'b0;
            // and the scan's results as the reset leaves them (a run with no window sets neither end, and the readout
            // shows A's width and last from here on)
            run <= '0; best <= '0; bend <= '0; run2 <= '0; best2 <= '0; bend2 <= '0;
          end
        end
        // NS 2: A's run waits for A's port to finish whatever partner transaction it still has (a write's beats would
        // otherwise be counted as A's first line's)
        C_QA: if (!s_req[0] && s_idle[0]) cs <= C_WREQ;
        C_END: ;
        default: cs <= C_IDLE;
      endcase
    end
  end

  // each capture clock's offset: the step under test while its run sweeps, Auto's choice or the OSD's after
  always_ff @(posedge clk) begin
    for (int k = 0; k < NS; k++) begin
      if (!EN) off[k] <= man(osd[k] == 5'd0 ? 5'd1 : osd[k]);
      else if (rst) off[k] <= 6'd0;
      else if (cs == C_STEP && pi == 1'(k)) off[k] <= st;
      else if (done[k] || (cs == C_DONE && pi == 1'(k)))
        off[k] <= (osd[k] != 5'd0) ? man(osd[k]) : (k == 1 && done[k]) ? rb_off : chosen;
    end
  end

  assign busy = EN && !(done[0] && (NS == 1 || done[NS-1] || skip_b));
  assign s_pb = (NS == 2) && ps;
  assign c_centre = cen;
  always_comb begin
    c_width[0] = best; c_last[0] = bend; c_off[0] = chosen;
    c_flags[0] = {flag_ver, flag_out, flag_clamp, flag_all, flag_narrow, flag_none};
    if (NS == 2) begin
      c_width[NS-1] = rb_width; c_last[NS-1] = rb_last; c_off[NS-1] = rb_off; c_flags[NS-1] = rb_flags;
    end
  end
endmodule
