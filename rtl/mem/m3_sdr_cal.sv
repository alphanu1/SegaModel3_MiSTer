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

module m3_sdr_cal
  import ppc_cache_pkg::*;
#(
  parameter int          AW        = 26,
  parameter bit          EN        = 1'b1,
  parameter int          NL        = 64,            // lines (32 bytes each), a power of two up to 64
  parameter int          REPS      = 16,            // read passes at each step (up to 64)
  parameter int          NSW       = 4,             // sweeps, their pass maps intersected (up to 8)
  parameter bit          HOSTILE   = 1'b1,          // every line read straight after a line write elsewhere
  parameter int          EDGE_M    = 12,            // steps of margin taken off the window's early edge
  parameter int          NSTEP     = 48,            // PLL steps in a period
  parameter int          BAND_LO   = 8,             // the analysed band: BAND_LO steps below the generated phase
  parameter int          BAND_HI   = 14,            //   to BAND_HI above (BAND_LO + BAND_HI < NSTEP)
  parameter int          WMIN      = 6,             // a window narrower than this is flagged
  parameter logic [26:0] BASE      = 27'h000_0000,
  parameter bit          TWO_CHIPS = 1'b1,
  parameter int          SETTLE    = 512            // clocks after the phase is there, before the reads
) (
  input  logic          clk,
  input  logic          rst,           // the memory's reset (PLL lock)
  input  logic          start,         // the controller is up (m2_sdram ready / m3_sdr2's probe passed)
  output logic          done,

  // m3_sdr_port's s_* (the bus's side, which m3_system / m3_sdr2 switch to this while !done)
  output logic          s_req,
  output logic [1:0]    s_cmd,
  output logic [AW:0]   s_sa,
  output logic [3:0]    s_be,
  output logic [31:0]   s_wdata,
  output logic [3:0]    s_wbe,
  input  logic          s_ack,
  input  logic          s_wnext,
  input  logic          s_rvalid,
  input  logic [31:0]   s_rdata,

  // the capture clock's offset (to the stepper) and its state
  input  logic [4:0]    osd,           // 0 Auto, 1-23 MAN[osd]
  output logic [5:0]    off,
  input  logic          settled,

  output logic [6:0]    c_width,
  output logic [5:0]    c_centre,
  output logic [5:0]    c_last,        // the window's last step (0 .. NSTEP - 1)
  output logic [5:0]    c_off,
  output logic [5:0]    c_flags,       // {verify, out, clamped, all, narrow, none}
  output logic [NSTEP-1:0] pass_map
);
  localparam int LB = $clog2(NL);
  // the manual offsets, single steps over the band: OSD 1 is 0, 2-15 are +1 .. +14, 16-23 are -8 .. -1 (as
  // NSTEP - 8 .. NSTEP - 1); 24-31 (not offered) are 0
  function automatic logic [5:0] man(input logic [4:0] o);
    if (o >= 5'd2 && o <= 5'd15) man = {1'b0, o - 5'd1};
    else if (o >= 5'd16 && o <= 5'd23) man = 6'(NSTEP) - {1'b0, 5'd24 - o};
    else man = 6'd0;
  endfunction

  // a line's byte address
  function automatic logic [AW:0] la(input logic [LB-1:0] i);
    logic [26:0] a, ii;
    ii = 27'(i);
    a = BASE ^ (ii << 5) ^ (27'(ii[2:1]) << 11) ^ (27'(ii >> 3) << 14);
    if (TWO_CHIPS) a = a ^ (27'(ii[0]) << 26);
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

  typedef enum logic [3:0] { C_IDLE, C_WREQ, C_WBEAT, C_STEP, C_WAIT, C_SET, C_RREQ, C_RBEAT, C_NEXT, C_SCAN, C_CEN, C_PICK, C_DONE } cs_t;
  logic          ver;             // the verify pass: one more step, at the chosen offset
  logic [2:0]    sw;              // the sweep
  logic          xw, xd;          // HOSTILE: the scratch write under way / done before this line's read
  logic [6:0]    run2, best2;     // the scan of the steps that count (inside the band, EDGE_M after the edge)
  logic [5:0]    bend2;
  logic [5:0]    cen;
  // HOSTILE's scratch lines: another bank (byte bit 11) and 4 MiB on (bit 22), so every line read follows a
  // line write of another line's pattern to another bank and row: read-after-write turnaround, both directions of
  // the bus at full rate, and (the two SDRAMs' calibrations running together) the other SDRAM's writes
  localparam logic [26:0] XOFF = 27'h040_0800;
  cs_t           cs;
  logic [LB-1:0] li;              // the line
  logic [5:0]    rep;             // the pass
  logic [5:0]    st;              // the step
  logic [9:0]    wt;              // the wait
  logic [1:0]    pq;              // s_wnext one and two clocks back
  logic [2:0]    sb, rb;          // write beat shown, read beat taken
  logic          bad;             // a beat wrong at this step
  logic [6:0]    run, best;       // the scan
  logic [6:0]    sp;              // its position (0 .. 2 NSTEP - 1)
  logic [5:0]    bend;            // the best window's last step
  logic          flag_none, flag_narrow, flag_all, flag_clamp, flag_out, flag_ver;
  logic [5:0]    chosen;

  wire  [2:0]    sb_n = (cs == C_WBEAT && pq[1]) ? sb + 3'd1 : sb;
  // one pattern generator for the writes (the beat shown next) and the reads (the beat taken): never both at once
  // (HOSTILE's scratch lines take another line's pattern, li ^ 2: the other half of the pattern set)
  wire  [31:0]   pw = patt(xw ? li ^ LB'(2) : li, (cs == C_RBEAT) ? rb : sb_n);
  wire  [AW:0]   xsa = la(li) ^ XOFF[AW:0];
  wire  [5:0]    sp6 = (sp >= 7'(NSTEP)) ? 6'(sp - 7'(NSTEP)) : sp[5:0];
  // the choice (R203), made on the pass map by the scan, not by arithmetic: a step counts if it passed, the EDGE_M
  // steps before it passed too (the early edge taken EDGE_M steps in: the power-up test sees it up to that much
  // earlier than the games do, cold and quiet) and it lies inside the band (-BAND_LO .. +BAND_HI: STA's limits,
  // the late one where the slow corner puts the capture word's transfer into clk_sys); the widest run of such steps
  // (best2, ending at bend2) is what is left, and the choice its middle (rounded up), as the old centre.
  wire           inb = (sp6 <= 6'(BAND_HI)) || (sp6 >= 6'(NSTEP - BAND_LO));
  wire           ok2 = pass_map[sp6] && run >= 7'(EDGE_M) && inb;
  wire  [6:0]    back = (best2 - 7'd1) >> 1;
  wire  [6:0]    cen_r = {1'b0, bend2} + 7'(NSTEP) - back;
  wire  [5:0]    cen_m = (cen_r >= 7'(NSTEP)) ? 6'(cen_r - 7'(NSTEP)) : cen_r[5:0];

  assign s_be = 4'hF; assign s_wbe = 4'hF;

  always_ff @(posedge clk) begin
    if (rst || !EN) begin
      cs <= C_IDLE; done <= !EN; s_req <= 1'b0; s_cmd <= MC_LINE_RD; s_sa <= '0; s_wdata <= '0;
      li <= '0; rep <= '0; st <= '0; wt <= '0; pq <= '0; sb <= '0; rb <= '0; bad <= 1'b0;
      run <= '0; best <= '0; sp <= '0; bend <= '0; pass_map <= '0;
      flag_none <= 1'b0; flag_narrow <= 1'b0; flag_all <= 1'b0; flag_clamp <= 1'b0; flag_out <= 1'b0; flag_ver <= 1'b0;
      chosen <= '0; ver <= 1'b0; sw <= '0; xw <= 1'b0; xd <= 1'b0; run2 <= '0; best2 <= '0; bend2 <= '0; cen <= '0;
      off <= 6'd0;
    end else begin
      s_req <= s_req && !s_ack;
      s_wdata <= pw;                     // beat k in s_wdata at t + k + 2 (m3_bus's timing for LW1)
      pq <= {pq[0], s_wnext};
      sb <= sb_n;
      case (cs)
        C_IDLE: if (start) begin cs <= C_WREQ; li <= '0; end

        // ---- the writes: each line once, a cast-out of eight beats
        C_WREQ: begin
          s_req <= 1'b1; s_cmd <= MC_LINE_WR; s_sa <= la(li); sb <= '0; pq <= '0;
          cs <= C_WBEAT;
        end
        C_WBEAT: if (pq[1] && sb == 3'd7) begin
          // the port took the last beat this clock
          if (xw) begin xw <= 1'b0; xd <= 1'b1; cs <= C_RREQ; end        // HOSTILE: now the line's read
          else if (li == LB'(NL - 1)) begin cs <= C_STEP; st <= '0; sw <= '0; end
          else begin li <= li + 1'd1; cs <= C_WREQ; end
        end

        // ---- a step: the phase there, then REPS passes of every line
        C_STEP: begin off <= st; wt <= '0; bad <= 1'b0; cs <= C_WAIT; end
        C_WAIT: begin
          wt <= wt + 10'd1;
          if (wt >= 10'd16 && settled) begin wt <= '0; cs <= C_SET; end
        end
        C_SET: begin
          wt <= wt + 10'd1;
          if (wt == 10'(SETTLE)) begin li <= '0; rep <= '0; cs <= C_RREQ; end
        end
        C_RREQ: if (HOSTILE && !xd) begin
            // the scratch line first: the inverted pattern, another bank and row
            s_req <= 1'b1; s_cmd <= MC_LINE_WR; s_sa <= xsa; sb <= '0; pq <= '0; xw <= 1'b1; cs <= C_WBEAT;
          end else begin
            s_req <= 1'b1; s_cmd <= MC_LINE_RD; s_sa <= la(li); rb <= '0; cs <= C_RBEAT;
          end
        C_RBEAT: if (s_rvalid) begin
          if (s_rdata != pw) bad <= 1'b1;
          rb <= rb + 3'd1;
          if (rb == 3'd7) begin
            xd <= 1'b0;
            if (li != LB'(NL - 1)) begin li <= li + 1'd1; cs <= C_RREQ; end
            else if (rep != 6'(REPS - 1)) begin li <= '0; rep <= rep + 6'd1; cs <= C_RREQ; end
            else cs <= C_NEXT;
          end
        end
        C_NEXT: begin
          if (ver) begin
            // the verify pass: wrong reads at the chosen offset, after the stepper took the clock there
            if (bad) begin flag_ver <= 1'b1; chosen <= 6'd0; end
            cs <= C_DONE;
          end else begin
            // NSW sweeps: a step passes only if it passed in every one
            if (sw == 3'd0 || bad) pass_map[st] <= !bad;           // (one shared enable term: cheap)
            if (st != 6'(NSTEP - 1)) begin st <= st + 6'd1; cs <= C_STEP; end
            else if (sw != 3'(NSW - 1)) begin st <= '0; sw <= sw + 3'd1; cs <= C_STEP; end
            else begin cs <= C_SCAN; sp <= '0; run <= '0; best <= '0; run2 <= '0; best2 <= '0; end
          end
        end

        // ---- the widest passing run over two turns (wrap-around), width at most NSTEP
        C_SCAN: begin
          if (pass_map[sp6]) begin
            if (run < 7'(NSTEP)) run <= run + 7'd1;
            if (run + 7'd1 > best && run < 7'(NSTEP)) begin best <= run + 7'd1; bend <= sp6; end
          end else run <= '0;
          if (ok2) begin
            run2 <= run2 + 7'd1;
            if (run2 + 7'd1 > best2) begin best2 <= run2 + 7'd1; bend2 <= sp6; end
          end else run2 <= '0;
          if (sp == 7'(2 * NSTEP - 1)) cs <= C_CEN;
          sp <= sp + 7'd1;
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
          done <= 1'b1;
          off <= (osd == 5'd0) ? chosen : man(osd);
        end
        default: cs <= C_IDLE;
      endcase
    end
    if (!EN) off <= man(osd == 5'd0 ? 5'd1 : osd);
  end

  assign c_width  = best;
  assign c_centre = cen;
  assign c_last   = bend;
  assign c_off    = chosen;
  assign c_flags  = {flag_ver, flag_out, flag_clamp, flag_all, flag_narrow, flag_none};
endmodule
