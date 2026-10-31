// SPDX-License-Identifier: GPL-3.0-or-later
//
// Sega Model 3 core for MiSTer FPGA
// Copyright (C) 2026 alphanu1
//
// MODEL 3's TWO SCSPs ON ONE ENGINE. Derived from m2_scsp.sv (Model 2 at
// 47edd9dc; THIRD_PARTY.md), which follows MAME 0.289's scsp.cpp /
// scspdsp.cpp (BSD-3-Clause, ElSemi, R. Belmont). The slot program, the
// arithmetic and every table are m2_scsp's; what is shared and what is not:
//
//   SHARED   the sequencer and its microcode ROMs, the ALU, the 27 x 18
//            multiplier, the envelope's second clock, the LFO and EG tables,
//            the register file (its entries are {chip, register}), the level
//            evaluator (m2_scsp_level, SW 6), the DSP (scsp_dual_dsp), the
//            68000's queue, one slot cache and one DSP cache (512 lines each,
//            the chip an index bit), and one memory port of each kind.
//   PER CHIP in RAM, the chip the top address bit: the slot registers
//            (1,024 x 16), the slot state (512 x 32), the level words, the FM
//            ring, and the DSP's MPRO, TEMP, MEMS, COEF and MADRS.
//   PER CHIP in logic: the common registers, timers, interrupts and MIDI
//            (m2_scsp_regs, one each; chip 1 without MIDI out), the active
//            bits, BUFPTR's parity, the 0x408 monitor, the outputs, the DSP's
//            MIXS and EFREG (an array each: the slots of one chip write MIXS
//            while the DSP runs the other's) and its DEC / LastStep / stop.
//
// THE SCHEDULE. One chip's sample at a time, never a slot of each: the engine
// is a microcoded sequence over a whole sample (slots, DSP, effect returns,
// output) and its per-sample sums live in the register file, so a turn is a
// sample. A turn (scsp_uasm.py dual) makes the chip's LAST sample's effect
// returns and output -- its DSP ran during the other chip's turn -- then this
// sample's slots, starts its DSP and hands over:
//
//     sequencer  | E0 O0 S0 | E1 O1 S1 | E0 O0 S0 | ...
//     DSP        |      D1  |      D0  |      D1  | ...
//
// so the DSP's run is hidden behind the other chip's slots, and a pair of
// samples costs about 2 x (slots + 100) clocks, not 2 x (slots + DSP + 100).
//
// WHERE IT DIFFERS FROM TWO m2_scsp (each also in the bench's notes):
//   1. a chip's output comes one turn later, at the start of its next turn,
//      not at the end of its own (the two outputs are not simultaneous);
//   2. a 68000 access to chip c is applied in an operation window of either
//      chip's turn. Between c's slots of sample n and its effect returns /
//      output -- the window in which its DSP runs -- a write to the level
//      words (EFSDL / EFPAN), MVOL or DAC18 reaches sample n's output, as a
//      write that lands in m2_scsp's DSP wait does. Writes applied at c's own
//      sample boundary (step_mode: where the engine waits for chip c) give
//      the two m2_scsp's output exactly (tb_scsp_dual);
//   3. an access to DSP registers (0x700-0xEFF) waits while the DSP runs
//      EITHER chip (one DSP), not only its own; queue order is kept;
//   4. the DSP's next run starts only when the last run's memory accesses are
//      all done (a late read could otherwise land in the other chip's run);
//   5. slot fetches and DSP accesses of the two chips share one port each,
//      with the chip beside the address (sm_ch, dm_ch);
//   6. KEYONEX's sweep, a word-5 write and the monitor's latch run as calls
//      from an operation window as before; they stop the sequencer for both
//      chips, not one.
// Model 3's board (MAME model3.cpp): each SCSP has its own 512 KB of sound
// RAM; only chip 0 drives the 68000's interrupt and the MIDI link (scsp2 has
// no irq or MIDI callback), so ipl1 is left for the board to ignore.

`timescale 1ns/1ps

module scsp_dual #(
  parameter bit FM        = 1'b1,
  parameter bit MIDI_OUT  = 1'b1,         // chip 0's MOBUF (chip 1 has none on Model 3)
  parameter bit LFO_NOISE = 1'b1,
  parameter bit MADRS_MIRROR = 1'b0,
  parameter int TIMER     = 0,
  // the caches: 9 = 512 lines, 256 a chip (as two m2_scsp); 8 = 256 lines
  // shared, the chip in the tag (half the M10K, more fills)
  parameter int unsigned SLOT_IDX = 9,
  parameter int unsigned DSP_IDX  = 9,
  // chip 1's common registers: 1 = m2_scsp_regs (timers, interrupts); 0 =
  // only what the engine reads (MVOL, DAC18, RBL / RBP, MSLC) and the words
  // as written. VF3's driver writes nothing else on chip 1 (0x400, 0x402,
  // 0x408 only, 180 s capture) and chip 1's interrupt is not wired on Model 3
  parameter bit CH1_TIMERS = 1'b1
) (
  input  logic        clk,
  input  logic        rst_n,

  // ---- the 68000 (both chips' 0x000-0xfff, cpu_ch the chip), one access per req
  input  logic        cpu_req,
  input  logic        cpu_we,
  input  logic        cpu_ch,
  input  logic [11:1] cpu_addr,
  input  logic [15:0] cpu_din,
  input  logic  [1:0] cpu_be,
  output logic        cpu_ack,
  output logic [15:0] cpu_dout,
  output logic  [2:0] ipl0,             // chip 0's interrupt level
  output logic  [2:0] ipl1,             // chip 1's (not wired on Model 3)

  // ---- MIDI (chip 0)
  input  logic  [7:0] midi_data,
  input  logic        midi_valid,
  output logic        midi_ack,
  output logic  [7:0] mout_data,
  output logic        mout_valid,
  input  logic        mout_ack,

  // ---- sound RAM, the chip beside each address
  output logic        sm_req,
  output logic        sm_ch,
  output logic [15:0] sm_line,
  input  logic        sm_ack,
  input  logic [63:0] sm_data,
  output logic        dm_req,
  output logic        dm_ch,
  output logic        dm_we,
  output logic [18:1] dm_addr,
  output logic [15:0] dm_wdata,
  input  logic        dm_ack,
  input  logic [63:0] dm_data,
  input  logic        snp,
  input  logic        snp_ch,
  input  logic [18:1] snp_addr,

  // ---- output, a chip each ([1] chip 1)
  input  logic        out_ready,
  output logic  [1:0] out_valid,
  output logic [1:0][31:0] out_raw_l, out_raw_r,
  output logic [1:0][15:0] out_l, out_r,
  input  logic        tick44,
  input  logic  [5:0] tph,
  input  logic        cpu_tick,
  input  logic        tarc,
  input  logic  [1:0] boost,

  // ---- benches
  input  logic        step_mode,        // the engine waits at each chip's sample boundary for step_go
  input  logic        step_go,
  output logic        at_go,            // waiting there (step_mode), for chip at_ch
  output logic        at_ch,
  output logic        quiet,
  output logic        dbg_slot_valid,
  output logic  [5:0] dbg_slot,         // {chip, slot}
  output logic signed [31:0] dbg_slot_smp,
  output logic [31:0] dbg_samples,
  output logic [31:0] dbg_miss
);

`include "scsp_tables.svh"
  /* verilator lint_off UNUSEDPARAM */
`include "scsp_d_uc.svh"
  /* verilator lint_on UNUSEDPARAM */

  localparam logic [1:0] ATT = 2'd0, DC1 = 2'd1, DC2 = 2'd2, REL = 2'd3;

  // the cache line with the chip as an index bit: {line[15:8], chip, line[7:0]}
  function automatic logic [16:0] pl(input logic c, input logic [15:0] l);
    pl = {l[15:8], c, l[7:0]};
  endfunction

  // =====================================================================
  // declarations
  // =====================================================================
  logic  [7:0] upc, upc_n, ret_addr;
  logic [UF_W-1:0] uf;
  logic [UE_W-1:0] ue;
  logic        hold;
  logic  [5:0] slot;
  logic  [4:0] sw, slot_e, sw_e;
  logic        ch, ch_e;                  // the turn's chip at F; the chip of the microinstruction in E
  logic        sw_ch;                     // a call's chip (the sweep, word 5, the monitor)
  logic  [1:0] par;                       // BUFPTR's top bit, a chip each
  logic [31:0] active [2];
  logic  [1:0] pend;                      // a chip's sample is waiting for its effect returns

  (* ramstyle = "logic" *) logic [29:0] wq [4];  // {chip, be 2, addr 11, data 16}
  logic  [1:0] wq_rd, wq_wr;
  logic  [2:0] wq_n;
  logic        rd_pend, rd_ch;
  logic [11:1] rd_a;
  logic        wq_pop, rd_done;
  logic        mon_fast, mon_go;
  logic  [1:0][4:0] mslc_c, mon_slot;
  logic  [1:0] mon_req, mon_pend, mon_rd;
  logic        op_busy, op_rd, o_ch;
  logic [11:1] o_addr;
  logic [15:0] o_data;
  logic  [1:0] o_be;
  logic  [2:0] oc;
  logic  [8:0] w6l;
  logic        call_go, call_ch;
  logic  [7:0] call_to;
  logic  [4:0] call_sw;

  logic [31:0] q, a_v, b_v, alu_r, sum;
  logic        borrow, scmp;
  logic [31:0] ra_q, rb_q, st_q;
  logic [15:0] sqa, sqb;
  logic [14:0] lv_q;
  logic signed [26:0] mul_a;
  logic signed [17:0] mul_b;
  logic signed [44:0] prod;
  logic [31:0] psl_v;
  logic        f1, f2, f3, f4, stop_rel, fk, fr;
  logic [15:0] w0l;
  logic [14:0] w9l;
  logic  [3:0] mdl;
  logic        fm_mod, lpslnk, d2r_zero, stwinh;
  logic  [3:0] isel;
  logic  [1:0] egst_l;
  logic        hold_r;
  logic        eg2, eg2_we, eg2_tadre, eg2_d2r, eg2_lpslnk, eg2_hold;
  logic [31:0] eg2_sum;
  logic  [1:0] eg2_pre;
  logic  [4:0] eg2_dl;
  logic [25:0] eg2_stq;
  logic  [8:0] eg2_wa;
  logic [31:0] eg2_r;
  logic        eg_lin, eg_hold;
  logic  [9:0] eg_lv;
  logic [11:0] frac;
  logic [13:0] tadr;
  logic  [8:0] tbl_q;
  logic  [9:0] egt_a;
  logic [12:0] egt_q;
  logic [26:0] rate_q;
  logic [19:0] fa1, fa2;
  logic        fch;                       // the chip of the fetch in flight
  logic        f2_known, p1_ok, p2_ok;
  logic signed [16:0] p1v, p2v;
  logic [15:0] ring_q;
  logic  [7:0] ring_ra;
  logic        dsp_go, dsp_done, dsp_busy;
  logic  [1:0] dsp_act;                   // the DSP is clearing or running chip c's MIXS / EFREG
  logic        dsp_pend, dsp_pch;          // a run asked for and not done; its chip
  logic        mix_we;
  logic [19:0] mix_val;
  logic signed [15:0] ef_q;
  logic        dw_en;
  logic        dsp_wr_done, dsp_wr_ch;
  logic [18:1] dsp_wr_addr;
  logic [15:0] dsp_rq;
  logic        lv_req, lv_busy;
  logic  [8:0] lv_raddr;
  logic  [3:0] mon_ca;
  logic [1:0][15:0] mon;
  logic  [1:0][3:0] mvol_c;
  logic  [1:0] dac18_c;
  logic  [1:0][1:0] rbl_c;
  logic  [1:0][5:0] rbp_c;
  logic  [1:0][4:0] mslc_r;
  logic  [1:0][15:0] regs_q;
  logic  [1:0] cw_en_c;
  logic        cw_en, mi_pop;

  wire  [3:0] f_ra    = uf[`UF_RA];
  wire  [3:0] f_rb    = uf[`UF_RB];
  wire        f_st    = uf[`UF_ST];
  wire  [2:0] f_strow = uf[`UF_STROW];
  wire        f_streg = uf[`UF_STREG];
  wire  [1:0] f_idx   = uf[`UF_IDX];
  wire  [3:0] f_swa   = uf[`UF_SWA];
  wire  [3:0] f_swb   = uf[`UF_SWB];
  wire  [2:0] f_lw    = uf[`UF_LW];
  wire  [2:0] f_flow  = uf[`UF_FLOW];
  wire  [4:0] f_bc    = uf[`UF_BC];
  wire  [7:0] f_tgt   = uf[`UF_TGT];
  wire  [3:0] e_op    = ue[`UE_OP];
  wire  [2:0] e_a     = ue[`UE_A];
  wire  [2:0] e_b     = ue[`UE_B];
  wire  [1:0] e_psl   = ue[`UE_PSL];
  wire [15:0] e_imm   = ue[`UE_IMM];
  wire        e_immhi = ue[`UE_IMMHI];
  wire        e_rdwe  = ue[`UE_RDWE];
  wire  [3:0] e_rd    = ue[`UE_RD];
  wire  [3:0] e_pred  = ue[`UE_PRED];
  wire        e_stwe  = ue[`UE_STWE];
  wire  [2:0] e_stwrow = ue[`UE_STWROW];
  wire  [1:0] e_widx  = ue[`UE_WIDX];
  wire        e_mul   = ue[`UE_MUL];
  wire  [1:0] e_ma    = ue[`UE_MA];
  wire  [2:0] e_mb    = ue[`UE_MB];
  wire  [2:0] e_fset  = ue[`UE_FSET];
  wire  [2:0] e_lata  = ue[`UE_LATA];
  wire  [1:0] e_latb  = ue[`UE_LATB];
  wire  [3:0] e_s1    = ue[`UE_S1];
  wire  [3:0] e_s2    = ue[`UE_S2];

  // the chip E's common registers belong to (MVOL, DAC18 in the output)
  wire  [3:0] mvol  = mvol_c[ch_e];
  wire        dac18 = dac18_c[ch_e];

  wire        pcm8    = w0l[4];
  wire        loop_on = w0l[6:5] != 2'd0;
  wire  [1:0] ssctl   = w0l[8:7];
  wire  [1:0] sbctl   = w0l[10:9];
  wire  [4:0] lfof    = w9l[14:10];
  wire  [1:0] plfows  = w9l[9:8];
  wire  [2:0] plfos   = w9l[7:5];
  wire  [1:0] alfows  = w9l[4:3];
  wire  [2:0] alfos   = w9l[2:0];
  wire  [1:0] eg_pre  = (f3 && lpslnk && egst_l == ATT) ? DC1 : egst_l;

  // =====================================================================
  // the 68000's queue (one for both chips: their accesses stay in order)
  // =====================================================================
  wire  [29:0] wq_h   = wq[wq_rd];
  wire  [11:1] h_addr = wq_h[26:16];
  wire   [4:0] o_slot = o_addr[9:5];
  wire   [3:0] o_word = o_addr[4:1];
  wire         op_ready = ((wq_n != 3'd0) || rd_pend) && !wq_pop && !rd_done;
  wire  [11:1] nx_addr  = (wq_n != 3'd0) ? h_addr : rd_a;
  wire         nx_ch    = (wq_n != 3'd0) ? wq_h[29] : rd_ch;
  wire         head_dsp = nx_addr >= 11'h380 && nx_addr < 11'h600;
  wire         op_ok    = op_ready && !(head_dsp && dsp_busy);
  wire         o_ring   = FM && o_addr >= 11'h300 && o_addr < 11'h380;
  wire         o_dsp    = o_addr >= 11'h380 && o_addr < 11'h600;

  logic [29:0] wh;
  logic        wh_v;
  wire         wq_room = wq_n != 3'd4;
  wire         wq_push = wq_room && ((cpu_req && cpu_we) || wh_v);
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      wq_rd <= '0; wq_wr <= '0; wq_n <= '0; rd_pend <= 1'b0; cpu_ack <= 1'b0; rd_a <= '0; rd_ch <= 1'b0;
      wh <= '0; wh_v <= 1'b0;
    end else begin
      cpu_ack <= 1'b0;
      if (cpu_req && cpu_we && !wq_room) begin wh <= {cpu_ch, cpu_be, cpu_addr, cpu_din}; wh_v <= 1'b1; end
      if (wq_push) begin
        wq[wq_wr] <= wh_v ? wh : {cpu_ch, cpu_be, cpu_addr, cpu_din};
        wq_wr <= wq_wr + 2'd1;
        cpu_ack <= 1'b1;
        wh_v <= 1'b0;
      end
      if (cpu_req && !cpu_we && !mon_fast) begin rd_pend <= 1'b1; rd_a <= cpu_addr; rd_ch <= cpu_ch; end
      if (rd_done || mon_go) begin rd_pend <= 1'b0; cpu_ack <= 1'b1; end
      if (wq_pop) wq_rd <= wq_rd + 2'd1;
      wq_n <= wq_n + {2'd0, wq_push} - {2'd0, wq_pop};
    end
  end

  // =====================================================================
  // the microcode ROMs and the sequencer
  // =====================================================================
  (* ramstyle = "M10K" *) logic [UF_W-1:0] ucf [256];
  (* ramstyle = "M10K" *) logic [UE_W-1:0] uce [256];
  initial begin
    $readmemh("scsp_d_ucf.hex", ucf);
    $readmemh("scsp_d_uce.hex", uce);
  end
  logic [UE_W-1:0] ue_f;
  always_ff @(posedge clk) begin
    uf   <= ucf[upc_n];
    ue_f <= uce[upc_n];
  end
  logic e_bub;
  always_ff @(posedge clk) ue <= e_bub ? UE_NOPW : ue_f;

  function automatic logic cond(input logic [4:0] c);
    case (c)
      5'(C_ALWAYS):  cond = 1'b1;
      5'(C_PLFO):    cond = plfos != 3'd0;
      5'(C_ALFO):    cond = alfos != 3'd0;
      5'(C_LOOP1):   cond = loop_on && f1;
      5'(C_LOOP2):   cond = loop_on && f2;
      5'(C_FMMOD):   cond = FM && fm_mod;
      5'(C_MVOL15):  cond = mvol == 4'hf;
      5'(C_NMVOL15): cond = mvol != 4'hf;
      5'(C_START):   cond = fk && fr;
      5'(C_NKB):     cond = !fk;
      5'(C_MORESW):  cond = sw != 5'd31;
      5'(C_MOREI):   cond = slot != 6'd16;
      5'(C_SSCTL3):  cond = ssctl == 2'd3;
      5'(C_NPEND):   cond = !pend[ch];
      default:       cond = 1'b0;
    endcase
  endfunction

  // GO: the FIFO has room (or the bench's step), no access waits, and the DSP
  // is not clearing this chip's MIXS (after reset). The DSP may be running the
  // other chip: that is the overlap.
  wire go_ok    = step_mode ? (step_go && !lv_busy && !lv_req) : out_ready;
  wire dsp_idle = !dsp_busy && !dsp_pend;
  wire dsp_done_ch = !(dsp_pend && dsp_pch == ch) && !dsp_act[ch];
  wire op_win   = (f_flow == 3'(FLOW_DISPATCH)) ||
                  (f_flow == 3'(FLOW_WAIT) && (f_bc == 5'(C_GO) || f_bc == 5'(C_DSPIDLE) || f_bc == 5'(C_DSPDONE)));
  wire op_start = op_win && !op_busy && !call_go && op_ok && (mon_req == 2'b00);
  wire mon_start = op_win && !op_busy && !call_go && (mon_req != 2'b00);
  wire stop_now = (!loop_on && ((f3 && f1) || (f4 && f2))) || stop_rel;
  // the slot an access names, and its chip: the sweep's / call's, MSLC's, or the turn's
  wire [4:0] idx_f  = (f_idx == 2'(IDX_SW)) ? sw : (f_idx == 2'(IDX_MSLC)) ? mslc_r[ch] : slot[4:0];
  wire       idxc_f = (f_idx == 2'(IDX_SW)) ? sw_ch : ch;

  logic boot;
  always_ff @(posedge clk or negedge rst_n) if (!rst_n) boot <= 1'b1; else boot <= 1'b0;
  always_comb begin
    upc_n = upc + 8'd1; hold = 1'b0;
    if (call_go) upc_n = call_to;
    else if (op_busy || op_start || mon_start) hold = 1'b1;
    else case (f_flow)
      3'(FLOW_BR):      if (cond(f_bc)) upc_n = f_tgt;
      3'(FLOW_JMP), 3'(FLOW_SWITCH): upc_n = f_tgt;
      3'(FLOW_RET):     upc_n = ret_addr;
      3'(FLOW_SLOTEND): upc_n = UA_DISPATCH;
      3'(FLOW_DISPATCH): begin
        if (slot[5]) ;
        else if (active[ch][slot[4:0]]) upc_n = UA_SLOT;
        else hold = 1'b1;
      end
      3'(FLOW_WAIT): begin
        case (f_bc)
          5'(C_GO):      hold = !(go_ok && !dsp_act[ch] && !op_ready);
          5'(C_DSPIDLE): hold = !dsp_idle;
          5'(C_DSPDONE): hold = !dsp_done_ch;
          5'(C_FETCHED): hold = !(p1_ok && p2_ok);
          default:       hold = 1'b0;
        endcase
      end
      default: ;
    endcase
    if (hold) upc_n = upc;
    e_bub = hold || call_go || boot;
    if (boot) upc_n = UA_TURN;
  end
  assign at_go = f_flow == 3'(FLOW_WAIT) && f_bc == 5'(C_GO) && !call_go && !boot;
  assign at_ch = ch;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      upc <= UA_TURN; ret_addr <= UA_TURN; slot <= 6'd32; sw <= '0; slot_e <= '0; sw_e <= '0;
      active[0] <= '0; active[1] <= '0; f1 <= 1'b0; f2 <= 1'b0; f3 <= 1'b0; f4 <= 1'b0; stop_rel <= 1'b0;
      ch <= 1'b0; ch_e <= 1'b0; sw_ch <= 1'b0; pend <= 2'b00;
    end else begin
      upc <= upc_n;
      if (!hold && !call_go) begin slot_e <= slot[4:0]; sw_e <= sw; ch_e <= ch; end
      if (call_go) begin ret_addr <= upc; sw <= call_sw; sw_ch <= call_ch; end
      if (!hold && !call_go && !op_busy) begin
        if (f_flow == 3'(FLOW_DISPATCH) && !slot[5] && active[ch][slot[4:0]]) begin
          f1 <= 1'b0; f2 <= 1'b0; f3 <= 1'b0; f4 <= 1'b0; stop_rel <= 1'b0;
        end
        if (f_flow == 3'(FLOW_SLOTEND)) begin
          slot <= slot + 6'd1;
          if (stop_now && upc != UA_SKIP) active[ch][slot[4:0]] <= 1'b0;
        end
        if (f_flow == 3'(FLOW_SWITCH)) ch <= ~ch;
      end
      if (hold && !op_busy && !op_start && !mon_start && f_flow == 3'(FLOW_DISPATCH) && !slot[5] && !active[ch][slot[4:0]])
        slot <= slot + 6'd1;
      if (e_s2 == 4'(S2_ICLR)) slot <= 6'd0;
      if (e_s2 == 4'(S2_IINC)) slot <= slot + 6'd1;
      if (e_s2 == 4'(S2_SWINC)) sw <= sw + 5'd1;
      if (e_s2 == 4'(S2_ACTSET)) active[sw_ch][sw_e] <= 1'b1;
      if (e_s2 == 4'(S2_DSPGO)) pend[ch_e] <= 1'b1;
      if (e_s2 == 4'(S2_OUTV))  pend[ch_e] <= 1'b0;
      if (e_fset == 3'(FSET_F1)) f1 <= !borrow;
      if (e_fset == 3'(FSET_F2)) f2 <= !borrow;
      if (e_fset == 3'(FSET_F3)) f3 <= scmp;
      if (e_fset == 3'(FSET_F4)) f4 <= scmp;
      if (eg2 && egst_n_rel_stop) stop_rel <= 1'b1;
    end
  end

  // =====================================================================
  // RAMs: slot registers, slot state, the register file ({chip, ...})
  // =====================================================================
  logic  [9:0] sra, srb;
  logic        srb_we;
  logic  [1:0] srb_be;
  logic [15:0] srb_d;
  (* ramstyle = "M10K" *) logic [1:0][7:0] sreg [1024];
  always_ff @(posedge clk) sqa <= sreg[sra];
  always_ff @(posedge clk) begin
    if (srb_we && srb_be[0]) sreg[srb][0] <= srb_d[7:0];
    if (srb_we && srb_be[1]) sreg[srb][1] <= srb_d[15:8];
    sqb <= sreg[srb];
  end

  logic  [8:0] st_ra, st_wa, st_wa2;
  logic        st_we;
  logic [31:0] st_wd;
  (* ramstyle = "M10K" *) logic [31:0] st [512];
  always_ff @(posedge clk) begin
    if (st_we) st[st_wa2] <= st_wd;
    st_q <= st[st_ra];
  end

  logic        rf_we;
  (* ramstyle = "M10K" *) logic [31:0] rfa [32];
  (* ramstyle = "M10K" *) logic [31:0] rfb [32];
  always_ff @(posedge clk) begin
    if (rf_we) begin rfa[{ch_e, e_rd}] <= alu_r; rfb[{ch_e, e_rd}] <= alu_r; end
    ra_q <= rfa[{ch, f_ra}];
    rb_q <= rfb[{ch, f_rb}];
  end

  // =====================================================================
  // tables
  // =====================================================================
  m2_scsp_lfo u_lfo (.clk(clk), .tadr(tadr), .tbl_q(tbl_q));
  (* ramstyle = "M10K" *) logic [12:0] eg_rom [1024];
  (* ramstyle = "M10K" *) logic [26:0] rate_rom [128];
  initial begin
    $readmemh("scsp_eg.hex", eg_rom);
    $readmemh("scsp_rate.hex", rate_rom);
  end
  always_ff @(posedge clk) begin
    egt_q  <= eg_rom[egt_a];
    rate_q <= rate_rom[{eg_pre != ATT, st_q[5:0]}];
    hold_r <= st_q[6];
  end

  // =====================================================================
  // the operands
  // =====================================================================
  wire signed [31:0] p_lo = prod[31:0];
  function automatic logic [31:0] psl_of(input logic signed [44:0] p, input logic [1:0] sel, input logic p8);
    logic signed [31:0] lo;
    lo = p[31:0];
    case (sel)
      2'(PSL_S6):  psl_of = p[37:6];
      2'(PSL_S12): psl_of = 32'(lo >>> 12);
      2'(PSL_S16): psl_of = 32'(lo >>> 16);
      default:     psl_of = 32'(p >>> 17) & ~{31'd0, !p8};
    endcase
  endfunction
  wire [8:0] lfo_step = scsp_lfostep(lfof);

  always_comb begin
    case (e_a)
      3'(A_RA):   a_v = ra_q;
      3'(A_Q):    a_v = q;
      3'(A_ST):   a_v = (e_op == 4'(OP_EG)) ? {6'd0, st_q[25:0]} : st_q;
      3'(A_CSH):  a_v = pcm8 ? {12'd0, st_q[31:12]} : {11'd0, st_q[31:12], 1'b0};
      3'(A_PSL):  a_v = psl_v;
      3'(A_P2):   a_v = 32'(p2v);
      3'(A_RING): a_v = FM ? 32'($signed(ring_q)) : 32'd0;
      default:    a_v = 32'(ef_q);
    endcase
    case (e_b)
      3'(B_RB):   b_v = rb_q;
      3'(B_IMM):  b_v = e_immhi ? {e_imm, 16'd0} : {16'd0, e_imm};
      3'(B_Q):    b_v = q;
      3'(B_SA):   b_v = {12'd0, sqb[3:0], sqa};
      3'(B_SQ12): b_v = {4'd0, sqa, 12'd0};
      3'(B_LFOI): b_v = {7'd0, (plfos != 3'd0) ? lfo_step : 9'd0, 7'd0, (alfos != 3'd0) ? lfo_step : 9'd0};
      3'(B_RATE): b_v = {5'd0, rate_q};
      default:    b_v = 32'(p1v);
    endcase
  end

  // =====================================================================
  // the ALU
  // =====================================================================
  logic [1:0] egst_n;
  logic       egst_n_rel_stop;
  logic [25:0] eg_v;
  wire  [4:0] eg_dl = 5'h1f - sqa[9:5];
  wire        eg_sub = eg_pre != ATT;

  always_comb begin
    logic [31:0] bb;
    logic        sub, split, cin;
    sub = e_op == 4'(OP_SUB) || (e_op == 4'(OP_EG) && eg_sub);
    split = e_op == 4'(OP_ADDSPLIT);
    cin = sub || (e_op == 4'(OP_ADDSGN) && a_v[31]);
    bb = (e_op == 4'(OP_ADDSGN)) ? 32'd0 : sub ? ~b_v : b_v;
    begin
      logic [16:0] lo;
      logic [16:0] hi;
      lo = {1'b0, a_v[15:0]} + {1'b0, bb[15:0]} + {16'd0, cin};
      hi = {1'b0, a_v[31:16]} + {1'b0, bb[31:16]} + {16'd0, lo[16] && !split};
      sum = {hi[15:0], lo[15:0]};
      borrow = sub && !hi[16];
    end
    scmp = sum[31:28] != 4'd0 || sum[27:12] >= sqa;
  end

  always_comb begin
    logic signed [31:0] v;
    v = $signed(eg2_sum);
    egst_n = eg2_pre; egst_n_rel_stop = 1'b0;
    case (eg2_pre)
      ATT: begin
        if (v >= 32'sh03ff0000) begin
          if (!eg2_lpslnk) egst_n = DC1;
          v = 32'sh03ff0000;
        end
      end
      DC1: begin
        if (v < 0) v = 0;
        if (v[25:21] <= eg2_dl) egst_n = DC2;
      end
      DC2: begin
        if (eg2_d2r) v = $signed({6'd0, eg2_stq});
        else if (v < 0) v = 0;
      end
      default: begin
        if (v <= 0) begin v = 0; egst_n_rel_stop = 1'b1; end
      end
    endcase
    eg_v = v[25:0];
  end
  assign eg2_r = {4'd0, ~egst_n, eg_v};

  function automatic logic signed [15:0] dac(input logic signed [31:0] s, input logic d18);
    logic signed [31:0] x;
    x = d18 ? s : (s >>> 2);
    if (d18) begin
      if (x[31:17] != {15{x[31]}}) x = x[31] ? -32'sd131072 : 32'sd131071;
      x = x >>> 2;
    end else if (x[31:15] != {17{x[31]}}) x = x[31] ? -32'sd32768 : 32'sd32767;
    dac = x[15:0];
  endfunction

  wire pred_ok = cond({1'b0, e_pred});
  always_comb begin
    case (e_op)
      4'(OP_PASSA): alu_r = a_v;
      4'(OP_PASSB): alu_r = b_v;
      4'(OP_ANDN):  alu_r = a_v & ~b_v;
      4'(OP_RATEIDX): begin
        logic [4:0] r5;
        logic signed [9:0] x;
        case (e_imm[1:0])
          2'(RI_AR):  r5 = sqa[4:0];
          2'(RI_D1R): r5 = sqa[10:6];
          2'(RI_D2R): r5 = sqa[15:11];
          default:    r5 = sqb[4:0];
        endcase
        x = $signed(a_v[9:0]) + $signed({4'd0, r5, 1'b0});
        if (x < 0) x = 0;
        else if (x > 63) x = 63;
        alu_r = {25'd0, e_imm[1:0] == 2'(RI_AR) && sqa[5], x[5:0]};
      end
      4'(OP_EGBASE): begin
        if (sqb[13:10] == 4'hf) alu_r = 32'd0;
        else alu_r = 32'($signed(sqa[14:11])) + {26'd0, sqb[13:10], 1'b0} + {31'd0, sqa[9]};
      end
      4'(OP_SBC): begin
        logic [17:0] s;
        s = sum[17:0];
        if (ssctl != 2'd0) s = 18'd0;
        if (sbctl[0]) s = s ^ 18'h07fff;
        if (sbctl[1]) s = {{2{~s[15]}}, ~s[15], s[14:0]};
        alu_r = 32'($signed(s));
      end
      4'(OP_DAC):   alu_r = 32'(dac($signed(ra_q), dac18));
      default:      alu_r = sum;
    endcase
  end

  wire go_e = pred_ok;
  assign rf_we = e_rdwe && go_e;
  wire   eg_go = e_op == 4'(OP_EG) && go_e;
  assign st_we = (e_stwe && go_e && !eg_go) || eg2_we;
  assign st_wa = (e_widx == 2'(IDX_SW)) ? {sw_ch, sw_e, e_stwrow} : {ch_e, slot_e, e_stwrow};
  assign st_wa2 = eg2_we ? eg2_wa : st_wa;
  assign st_wd  = eg2_we ? eg2_r : alu_r;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      eg2 <= 1'b0; eg2_we <= 1'b0; eg2_tadre <= 1'b0; eg2_d2r <= 1'b0; eg2_lpslnk <= 1'b0; eg2_hold <= 1'b0;
      eg2_sum <= '0; eg2_pre <= ATT; eg2_dl <= '0; eg2_stq <= '0; eg2_wa <= '0;
    end else begin
      eg2 <= eg_go; eg2_we <= eg_go && e_stwe; eg2_tadre <= eg_go && e_s1 == 4'(S1_TADRE);
      if (eg_go) begin
        eg2_sum <= sum; eg2_pre <= eg_pre; eg2_dl <= eg_dl; eg2_d2r <= d2r_zero; eg2_lpslnk <= lpslnk;
        eg2_stq <= st_q[25:0]; eg2_wa <= st_wa; eg2_hold <= hold_r;
      end
    end
  end

  assign st_ra = {idxc_f, idx_f, f_streg ? {1'b1, eg_pre} : f_strow};

  // =====================================================================
  // the multiplier
  // =====================================================================
  wire [16:0] pow2p = 17'(1) << {~sqa[14], sqa[13:11]};
  wire [16:0] pow2f = 17'(1) << ({1'b0, mdl} + {4'd0, !pcm8});
  wire [12:0] egv   = eg_lin ? (eg_hold ? 13'h0ffc : {1'b0, eg_lv, 2'b00}) : egt_q;
  function automatic logic [16:0] mvol_gain(input logic [3:0] mv);
    mvol_gain = 17'((({13'd0, mv} * 17'd65536) + 17'd7) / 17'd15);
  endfunction
  always_ff @(posedge clk) begin
    if (e_mul && go_e) begin
      case (e_ma)
        2'(MA_RA):  mul_a <= $signed(ra_q[26:0]);
        2'(MA_Q):   mul_a <= $signed(q[26:0]);
        2'(MA_PSL): mul_a <= $signed(psl_v[26:0]);
        default:    mul_a <= $signed({16'd0, 1'b1, sqa[9:0]});
      endcase
      case (e_mb)
        3'(MB_POW2P): mul_b <= $signed({1'b0, pow2p});
        3'(MB_POW2F): mul_b <= $signed({1'b0, pow2f});
        3'(MB_TBL4):  mul_b <= $signed({5'd0, tbl_q, 4'd0});
        3'(MB_EGV):   mul_b <= $signed({5'd0, egv});
        3'(MB_LV):    mul_b <= $signed({3'd0, lv_q});
        3'(MB_FRAC):  mul_b <= $signed({6'd0, frac});
        3'(MB_MVG):   mul_b <= $signed({1'b0, mvol_gain(mvol)});
        default:      mul_b <= $signed({8'd0, 10'(10'd64 << boost)});
      endcase
    end
  end
  wire signed [44:0] prod_n = mul_a * mul_b;
  wire  [1:0] psl_n  = ue_f[`UE_PSL];
  wire        pcm8_n = (e_latb == 2'(LATB_W0)) ? sqb[4] : w0l[4];
  always_ff @(posedge clk) begin
    prod  <= prod_n;
    psl_v <= psl_of(prod_n, psl_n, pcm8_n);
  end

  // =====================================================================
  // E: registers, latches, side effects
  // =====================================================================
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      q <= '0; w0l <= '0; w9l <= '0; mdl <= '0; fm_mod <= 1'b0; lpslnk <= 1'b0; d2r_zero <= 1'b0;
      stwinh <= 1'b0; isel <= '0; egst_l <= REL; eg_lin <= 1'b0; eg_hold <= 1'b0; eg_lv <= '0;
      tadr <= '0; egt_a <= '0; frac <= '0; fk <= 1'b0; fr <= 1'b0;
      dsp_go <= 1'b0; dsp_pend <= 1'b0; dsp_pch <= 1'b0; out_valid <= 2'b00; dbg_slot_valid <= 1'b0;
      dbg_samples <= '0; par <= 2'b00; mon <= '0; mon_ca <= '0;
      out_raw_l <= '0; out_raw_r <= '0; out_l <= '0; out_r <= '0; dbg_slot <= '0; dbg_slot_smp <= '0;
    end else begin
      dsp_go <= 1'b0; out_valid <= 2'b00; dbg_slot_valid <= 1'b0;
      if (dsp_done) dsp_pend <= 1'b0;
      q <= alu_r;
      case (e_lata)
        3'(LATA_W9):  w9l <= sqa[14:0];
        3'(LATA_W4):  d2r_zero <= sqa[15:11] == 5'd0;
        3'(LATA_W5):  lpslnk <= sqa[14];
        3'(LATA_W6):  stwinh <= sqa[9];
        3'(LATA_W10): isel <= sqa[6:3];
        default: ;
      endcase
      case (e_latb)
        2'(LATB_W0): w0l <= sqb;
        2'(LATB_W7): begin mdl <= sqb[15:12]; fm_mod <= sqb != 16'd0; end
        default: ;
      endcase
      if (e_fset == 3'(FSET_KBREL)) begin fk <= sqb[11]; fr <= st_q[27:26] == 2'd0; end
      if (go_e) begin
        case (e_s1)
          4'(S1_TADRP): tadr <= {1'b0, plfos, plfows, sum[31:24]};
          4'(S1_TADRA): tadr <= {1'b1, alfos, alfows, a_v[15:8]};
          4'(S1_LATEG): egst_l <= ~st_q[27:26];
          4'(S1_FETCH1): frac <= st_q[11:0];
          4'(S1_DBG): begin dbg_slot_valid <= 1'b1; dbg_slot <= {ch_e, slot_e}; dbg_slot_smp <= a_v; end
          4'(S1_OUTLRAW): out_raw_l[ch_e] <= a_v;
          4'(S1_OUTRRAW): out_raw_r[ch_e] <= a_v;
          4'(S1_OUTL): out_l[ch_e] <= a_v[15:0];
          4'(S1_OUTR): out_r[ch_e] <= a_v[15:0];
          default: ;
        endcase
      end
      if (eg2_tadre) begin
        egt_a <= eg2_r[25:16]; eg_lv <= eg2_r[25:16];
        eg_lin <= eg2_pre == ATT; eg_hold <= eg2_hold;
      end
      case (e_s2)
        4'(S2_DSPGO): begin dsp_go <= 1'b1; dsp_pend <= 1'b1; dsp_pch <= ch_e; end
        4'(S2_OUTV): begin
          out_valid[ch_e] <= 1'b1; par[ch_e] <= ~par[ch_e];
          if (!(&dbg_samples)) dbg_samples <= dbg_samples + 32'd1;
        end
        4'(S2_MON1): mon_ca <= st_q[27:24];
        4'(S2_MON2): if (step_mode) mon[ch_e] <= {5'd0, mon_ca, ~st_q[27:26], 5'h1f - st_q[25:21]};
        4'(S2_MON3): mon[sw_ch] <= {5'd0, mon_ca, ~st_q[27:26], 5'h1f - st_q[25:21]};
        default: ;
      endcase
    end
  end

  // =====================================================================
  // the sample bytes: a1 then a2 through the slot cache
  // =====================================================================
  typedef enum logic [2:0] { G_IDLE, G_L1, G_W1, G_F1, G_L2, G_W2, G_F2 } gst_t;
  gst_t        gs;
  logic        lk_req, lk_hit, fill_req, fill_done;
  logic [16:0] lk_line, fill_line, c_line;
  logic [63:0] lk_data, fill_data;
  function automatic logic signed [16:0] pick(input logic [63:0] ln, input logic [2:0] a, input logic p8);
    logic [15:0] wd;
    wd = ln[{a[2:1], 4'b0} +: 16];
    if (p8) pick = a[0] ? $signed({wd[7], wd[7:0], 8'd0}) : $signed({wd[15], wd[15:8], 8'd0});
    else    pick = $signed({wd[15], wd});
  endfunction
  wire fetch1 = go_e && e_s1 == 4'(S1_FETCH1);
  wire fetch2 = go_e && e_s1 == 4'(S1_FETCH2);
  wire signed [16:0] pk = pick(lk_data, (gs == G_W2) ? fa2[2:0] : fa1[2:0], pcm8);
  wire second = gs == G_L2 || gs == G_W2 || gs == G_F2;
  always_comb begin
    lk_line = pl(fch, second ? fa2[18:3] : fa1[18:3]);
    fill_line = lk_line;
    lk_req = gs == G_L1 || (gs == G_L2 && f2_known && !fa2[19]);
    fill_req = gs == G_F1 || gs == G_F2;
  end
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      gs <= G_IDLE; p1_ok <= 1'b0; p2_ok <= 1'b0; f2_known <= 1'b0; fa1 <= '0; fa2 <= '0; fch <= 1'b0;
      p1v <= '0; p2v <= '0;
    end else begin
      if (fetch1) begin
        fa1 <= sum[19:0]; fch <= ch_e; p1_ok <= 1'b0; p2_ok <= 1'b0; f2_known <= 1'b0;
        if (ssctl != 2'd0) begin p1v <= '0; p2v <= '0; p1_ok <= 1'b1; p2_ok <= 1'b1; gs <= G_IDLE; end
        else if (sum[19]) begin p1v <= '0; p1_ok <= 1'b1; gs <= G_L2; end
        else gs <= G_L1;
      end
      if (fetch2) begin fa2 <= sum[19:0]; f2_known <= 1'b1; end
      case (gs)
        G_L1: gs <= G_W1;
        G_W1: if (lk_hit) begin p1v <= pk; p1_ok <= 1'b1; gs <= G_L2; end else gs <= G_F1;
        G_F1: if (fill_done) gs <= G_L1;
        G_L2: if (f2_known) begin
          if (fa2[19]) begin p2v <= '0; p2_ok <= 1'b1; gs <= G_IDLE; end
          else gs <= G_W2;
        end
        G_W2: if (lk_hit) begin p2v <= pk; p2_ok <= 1'b1; gs <= G_IDLE; end else gs <= G_F2;
        G_F2: if (fill_done) gs <= G_L2;
        default: ;
      endcase
    end
  end

  m2_scsp_cache #(.IDX(SLOT_IDX), .LW(17)) u_cache (
    .clk(clk), .rst_n(rst_n),
    .lk_req(lk_req), .lk_line(lk_line), .lk_hit(lk_hit), .lk_data(lk_data),
    .fill_req(fill_req), .fill_line(fill_line), .fill_done(fill_done), .fill_data(fill_data),
    .m_req(sm_req), .m_line(c_line), .m_ack(sm_ack), .m_data(sm_data),
    .snp(snp), .snp_line(pl(snp_ch, snp_addr[18:3])),
    .snp2(dsp_wr_done), .snp2_line(pl(dsp_wr_ch, dsp_wr_addr[18:3])),
    .dbg_fills(dbg_miss)
  );
  assign sm_ch   = c_line[8];
  assign sm_line = {c_line[16:9], c_line[7:0]};

  // =====================================================================
  // FM: the ring buffer, a chip's 128 words in each half
  // =====================================================================
  logic        ring_we;
  logic  [7:0] ring_wa;
  logic [15:0] ring_wd;
  always_comb begin
    ring_ra = {ch_e, 1'b0, 6'({par[ch_e], slot_e} + ((e_s1 == 4'(S1_RINGX)) ? sqa[11:6] : sqa[5:0]))};
    ring_we = e_s2 == 4'(S2_RINGW) && FM && !stwinh;
    ring_wa = {ch_e, 1'b0, par[ch_e], slot_e};
    ring_wd = 16'(p_lo >>> 13);
    if (op_busy) begin
      ring_ra = {o_ch, o_addr[7:1]};
      ring_we = !op_rd && o_ring && ((oc == 3'd0 && o_be == 2'b11) || oc == 3'd1);
      ring_wa = {o_ch, o_addr[7:1]};
      ring_wd = (oc == 3'd0) ? o_data : {o_be[1] ? o_data[15:8] : ring_q[15:8], o_be[0] ? o_data[7:0] : ring_q[7:0]};
    end
  end
  generate if (FM) begin : g_fm
    (* ramstyle = "M10K" *) logic [15:0] ring [256];   // (D-27: M10K, not m2_scsp's MLAB)
    always_ff @(posedge clk) begin
      if (ring_we) ring[ring_wa] <= ring_wd;
      ring_q <= ring[ring_ra];
    end
  end else begin : g_nofm
    assign ring_q = 16'd0;
  end endgenerate

  // =====================================================================
  // the level words, the DSP
  // =====================================================================
  assign lv_raddr = {idxc_f, idx_f, f_lw};
  m2_scsp_level #(.SW(6)) u_level (
    .clk(clk), .rst_n(rst_n),
    .req(lv_req), .req_slot({o_ch, o_slot}), .req_w6({7'd0, w6l}), .req_w10(sqb), .req_w11(sqa),
    .busy(lv_busy), .rd_addr(lv_raddr), .rd_q(lv_q)
  );

  assign mix_we  = e_s2 == 4'(S2_MIXW);
  assign mix_val = p_lo[29:10];
  scsp_dual_dsp #(.MADRS_MIRROR(MADRS_MIRROR), .IDX(DSP_IDX)) u_dsp (
    .clk(clk), .rst_n(rst_n),
    .go(dsp_go), .go_ch(dsp_pch), .done(dsp_done), .busy(dsp_busy), .act(dsp_act),
    .rbl0(rbl_c[0]), .rbp0(rbp_c[0]), .rbl1(rbl_c[1]), .rbp1(rbp_c[1]),
    .mix_ch(ch_e), .mix_sel(isel), .mix_we(mix_we), .mix_val(mix_val),
    .ef_ch(ch), .ef_idx(slot[3:0]), .ef_q(ef_q),
    .w_en(dw_en), .w_ch(o_ch), .w_addr(o_addr), .w_data(o_data), .w_be(o_be), .r_addr(o_addr), .r_q(dsp_rq),
    .m_req(dm_req), .m_ch(dm_ch), .m_we(dm_we), .m_addr(dm_addr), .m_wdata(dm_wdata),
    .m_ack(dm_ack), .m_data(dm_data), .snp(snp), .snp_ch(snp_ch), .snp_addr(snp_addr),
    .wr_done(dsp_wr_done), .wr_ch(dsp_wr_ch), .wr_addr(dsp_wr_addr)
  );

  // =====================================================================
  // the 68000's accesses (o_ch the chip)
  // =====================================================================
  wire slotend_clr = !op_busy && f_flow == 3'(FLOW_SLOTEND) && !hold && !call_go && stop_now && upc != UA_SKIP;
  always_comb begin
    sra = {idxc_f, idx_f, f_swa}; srb = {idxc_f, idx_f, f_swb};
    srb_we = 1'b0; srb_be = 2'b11; srb_d = o_data;
    if (slotend_clr) begin
      srb = {ch, slot[4:0], 4'd0}; srb_we = 1'b1; srb_be = 2'b10; srb_d = {w0l[15:12], 1'b0, w0l[10:0]};
    end
    if (op_busy) begin
      sra = {o_ch, o_slot, (oc >= 3'd2) ? 4'd11 : 4'd6};
      srb = {o_ch, o_slot, (oc >= 3'd1 && !op_rd) ? 4'd10 : o_word};
      if (!op_rd && oc == 3'd0 && o_addr < 11'h200) begin
        srb_we = 1'b1; srb_be = o_be; srb_d = (o_word == 4'd0) ? (o_data & 16'hefff) : o_data;
      end
    end
  end
  assign dw_en = op_busy && !op_rd && oc == 3'd0 && o_dsp;
  wire   o_com = o_addr >= 11'h200 && o_addr < 11'h218;
  logic  o_com_r;
  always_ff @(posedge clk) o_com_r <= o_com;
  assign cw_en = op_busy && !op_rd && oc == 3'd1 && o_com_r;
  assign cw_en_c = {cw_en && o_ch, cw_en && !o_ch};

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      op_busy <= 1'b0; op_rd <= 1'b0; o_addr <= '0; o_data <= '0; o_be <= '0; oc <= '0; o_ch <= 1'b0;
      wq_pop <= 1'b0; rd_done <= 1'b0; mi_pop <= 1'b0; lv_req <= 1'b0; w6l <= '0;
      call_go <= 1'b0; call_to <= UA_TURN; call_sw <= '0; call_ch <= 1'b0; cpu_dout <= '0;
    end else begin
      wq_pop <= 1'b0; rd_done <= 1'b0; mi_pop <= 1'b0; call_go <= 1'b0;
      if (lv_req && !lv_busy) lv_req <= 1'b0;
      if (mon_start) begin
        call_go <= 1'b1; call_to <= UA_MON;
        call_ch <= !mon_req[0]; call_sw <= mon_req[0] ? mon_slot[0] : mon_slot[1];
      end
      if (mon_go) cpu_dout <= mon_go_c[0] ? mon[0] : mon[1];
      if (op_start) begin
        op_busy <= 1'b1; oc <= '0; op_rd <= wq_n == 3'd0;
        o_addr <= nx_addr; o_ch <= nx_ch; o_data <= wq_h[15:0]; o_be <= wq_h[28:27];
      end else if (op_busy) begin
        if (!op_rd) begin
          if (o_addr < 11'h200) begin
            case (oc)
              3'd0: begin
                wq_pop <= 1'b1;
                if (o_word == 4'd0 && o_be[1] && o_data[12]) begin
                  call_go <= 1'b1; call_to <= UA_SWEEP; call_sw <= 5'd0; call_ch <= o_ch; op_busy <= 1'b0;
                end else if (o_word == 4'd5) begin
                  call_go <= 1'b1; call_to <= UA_W5; call_sw <= o_slot; call_ch <= o_ch; op_busy <= 1'b0;
                end else if (o_word == 4'd6 || o_word == 4'd10 || o_word == 4'd11) oc <= 3'd1;
                else op_busy <= 1'b0;
              end
              3'd1: oc <= 3'd2;
              3'd2: begin w6l <= {sqa[8], sqa[7:0]}; oc <= 3'd3; end
              default: if (!lv_busy && !lv_req) begin lv_req <= 1'b1; op_busy <= 1'b0; end
            endcase
          end else if (((o_ring && o_be != 2'b11) || o_com) && oc == 3'd0) begin
            wq_pop <= 1'b1; oc <= 3'd1;
          end else begin
            if (oc == 3'd0) wq_pop <= 1'b1;
            op_busy <= 1'b0;
          end
        end else begin
          if (oc == 3'd0) oc <= 3'd1;
          else begin
            if (o_addr < 11'h200) cpu_dout <= sqb;
            else if (o_ring) cpu_dout <= ring_q;
            else if (o_dsp) cpu_dout <= dsp_rq;
            else if (o_addr < 11'h218) cpu_dout <= regs_q[o_ch];
            else cpu_dout <= 16'd0;
            if (o_addr == 11'h202 && !o_ch) mi_pop <= 1'b1;
            rd_done <= 1'b1; op_busy <= 1'b0;
          end
        end
      end
    end
  end

  // =====================================================================
  // THE 0x408 MONITOR IN THE 68000's TIME (m2_scsp R885), a chip each
  // =====================================================================
  assign mon_fast = !step_mode && cpu_addr == 11'h204;
  wire  [1:0] mon_go_c = mon_rd & ~mon_pend;
  assign mon_go   = mon_go_c != 2'b00;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      mslc_c <= '0; mon_slot <= '0; mon_req <= 2'b00; mon_pend <= 2'b00; mon_rd <= 2'b00;
    end else begin
      if (cpu_req && cpu_we && cpu_addr == 11'h204 && cpu_be[1]) mslc_c[cpu_ch] <= cpu_din[15:11];
      for (int c = 0; c < 2; c++) begin
        if (cpu_tick && !step_mode) begin mon_slot[c] <= mslc_c[c]; mon_req[c] <= 1'b1; mon_pend[c] <= 1'b1; end
        else begin
          if (mon_start && (c == 0 ? mon_req[0] : !mon_req[0])) mon_req[c] <= 1'b0;
          if (e_s2 == 4'(S2_MON3) && sw_ch == 1'(c) && !mon_req[c]) mon_pend[c] <= 1'b0;
        end
      end
      if (cpu_req && !cpu_we && mon_fast) mon_rd[cpu_ch] <= 1'b1;
      else mon_rd <= mon_rd & ~mon_go_c;
    end
  end

  // =====================================================================
  // common registers, timers, interrupts, MIDI: one block a chip
  // =====================================================================
  m2_scsp_regs #(.MIDI_OUT(MIDI_OUT), .TIMER(TIMER)) u_regs0 (
    .clk(clk), .rst_n(rst_n),
    .a(o_addr[5:1]), .we(cw_en_c[0]), .d(o_data), .be(o_be), .q(regs_q[0]), .mi_pop(mi_pop), .mon(mon[0]),
    .tick44(tick44), .exact(!step_mode), .tph(tph), .tarc(tarc),
    .midi_data(midi_data), .midi_valid(midi_valid), .midi_ack(midi_ack),
    .mout_data(mout_data), .mout_valid(mout_valid), .mout_ack(mout_ack),
    .ipl(ipl0), .mvol(mvol_c[0]), .dac18(dac18_c[0]), .rbl(rbl_c[0]), .rbp(rbp_c[0]), .mslc(mslc_r[0])
  );
  generate if (CH1_TIMERS) begin : g_r1
  m2_scsp_regs #(.MIDI_OUT(1'b0), .TIMER(TIMER)) u_regs1 (
      .clk(clk), .rst_n(rst_n),
      .a(o_addr[5:1]), .we(cw_en_c[1]), .d(o_data), .be(o_be), .q(regs_q[1]), .mi_pop(1'b0), .mon(mon[1]),
      .tick44(tick44), .exact(!step_mode), .tph(tph), .tarc(tarc),
      .midi_data(8'd0), .midi_valid(1'b0), .midi_ack(),
      .mout_data(), .mout_valid(), .mout_ack(1'b0),
      .ipl(ipl1), .mvol(mvol_c[1]), .dac18(dac18_c[1]), .rbl(rbl_c[1]), .rbp(rbp_c[1]), .mslc(mslc_r[1])
    );
  end else begin : g_r1lite
    (* ramstyle = "MLAB" *) logic [15:0] ud1 [32];
    logic [15:0] ud1_q;
    logic  [5:1] ar1;
    wire  [15:0] q1 = (ar1 == 5'h04) ? mon[1] : (ar1 >= 5'h18) ? 16'd0 : ud1_q;
    wire  [15:0] mrg1 = {o_be[1] ? o_data[15:8] : q1[15:8], o_be[0] ? o_data[7:0] : q1[7:0]};
    always_ff @(posedge clk) begin
      ar1 <= o_addr[5:1];
      if (cw_en_c[1]) ud1[ar1] <= mrg1;
      ud1_q <= ud1[o_addr[5:1]];
    end
    assign regs_q[1] = q1;
    always_ff @(posedge clk or negedge rst_n) begin
      if (!rst_n) begin mvol_c[1] <= '0; dac18_c[1] <= 1'b0; rbl_c[1] <= '0; rbp_c[1] <= '0; mslc_r[1] <= '0; end
      else if (cw_en_c[1]) begin
        if (ar1 == 5'h00) begin mvol_c[1] <= mrg1[3:0]; dac18_c[1] <= mrg1[8]; end
        if (ar1 == 5'h01) begin rbl_c[1] <= mrg1[8:7]; rbp_c[1] <= mrg1[5:0]; end
        if (ar1 == 5'h04) mslc_r[1] <= mrg1[15:11];
      end
    end
    assign ipl1 = 3'd0;
  end endgenerate

  // (the DSP may still be running the other chip)
  assign quiet = upc == UA_IDLE && !op_busy && !call_go && !op_ready && !wq_pop && !rd_done &&
                 !lv_busy && !lv_req;

  /* verilator lint_off UNUSEDSIGNAL */
  wire _unused = &{1'b0, LFO_NOISE, prod[44:32], e_psl, w0l[11], egst_n_rel_stop, ue[`UE_IMM], ret_addr};
  /* verilator lint_on UNUSEDSIGNAL */

endmodule
