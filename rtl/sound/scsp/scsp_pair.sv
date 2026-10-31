// SPDX-License-Identifier: GPL-3.0-or-later
//
// Sega Model 3 core for MiSTer FPGA
// Copyright (C) 2026 alphanu1
//
// Model 3's two SCSPs, as the sound board sees them: one 68000 port (cpu_ch
// the chip, 0x100000 / 0x300000 decoded outside), one slot-fetch port and one
// DSP port to sound RAM (the chip beside each address), and a chip's output
// each. SCSP_SHARED selects the build:
//   1  scsp_dual: one engine for both chips;
//   0  two m2_scsp, with a 2:1 arbiter on each memory port so the board sees
//      the same ports (the arbiters are this build's cost, not the engines').
// Chip 1's interrupt level is not wired on Model 3 (MAME model3.cpp: scsp2
// has no irq callback); it is an output for the bench only.

`timescale 1ns/1ps

module scsp_pair #(
  parameter bit SCSP_SHARED = 1'b1,
  parameter bit FM          = 1'b1,
  parameter bit MIDI_OUT    = 1'b1,
  parameter int TIMER       = 0,
  parameter int unsigned SLOT_IDX = 9,      // the shared build's caches (scsp_dual)
  parameter int unsigned DSP_IDX  = 9,
  parameter bit CH1_TIMERS        = 1'b1     // (scsp_dual)
) (
  input  logic        clk,
  input  logic        rst_n,
  input  logic        cpu_req,
  input  logic        cpu_we,
  input  logic        cpu_ch,
  input  logic [11:1] cpu_addr,
  input  logic [15:0] cpu_din,
  input  logic  [1:0] cpu_be,
  output logic        cpu_ack,
  output logic [15:0] cpu_dout,
  output logic  [2:0] ipl,
  input  logic  [7:0] midi_data,
  input  logic        midi_valid,
  output logic        midi_ack,
  output logic  [7:0] mout_data,
  output logic        mout_valid,
  input  logic        mout_ack,
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
  input  logic        out_ready,
  output logic  [1:0] out_valid,
  output logic [1:0][15:0] out_l, out_r,
  input  logic        tick44,
  input  logic  [5:0] tph,
  input  logic        cpu_tick,
  input  logic        tarc,
  input  logic  [1:0] boost
);

  generate if (SCSP_SHARED) begin : g_one
    scsp_dual #(.FM(FM), .MIDI_OUT(MIDI_OUT), .TIMER(TIMER), .SLOT_IDX(SLOT_IDX), .DSP_IDX(DSP_IDX), .CH1_TIMERS(CH1_TIMERS)) u_dual (
      .clk(clk), .rst_n(rst_n),
      .cpu_req(cpu_req), .cpu_we(cpu_we), .cpu_ch(cpu_ch), .cpu_addr(cpu_addr), .cpu_din(cpu_din), .cpu_be(cpu_be),
      .cpu_ack(cpu_ack), .cpu_dout(cpu_dout), .ipl0(ipl), .ipl1(),
      .midi_data(midi_data), .midi_valid(midi_valid), .midi_ack(midi_ack),
      .mout_data(mout_data), .mout_valid(mout_valid), .mout_ack(mout_ack),
      .sm_req(sm_req), .sm_ch(sm_ch), .sm_line(sm_line), .sm_ack(sm_ack), .sm_data(sm_data),
      .dm_req(dm_req), .dm_ch(dm_ch), .dm_we(dm_we), .dm_addr(dm_addr), .dm_wdata(dm_wdata),
      .dm_ack(dm_ack), .dm_data(dm_data),
      .snp(snp), .snp_ch(snp_ch), .snp_addr(snp_addr),
      .out_ready(out_ready), .out_valid(out_valid), .out_raw_l(), .out_raw_r(), .out_l(out_l), .out_r(out_r),
      .tick44(tick44), .tph(tph), .cpu_tick(cpu_tick), .tarc(tarc), .boost(boost),
      .step_mode(1'b0), .step_go(1'b0), .at_go(), .at_ch(), .quiet(),
      .dbg_slot_valid(), .dbg_slot(), .dbg_slot_smp(), .dbg_samples(), .dbg_miss()
    );
  end else begin : g_two
    logic  [1:0] e_ack;
    logic [1:0][15:0] e_dout;
    logic  [1:0] e_smreq, e_dmreq, e_dmwe;
    logic [1:0][15:0] e_smline, e_dmwd;
    logic [1:0][17:0] e_dmaddr;
    logic  [1:0][2:0] e_ipl;
    logic  [1:0] e_mack, e_mov;
    logic  [1:0][7:0] e_mod;
    logic        rd_c;                       // the chip of the access in flight
    always_ff @(posedge clk) if (cpu_req) rd_c <= cpu_ch;
    assign cpu_ack  = |e_ack;
    assign cpu_dout = e_dout[rd_c];

    // ---- a 2:1 arbiter a port kind: one burst at a time, the request dropped
    // for a clock after each acknowledge (the ports' own convention)
    logic sm_busy, sm_own, sm_gap, dm_busy, dm_own, dm_gap;
    always_ff @(posedge clk or negedge rst_n) begin
      if (!rst_n) begin
        sm_busy <= 1'b0; sm_own <= 1'b0; sm_gap <= 1'b0; dm_busy <= 1'b0; dm_own <= 1'b0; dm_gap <= 1'b0;
      end else begin
        sm_gap <= 1'b0; dm_gap <= 1'b0;
        if (sm_busy && sm_ack) begin sm_busy <= 1'b0; sm_gap <= 1'b1; end
        else if (!sm_busy && !sm_gap && (e_smreq != 2'b00)) begin
          sm_busy <= 1'b1; sm_own <= (e_smreq == 2'b11) ? ~sm_own : e_smreq[1];
        end
        if (dm_busy && dm_ack) begin dm_busy <= 1'b0; dm_gap <= 1'b1; end
        else if (!dm_busy && !dm_gap && (e_dmreq != 2'b00)) begin
          dm_busy <= 1'b1; dm_own <= (e_dmreq == 2'b11) ? ~dm_own : e_dmreq[1];
        end
      end
    end
    assign sm_req = sm_busy;   assign sm_ch = sm_own;  assign sm_line = e_smline[sm_own];
    assign dm_req = dm_busy;   assign dm_ch = dm_own;  assign dm_we = e_dmwe[dm_own];
    assign dm_addr = e_dmaddr[dm_own]; assign dm_wdata = e_dmwd[dm_own];

    genvar c;
    for (c = 0; c < 2; c++) begin : g_e
      m2_scsp #(.FM(FM), .MIDI_OUT(c == 0 ? MIDI_OUT : 1'b0), .TIMER(TIMER)) u (
        .clk(clk), .rst_n(rst_n),
        .cpu_req(cpu_req && cpu_ch == 1'(c)), .cpu_we(cpu_we), .cpu_addr(cpu_addr), .cpu_din(cpu_din), .cpu_be(cpu_be),
        .cpu_ack(e_ack[c]), .cpu_dout(e_dout[c]), .ipl(e_ipl[c]),
        .midi_data(midi_data), .midi_valid(c == 0 ? midi_valid : 1'b0), .midi_ack(e_mack[c]),
        .mout_data(e_mod[c]), .mout_valid(e_mov[c]), .mout_ack(c == 0 ? mout_ack : 1'b0),
        .sm_req(e_smreq[c]), .sm_line(e_smline[c]), .sm_ack(sm_busy && sm_ack && sm_own == 1'(c)), .sm_data(sm_data),
        .dm_req(e_dmreq[c]), .dm_we(e_dmwe[c]), .dm_addr(e_dmaddr[c]), .dm_wdata(e_dmwd[c]),
        .dm_ack(dm_busy && dm_ack && dm_own == 1'(c)), .dm_data(dm_data),
        .snp(snp && snp_ch == 1'(c)), .snp_addr(snp_addr),
        .out_ready(out_ready), .out_valid(out_valid[c]), .out_l_raw(), .out_r_raw(), .out_l(out_l[c]), .out_r(out_r[c]),
        .tick44(tick44), .tph(tph), .cpu_tick(cpu_tick), .tarc(tarc), .boost(boost),
        .step_mode(1'b0), .step_go(1'b0), .quiet(),
        .dbg_slot_valid(), .dbg_slot(), .dbg_slot_smp(), .dbg_samples(), .dbg_miss(), .dbg_mvol()
      );
    end
    assign ipl        = e_ipl[0];
    assign midi_ack   = e_mack[0];
    assign mout_data  = e_mod[0];
    assign mout_valid = e_mov[0];
  end endgenerate

endmodule
