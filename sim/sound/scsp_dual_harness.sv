// SPDX-License-Identifier: GPL-3.0-or-later
// Sega Model 3 core for MiSTer FPGA -- Copyright (C) 2026 alphanu1
//
// tb_scsp_dual's top: two m2_scsp (the reference, a chip each) beside one
// scsp_dual, every port at the top so the bench serves them all. The two
// sides share nothing: each has its own sound RAMs in the bench.
`timescale 1ns/1ps
module scsp_dual_harness #(
  parameter int unsigned SLOT_IDX = 9,
  parameter int unsigned DSP_IDX  = 9,
  parameter bit FM = 1'b1,
  parameter bit CH1_TIMERS = 1'b1
) (
  input  logic        clk, rst_n,
  input  logic        step_mode,
  input  logic        tick44, d_tick44, cpu_tick,   // tick44: the references; d_tick44: the dual
  input  logic  [5:0] tph,
  input  logic        out_ready,
  // ---- the reference, chip c in [c]
  input  logic  [1:0] r_req, r_we,
  input  logic [1:0][10:0] r_addr,
  input  logic [1:0][15:0] r_din,
  input  logic [1:0][1:0] r_be,
  output logic  [1:0] r_ack,
  output logic [1:0][15:0] r_dout,
  output logic [1:0][2:0] r_ipl,
  input  logic  [1:0] r_go,
  output logic  [1:0] r_quiet,
  output logic  [1:0] r_ov,
  output logic [1:0][31:0] r_raw_l, r_raw_r,
  output logic [1:0][15:0] r_l, r_r,
  output logic  [1:0] r_sv,
  output logic [1:0][4:0] r_slot,
  output logic [1:0][31:0] r_smp,
  output logic  [1:0] r_sm_req,
  output logic [1:0][15:0] r_sm_line,
  input  logic  [1:0] r_sm_ack,
  input  logic [1:0][63:0] r_sm_data,
  output logic  [1:0] r_dm_req, r_dm_we,
  output logic [1:0][17:0] r_dm_addr,
  output logic [1:0][15:0] r_dm_wdata,
  input  logic  [1:0] r_dm_ack,
  input  logic [1:0][63:0] r_dm_data,
  input  logic  [1:0] r_snp,
  input  logic [1:0][17:0] r_snp_addr,
  // MIDI (chip 0 on both sides)
  input  logic  [7:0] midi_data,
  input  logic        r_midi_valid, d_midi_valid,
  output logic        r_midi_ack, d_midi_ack,
  output logic  [7:0] r_mout_data, d_mout_data,
  output logic        r_mout_valid, d_mout_valid,
  input  logic        r_mout_ack, d_mout_ack,
  // ---- the dual
  input  logic        d_req, d_we, d_ch,
  input  logic [10:0] d_addr,
  input  logic [15:0] d_din,
  input  logic  [1:0] d_be,
  output logic        d_ack,
  output logic [15:0] d_dout,
  output logic [1:0][2:0] d_ipl,
  input  logic        d_go,
  output logic        d_at_go, d_at_ch, d_quiet,
  output logic  [1:0] d_ov,
  output logic [1:0][31:0] d_raw_l, d_raw_r,
  output logic [1:0][15:0] d_l, d_r,
  output logic        d_sv,
  output logic  [5:0] d_slot,
  output logic [31:0] d_smp,
  output logic        d_sm_req, d_sm_ch,
  output logic [15:0] d_sm_line,
  input  logic        d_sm_ack,
  input  logic [63:0] d_sm_data,
  output logic        d_dm_req, d_dm_ch, d_dm_we,
  output logic [17:0] d_dm_addr,
  output logic [15:0] d_dm_wdata,
  input  logic        d_dm_ack,
  input  logic [63:0] d_dm_data,
  input  logic        d_snp, d_snp_ch,
  input  logic [17:0] d_snp_addr
);
  for (genvar c = 0; c < 2; c++) begin : g_ref
    m2_scsp #(.MIDI_OUT(c == 0)) u (
      .clk(clk), .rst_n(rst_n),
      .cpu_req(r_req[c]), .cpu_we(r_we[c]), .cpu_addr(r_addr[c]), .cpu_din(r_din[c]), .cpu_be(r_be[c]),
      .cpu_ack(r_ack[c]), .cpu_dout(r_dout[c]), .ipl(r_ipl[c]),
      .midi_data(midi_data), .midi_valid(c == 0 ? r_midi_valid : 1'b0), .midi_ack(),
      .mout_data(), .mout_valid(), .mout_ack(c == 0 ? r_mout_ack : 1'b0),
      .sm_req(r_sm_req[c]), .sm_line(r_sm_line[c]), .sm_ack(r_sm_ack[c]), .sm_data(r_sm_data[c]),
      .dm_req(r_dm_req[c]), .dm_we(r_dm_we[c]), .dm_addr(r_dm_addr[c]), .dm_wdata(r_dm_wdata[c]),
      .dm_ack(r_dm_ack[c]), .dm_data(r_dm_data[c]),
      .snp(r_snp[c]), .snp_addr(r_snp_addr[c]),
      .out_ready(out_ready), .out_valid(r_ov[c]), .out_l_raw(r_raw_l[c]), .out_r_raw(r_raw_r[c]),
      .out_l(r_l[c]), .out_r(r_r[c]),
      .tick44(tick44), .tph(tph), .cpu_tick(cpu_tick), .tarc(1'b0), .boost(2'd0),
      .step_mode(step_mode), .step_go(r_go[c]), .quiet(r_quiet[c]),
      .dbg_slot_valid(r_sv[c]), .dbg_slot(r_slot[c]), .dbg_slot_smp(r_smp[c]),
      .dbg_samples(), .dbg_miss(), .dbg_mvol()
    );
  end
  assign r_midi_ack = g_ref[0].u.midi_ack;
  assign r_mout_data = g_ref[0].u.mout_data;
  assign r_mout_valid = g_ref[0].u.mout_valid;

  scsp_dual #(.SLOT_IDX(SLOT_IDX), .DSP_IDX(DSP_IDX), .FM(FM), .CH1_TIMERS(CH1_TIMERS)) u_d (
    .clk(clk), .rst_n(rst_n),
    .cpu_req(d_req), .cpu_we(d_we), .cpu_ch(d_ch), .cpu_addr(d_addr), .cpu_din(d_din), .cpu_be(d_be),
    .cpu_ack(d_ack), .cpu_dout(d_dout), .ipl0(d_ipl[0]), .ipl1(d_ipl[1]),
    .midi_data(midi_data), .midi_valid(d_midi_valid), .midi_ack(d_midi_ack),
    .mout_data(d_mout_data), .mout_valid(d_mout_valid), .mout_ack(d_mout_ack),
    .sm_req(d_sm_req), .sm_ch(d_sm_ch), .sm_line(d_sm_line), .sm_ack(d_sm_ack), .sm_data(d_sm_data),
    .dm_req(d_dm_req), .dm_ch(d_dm_ch), .dm_we(d_dm_we), .dm_addr(d_dm_addr), .dm_wdata(d_dm_wdata),
    .dm_ack(d_dm_ack), .dm_data(d_dm_data),
    .snp(d_snp), .snp_ch(d_snp_ch), .snp_addr(d_snp_addr),
    .out_ready(out_ready), .out_valid(d_ov), .out_raw_l(d_raw_l), .out_raw_r(d_raw_r), .out_l(d_l), .out_r(d_r),
    .tick44(d_tick44), .tph(tph), .cpu_tick(cpu_tick), .tarc(1'b0), .boost(2'd0),
    .step_mode(step_mode), .step_go(d_go), .at_go(d_at_go), .at_ch(d_at_ch), .quiet(d_quiet),
    .dbg_slot_valid(d_sv), .dbg_slot(d_slot), .dbg_slot_smp(d_smp), .dbg_samples(), .dbg_miss()
  );
endmodule
