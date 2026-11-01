// SPDX-License-Identifier: GPL-3.0-or-later
//
// Sega Model 3 core for MiSTer FPGA
// Copyright (C) 2026 alphanu1
//
// Standalone fit of ppc_core (M0-A2): ALM, M10K, DSP and Fmax at 100 MHz.
// Every port but the clock is a virtual pin (cpu_spike.qsf); the retire trace
// is left unconnected so it is trimmed, as it is in the core build.

module cpu_spike_top (
  input  logic        clk,
  input  logic        rst,
  output logic        i_req,
  output logic [31:0] i_addr,
  input  logic        i_valid,
  input  logic [31:0] i_data,
  input  logic [1:0]  i_way,
  input  logic        i_arr,
  input  logic [19:0] i_pa,
  output logic        i_pv,
  output logic [11:2] i_pidx,
  output logic [1:0]  i_pway,
  output logic [19:0] i_ppa,
  output logic        i_pre,
  input  logic        i_two,
  output logic        i_w2,
  input  logic [31:0] i_data2,
  output logic        d_req,
  output logic        d_we,
  output logic [31:0] d_addr,
  output logic [3:0]  d_be,
  output logic [31:0] d_wdata,
  output logic [3:0]  d_wimg,
  output logic [2:0]  d_op,
  input  logic        d_rdy,
  input  logic        d_idle,
  output logic [3:0]  i_wimg,
  output logic        i_seq,
  output logic [6:0]  hid,
  output logic        icbi_req,
  output logic [6:0]  icbi_set,
  input  logic        icbi_ack,
  input  logic        d_valid,
  input  logic [31:0] d_rdata,
  input  logic        tb_tick,
  input  logic        ext_int
);

  ppc_core u_cpu (
    .clk (clk), .rst (rst),
    .i_req (i_req), .i_addr (i_addr), .i_wimg (i_wimg), .i_seq (i_seq), .i_valid (i_valid), .i_data (i_data),
    .i_way (i_way), .i_arr (i_arr), .i_pa (i_pa), .i_pv (i_pv), .i_pidx (i_pidx), .i_pway (i_pway), .i_ppa (i_ppa),
    .i_pre (i_pre), .i_two (i_two), .i_w2 (i_w2), .i_data2 (i_data2),
    .d_req (d_req), .d_we (d_we), .d_addr (d_addr), .d_be (d_be), .d_wdata (d_wdata),
    .d_wimg (d_wimg), .d_op (d_op), .d_rdy (d_rdy), .d_idle (d_idle), .d_valid (d_valid), .d_rdata (d_rdata),
    .hid_ice (hid[0]), .hid_dce (hid[1]), .hid_ilock (hid[2]), .hid_dlock (hid[3]), .hid_icfi (hid[4]),
    .hid_dcfi (hid[5]), .hid_noopti (hid[6]),
    .icbi_req (icbi_req), .icbi_set (icbi_set), .icbi_ack (icbi_ack),
    .tb_tick (tb_tick), .ext_int (ext_int), .ev (), .ev_mmu (), .ev_btb (),
    .rt_valid (), .rt_last (), .rt_pc (), .rt_insn (), .rt_gpr_we (), .rt_gpr (), .rt_gpr_sh (), .rt_irq (), .rt_gpr_val (),
    .rt_cr (), .rt_xer (), .rt_lr (), .rt_ctr (), .rt_msr (), .rt_srr0 (), .rt_srr1 (),
    .rt_fpscr (), .rt_fpq (), .rt_fw_en (), .rt_fw_idx (), .rt_fw_val (), .rt_dly ()
  );

endmodule
