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
// The CPU subsystem: ppc_core with its instruction and data caches (study
// D-15, R36), presenting one memory port in the caches' protocol
// (ppc_cache_pkg.sv: line fills, cast-outs, single reads and writes, one
// transaction at a time, completed in order).
//
// The two caches' ports are arbitrated here, one transaction at a time; the
// data cache goes first when both ask. A transaction belongs to the cache
// that made it until it completes: eight read beats (line fill), eight write
// beats (cast-out), one read beat (single read), or the acknowledge (single
// write). Every response is routed to that cache alone.
//
// HID0's cache bits come from the core's flip-flops; the flash invalidates
// are one-cycle pulses (the caches latch them). icbi goes from the core's MEM
// to the I-cache. sync waits on the data cache's d_idle.
//
// Snoop (SNOOP = 1, study D-41): the data cache's snoop port, for another
// master's access to memory (ppc_dcache.sv, "Snoop port"). sn_req is held
// until the sn_ack pulse with sn_addr (the physical line) and sn_inv stable.
// A push goes out on the memory port as an ordinary cast-out (MC_LINE_WR)
// before sn_ack, so the port must keep being served while sn_req waits. After
// an sn_ack with sn_inv = 1 the master's write must reach memory before any
// later transaction of this port. The instruction cache is not snooped (603e
// 3.1). SNOOP = 0: sn_ack answers a cycle later and nothing is coherent; tie
// sn_req low.

module ppc_cpu
  import ppc_pkg::*, ppc_cache_pkg::*;
#(
  parameter logic [31:0] RESET_PC = 32'hFFF0_0100,
  parameter logic [31:0] PVR      = 32'h0006_0101,
  parameter bit          BTB      = 1'b1,             // the core's BTB and the I-cache's pre-read port (D-30)
  parameter bit          FETCH2   = 1'b0,             // two-word fetch (D-30; off by default, D-36)
  parameter bit          LA       = 1'b1,             // the 603e's CR look-ahead (D-30)
  parameter bit          SNOOP    = 1'b1,             // the data cache's snoop port (D-41)
  parameter bit          LU2      = 1'b1,             // load-use 2: the D-cache's early word into the core (ppc_core LU2)
  parameter bit          ISB      = 1'b0,             // a 32 KiB I-cache (ppc_icache ISB; the 603e's is 16 KiB)
  parameter bit          FAST     = 1'b1              // the single-precision unit (ppc_core FAST; 0: one FMA)
)(
  input  logic        clk,
  input  logic        rst,

  // memory port (ppc_cache_pkg)
  output logic        m_req,
  output logic [1:0]  m_cmd,
  output logic [31:0] m_addr,
  output logic [3:0]  m_be,
  output logic [31:0] m_wdata,
  input  logic        m_ack,
  input  logic        m_wnext,
  input  logic        m_rvalid,
  input  logic [31:0] m_rdata,

  // snoop (above)
  input  logic        sn_req,
  input  logic [31:5] sn_addr,
  input  logic        sn_inv,
  output logic        sn_ack,

  input  logic        tb_tick,
  input  logic        ext_int,
  output logic [7:0]  ev,
  output logic [5:0]  ev_mmu,
  output logic [3:0]  ev_btb,

  // retire trace (ppc_core), and the core's data requests for a bench
  output logic        rt_valid,
  output logic        rt_last,
  output logic [31:0] rt_pc,
  output logic [31:0] rt_insn,
  output logic        rt_gpr_we,
  output logic [4:0]  rt_gpr,
  output logic        rt_gpr_sh,
  output logic [1:0]  rt_irq,
  output logic [31:0] rt_gpr_val,
  output logic [31:0] rt_cr,
  output logic [31:0] rt_xer,
  output logic [31:0] rt_lr,
  output logic [31:0] rt_ctr,
  output logic [31:0] rt_msr,
  output logic [31:0] rt_srr0,
  output logic [31:0] rt_srr1,
  output logic [31:0] rt_fpscr,
  output logic        rt_fpq,
  output logic        rt_fw_en,
  output logic [4:0]  rt_fw_idx,
  output logic [63:0] rt_fw_val,
  output logic [10:0] rt_dly,
  output logic        t_dreq,
  output logic        t_dwe,
  output logic [31:0] t_daddr,
  output logic [3:0]  t_dwimg,
  output logic [2:0]  t_dop,
  output logic        t_dvalid,
  output logic [31:0] t_drdata,
  output logic [3:0]  t_dbe,
  output logic [31:0] t_dwdata,
  output logic        t_drsv,
  output logic        t_dcnd
);

  // ------------------------------------------------------------ the core
  logic        i_req, i_seq, i_valid;
  logic [31:0] i_addr, i_data;
  logic [1:0]  i_way, i_pway;
  logic        i_arr, i_pv, i_pre, i_eix;
  logic [19:0] i_pa, i_ppa;
  logic [12:2] i_pidx;
  logic        i_two, i_w2;
  logic [31:0] i_data2;
  logic [3:0]  i_wimg;
  logic        d_req, d_we, d_rdy, d_valid, d_idle;
  logic [31:0] d_addr, d_wdata, d_rdata;
  logic [31:0] d_eword;
  logic        d_eok;
  logic [3:0]  d_be, d_wimg;
  logic [2:0]  d_op;
  logic        hid_ice, hid_dce, hid_ilock, hid_dlock, hid_icfi, hid_dcfi, hid_noopti;
  logic        icbi_req, icbi_ack;
  logic [6:0]  icbi_set;
  logic        d_resv, d_cond, res_kill;

  ppc_core #(.RESET_PC (RESET_PC), .PVR (PVR), .BTB (BTB), .FETCH2 (FETCH2), .LA (LA), .LU2 (LU2), .FAST (FAST)) u_core (
    .clk (clk), .rst (rst),
    .i_req (i_req), .i_addr (i_addr), .i_wimg (i_wimg), .i_seq (i_seq), .i_valid (i_valid), .i_data (i_data),
    .i_way (i_way), .i_arr (i_arr), .i_pa (i_pa), .i_pv (i_pv), .i_pidx (i_pidx), .i_pway (i_pway), .i_ppa (i_ppa),
    .i_pre (i_pre), .i_eix (i_eix), .i_two (i_two), .i_w2 (i_w2), .i_data2 (i_data2),
    .d_req (d_req), .d_we (d_we), .d_addr (d_addr), .d_be (d_be), .d_wdata (d_wdata),
    .d_wimg (d_wimg), .d_op (d_op), .d_rdy (d_rdy), .d_idle (d_idle), .d_valid (d_valid), .d_rdata (d_rdata),
    .d_eword (d_eword), .d_eok (d_eok),
    .d_resv (d_resv), .d_cond (d_cond), .res_kill (res_kill),
    .hid_ice (hid_ice), .hid_dce (hid_dce), .hid_ilock (hid_ilock), .hid_dlock (hid_dlock),
    .hid_icfi (hid_icfi), .hid_dcfi (hid_dcfi), .hid_noopti (hid_noopti),
    .icbi_req (icbi_req), .icbi_set (icbi_set), .icbi_ack (icbi_ack),
    .tb_tick (tb_tick), .ext_int (ext_int), .ev (ev), .ev_mmu (ev_mmu), .ev_btb (ev_btb),
    .rt_valid (rt_valid), .rt_last (rt_last), .rt_pc (rt_pc), .rt_insn (rt_insn),
    .rt_gpr_we (rt_gpr_we), .rt_gpr (rt_gpr), .rt_gpr_sh (rt_gpr_sh), .rt_irq (rt_irq), .rt_gpr_val (rt_gpr_val),
    .rt_cr (rt_cr), .rt_xer (rt_xer), .rt_lr (rt_lr), .rt_ctr (rt_ctr), .rt_msr (rt_msr),
    .rt_srr0 (rt_srr0), .rt_srr1 (rt_srr1), .rt_fpscr (rt_fpscr), .rt_fpq (rt_fpq),
    .rt_fw_en (rt_fw_en), .rt_fw_idx (rt_fw_idx), .rt_fw_val (rt_fw_val), .rt_dly (rt_dly)
  );
  assign t_dreq  = d_req;
  assign t_dwe   = d_we;
  assign t_daddr = d_addr;
  assign t_dwimg = d_wimg;
  assign t_dop   = d_op;
  assign t_dvalid = d_valid;
  assign t_drdata = d_rdata;
  assign t_dbe    = d_be;
  assign t_dwdata = d_wdata;
  assign t_drsv   = d_resv;
  assign t_dcnd   = d_cond;

  // ------------------------------------------------------------ the caches
  logic        im_req, im_ack, im_rvalid;
  logic [1:0]  im_cmd;
  logic [31:0] im_addr;
  logic [3:0]  im_be;
  logic        dm_req, dm_ack, dm_wnext, dm_rvalid;
  logic [1:0]  dm_cmd;
  logic [31:0] dm_addr, dm_wdata;
  logic [3:0]  dm_be;

  ppc_icache #(.PRE (BTB), .FETCH2 (FETCH2), .ISB (ISB)) u_ic (
    .clk (clk), .rst (rst),
    .i_req (i_req), .i_addr (i_addr), .i_seq (i_seq), .i_wimg (i_wimg), .i_valid (i_valid), .i_data (i_data),
    .i_way (i_way), .i_arr (i_arr), .i_pa (i_pa), .i_pv (i_pv), .i_pidx (i_pidx), .i_pway (i_pway), .i_ppa (i_ppa),
    .i_pre (i_pre), .i_eix (i_eix), .i_two (i_two), .i_w2 (i_w2), .i_data2 (i_data2),
    .hid_ice (hid_ice), .hid_ilock (hid_ilock), .hid_icfi (hid_icfi), .icfi_ack (),
    .icbi_req (icbi_req), .icbi_set (icbi_set), .icbi_ack (icbi_ack), .i_idle (),
    .m_req (im_req), .m_cmd (im_cmd), .m_addr (im_addr), .m_be (im_be),
    .m_ack (im_ack), .m_rvalid (im_rvalid), .m_rdata (m_rdata),
    .t_alloc (), .t_alloc_set (), .t_alloc_way (), .t_alloc_tag ()
  );

  ppc_dcache #(.SNOOP (SNOOP), .EARLY (LU2)) u_dc (
    .clk (clk), .rst (rst),
    .d_req (d_req), .d_we (d_we), .d_addr (d_addr), .d_be (d_be), .d_wdata (d_wdata), .d_wimg (d_wimg),
    .d_op (d_op), .d_rdy (d_rdy), .d_valid (d_valid), .d_rdata (d_rdata), .d_eword (d_eword), .d_eok (d_eok),
    .hid_dce (hid_dce), .hid_dlock (hid_dlock), .hid_dcfi (hid_dcfi), .hid_noopti (hid_noopti),
    .dcfi_ack (), .d_idle (d_idle),
    .d_rsv (d_resv), .res_kill (res_kill), .sn_req (sn_req), .sn_addr (sn_addr), .sn_inv (sn_inv), .sn_ack (sn_ack),
    .m_req (dm_req), .m_cmd (dm_cmd), .m_addr (dm_addr), .m_be (dm_be), .m_wdata (dm_wdata),
    .m_ack (dm_ack), .m_wnext (dm_wnext), .m_rvalid (dm_rvalid), .m_rdata (m_rdata),
    .t_alloc (), .t_alloc_set (), .t_alloc_way (), .t_alloc_tag ()
  );

  // ------------------------------------------------------------ the arbiter
  // own: a transaction is the owner's from its grant to its completion;
  // od: the owner is the data cache. acked: its acknowledge has come.
  logic        own, od, acked;
  logic [2:0]  beats;                           // beats still to come after the acknowledge
  wire         sel_d = own ? od : dm_req;       // the port shows the owner, else the data cache's ask
  wire         ask   = own ? !acked : (dm_req || im_req);
  assign m_req   = ask && (sel_d ? dm_req : im_req);
  assign m_cmd   = sel_d ? dm_cmd  : im_cmd;
  assign m_addr  = sel_d ? dm_addr : im_addr;
  assign m_be    = sel_d ? dm_be   : im_be;
  assign m_wdata = dm_wdata;                    // only the data cache writes
  assign dm_ack    = m_ack && sel_d;
  assign im_ack    = m_ack && !sel_d;
  assign dm_rvalid = m_rvalid && own && od;
  assign im_rvalid = m_rvalid && own && !od;
  assign dm_wnext  = m_wnext && own && od;

  wire one_beat = m_cmd == MC_RD;
  always_ff @(posedge clk) begin
    if (rst) begin
      own <= 1'b0; acked <= 1'b0;
    end else if (!own) begin
      if (m_req && m_ack) begin                 // acknowledged in its first cycle
        own <= m_cmd != MC_WR; od <= sel_d; acked <= 1'b1;
        beats <= one_beat ? 3'd0 : 3'd7;
      end else if (m_req) begin
        own <= 1'b1; od <= sel_d; acked <= 1'b0;
      end
    end else if (!acked) begin
      if (m_ack) begin
        acked <= 1'b1;
        beats <= one_beat ? 3'd0 : 3'd7;
        if (m_cmd == MC_WR) own <= 1'b0;        // a single write is done at its acknowledge
      end
    end else if (m_rvalid || m_wnext) begin
      if (beats == 3'd0) own <= 1'b0;
      else beats <= beats - 3'd1;
    end
  end

endmodule
