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
// The Model 3 Step 1.0 system, study D-37 stages A and B: everything below
// the MiSTer framework, so the Verilator bench runs this module whole (as the
// viewer's m3v_core). One 100 MHz clock (D-3, D-28).
//
//   ppc_cpu (603e, caches) --+
//   m3_scripts (53C810 DMA) -+-> m3_bus --> m3_sdr_port --> m2_sdram port 0
//                                     |--> m3_scripts registers, PCI config
//                                     |--> m3_io (315-5649, 93C46), m3_jtag
//                                     |--> m3_tilegen registers and palette
//                                     `--> r3d_cmd (the Real3D's status and windows)
//   r3d_cmd -> r3d_mem (8C, 8E: SDRAM port 4; polygon RAM: DDR3, two copies)
//           -> r3d_texup (texture FIFO, VROM port: SDRAM port 5, DDR3)
//           -> r3d_geo (culling RAM: port 2, VROM: port 3, polygon RAM: DDR3)
//              -> r3d_rast (MIP; store, layer, texture cache: DDR3)
//   m3_tilegen (VRAM in SDRAM, port 1) + m3_compose (the layer from DDR3) -> video
//   m3_loader: the MRA's stream -> SDRAM (download port)
//
// DDR3 (64-bit words from DDR_BASE; sim/system/notes.md, Stage B): layer
// buffers 0-3 at 0x20000 k, polygon RAM copies at 0x100000 and 0x180000, the
// texture ring at 0x200000, the rasteriser's store at 0x400000, the texture
// sheets at 0xC00000. Nothing touches DDR3 before ddr_ok (R71: hps_io's first
// command and RESET low for 100 ms): the arbiter's request is gated on it.
//
// SDRAM layout (bytes; study 7.3, sim/system/notes.md, the MRA):
//   0x0000000  banked CROM, 64 MiB (CROM0-CROM3, 8 MiB a bank)
//   0x4000000  VROM, 32 MiB (stage B; four 8 MiB blocks of four ROMs)
//   0x6000000  fixed CROM, 2 MiB (FFE00000-FFFFFFFF)
//   0x6200000  backup RAM, 128 KiB (the MRA fills it with FF)
//   0x6400000  work RAM, 8 MiB
//   0x6C00000  tile VRAM, 1 MiB
//   0x6D00000  culling RAM 8C, 4 MiB
//   0x7100000  8E, two copies of 1 MiB (r3d_mem)
//
// Reset: the memories come out of reset on PLL lock and stay out
// (mem_rst_n); the game (CPU, bus, devices) is held in reset until the ROMs
// are in (rom_loaded) and during any download, OSD reset or RESET.

module m3_system
  import ppc_cache_pkg::*;
#(
  parameter int          COL_BITS = 10,
  parameter bit          DUAL     = 1'b1,        // the 128 MB module
  parameter logic [4:0]  CROM_BANKS = 5'd8,
  parameter int          EE_TWP   = 0,           // the EEPROM's write time, clocks (0: 2 ms at CLK_HZ)
  parameter int          MIDI_GAP = 0,           // a MIDI byte, clocks (0: 320 us at CLK_HZ)
  parameter bit          OVERLAY  = 1'b1,
  // CPU performance on the board (stage A): m3_cpu_mon's counters, latched and
  // reported once a second on the debug UART ("P nn value", tools/uart_capture.py)
  // and shown on the overlay with CPI x 256. 0 drops it (the release).
  parameter bit          CPI_MON  = 1'b1,
  parameter int          CLK_HZ   = 100_000_000,
  // D-41: the 603e revision the game reads (Step 1.0: rev 1.3, as MAME). VF3's
  // boot code and TLB-miss handler set W in every BAT and RPA on revisions below
  // 1.3; at 1.3 RAM is copy-back (D-14: per step)
  parameter logic [31:0] PVR      = 32'h0006_0101,  // 0x00060103 once the D-cache snoops the 53C810 (D-41)
  parameter logic [28:0] DDR_BASE = 29'h0600_0000   // byte 0x30000000, the cores' DDR3 (Model 2 R654)
) (
  input  logic        clk,
  input  logic        mem_rst_n,
  input  logic        reset,         // RESET | OSD reset | buttons

  input  logic        ioctl_download,
  input  logic [15:0] ioctl_index,
  input  logic        ioctl_wr,
  input  logic [26:0] ioctl_addr,
  input  logic [15:0] ioctl_dout,
  output logic        ioctl_wait,

  input  logic [2:0]  rd_lat_sel,
  output logic        sd_cke, sd_cs_n, sd_ras_n, sd_cas_n, sd_we_n,
  output logic [1:0]  sd_ba,
  output logic [12:0] sd_a,
  output logic [1:0]  sd_dqm,
  output logic [15:0] sd_dq_o,
  output logic        sd_dq_oe,
  input  logic [15:0] sd_dq_i,

  // DDR3 (the framework's DDRAM port), and the gate on it (R71)
  input  logic        ddr_ok,
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

  output logic        ce_pix,
  output logic        v_hs, v_vs, v_hblank, v_vblank,
  output logic [7:0]  v_r, v_g, v_b,
  input  logic        overlay_en,

  output logic        led,
  output logic        uart_tx,
  input  logic        tb_fast,       // bench only (0 on the board): the timebase at the clock rate

  // bench
  output logic        rom_loaded,
  output logic [31:0] frame,
  output logic        tr_v,
  output logic        tr_master,
  output logic        tr_we,
  output logic [31:0] tr_addr,
  output logic [3:0]  tr_be,
  output logic [31:0] tr_data,
  output logic        ev_v,          // events: 1 IRQ set (bits), 2 ping-pong flip, 3 frame start
  output logic [1:0]  ev_kind,
  output logic [7:0]  ev_data,
  output logic        rt_valid_o,
  output logic [31:0] rt_pc_o,
  output logic [31:0] rt_insn_o,
  output logic        rt_gpr_we_o,
  output logic [4:0]  rt_gpr_o,
  output logic        rt_gpr_sh_o,
  output logic [31:0] rt_gpr_val_o
);

  localparam int AW = (DUAL ? 1 : 0) + 2 + 13 + COL_BITS;
  // everything that counts real time follows CLK_HZ (100 MHz; 90 MHz for the
  // diagnostic build): the pixel enable (16 MHz), the timebase (16.5 MHz), the
  // SDRAM refresh (7.7 us), the EEPROM's write time, a MIDI byte, the UART
  localparam int CLK_MHZ = CLK_HZ / 1_000_000;
  localparam int EE_TWP_C   = (EE_TWP != 0)   ? EE_TWP   : CLK_HZ / 500;
  localparam int MIDI_GAP_C = (MIDI_GAP != 0) ? MIDI_GAP : CLK_HZ / 3125;
  localparam int T_REFI_C   = CLK_MHZ * 77 / 10;
  localparam logic [26:0] SDR_CROM = 27'h000_0000, SDR_FIXED = 27'h600_0000, SDR_BACKUP = 27'h620_0000,
                          SDR_RAM = 27'h640_0000, SDR_VRAM = 27'h6C0_0000;

  // ===========================================================================
  // reset
  // ===========================================================================
  logic loader_busy;
  logic [2:0] rst_s = 3'b111;
  always_ff @(posedge clk) rst_s <= {rst_s[1:0], reset || !mem_rst_n || !rom_loaded || loader_busy};
  wire rst = rst_s[2];
  logic por_n;                       // power-on: the EEPROM and the I/O latches
  always_ff @(posedge clk) por_n <= mem_rst_n;

  // ===========================================================================
  // SDRAM: port 0 the bus, 1 the tile generator, 2 the geometry's culling
  // reads, 3 its VROM reads, 4 r3d_mem, 5 r3d_texup; the loader on wr_*
  // ===========================================================================
  localparam int NSP = 6;
  logic          mem_ready;
  logic          ld_req, ld_ack;
  logic [AW:1]   ld_addr;
  logic [15:0]   ld_din;
  logic [1:0]    ld_be;
  logic [NSP-1:0]          p_req, p_we, p_long, p_lo, p_ack;
  logic [NSP-1:0][AW:1]    p_addr;
  logic [NSP-1:0][15:0]    p_din;
  logic [NSP-1:0][1:0]     p_be;
  logic [NSP-1:0][63:0]    p_dout;

  m2_sdram #(.COL_BITS(COL_BITS), .NP(NSP), .T_REFI(T_REFI_C), .ACK_HOLD(1), .DUAL(DUAL), .MASK_ON_A(DUAL)) u_sdram (
    .clk, .rst_n(mem_rst_n), .ready(mem_ready), .rd_lat_sel,
    .sd_cke, .sd_cs_n, .sd_ras_n, .sd_cas_n, .sd_we_n, .sd_ba, .sd_a, .sd_dqm, .sd_dq_o, .sd_dq_oe, .sd_dq_i,
    .wr_req(ld_req), .wr_addr(ld_addr), .wr_din(ld_din), .wr_be(ld_be), .wr_ack(ld_ack),
    .p_req, .p_we, .p_addr, .p_din, .p_be, .p_dout, .p_ack, .p_long, .p_lo,
    .dbg_req(), .dbg_grant());

  logic [26:0] ld_bytes;
  logic        ld_ovf;
  m3_loader #(.AW(AW)) u_ld (
    .clk, .rst(!mem_rst_n), .mem_ready,
    .ioctl_download, .ioctl_index, .ioctl_wr, .ioctl_addr, .ioctl_dout, .ioctl_wait,
    .sdr_wr_req(ld_req), .sdr_wr_addr(ld_addr), .sdr_wr_din(ld_din), .sdr_wr_be(ld_be), .sdr_wr_ack(ld_ack),
    .rom_loaded, .busy(loader_busy), .overflow(ld_ovf), .bytes(ld_bytes));

  // ===========================================================================
  // the CPU
  // ===========================================================================
  // the timebase and decrementer count at the 66 MHz bus / 4 (D-14): 16.5 MHz
  logic [7:0] tb_acc;
  logic       tb_tick;
  always_ff @(posedge clk) begin
    if (rst) begin tb_acc <= '0; tb_tick <= 1'b0; end
    else if (tb_fast) tb_tick <= 1'b1;
    else if (tb_acc + 8'd33 >= 8'(2 * CLK_MHZ)) begin tb_acc <= tb_acc + 8'd33 - 8'(2 * CLK_MHZ); tb_tick <= 1'b1; end
    else begin tb_acc <= tb_acc + 8'd33; tb_tick <= 1'b0; end
  end

  logic [1:0]       m_req, m_ack, m_wnext, m_rvalid;
  logic [1:0][1:0]  m_cmd;
  logic [1:0][31:0] m_addr, m_wdata;
  logic [1:0][3:0]  m_be;
  logic [31:0]      m_rdata;
  logic             ext_int;
  logic             rt_valid, rt_gpr_we, rt_gpr_sh;
  logic [4:0]       rt_gpr;
  logic [31:0]      rt_gpr_val;
  logic [7:0]       cpu_ev;
  logic [5:0]       cpu_ev_mmu;
  logic [1:0]       rt_irq;
  logic [31:0]      rt_lr, rt_msr, rt_srr0, rt_srr1;
  logic [31:0]      rt_pc;

  ppc_cpu #(.PVR(PVR)) u_cpu (
    .clk, .rst,
    .m_req(m_req[0]), .m_cmd(m_cmd[0]), .m_addr(m_addr[0]), .m_be(m_be[0]), .m_wdata(m_wdata[0]),
    .m_ack(m_ack[0]), .m_wnext(m_wnext[0]), .m_rvalid(m_rvalid[0]), .m_rdata,
    // the D-cache snoop (D-41): tied off until the 53C810's RAM writes drive it
    .sn_req(1'b0), .sn_addr(27'd0), .sn_inv(1'b0), .sn_ack(),
    .tb_tick, .ext_int, .ev(cpu_ev), .ev_mmu(cpu_ev_mmu), .ev_btb(),
    .rt_valid, .rt_last(), .rt_pc, .rt_insn(rt_insn_o), .rt_gpr_we, .rt_gpr, .rt_gpr_sh,
    .rt_irq(rt_irq), .rt_gpr_val,
    .rt_cr(), .rt_xer(), .rt_lr(rt_lr), .rt_ctr(), .rt_msr(rt_msr), .rt_srr0(rt_srr0), .rt_srr1(rt_srr1), .rt_fpscr(), .rt_fpq(),
    .rt_fw_en(), .rt_fw_idx(), .rt_fw_val(), .rt_dly(),
    .t_dreq(), .t_dwe(), .t_daddr(), .t_dwimg(), .t_dop(), .t_dvalid(), .t_drdata(),
    .t_dbe(), .t_dwdata(), .t_drsv(), .t_dcnd());
  assign rt_valid_o = rt_valid;
  assign rt_gpr_we_o = rt_gpr_we; assign rt_gpr_o = rt_gpr; assign rt_gpr_sh_o = rt_gpr_sh; assign rt_gpr_val_o = rt_gpr_val;

  // ---- CPU performance: m3_cpu_mon, a report a second ----
  //   00 cycles  01 instructions  02 uops  03 decode starved  04 load-use
  //   05 mul/div wait  06 data memory wait  07 redirects  08 fetch waiting
  logic [8:0][31:0] perf;
  logic [15:0]      cpi_q8;          // CPI x 256 of the last report
  generate if (CPI_MON) begin : g_mon
    logic [31:0] sec;
    logic        rep, clr;
    always_ff @(posedge clk) begin
      rep <= 1'b0; clr <= rep;
      if (rst) sec <= '0;
      else if (sec == 32'(CLK_HZ - 1)) begin sec <= '0; rep <= 1'b1; end
      else sec <= sec + 32'd1;
    end
    m3_cpu_mon #(.DIVISOR((CLK_HZ + 57_600) / 115_200), .SIG(1'b0)) u_mon (
      .clk, .rst, .clear(clr), .report(rep), .ev(cpu_ev),
      .rt_valid, .rt_gpr_we, .rt_gpr, .rt_gpr_val, .busy(), .tx(uart_tx), .counts(perf));
    // CPI x 256 = (cycles << 8) / instructions, a bit a clock after each report
    logic [39:0] num;
    logic [31:0] den;
    logic [40:0] rem;
    logic [5:0]  k;
    logic [23:0] quo;
    logic        div_go, div_run;
    wire  [40:0] r2 = {rem[39:0], num[39]};
    always_ff @(posedge clk) begin
      div_go <= clr;
      if (rst) begin div_run <= 1'b0; cpi_q8 <= '0; end
      else if (div_go) begin
        num <= {perf[0], 8'd0}; den <= perf[1]; rem <= '0; quo <= '0; k <= 6'd40; div_run <= 1'b1;
      end else if (div_run) begin
        num <= {num[38:0], 1'b0};
        if (r2 >= {9'd0, den}) begin rem <= r2 - {9'd0, den}; quo <= {quo[22:0], 1'b1}; end
        else begin rem <= r2; quo <= {quo[22:0], 1'b0}; end
        k <= k - 6'd1;
        if (k == 6'd1) begin
          div_run <= 1'b0;
          cpi_q8 <= (den == 32'd0) ? 16'hFFFF : (quo[22:15] != 8'd0) ? 16'hFFFF : {quo[14:0], (r2 >= {9'd0, den})};
        end
      end
    end
  end else begin : g_nomon
    assign perf = '0; assign cpi_q8 = '0; assign uart_tx = 1'b1;
  end endgenerate
  assign rt_pc_o = rt_pc;

  // ===========================================================================
  // the 53C810
  // ===========================================================================
  logic        sc_req, sc_we, sc_ack, sc_win_en, cf_req, cf_we, cf_ack, scsi_irq, scsi_busy;
  logic [6:2]  sc_addr;
  logic [3:0]  sc_be, cf_be;
  logic [31:0] sc_wdata, sc_rdata, cf_wdata, cf_rdata;
  logic [31:8] sc_win_base;
  logic [5:0]  cf_idx;

  m3_scripts u_scsi (
    .clk, .rst,
    .c_req(cf_req), .c_we(cf_we), .c_idx(cf_idx), .c_be(cf_be), .c_wdata(cf_wdata), .c_ack(cf_ack), .c_rdata(cf_rdata),
    .win_en(sc_win_en), .win_base(sc_win_base),
    .s_req(sc_req), .s_we(sc_we), .s_addr(sc_addr), .s_be(sc_be), .s_wdata(sc_wdata), .s_ack(sc_ack), .s_rdata(sc_rdata),
    .m_req(m_req[1]), .m_cmd(m_cmd[1]), .m_addr(m_addr[1]), .m_be(m_be[1]), .m_wdata(m_wdata[1]),
    .m_ack(m_ack[1]), .m_wnext(m_wnext[1]), .m_rvalid(m_rvalid[1]), .m_rdata,
    .irq(scsi_irq), .busy(scsi_busy));

  // ===========================================================================
  // the bus and its devices
  // ===========================================================================
  logic        s_req, s_swap, s_zero, s_ack, s_wnext, s_rvalid, s_idle;
  logic [1:0]  s_cmd;
  logic [AW:0] s_sa;
  logic [3:0]  s_be;
  logic [31:0] s_wdata, s_rdata;
  logic        io_sel, io_we;
  logic [3:0]  io_off;
  logic [7:0]  io_wbyte, io_rbyte, io_pa;
  logic        tg_reg_we, tg_pal_we;
  logic [5:0]  tg_reg_addr;
  logic [31:0] tg_reg_wdata;
  logic [14:0] tg_pal_addr;
  logic [15:0] tg_pal_wdata;
  logic        ln_start;
  logic [9:0]  ln;
  logic [7:0]  irq_state, irq_enable, crom_bank;
  logic [23:0] n_submit, n_cull_lo, n_cull_hi, n_poly, n_tex, n_vrom;
  logic [15:0] n_unmapped;
  logic [31:0] last_unmapped;
  logic        pingpong;
  logic [18:0] r3d_cfg0, r3d_cfg1, r3d_cfg2, jt_cfg0, jt_cfg1, jt_cfg2;
  logic        jt_wr, jt_busy, jt_tdo, jt_cfg_we;
  logic [7:0]  jt_wdata;
  logic        h_go, h_rdy, h_we, h_rv, r3_pingpong, r3_flip;
  logic [31:0] h_addr, h_wdata, h_rdata;
  logic        mw_we;
  logic [3:0]  mw_dev;
  logic [31:0] mw_val;

  m3_bus #(.AW(AW), .SDR_CROM(SDR_CROM), .SDR_FIXED(SDR_FIXED), .SDR_BACKUP(SDR_BACKUP), .SDR_RAM(SDR_RAM),
           .SDR_VRAM(SDR_VRAM), .CROM_BANKS(CROM_BANKS), .MIDI_GAP(MIDI_GAP_C)) u_bus (
    .clk, .rst,
    .m_req, .m_cmd, .m_addr, .m_be, .m_wdata, .m_ack, .m_wnext, .m_rvalid, .m_rdata,
    .s_req, .s_cmd, .s_sa, .s_swap, .s_zero, .s_be, .s_wdata, .s_ack, .s_wnext, .s_rvalid, .s_rdata,
    .sc_req, .sc_we, .sc_addr, .sc_be, .sc_wdata, .sc_ack, .sc_rdata, .sc_win_en, .sc_win_base,
    .cf_req, .cf_we, .cf_idx, .cf_be, .cf_wdata, .cf_ack, .cf_rdata, .scsi_irq,
    .io_sel, .io_we, .io_off, .io_wbyte, .io_rbyte,
    .tg_reg_we, .tg_reg_addr, .tg_reg_wdata, .tg_pal_we, .tg_pal_addr, .tg_pal_wdata,
    .ln_start, .ln, .ext_int,
    .irq_state_o(irq_state), .irq_enable_o(irq_enable), .crom_bank_o(crom_bank),
    .n_submit, .n_cull_lo, .n_cull_hi, .n_poly, .n_tex, .n_vrom, .n_unmapped, .last_unmapped,
    .pingpong_o(pingpong), .pingpong_i(r3_pingpong), .h_go, .h_rdy, .h_we, .h_addr, .h_wdata, .h_rv, .h_rdata,
    .r3d_cfg0, .r3d_cfg1, .r3d_cfg2,
    .jt_wr, .jt_wdata, .jt_busy, .jt_tdo, .jt_cfg_we, .jt_cfg0, .jt_cfg1, .jt_cfg2,
    .tr_v, .tr_master, .tr_we, .tr_addr, .tr_be, .tr_data);

  m3_sdr_port #(.AW(AW)) u_sport (
    .clk, .rst(!mem_rst_n),
    .s_req, .s_cmd, .s_sa, .s_swap, .s_zero, .s_be, .s_wdata, .s_ack, .s_wnext, .s_rvalid, .s_rdata, .s_idle,
    .p_req(p_req[0]), .p_we(p_we[0]), .p_addr(p_addr[0]), .p_din(p_din[0]), .p_be(p_be[0]), .p_long(p_long[0]),
    .p_dout(p_dout[0]), .p_lo(p_lo[0]), .p_ack(p_ack[0]));

  m3_io #(.EE_TWP(EE_TWP_C)) u_io (
    .clk, .por_n, .rst_n(!rst), .sel(io_sel), .we(io_we), .off(io_off), .wbyte(io_wbyte), .rbyte(io_rbyte),
    .in0(8'hFF), .test_b_n(1'b1), .service_b_n(1'b1), .in2(8'hFF), .in3(8'hFF), .pa_q(io_pa));

  m3_jtag u_jtag (
    .clk, .rst, .wr(jt_wr), .wdata(jt_wdata), .busy(jt_busy), .tdo(jt_tdo),
    .cfg0(r3d_cfg0), .cfg1(r3d_cfg1), .cfg2(r3d_cfg2),
    .cfg_we(jt_cfg_we), .cfg0_w(jt_cfg0), .cfg1_w(jt_cfg1), .cfg2_w(jt_cfg2),
    .mw_we, .mw_dev, .mw_val);

  // ===========================================================================
  // the Real3D (D-37 stage B; sim/system/notes.md, Stage B)
  // ===========================================================================
  localparam logic [24:0] D_POLY0 = 25'h10_0000, D_POLY1 = 25'h18_0000, D_STG = 25'h20_0000,
                          D_STORE = 25'h40_0000, D_TEX = 25'hC0_0000;
  localparam logic [26:0] SDR_8C = 27'h6D0_0000, SDR_8E0 = 27'h710_0000, SDR_8E1 = 27'h720_0000, SDR_VROM = 27'h400_0000;

  // ---- DDR3: m3_ddr3p (the texture cache on its pipelined port) behind
  // m3_ddr_arb: 0 the layer's scan-out, 1 the store, 2 the layer, 3 the
  // geometry's polygon reads, 4 r3d_mem, 5 r3d_texup
  localparam int NDP = 6;
  logic [NDP-1:0]       d_req, d_we, d_wacc, d_rvalid, d_ack;
  logic [NDP-1:0][24:0] d_addr;
  logic [NDP-1:0][7:0]  d_blen, d_be;
  logic [NDP-1:0][63:0] d_din;
  logic        dm_req, dm_we, dm_wacc, dm_rvalid, dm_ack;
  logic [24:0] dm_addr;
  logic [7:0]  dm_blen, dm_be;
  logic [63:0] dm_din, dm_dout;
  logic        tx_req, tx_ack, tx_rvalid;
  logic [24:0] tx_addr;
  m3_ddr_arb #(.NP(NDP)) u_darb (
    .clk, .rst(!mem_rst_n), .req(d_req), .we(d_we), .addr(d_addr), .blen(d_blen), .din(d_din), .be(d_be),
    .wacc(d_wacc), .rvalid(d_rvalid), .ack(d_ack),
    .m_req(dm_req), .m_we(dm_we), .m_addr(dm_addr), .m_blen(dm_blen), .m_din(dm_din), .m_be(dm_be),
    .m_wacc(dm_wacc), .m_rvalid(dm_rvalid), .m_ack(dm_ack));
  m3_ddr3p #(.PIPE(1'b1), .NP(1)) u_ddr3 (
    .clk, .rst_n(mem_rst_n), .base(DDR_BASE),
    .req(dm_req && ddr_ok), .we(dm_we), .addr(dm_addr), .blen(dm_blen), .din(dm_din), .be(dm_be),
    .wnext(), .wacc(dm_wacc), .rvalid(dm_rvalid), .ack(dm_ack),
    .p_req(tx_req && ddr_ok), .p_addr(tx_addr), .p_blen(8'd4), .p_we(1'b0), .p_din('0),
    .p_ack(tx_ack), .p_rvalid(tx_rvalid), .p_wacc(),
    .dout(dm_dout),
    .DDRAM_CLK, .DDRAM_BUSY, .DDRAM_BURSTCNT, .DDRAM_ADDR, .DDRAM_DIN, .DDRAM_BE,
    .DDRAM_WE, .DDRAM_RD, .DDRAM_DOUT, .DDRAM_DOUT_READY,
    .dbg_lat_v(), .dbg_lat(), .dbg_lat_src(), .dbg_lat_max(), .dbg_acc(), .dbg_acc_max(),
    .dbg_take(), .dbg_take_src(), .dbg_done(), .dbg_done_src(), .dbg_idle(), .dbg_st(), .dbg_rdy(), .dbg_rq());

  // ---- command, status, the frame sequence
  logic        m3_valid, m3_ready, m3_can_swap, m3_rbusy;
  logic        m3_swap /* verilator public_flat_rd */, m3_bank /* verilator public_flat_rd */;
  logic [1:0]  m3_reg;
  logic [19:0] m3_addr;
  logic [31:0] m3_data;
  logic        t3_valid, t3_ready, t3_vrom, t3_flush, t3_pend, t3_mark, t3_mpend, t3_go, t3_busy;
  logic [31:0] t3_data;
  logic        g_start, g_busy, f_ready, f_begin, r_busy, show_3d;
  logic        f_end /* verilator public_flat_rd */, r_done /* verilator public_flat_rd */;
  logic [1:0]  f_buf /* verilator public_flat_rd */, show_buf;
  logic [15:0] n_draw, n_drop, n_runs, n_copy, n_tex_up, n_tex_skip;
  r3d_cmd u_r3cmd (
    .clk, .rst, .h_go, .h_rdy, .h_we, .h_addr, .h_wdata, .h_rv, .h_rdata,
    .tg_reg_we, .tg_reg_addr, .tg_reg_wdata, .ln_start, .ln, .mw_we, .mw_dev, .mw_val,
    .m_valid(m3_valid), .m_ready(m3_ready), .m_reg(m3_reg), .m_addr(m3_addr), .m_data(m3_data),
    .m_swap(m3_swap), .m_can_swap(m3_can_swap), .m_rbusy(m3_rbusy),
    .t_valid(t3_valid), .t_ready(t3_ready), .t_vrom(t3_vrom), .t_data(t3_data), .t_flush(t3_flush), .t_pend(t3_pend),
    .t_mark(t3_mark), .t_mpend(t3_mpend), .t_go(t3_go), .t_busy(t3_busy),
    .g_start, .g_busy, .f_ready, .f_begin, .f_end, .f_buf, .r_done, .r_busy,
    .show_buf, .show_3d, .pingpong(r3_pingpong), .flip(r3_flip), .n_draw, .n_drop);

  // ---- culling and polygon RAM windows
  r3d_mem #(.AW(AW), .SDR_8C(SDR_8C), .SDR_8E0(SDR_8E0), .SDR_8E1(SDR_8E1), .DDR_P0(D_POLY0), .DDR_P1(D_POLY1)) u_r3mem (
    .clk, .rst, .w_valid(m3_valid), .w_ready(m3_ready), .w_reg(m3_reg), .w_addr(m3_addr), .w_data(m3_data),
    .swap(m3_swap), .r_busy(m3_rbusy), .bank(m3_bank), .can_swap(m3_can_swap),
    .s_req(p_req[4]), .s_we(p_we[4]), .s_addr(p_addr[4]), .s_din(p_din[4]), .s_be(p_be[4]), .s_ack(p_ack[4]), .s_dout(p_dout[4]),
    .d_req(d_req[4]), .d_we(d_we[4]), .d_addr(d_addr[4]), .d_blen(d_blen[4]), .d_din(d_din[4]), .d_be(d_be[4]),
    .d_wacc(d_wacc[4]), .d_rvalid(d_rvalid[4]), .d_ack(d_ack[4]), .d_dout(dm_dout),
    .n_runs, .n_copy);
  assign p_long[4] = 1'b0;

  // ---- textures
  r3d_texup #(.AW(AW), .SDR_VROM(SDR_VROM), .DDR_STG(D_STG), .DDR_TEX(D_TEX)) u_r3tex (
    .clk, .rst, .t_valid(t3_valid), .t_ready(t3_ready), .t_vrom(t3_vrom), .t_data(t3_data), .t_flush(t3_flush),
    .t_pend(t3_pend), .mark(t3_mark), .t_mpend(t3_mpend), .go(t3_go), .busy(t3_busy),
    .s_req(p_req[5]), .s_addr(p_addr[5]), .s_ack(p_ack[5]), .s_dout(p_dout[5]),
    .d_req(d_req[5]), .d_we(d_we[5]), .d_addr(d_addr[5]), .d_blen(d_blen[5]), .d_din(d_din[5]), .d_be(d_be[5]),
    .d_wacc(d_wacc[5]), .d_rvalid(d_rvalid[5]), .d_ack(d_ack[5]), .d_dout(dm_dout),
    .n_tex(n_tex_up), .n_skip(n_tex_skip));
  assign p_we[5] = 1'b0; assign p_din[5] = '0; assign p_be[5] = '0; assign p_long[5] = 1'b0;

  // ---- the geometry (set-up on its pool, the LOD word: R76, R75)
  logic        geo_busy, cr_req, cr_ack, cr_rv, pr_req, pr_ack, pr_rv, vr_req, vr_ack, vr_rv;
  logic [7:0]  geo_err;
  logic [23:0] cr_addr, pr_addr, vr_addr;
  logic [2:0]  cr_len, pr_len, vr_len;
  logic [31:0] cr_rd, pr_rd, vr_rd;
  logic        gq_valid, gq_ready, gq_last;
  logic [31:0] gq_data;
  r3d_geo #(.SETUP_OUT(1'b1), .LOD_WORD(1'b1)) u_geo (
    .clk, .rst, .start(g_start), .step15(1'b0), .busy(geo_busy), .err(geo_err),
    .cull_req(cr_req), .cull_addr(cr_addr), .cull_len(cr_len), .cull_ack(cr_ack), .cull_rvalid(cr_rv), .cull_rdata(cr_rd),
    .pram_req(pr_req), .pram_addr(pr_addr), .pram_len(pr_len), .pram_ack(pr_ack), .pram_rvalid(pr_rv), .pram_rdata(pr_rd),
    .vrom_req(vr_req), .vrom_addr(vr_addr), .vrom_len(vr_len), .vrom_ack(vr_ack), .vrom_rvalid(vr_rv), .vrom_rdata(vr_rd),
    .out_valid(gq_valid), .out_ready(gq_ready), .out_data(gq_data), .out_last(gq_last));
  assign g_busy = geo_busy || gq_valid;

  // culling RAM: words 0-0xFFFFF are 8C, 0x800000-0x83FFFF 8E (bank R); else zero
  logic [23:0] cb_addr;
  logic [AW:1] cb_sdr;
  logic        cb_zero;
  always_comb begin
    cb_zero = 1'b0; cb_sdr = '0;
    if (cb_addr < 24'h10_0000) cb_sdr = AW'(SDR_8C[26:1]) + AW'({cb_addr[19:0], 1'b0});
    else if (cb_addr >= 24'h80_0000 && cb_addr < 24'h84_0000)
      cb_sdr = AW'((m3_bank ? SDR_8E1 : SDR_8E0) >> 1) + AW'({cb_addr[17:0], 1'b0});
    else cb_zero = 1'b1;
  end
  m3v_sdr_rd #(.AW(AW)) u_crd (
    .clk, .rst,
    .g_req(cr_req), .g_addr(cr_addr), .g_len(cr_len), .g_ack(cr_ack), .g_rvalid(cr_rv), .g_rdata(cr_rd),
    .blk_addr(cb_addr), .blk_sdr(cb_sdr), .blk_zero(cb_zero),
    .p_req(p_req[2]), .p_addr(p_addr[2]), .p_lo(p_lo[2]), .p_ack(p_ack[2]), .p_dout(p_dout[2]), .n_req());
  assign p_we[2] = 1'b0; assign p_din[2] = '0; assign p_be[2] = '0; assign p_long[2] = 1'b1;
  r3d_vrom_rd #(.AW(AW), .SDR_VROM(SDR_VROM)) u_vrd (
    .clk, .rst,
    .g_req(vr_req), .g_addr(vr_addr), .g_len(vr_len), .g_ack(vr_ack), .g_rvalid(vr_rv), .g_rdata(vr_rd),
    .p_req(p_req[3]), .p_addr(p_addr[3]), .p_ack(p_ack[3]), .p_dout(p_dout[3]), .n_req());
  assign p_we[3] = 1'b0; assign p_din[3] = '0; assign p_be[3] = '0; assign p_long[3] = 1'b0;
  m3v_ddr_rd u_prd (
    .clk, .rst, .base(m3_bank ? D_POLY1 : D_POLY0),
    .g_req(pr_req), .g_addr(pr_addr), .g_len(pr_len), .g_ack(pr_ack), .g_rvalid(pr_rv), .g_rdata(pr_rd),
    .m_req(d_req[3]), .m_addr(d_addr[3]), .m_blen(d_blen[3]), .m_rvalid(d_rvalid[3]), .m_ack(d_ack[3]), .m_dout(dm_dout),
    .n_req());
  assign d_we[3] = 1'b0; assign d_din[3] = '0; assign d_be[3] = 8'hFF;

  // ---- the rasteriser (nearest mip level, R79/R88); scroll fog off for now
  r3d_rast #(.MIP(1'b1)) u_rast (
    .clk, .rst,
    .i_valid(gq_valid), .i_ready(gq_ready), .i_data(gq_data), .i_last(gq_last),
    .f_ready, .f_binning(), .f_begin, .f_end, .fb_base(25'({f_buf, 17'd0})),
    .sf_x0(9'd0), .sf_x1(9'd0), .sf_y0(9'd0), .sf_y1(9'd0), .sf_col(24'd0), .sf_out(24'd0),
    .done(r_done), .busy(r_busy),
    .store_base(D_STORE), .tex_base(D_TEX),
    .s_req(d_req[1]), .s_we(d_we[1]), .s_addr(d_addr[1]), .s_blen(d_blen[1]), .s_din(d_din[1]), .s_be(d_be[1]),
    .s_wacc(d_wacc[1]), .s_rvalid(d_rvalid[1]), .s_ack(d_ack[1]), .s_dout(dm_dout),
    .t_req(tx_req), .t_addr(tx_addr), .t_ack(tx_ack), .t_rvalid(tx_rvalid), .t_dout(dm_dout),
    .l_req(d_req[2]), .l_addr(d_addr[2]), .l_blen(d_blen[2]), .l_din(d_din[2]), .l_wacc(d_wacc[2]), .l_ack(d_ack[2]),
    .n_pix(), .n_frames(), .f_cycles(), .f_eobw(), .st_rec(), .st_drop(), .st_run(), .st_pair(), .st_ovf(), .vp_ovf());
  assign d_we[2] = 1'b1; assign d_be[2] = 8'hFF;

  // ===========================================================================
  // video: timing, the tile generator, the overlay
  // ===========================================================================
  logic [6:0] ce_acc;
  always_ff @(posedge clk) begin
    if (!mem_rst_n) begin ce_acc <= '0; ce_pix <= 1'b0; end
    else if (ce_acc + 7'd16 >= 7'(CLK_MHZ)) begin ce_acc <= ce_acc + 7'd16 - 7'(CLK_MHZ); ce_pix <= 1'b1; end
    else begin ce_acc <= ce_acc + 7'd16; ce_pix <= 1'b0; end
  end

  logic [9:0] hcnt, vcnt;
  logic       hblank, vblank, hsync, vsync, vblank_start;
  m2_video_timing u_vt (
    .clk, .ce_pix, .rst_n(mem_rst_n), .interlace(1'b0), .ntsc_lines(1'b0), .mid_lines(1'b0), .narrow(1'b0),
    .hcnt, .vcnt, .hblank, .vblank, .hsync, .vsync, .visible(), .line_start(), .line_number(),
    .vblank_start, .field(), .ypos(), .vtotal());

  // line events for the bus: the start of each line (hcnt 0)
  always_ff @(posedge clk) begin
    ln_start <= ce_pix && hcnt == 10'd655;
    ln <= (vcnt == 10'd423) ? 10'd0 : vcnt + 10'd1;
  end

  // tile generator: at the start of the horizontal blank of line L, the line
  // built during L is shown next (swap), and line L + 2 is started (go)
  logic        tg_ready, tg_busy, tg_ovr, tg_swap, tg_go, tg_go_n;
  logic [8:0]  tg_go_y;
  logic        o_v, o_drawn, o_above;
  logic [7:0]  o_r, o_g, o_b;
  logic        t_req, t_ack;
  logic [AW:1] t_addr;
  logic [63:0] t_dout;
  wire  [9:0]  l1 = (vcnt == 10'd423) ? 10'd0 : vcnt + 10'd1;
  wire  [9:0]  l2 = (vcnt >= 10'd422) ? vcnt - 10'd422 : vcnt + 10'd2;
  always_ff @(posedge clk) begin
    tg_swap <= 1'b0; tg_go <= tg_go_n; tg_go_n <= 1'b0;
    if (ce_pix && hcnt == 10'd496) begin
      if (l1 < 10'd384) tg_swap <= 1'b1;
      if (l2 < 10'd384) begin tg_go_n <= 1'b1; tg_go_y <= l2[8:0]; end
    end
  end
  logic [15:0] tg_ovr_n;
  always_ff @(posedge clk) if (rst) tg_ovr_n <= '0; else if (tg_swap && tg_busy) tg_ovr_n <= tg_ovr_n + 16'd1;

  m3_tilegen #(.AW(AW), .VBASE(AW'(SDR_VRAM >> 1))) u_tg (
    .clk, .rst_n(!rst), .ready(tg_ready),
    .reg_we(tg_reg_we), .reg_addr(tg_reg_addr), .reg_wdata(tg_reg_wdata),
    .pal_we(tg_pal_we), .pal_addr(tg_pal_addr), .pal_wdata(tg_pal_wdata),
    .swap(tg_swap), .go(tg_go), .go_y(tg_go_y), .busy(tg_busy), .overrun(tg_ovr),
    .sc_rd(ce_pix && !hblank && !vblank), .sc_x(hcnt[8:0]),
    .o_v, .o_drawn, .o_above, .o_r, .o_g, .o_b,
    .p_req(t_req), .p_addr(t_addr), .p_dout(t_dout), .p_ack(t_ack),
    .st_cycles(), .st_reqs());
  assign p_req[1] = t_req; assign p_we[1] = 1'b0; assign p_addr[1] = t_addr; assign p_din[1] = '0;
  assign p_be[1] = '0; assign p_long[1] = 1'b0; assign t_ack = p_ack[1]; assign t_dout = p_dout[1];

  // ---- the frame count and the bench's events ----
  always_ff @(posedge clk) begin
    ev_v <= 1'b0;
    if (rst) frame <= 32'd1;           // Supermodel's numbering: frame 1 is the first
    else if (ln_start && ln == 10'd384) begin frame <= frame + 32'd1; ev_v <= 1'b1; ev_kind <= 2'd3; ev_data <= 8'd0; end
    else if (r3_flip) begin ev_v <= 1'b1; ev_kind <= 2'd2; ev_data <= 8'd0; end
  end

  // ---- the overlay (hex; rows of four words) ----
  logic [31:0] pc_v, pc_last;
  logic [15:0] n_scsi_irq;
  logic        scsi_irq_d;
  logic [31:0] cpu_wd_cnt, cpu_wd_max;  // the CPU's bus wait: cycles since its request
  logic        wd_flag;
  always_ff @(posedge clk) begin
    scsi_irq_d <= scsi_irq;
    if (rt_valid) pc_last <= rt_pc;
    if (vblank_start) pc_v <= pc_last;
    if (rst) begin n_scsi_irq <= '0; cpu_wd_cnt <= '0; cpu_wd_max <= '0; wd_flag <= 1'b0; end
    else begin
      if (scsi_irq && !scsi_irq_d) n_scsi_irq <= n_scsi_irq + 16'd1;
      if (m_req[0] && !m_ack[0]) cpu_wd_cnt <= cpu_wd_cnt + 32'd1; else cpu_wd_cnt <= '0;
      if (cpu_wd_cnt > cpu_wd_max) cpu_wd_max <= cpu_wd_cnt;
      if (cpu_wd_cnt == 32'd10_000_000) wd_flag <= 1'b1;
    end
  end

  // Exceptions, for the overlay (rows 8-11).
  // An exception entry is found from the retirement stream. The core retires
  // the instruction an exception replaces as a record of its own, and that
  // record already carries the new SRR0 / SRR1 (ppc_core tr_srr0), so the next
  // record is the vector's first instruction. (Build 5 compared each record's
  // SRR0 / SRR1 with the one before and asked for the change on the vector's
  // record: the change is on the record before, so it never fired. Row 8 was
  // zero on the board for that reason.)
  // The record is marked by the core's own taken-exception events: ev_mmu
  // [3:0] (an interrupt, DSI or ISI, ITLB miss, DTLB miss, in that record) and
  // rt_irq. The other kinds (program, alignment, sc, FP unavailable, trap)
  // have no exported strobe. For those the record is the one whose SRR0 or SRR1
  // differ from the previous record's, unless it is an mtspr to either.
  // A repeated TLB miss or DSI at one PC is still counted, because it comes
  // through ev_mmu. The vector is the PC of the record that follows.
  //   first slot (rows 8, 9): the first entry after reset
  //   last slots (rows 10, 11): the most recent entry and the one before it,
  //   frozen at the boot ROM's fatal handler (FFF00000). The fatal entry is then
  //   the most recent, and the one before it is in row 10.
  //   DAR / DSISR: the last values an mfspr read, frozen with them.
  logic [31:0] ex_vec, ex_srr0, ex_srr1, ex_msr, ex_pc, ex_ins;
  logic [31:0] lc_vec, lc_srr0, lc_srr1, lc_ins;     // the most recent entry
  logic [31:0] lp_vec, lp_srr0, lp_srr1, lp_ins;     // the one before it
  logic [31:0] xn_cnt, x_dar, x_dsisr;
  logic [31:0] fx_vec, fx_srr0, pc_prev;
  logic        ex_seen, fx_seen, x_arm;
  logic [31:0] x_srr0, x_srr1, x_msr, x_pc, x_ins;   // the armed record
  logic [31:0] rt_insn_q, srr0_prev, srr1_prev;
  assign rt_insn_q = rt_insn_o;
  // mtspr SRR0 / SRR1 (spr 26, 27): the split field is insn[20:16] = spr[4:0], insn[15:11] = spr[9:5]
  wire is_mtsrr = (rt_insn_q[31:26] == 6'd31) && (rt_insn_q[10:1] == 10'd467) &&
                  (rt_insn_q[20:17] == 4'b1101) && (rt_insn_q[15:11] == 5'd0);
  // mfspr DAR (19) / DSISR (18)
  wire is_mfdx  = (rt_insn_q[31:26] == 6'd31) && (rt_insn_q[10:1] == 10'd339) &&
                  (rt_insn_q[20:17] == 4'b1001) && (rt_insn_q[15:11] == 5'd0);
  wire x_take   = (cpu_ev_mmu[3:0] != 4'd0) || (rt_irq != 2'd0) ||
                  (((rt_srr0 != srr0_prev) || (rt_srr1 != srr1_prev)) && !is_mtsrr);
  always_ff @(posedge clk) begin
    if (rst) begin
      ex_seen <= 1'b0; fx_seen <= 1'b0; x_arm <= 1'b0; xn_cnt <= '0;
      ex_vec <= '0; ex_srr0 <= '0; ex_srr1 <= '0; ex_msr <= '0; ex_pc <= '0; ex_ins <= '0;
      lc_vec <= '0; lc_srr0 <= '0; lc_srr1 <= '0; lc_ins <= '0;
      lp_vec <= '0; lp_srr0 <= '0; lp_srr1 <= '0; lp_ins <= '0;
      x_dar <= '0; x_dsisr <= '0;
      fx_vec <= '0; fx_srr0 <= '0;
      srr0_prev <= '0; srr1_prev <= '0;
    end else if (rt_valid) begin
      pc_prev <= rt_pc; srr0_prev <= rt_srr0; srr1_prev <= rt_srr1;
      x_arm <= x_take;
      if (x_take) begin
        x_srr0 <= rt_srr0; x_srr1 <= rt_srr1; x_msr <= rt_msr; x_pc <= rt_pc; x_ins <= rt_insn_q;
      end
      if (x_arm) begin                                // this record is the vector's first
        if (!ex_seen) begin
          ex_seen <= 1'b1; ex_vec <= rt_pc; ex_srr0 <= x_srr0; ex_srr1 <= x_srr1;
          ex_msr <= x_msr; ex_pc <= x_pc; ex_ins <= x_ins;
        end
        if (!fx_seen) begin
          xn_cnt <= xn_cnt + 32'd1;
          lp_vec <= lc_vec; lp_srr0 <= lc_srr0; lp_srr1 <= lc_srr1; lp_ins <= lc_ins;
          lc_vec <= rt_pc;  lc_srr0 <= x_srr0;  lc_srr1 <= x_srr1;  lc_ins <= x_ins;
        end
      end
      if (is_mfdx && rt_gpr_we && !fx_seen) begin
        if (rt_insn_q[16]) x_dar <= rt_gpr_val; else x_dsisr <= rt_gpr_val;
      end
      if (rt_pc == 32'hFFF0_0000 && !fx_seen && rom_loaded) begin
        fx_seen <= 1'b1; fx_vec <= pc_prev; fx_srr0 <= rt_srr0;
      end
`ifdef M3_XLOG
      if (x_arm && !fx_seen)
        $display("xlog n=%0d vec=%08x pc=%08x ins=%08x srr0=%08x srr1=%08x msr=%08x mmu=%x",
                 xn_cnt + 32'd1, rt_pc, x_pc, x_ins, x_srr0, x_srr1, x_msr, cpu_ev_mmu);
`endif
    end
  end

  logic [9:0]  x_d, y_d;
  logic        hb_d, vb_d, hs_d, vs_d;
  always_ff @(posedge clk) if (ce_pix) begin
    x_d <= hcnt; y_d <= vcnt; hb_d <= hblank; vb_d <= vblank; hs_d <= hsync; vs_d <= vsync;
  end

  // the tile pixel and the 3D layer (scan-out on DDR3 port 0)
  logic [23:0] px;
  logic [15:0] fb_late;
  m3_compose u_comp (
    .clk, .rst_n(mem_rst_n), .ddr_ok, .hcnt, .vcnt, .sc_rd(ce_pix && !hblank && !vblank),
    .show_buf, .show_3d, .o_v, .o_drawn, .o_above, .o_r, .o_g, .o_b, .px,
    .m_req(d_req[0]), .m_addr(d_addr[0]), .m_blen(d_blen[0]), .m_rvalid(d_rvalid[0]), .m_ack(d_ack[0]), .m_dout(dm_dout),
    .n_late(fb_late));
  assign d_we[0] = 1'b0; assign d_din[0] = '0; assign d_be[0] = 8'hFF;

  logic [5:0]  ov_idx;
  logic [31:0] ov_v;
  logic        ov_box, ov_ink;
  generate if (OVERLAY) begin : g_ov
    logic [31:0] tgr20, tgr60, tgr64, tgr68, tgr6c, tgr08;
    always_ff @(posedge clk) if (tg_reg_we) begin
      case (tg_reg_addr)
        6'h02: tgr08 <= tg_reg_wdata;
        6'h08: tgr20 <= tg_reg_wdata;
        6'h18: tgr60 <= tg_reg_wdata;
        6'h19: tgr64 <= tg_reg_wdata;
        6'h1A: tgr68 <= tg_reg_wdata;
        6'h1B: tgr6c <= tg_reg_wdata;
        default: ;
      endcase
    end
    always_ff @(posedge clk) begin
      case (ov_idx)
        // row 0: CPU PC (at vblank), IRQ state/enable and CROM bank, frames, the 53C810 interrupts
        6'd0:  ov_v <= pc_v;
        6'd1:  ov_v <= {irq_state, irq_enable, crom_bank, 7'd0, pingpong};
        6'd2:  ov_v <= frame;
        6'd3:  ov_v <= {n_scsi_irq, 16'd0};
        // row 1: Real3D: frames submitted; frames drawn and dropped; the geometry's
        // error bits, layer lines late, textures uploaded; runs deferred, copies
        6'd4:  ov_v <= {8'h88, n_submit};
        6'd5:  ov_v <= {n_draw, n_drop};
        6'd6:  ov_v <= {geo_err, fb_late[7:0], n_tex_up};
        6'd7:  ov_v <= {n_runs, n_copy};
        // row 2: texture FIFO, VROM port, unmapped accesses (count, last address)
        6'd8:  ov_v <= {8'h94, n_tex};
        6'd9:  ov_v <= {8'h90, n_vrom};
        6'd10: ov_v <= {n_unmapped, 16'd0};
        6'd11: ov_v <= last_unmapped;
        // row 3: tile registers 20, 60, 64, 68
        6'd12: ov_v <= tgr20;
        6'd13: ov_v <= tgr60;
        6'd14: ov_v <= tgr64;
        6'd15: ov_v <= tgr68;
        // row 4: tile register 6C, 08; overruns; the I/O port A
        6'd16: ov_v <= tgr6c;
        6'd17: ov_v <= tgr08;
        6'd18: ov_v <= {tg_ovr_n, 8'd0, io_pa};
        6'd19: ov_v <= {r3d_cfg0[15:0], r3d_cfg2[15:0]};
        // row 5: SDRAM and loader: bytes loaded; flags; the CPU's longest bus wait
        6'd20: ov_v <= {5'd0, ld_bytes};
        6'd21: ov_v <= {3'd0, mem_ready, 3'd0, rom_loaded, 3'd0, ld_ovf, 3'd0, wd_flag, 16'd0};
        6'd22: ov_v <= cpu_wd_max;
        6'd23: ov_v <= {16'd0, cpi_q8};             // CPI x 256, the last second
        // row 6: CPU performance, the last second: cycles, instructions, data memory wait, fetch waiting
        6'd24: ov_v <= perf[0];
        6'd25: ov_v <= perf[1];
        6'd26: ov_v <= perf[6];
        6'd27: ov_v <= perf[8];
        // row 7: redirects, load-use, mul/div wait, decode starved
        6'd28: ov_v <= perf[7];
        6'd29: ov_v <= perf[4];
        6'd30: ov_v <= perf[5];
        6'd31: ov_v <= perf[3];
        // row 8: the first exception entry: vector, SRR0, SRR1, MSR after entry
        6'd32: ov_v <= ex_vec;
        6'd33: ov_v <= ex_srr0;
        6'd34: ov_v <= ex_srr1;
        6'd35: ov_v <= ex_msr;
        // row 9: its record's PC and instruction; the boot ROM's fatal handler:
        // the PC before FFF00000 (the vector it came from), SRR0 then
        6'd36: ov_v <= ex_pc;
        6'd37: ov_v <= ex_ins;
        6'd38: ov_v <= fx_vec;
        6'd39: ov_v <= fx_srr0;
        // row 10: the entry before the most recent one (frozen at the fatal
        // handler: the last entry before the fatal one): vector, SRR0, SRR1, instruction
        6'd40: ov_v <= lp_vec;
        6'd41: ov_v <= lp_srr0;
        6'd42: ov_v <= lp_srr1;
        6'd43: ov_v <= lp_ins;
        // row 11: entries counted; the last DAR and DSISR read by mfspr; the most recent entry's vector
        6'd44: ov_v <= xn_cnt;
        6'd45: ov_v <= x_dar;
        6'd46: ov_v <= x_dsisr;
        6'd47: ov_v <= lc_vec;
        default: ov_v <= 32'h3B00_0001;   // stage B, build 1
      endcase
    end
    m3v_overlay #(.NR(12)) u_ov (.clk, .en(overlay_en), .x(x_d), .y(y_d), .idx(ov_idx), .val(ov_v), .box(ov_box), .ink(ov_ink));
  end else begin : g_noov
    assign ov_box = 1'b0; assign ov_ink = 1'b0; assign ov_idx = '0; assign ov_v = '0;
  end endgenerate

  always_ff @(posedge clk) if (ce_pix) begin
    logic [23:0] c;
    c = px;
    if (ov_box) c = ov_ink ? 24'hffffff : {1'b0, c[23:17], 1'b0, c[15:9], 1'b0, c[7:1]} & 24'h3f3f3f;
    {v_r, v_g, v_b} <= (hb_d || vb_d) ? 24'd0 : c;
    v_hs <= hs_d; v_vs <= vs_d; v_hblank <= hb_d; v_vblank <= vb_d;
  end

  assign led = !rst;

  /* verilator lint_off UNUSEDSIGNAL */
  wire _unused = &{1'b0, tg_ready, tg_ovr, s_idle, scsi_busy, r3d_cfg1, x_d[9], y_d[9], n_cull_lo, n_cull_hi, n_poly,
                    n_tex_skip, p_lo[5:3], p_lo[1:0], d_wacc[3], d_wacc[0], d_rvalid[2], d_rvalid[5], fb_late[15:8]};
  /* verilator lint_on UNUSEDSIGNAL */

endmodule
