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
// CPU test system for the board: ppc_core, a code ROM, a data RAM and the
// monitor, running one generated program again and again.
//
//   code ROM  32 KiB  0x00000000-0x00007FFF, and the vectors at 0xFFF0xxxx
//   data RAM  16 KiB  0x00100000-0x00103FFF, reloaded from a pristine copy
//                     before every run so each run starts identically
//
// Both answer two cycles after the request, as the bench's memory does with a
// wait of 2, so the board's cycle count and retire signature must equal the
// simulator's exactly (sim/cpu/tb_ppc_core.cpp --emit prints both). A
// difference is a hardware or timing fault, not a modelling one.
//
// CACHED = 1 builds the CPU subsystem instead (rtl/cpu/ppc_cpu.sv: the core
// with its caches), with the ROM and RAM behind the caches' memory port
// (ppc_cache_pkg.sv): the acknowledge a cycle after the request, the first
// word about MEM_LAT cycles after that, then a word a cycle. The program
// enables both caches first (HID0), and every run starts from caches that
// reset invalidated, so this form is as deterministic as the other.
//
// Run: reset the CPU, clear the counters, run to the program's final `b .`,
// print the report on the UART, reload the data RAM, repeat.

module m3_cputest
  import ppc_cache_pkg::*;
#(
  parameter              CODE_HEX = "cputest_code.hex",   // untyped: Quartus 17 rejects a string type here
  parameter              DATA_HEX = "cputest_data.hex",
  parameter logic [31:0] END_INSN = 32'h4800_0000,
  parameter int unsigned DIVISOR  = 868,
  parameter bit          CACHED   = 1'b0,
  parameter int unsigned MEM_LAT  = 4
)(
  input  logic clk,
  input  logic rst,
  input  logic tb_tick,
  output logic uart_tx,
  output logic running
);

  // ------------------------------------------------------------ memories
  // The code ROM in blocks 1,024 deep (max_depth): its address bits 14-12
  // then select among the blocks' outputs from a register, and only bits
  // 11-2 reach the blocks. Built 8,192 x 1 per block, bits 14-12 went to all
  // 32 blocks' address ports, and those bits are the fetch translation's
  // physical page (ppc_core i_pahi): the D-36 uncached full builds' worst
  // path, redir_a_q through the IBAT compare into the ROM. The core with
  // its I-cache has no such path (the cache indexes with bits 11-5 and
  // registers the page). Same 32 M10K.
  (* ramstyle = "M10K", max_depth = 1024 *) logic [31:0] rom   [0:8191];
  (* ramstyle = "M10K" *) logic [31:0] dinit [0:4095];
  (* ramstyle = "M10K" *) logic [7:0]  dram0 [0:4095];   // byte lanes, big-endian:
  (* ramstyle = "M10K" *) logic [7:0]  dram1 [0:4095];   // lane 0 is bits 31:24
  (* ramstyle = "M10K" *) logic [7:0]  dram2 [0:4095];
  (* ramstyle = "M10K" *) logic [7:0]  dram3 [0:4095];

  initial begin
    $readmemh(CODE_HEX, rom);
    $readmemh(DATA_HEX, dinit);
  end

  // ------------------------------------------------------------ the CPU
  logic        cpu_rst;
  logic        i_req, i_valid, d_req, d_we, d_valid;
  logic [31:0] i_addr, d_addr, d_wdata, d_rdata;
  (* preserve *) logic [31:0] i_data;   // a fabric register, see the instruction port
  logic [3:0]  d_be;
  logic        icbi_req;
  logic [7:0]  ev;
  logic        rt_valid, rt_last, rt_gpr_we;
  logic [4:0]  rt_gpr;
  logic [31:0] rt_pc, rt_insn, rt_gpr_val;

  logic        reload;                   // the data RAM is reloaded (the CPU in reset)
  logic [11:0] rl_addr;
  logic [31:0] rl_word;
  logic        rl_we;

  generate if (!CACHED) begin : g_core
  ppc_core u_cpu (
    .clk (clk), .rst (cpu_rst),
    .i_req (i_req), .i_addr (i_addr), .i_wimg (), .i_seq (), .i_valid (i_valid), .i_data (i_data),
    .i_way (2'd0), .i_arr (1'b0), .i_pa (20'd0), .i_pv (), .i_pidx (), .i_pway (), .i_ppa (), .i_pre (), .ev_btb (),
    .i_two (1'b0), .i_w2 (), .i_data2 (32'd0),
    .d_req (d_req), .d_we (d_we), .d_addr (d_addr), .d_be (d_be), .d_wdata (d_wdata),
    .d_wimg (), .d_op (), .d_rdy (1'b1), .d_idle (1'b1), .d_valid (d_valid), .d_rdata (d_rdata),
    .d_resv (), .d_cond (), .res_kill (1'b0),
    .hid_ice (), .hid_dce (), .hid_ilock (), .hid_dlock (), .hid_icfi (), .hid_dcfi (), .hid_noopti (),
    .icbi_req (icbi_req), .icbi_set (), .icbi_ack (icbi_req),
    .tb_tick (tb_tick), .ext_int (1'b0), .ev (ev), .ev_mmu (),
    .rt_valid (rt_valid), .rt_last (rt_last), .rt_pc (rt_pc), .rt_insn (rt_insn),
    .rt_gpr_we (rt_gpr_we), .rt_gpr (rt_gpr), .rt_gpr_sh (), .rt_irq (), .rt_gpr_val (rt_gpr_val),
    .rt_cr (), .rt_xer (), .rt_lr (), .rt_ctr (), .rt_msr (), .rt_srr0 (), .rt_srr1 (),
    .rt_fpscr (), .rt_fpq (), .rt_fw_en (), .rt_fw_idx (), .rt_fw_val (), .rt_dly ()
  );

  // instruction port: the ROM, two cycles. The M10K's output is registered
  // once more before the core: the core predecodes i_data at fetch, and the
  // block's own clock-to-out in front of that missed 120 MHz by 2.85 ns (R28).
  // The fitter packed a plain i_data <= i_q back into the M10K's output
  // register, which put the clock-to-out in front of predecode again (R41);
  // the mux below, on a word the core ignores while i_valid is low, keeps
  // it in the fabric.
  logic        i_v1;
  logic [31:0] i_q;
  always_ff @(posedge clk) begin
    i_v1    <= i_req && !cpu_rst;
    i_valid <= i_v1 && !cpu_rst;
    if (i_req) i_q <= rom[i_addr[14:2]];
    i_data  <= i_v1 ? i_q : {i_q[0], i_q[31:1]};
  end

  // data port: the RAM, two cycles; the reload walks the same write port
  wire  [11:0] d_word = reload ? rl_addr : d_addr[13:2];
  wire  [31:0] d_wd   = reload ? rl_word : d_wdata;

  // The read is registered once more for the same reason as the ROM's. A
  // write is posted: registered, then written a cycle later. The core's
  // translated store request reaches the M10K write enable straight from EX
  // and missed 100 MHz by 0.32 ns (R32). A read of the word being written
  // in that cycle takes the posted bytes over the RAM's.
  logic        d_v1, w_hit;
  logic [31:0] d_q, w_wd, h_wd;
  logic [11:0] w_word;
  logic [3:0]  w_be, h_be;
  // the posted write's lanes and its enable registered apart and ANDed after:
  // the store's lanes come from EX's address sum and the request from its
  // hold, and the two together into w_be missed 100 MHz by 0.04 ns (a
  // round-2 uncached full build, op_x into w_be)
  logic [3:0]  w_be_r;
  logic        w_en;
  assign w_be = w_en ? w_be_r : 4'd0;
  always_ff @(posedge clk) begin
    d_v1    <= d_req && !cpu_rst;
    d_valid <= d_v1 && !cpu_rst;
    w_word  <= d_word;
    w_be_r  <= reload ? {4{rl_we}} : d_be;
    w_en    <= reload || (d_req && d_we);
    w_wd    <= d_wd;
    if (w_be[3]) dram0[w_word] <= w_wd[31:24];
    if (w_be[2]) dram1[w_word] <= w_wd[23:16];
    if (w_be[1]) dram2[w_word] <= w_wd[15:8];
    if (w_be[0]) dram3[w_word] <= w_wd[7:0];
    d_q     <= {dram0[d_word], dram1[d_word], dram2[d_word], dram3[d_word]};
    w_hit   <= (w_be != 4'd0) && (w_word == d_word);
    h_be    <= w_be;
    h_wd    <= w_wd;
    for (int b = 0; b < 4; b++)
      d_rdata[8*b +: 8] <= (w_hit && h_be[b]) ? h_wd[8*b +: 8] : d_q[8*b +: 8];
  end

  end else begin : g_sub
  // ------------------------------------------------------------ CACHED = 1
  // The CPU subsystem, and a controller serving the caches' port from the
  // ROM and the RAM (ppc_cache_pkg.sv). Addresses 0x001xxxxx are the RAM,
  // all else the ROM (writes to it are dropped). Every RAM write enable and
  // address, and every read address, comes from a register; reads take two
  // edges (the M10K, then m_rdata).
  logic        m_req, m_ack, m_wnext, m_rvalid;
  logic [1:0]  m_cmd;
  logic [31:0] m_addr, m_wdata, m_rdata;
  logic [3:0]  m_be;

  ppc_cpu #(.SNOOP (1'b0)) u_cpu (
    .clk (clk), .rst (cpu_rst),
    .m_req (m_req), .m_cmd (m_cmd), .m_addr (m_addr), .m_be (m_be), .m_wdata (m_wdata),
    .m_ack (m_ack), .m_wnext (m_wnext), .m_rvalid (m_rvalid), .m_rdata (m_rdata),
    .sn_req (1'b0), .sn_addr (27'd0), .sn_inv (1'b0), .sn_ack (),   // no DMA here
    .tb_tick (tb_tick), .ext_int (1'b0), .ev (ev), .ev_mmu (), .ev_btb (),
    .rt_valid (rt_valid), .rt_last (rt_last), .rt_pc (rt_pc), .rt_insn (rt_insn),
    .rt_gpr_we (rt_gpr_we), .rt_gpr (rt_gpr), .rt_gpr_sh (), .rt_irq (), .rt_gpr_val (rt_gpr_val),
    .rt_cr (), .rt_xer (), .rt_lr (), .rt_ctr (), .rt_msr (), .rt_srr0 (), .rt_srr1 (),
    .rt_fpscr (), .rt_fpq (), .rt_fw_en (), .rt_fw_idx (), .rt_fw_val (), .rt_dly (),
    .t_dreq (), .t_dwe (), .t_daddr (), .t_dwimg (), .t_dop (), .t_dvalid (), .t_drdata (),
    .t_dbe (), .t_dwdata (), .t_drsv (), .t_dcnd ()
  );

  localparam logic [1:0] C_IDLE = 2'd0, C_ACK = 2'd1, C_RD = 2'd2, C_WB = 2'd3;
  logic [1:0]  c_st;
  logic [1:0]  c_cmd;
  logic [31:0] c_addr, c_wd;
  logic [3:0]  c_be;
  logic [7:0]  c_wait;
  logic [3:0]  c_n;                       // beats issued
  wire         c_line = c_cmd == MC_LINE_RD || c_cmd == MC_LINE_WR;
  wire         c_ram  = c_addr[31:20] == 12'h001;
  wire  [2:0]  c_w    = c_addr[4:2] + c_n[2:0];   // critical word first, wrapping (figure 3-3, in words)
  logic        r_iss, r_v1, r_ram1;        // a read issued; its word one edge on
  logic [12:0] r_idx;
  logic [31:0] q_rom, q_ram;
  logic        w_v;                        // a RAM write registered
  logic [11:0] w_idx;
  logic [31:0] w_d;
  logic [3:0]  w_be;
  wire  [11:0] wa  = reload ? rl_addr : w_idx;
  wire  [3:0]  wbe = reload ? {4{rl_we}} : (w_v ? w_be : 4'd0);
  wire  [31:0] wd  = reload ? rl_word : w_d;
  always_ff @(posedge clk) begin
    if (wbe[3]) dram0[wa] <= wd[31:24];
    if (wbe[2]) dram1[wa] <= wd[23:16];
    if (wbe[1]) dram2[wa] <= wd[15:8];
    if (wbe[0]) dram3[wa] <= wd[7:0];
    q_ram <= {dram0[r_idx[11:0]], dram1[r_idx[11:0]], dram2[r_idx[11:0]], dram3[r_idx[11:0]]};
    q_rom <= rom[r_idx];
    r_v1  <= r_iss && !cpu_rst;
    m_rvalid <= r_v1 && !cpu_rst;
    m_rdata  <= r_ram1 ? q_ram : q_rom;
  end
  always_ff @(posedge clk) begin
    m_ack   <= 1'b0;
    m_wnext <= 1'b0;
    r_iss   <= 1'b0;
    w_v     <= 1'b0;
    if (cpu_rst) begin
      c_st <= C_IDLE;
    end else begin
      // a cast-out beat taken: written at the next edge
      if (m_wnext) begin
        w_v <= c_ram; w_idx <= {c_addr[13:5], c_n[2:0] - 3'd1}; w_d <= m_wdata; w_be <= 4'hF;
      end
      case (c_st)
        C_IDLE: if (m_req) begin
          c_cmd <= m_cmd; c_addr <= m_addr; c_be <= m_be; c_wd <= m_wdata;
          m_ack <= 1'b1; c_st <= C_ACK;
        end
        C_ACK: begin                       // m_ack is high this cycle
          c_n <= 4'd0; c_wait <= 8'(MEM_LAT);
          if (c_cmd == MC_WR) begin
            w_v <= c_ram; w_idx <= c_addr[13:2]; w_d <= c_wd; w_be <= c_be;
            c_st <= C_IDLE;
          end else c_st <= (c_cmd == MC_LINE_WR) ? C_WB : C_RD;
        end
        C_RD: if (c_wait > 8'd2) c_wait <= c_wait - 8'd1;   // the read pipeline's two edges count
          else if (c_n != (c_line ? 4'd8 : 4'd1)) begin
            r_iss <= 1'b1; r_idx <= c_ram ? {1'b0, c_addr[13:5], c_line ? c_w : c_addr[4:2]} :
                                            {c_addr[14:5], c_line ? c_w : c_addr[4:2]};
            r_ram1 <= c_ram;
            c_n <= c_n + 4'd1;
          end else if (!r_iss && !r_v1) c_st <= C_IDLE;      // the last word is out
        C_WB: if (c_wait > 8'd1) c_wait <= c_wait - 8'd1;
          else if (c_n != 4'd8) begin
            m_wnext <= 1'b1; c_n <= c_n + 4'd1;
          end else if (!m_wnext) c_st <= C_IDLE;
        default: c_st <= C_IDLE;
      endcase
    end
  end
  end endgenerate

  // ------------------------------------------------------------ run control
  typedef enum logic [1:0] { S_RELOAD, S_RUN, S_REPORT } state_t;
  state_t      st;
  logic        mon_busy, report, clear;
  logic [12:0] rl_cnt;

  always_ff @(posedge clk) begin
    report <= 1'b0;
    clear  <= 1'b0;
    rl_we  <= 1'b0;
    if (rst) begin
      st      <= S_RELOAD;
      reload  <= 1'b1;
      rl_cnt  <= 13'd0;
      cpu_rst <= 1'b1;
    end else begin
      case (st)
        S_RELOAD: begin
          cpu_rst <= 1'b1;
          // read the pristine word, write it one cycle later
          rl_word <= dinit[rl_cnt[11:0]];
          rl_addr <= rl_cnt[11:0];
          rl_we   <= !rl_cnt[12];
          if (rl_cnt == 13'd4096) begin
            reload  <= 1'b0;
            cpu_rst <= 1'b0;
            clear   <= 1'b1;
            st      <= S_RUN;
          end else rl_cnt <= rl_cnt + 13'd1;
        end
        S_RUN: if (rt_valid && rt_last && rt_insn == END_INSN) begin
          cpu_rst <= 1'b1;
          report  <= 1'b1;
          st      <= S_REPORT;
        end
        default: if (!report && !mon_busy) begin
          reload <= 1'b1;
          rl_cnt <= 13'd0;
          st     <= S_RELOAD;
        end
      endcase
    end
  end

  assign running = (st == S_RUN);

  m3_cpu_mon #(.DIVISOR(DIVISOR)) u_mon (
    .clk (clk), .rst (rst), .clear (clear), .report (report), .ev (ev),
    .rt_valid (rt_valid), .rt_gpr_we (rt_gpr_we), .rt_gpr (rt_gpr), .rt_gpr_val (rt_gpr_val),
    .busy (mon_busy), .tx (uart_tx), .counts ()
  );

endmodule
