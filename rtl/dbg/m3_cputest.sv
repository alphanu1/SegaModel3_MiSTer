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
// Both answer one cycle after the request, as the bench's memory does with a
// wait of 1, so the board's cycle count and retire signature must equal the
// simulator's exactly (sim/cpu/tb_ppc_core.cpp --emit prints both). A
// difference is a hardware or timing fault, not a modelling one.
//
// Run: reset the CPU, clear the counters, run to the program's final `b .`,
// print the report on the UART, reload the data RAM, repeat.

module m3_cputest #(
  parameter              CODE_HEX = "cputest_code.hex",   // untyped: Quartus 17 rejects a string type here
  parameter              DATA_HEX = "cputest_data.hex",
  parameter logic [31:0] END_INSN = 32'h4800_0000,
  parameter int unsigned DIVISOR  = 868
)(
  input  logic clk,
  input  logic rst,
  input  logic tb_tick,
  output logic uart_tx,
  output logic running
);

  // ------------------------------------------------------------ memories
  (* ramstyle = "M10K" *) logic [31:0] rom   [0:8191];
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
  logic [31:0] i_addr, i_data, d_addr, d_wdata, d_rdata;
  logic [3:0]  d_be;
  logic [7:0]  ev;
  logic        rt_valid, rt_last, rt_gpr_we;
  logic [4:0]  rt_gpr;
  logic [31:0] rt_pc, rt_insn, rt_gpr_val;

  ppc_core u_cpu (
    .clk (clk), .rst (cpu_rst),
    .i_req (i_req), .i_addr (i_addr), .i_valid (i_valid), .i_data (i_data),
    .d_req (d_req), .d_we (d_we), .d_addr (d_addr), .d_be (d_be), .d_wdata (d_wdata),
    .d_valid (d_valid), .d_rdata (d_rdata),
    .tb_tick (tb_tick), .ev (ev),
    .rt_valid (rt_valid), .rt_last (rt_last), .rt_pc (rt_pc), .rt_insn (rt_insn),
    .rt_gpr_we (rt_gpr_we), .rt_gpr (rt_gpr), .rt_gpr_val (rt_gpr_val),
    .rt_cr (), .rt_xer (), .rt_lr (), .rt_ctr (), .rt_msr (), .rt_srr0 (), .rt_srr1 ()
  );

  // instruction port: the ROM, one cycle
  always_ff @(posedge clk) begin
    i_valid <= i_req && !cpu_rst;
    if (i_req) i_data <= rom[i_addr[14:2]];
  end

  // data port: the RAM, one cycle; the reload walks the same write port
  logic        reload;
  logic [11:0] rl_addr;
  logic [31:0] rl_word;
  logic        rl_we;
  wire  [11:0] d_word = reload ? rl_addr : d_addr[13:2];
  wire  [3:0]  d_wbe  = reload ? {4{rl_we}} : ((d_req && d_we) ? d_be : 4'd0);
  wire  [31:0] d_wd   = reload ? rl_word : d_wdata;

  always_ff @(posedge clk) begin
    d_valid <= d_req && !cpu_rst;
    if (d_wbe[3]) dram0[d_word] <= d_wd[31:24];
    if (d_wbe[2]) dram1[d_word] <= d_wd[23:16];
    if (d_wbe[1]) dram2[d_word] <= d_wd[15:8];
    if (d_wbe[0]) dram3[d_word] <= d_wd[7:0];
    d_rdata <= {dram0[d_word], dram1[d_word], dram2[d_word], dram3[d_word]};
  end

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
    .busy (mon_busy), .tx (uart_tx)
  );

endmodule
