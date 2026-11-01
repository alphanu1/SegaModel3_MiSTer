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
// CPU monitor: performance counters and a retire signature, reported on the
// debug UART (ttyS1 on the board, tools/uart_capture.py).
//
// Counters run from `clear` to `report`. On `report` the values are latched and
// printed one per line, nothing dropped (the report is short and the CPU is
// idle by then):
//
//     P 00 0001A2B3      tag, counter number, value in hex
//
//   00 cycles             01 instructions retired   02 uops retired
//   03 decode starved     04 load-use bubbles       05 mul/div wait
//   06 data memory wait   07 redirects              08 fetch waiting
//   09 retire signature   0A run number
//
// CPI is 00 / 01. The signature is a CRC-32 over every retired GPR write
// (register number and value), computed the same way by the reference model
// (sim/cpu/tb_ppc_core.cpp), so the board run is checked, not only timed.

module m3_cpu_mon #(
  parameter int unsigned DIVISOR = 868,         // 100 MHz / 115200
  parameter bit          SIG     = 1'b1         // the retire signature (0: reads FFFFFFFF; the system has no use for it)
)(
  input  logic        clk,
  input  logic        rst,
  input  logic        clear,
  input  logic        report,
  input  logic [7:0]  ev,
  input  logic        rt_valid,
  input  logic        rt_gpr_we,
  input  logic [4:0]  rt_gpr,
  input  logic [31:0] rt_gpr_val,
  output logic        busy,                      // a report is printing (or about to)
  output logic        tx,
  output logic [8:0][31:0] counts                // the last report's counters 00-08 (the system's overlay)
);

  localparam int N = 11;

  // Every input is registered once, clear and report with the events, so
  // the counts are those of the unregistered form: the trace's GPR value is
  // WB's load alignment, straight from the data port, and in front of the
  // CRC it missed 100 MHz in the full build
  logic        clear_q, report_q, rt_valid_q, rt_gpr_we_q;
  logic [7:0]  ev_q;
  logic [4:0]  rt_gpr_q;
  logic [31:0] rt_gpr_val_q;
  always_ff @(posedge clk) begin
    clear_q      <= clear;
    report_q     <= report && !rst;
    ev_q         <= ev;
    rt_valid_q   <= rt_valid;
    rt_gpr_we_q  <= rt_gpr_we;
    rt_gpr_q     <= rt_gpr;
    rt_gpr_val_q <= rt_gpr_val;
  end

  logic [31:0] cnt [0:8];
  logic [31:0] sig, runs;
  logic [31:0] lat [0:N-1];

  // CRC-32 (reflected 0xEDB88320) over 40 bits: {3'b0, reg, value}
  function automatic logic [31:0] crc40(input logic [31:0] c, input logic [39:0] d);
    logic [31:0] r;
    r = c;
    for (int i = 0; i < 40; i++)
      r = (r[0] ^ d[i]) ? ((r >> 1) ^ 32'hEDB8_8320) : (r >> 1);
    return r;
  endfunction

  // The CRC is linear, crc40(c, d) = crc40(c, 0) ^ crc40(0, d): the data's
  // part is registered first (sd_q), so the signature register's own loop
  // is only its 32-bit part (the 72-input form missed 100 MHz in the R63
  // batch-6 full build). Each update lands a cycle later; the report latches
  // the signature two cycles after the final branch retires, after the last
  // GPR write's update.
  logic [31:0] sd_q;
  logic        su_q;
  // crc40(sig, 0) as an XOR of sig's bits per output bit, the columns being
  // crc40 of each unit vector (constants): Quartus built the loop's form as
  // a chain seven LUTs deep (R63 final-candidate full build, sig to sig)
  logic [31:0] a_col [0:31];
  logic [31:0] sig_a;
  always_comb begin
    for (int j = 0; j < 32; j++) a_col[j] = crc40(32'd1 << j, 40'd0);
    for (int i = 0; i < 32; i++) begin
      sig_a[i] = 1'b0;
      for (int j = 0; j < 32; j++) sig_a[i] = sig_a[i] ^ (a_col[j][i] & sig[j]);
    end
  end
  // the data's part the same way: crc40(0, d) as a flat XOR of d's bits per
  // output bit (the loop's form was eight LUTs deep, rt_gpr_val_q into sd_q,
  // a D-36 cached full build)
  logic [31:0] d_col [0:36];
  logic [36:0] sd_d;
  logic [31:0] sd_n;
  assign sd_d = {rt_gpr_q, rt_gpr_val_q};
  always_comb begin
    for (int j = 0; j < 37; j++) d_col[j] = crc40(32'd0, 40'd1 << j);
    for (int i = 0; i < 32; i++) begin
      sd_n[i] = 1'b0;
      for (int j = 0; j < 37; j++) sd_n[i] = sd_n[i] ^ (d_col[j][i] & sd_d[j]);
    end
  end
`ifdef VERILATOR
  always_ff @(posedge clk)
    if (sd_n != crc40(32'd0, {3'b000, rt_gpr_q, rt_gpr_val_q})) $fatal(1, "monitor CRC matrix");
`endif
  always_ff @(posedge clk) begin
    sd_q <= sd_n;
    su_q <= SIG && rt_valid_q && rt_gpr_we_q && !rst && !clear_q;
  end
  always_ff @(posedge clk) begin
    if (rst) begin
      runs <= 32'd0;
      sig  <= 32'hFFFF_FFFF;
      for (int i = 0; i < 9; i++) cnt[i] <= 32'd0;
    end else if (clear_q) begin
      sig  <= 32'hFFFF_FFFF;
      for (int i = 0; i < 9; i++) cnt[i] <= 32'd0;
    end else begin
      cnt[0] <= cnt[0] + 32'd1;
      for (int i = 0; i < 8; i++) if (ev_q[i]) cnt[i + 1] <= cnt[i + 1] + 32'd1;
      if (su_q) sig <= sig_a ^ sd_q;
      if (report_q) runs <= runs + 32'd1;
    end
  end

  // ------------------------------------------------------------- printer
  logic       p_busy;                          // the printer's
  assign busy = p_busy || report_q;
  logic       u_valid, u_ready;
  logic [7:0] u_data, u_data_q;
  logic [3:0] line;
  logic [3:0] ch;          // 0 'P', 1 ' ', 2-3 index, 4 ' ', 5-12 value, 13 CR, 14 LF
  logic [31:0] wq;         // lat[line], a cycle on (below)

  function automatic logic [7:0] hex(input logic [3:0] n);
    return (n < 4'd10) ? (8'h30 + {4'd0, n}) : (8'h37 + {4'd0, n});
  endfunction

  always_comb begin
    case (ch)
      4'd0:    u_data = "P";
      4'd1:    u_data = " ";
      4'd2:    u_data = "0";
      4'd3:    u_data = hex(line);
      4'd4:    u_data = " ";
      4'd13:   u_data = 8'h0D;
      4'd14:   u_data = 8'h0A;
      default: u_data = hex(wq[31 - 4*(ch - 5) -: 4]);
    endcase
  end
  // the line's word, registered every cycle: the value's first digit (ch 5)
  // is chosen five characters after the line changes, so wq has long been
  // lat[line] by then. Read through the 11-way select in front of the digit
  // select and hex, line into u_data_q missed 100 MHz (a round-2 uncached
  // full build, -0.23 ns)
  always_ff @(posedge clk) wq <= lat[line];
`ifdef VERILATOR
  always_ff @(posedge clk)
    if (!rst && p_busy && u_ready && !u_valid && ch >= 4'd5 && ch <= 4'd12 && wq != lat[line])
      $fatal(1, "monitor printer's registered word");
`endif

  always_ff @(posedge clk) begin
    if (rst) begin
      p_busy  <= 1'b0;
      u_valid <= 1'b0;
      line    <= 4'd0;
      ch      <= 4'd0;
    end else begin
      u_valid <= 1'b0;
      if (!p_busy && report_q) begin
        for (int i = 0; i < 9; i++) lat[i] <= cnt[i];
        lat[9]  <= sig;
        lat[10] <= runs;
        p_busy  <= 1'b1;
        line    <= 4'd0;
        ch      <= 4'd0;
      end else if (p_busy && u_ready && !u_valid) begin
        u_valid <= 1'b1;
        if (ch == 4'd14) begin
          ch <= 4'd0;
          if (line == 4'(N - 1)) p_busy <= 1'b0;
          else line <= line + 4'd1;
        end else ch <= ch + 4'd1;
      end
    end
  end

  always_comb for (int i = 0; i < 9; i++) counts[i] = lat[i];

  m3_uart_tx #(.DIVISOR(DIVISOR)) u_tx (
    .clk (clk), .rst_n (!rst),
    .data (u_data_q), .valid (u_valid), .ready (u_ready),
    .tx (tx)
  );

  // the byte goes out the cycle after it is chosen
  always_ff @(posedge clk) if (p_busy && u_ready && !u_valid) u_data_q <= u_data;

endmodule
