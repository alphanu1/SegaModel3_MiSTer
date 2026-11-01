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
// The DDR3 probe core's reporter: one line printer for the screen and the
// UART. A line is a template (dp_text_pkg's tmpl, from tools/probe/gen_text.py)
// of characters and fields; a field is a value from `val`, in decimal (one
// time-shared double dabble) or hex. Each character goes to the screen's
// character RAM at its row and column, to the UART, or both.
//
//   start  once, after reset: build date, DDR3 base, clock, mode, the status
//          word and its bit map
//   debug  the progress counters (screen row 20; UART "D ..." lines)
//   case   an automatic case's results (its row, the title; UART "C ...")
//   manual a manual window's settings and results (UART "M ...")
//
// The UART takes bytes from a 1024-byte FIFO (M10K) into m3_uart_tx (8N1).
// A full FIFO drops the byte and counts it: the probe is never held up.

module dp_report
  import dp_text_pkg::*;
#(
  parameter logic [47:0] BUILD   = "000000",
  parameter int unsigned DIVISOR = 868          // 100 MHz / 115200
) (
  input  logic        clk,
  input  logic        rst_n,

  input  logic        go_start, go_dbg, go_case, go_man,   // one cycle, while idle
  output logic        busy,
  input  logic [4:0]  row_case,                    // the case's screen row

  output logic [5:0]  f_idx,                       // the field wanted; its value the same cycle
  input  logic [31:0] f_val,

  output logic        c_we,
  output logic [9:0]  c_wa,
  output logic [5:0]  c_wd,

  output logic        uart_tx,
  output logic [15:0] drops
);

  // ---- the UART: FIFO and transmitter --------------------------------------------
  (* ramstyle = "M10K, no_rw_check" *) logic [7:0] uf [1024];
  logic [10:0] uw, ur;
  logic        u_push;
  logic [7:0]  u_byte, u_q;
  logic        tx_ready, tx_valid, u_rd;
  wire         u_full = (uw - ur) == 11'd1024;
  wire         u_go = !u_rd && !tx_valid && tx_ready && uw != ur;   // a byte out of the FIFO
  always_ff @(posedge clk) begin
    if (u_push && !u_full) uf[uw[9:0]] <= u_byte;
    if (u_go) u_q <= uf[ur[9:0]];
  end
  always_ff @(posedge clk) begin
    if (!rst_n) begin uw <= '0; ur <= '0; drops <= '0; tx_valid <= 1'b0; u_rd <= 1'b0; end
    else begin
      if (u_push) begin
        if (!u_full) uw <= uw + 11'd1;
        else if (drops != 16'hffff) drops <= drops + 16'd1;
      end
      // a byte: read (a cycle), then offered until the transmitter takes it
      tx_valid <= 1'b0;
      if (u_rd) begin tx_valid <= 1'b1; u_rd <= 1'b0; end
      else if (tx_valid && !tx_ready) tx_valid <= 1'b1;
      else if (u_go) begin u_rd <= 1'b1; ur <= ur + 11'd1; end
    end
  end
  m3_uart_tx #(.DIVISOR(DIVISOR)) u_tx (
    .clk, .rst_n, .data(u_q), .valid(tx_valid), .ready(tx_ready), .tx(uart_tx));

  // ---- the printer ---------------------------------------------------------------
  typedef enum logic [2:0] { P_IDLE, P_EL, P_CONV, P_DIG, P_CR, P_LF } pst_t;
  pst_t ps;
  logic [8:0]  ti;                     // template element
  logic [31:0] e;
  logic [71:0] dd;                     // {BCD 40, binary 32}
  logic [5:0]  dk;
  logic [3:0]  dg;                     // digit being printed
  logic [5:0]  cc;                     // column of the next character
  logic        dot_done;

  assign busy = (ps != P_IDLE);
  wire [31:0] te = tmpl(ti);
  assign f_idx = te[16:11];

  function automatic logic [9:0] scr_a(input logic [4:0] row, input logic [5:0] col, input logic [4:0] rc);
    logic [4:0] r;
    r = (row == 5'd31) ? rc : row;
    scr_a = {r, 5'd0} + {1'b0, r, 4'd0} + 10'(col);
  endfunction

  // one character out: the screen (if the element has a row) and the UART (if marked)
`define DP_EMIT(el, col, a) begin c_we <= el[24]; c_wa <= scr_a(el[29:25], (col), row_case); c_wd <= 6'((a) - 8'd32); u_push <= el[17]; u_byte <= (a); end

  function automatic logic [3:0] add3(input logic [3:0] d);
    add3 = (d >= 4'd5) ? d + 4'd3 : d;
  endfunction

  always_ff @(posedge clk) begin
    if (!rst_n) begin ps <= P_IDLE; c_we <= 1'b0; u_push <= 1'b0; end
    else begin
      c_we <= 1'b0; u_push <= 1'b0;
      case (ps)
        P_IDLE: begin
          if (go_start) begin ti <= T_START; ps <= P_EL; end
          else if (go_case) begin ti <= T_CASE; ps <= P_EL; end
          else if (go_man) begin ti <= T_MAN; ps <= P_EL; end
          else if (go_dbg) begin ti <= T_DBG; ps <= P_EL; end
        end
        P_EL: begin
          e <= te;
          case (te[31:30])
            2'd0: ps <= P_CR;
            2'd1: begin
              `DP_EMIT(te, te[23:18], (te[7:0] >= 8'd1 && te[7:0] <= 8'd6) ? BUILD[47 - 8 * (te[2:0] - 3'd1) -: 8] :
                                      (te[7:0] == 8'd7) ? f_val[7:0] : te[7:0])
              ti <= ti + 9'd1;
            end
            default: begin
              dd <= {40'd0, f_val}; dk <= '0; dg <= te[10:7]; cc <= te[23:18]; dot_done <= 1'b0;
              ps <= (te[31:30] == 2'd3) ? P_DIG : P_CONV;
              if (te[31:30] == 2'd3) dd <= {8'd0, f_val, 32'd0};   // hex: the nibbles stand as digits
            end
          endcase
        end
        P_CONV: begin
          logic [39:0] b;
          for (int i = 0; i < 10; i++) b[4*i +: 4] = add3(dd[32 + 4*i +: 4]);
          dd <= {b, dd[31:0]} << 1;
          dk <= dk + 6'd1;
          if (dk == 6'd31) ps <= P_DIG;
        end
        P_DIG: begin
          logic [3:0] d;
          d = dd[32 + 4 * dg +: 4];
          cc <= cc + 6'd1;
          if (e[2] && dg == 4'd5 && !dot_done) begin
            `DP_EMIT(e, cc, ".")
            dot_done <= 1'b1;
          end else begin
            `DP_EMIT(e, cc, (d < 4'd10) ? 8'd48 + 8'(d) : 8'd55 + 8'(d))
            if (dg == e[6:3]) begin ti <= ti + 9'd1; ps <= P_EL; end
            else dg <= dg - 4'd1;
          end
        end
        P_CR: begin u_push <= 1'b1; u_byte <= 8'd13; ps <= P_LF; end
        default: begin u_push <= 1'b1; u_byte <= 8'd10; ps <= P_IDLE; end
      endcase
    end
  end

endmodule
`undef DP_EMIT
