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
// PowerPC multiply and divide.
//
// Multiply: 33 x 33 signed on the DSPs, operands and product registered,
// result three cycles after start (the CPI model's figure, study R22).
// mullw, mulhw, mulhwu and mulli share the one product.
//
// Divide: radix-2 restoring on magnitudes, one quotient bit a cycle, 34 cycles.
// divw / divwu are 0.2% of VF3's divide-free instruction stream (R12); area
// matters more than speed here.
//
// The manual leaves divide by zero and 0x80000000 / -1 undefined. This follows
// Supermodel, the emulator the games are known to run on: divw by zero gives 0
// for a non-negative dividend and -1 for a negative one, 0x80000000 / -1 gives
// -1, divwu by zero gives 0, and each sets OV.

module ppc_muldiv
  import ppc_pkg::*;
(
  input  logic        clk,
  input  logic        rst,
  input  logic        start,
  input  alu_op_t     op,
  input  logic [31:0] x,           // rA
  input  logic [31:0] y,           // rB or the mulli immediate
  output logic        done,        // result valid this cycle
  output logic [31:0] result,
  output logic        ov
);

  // ------------------------------------------------------------- multiply
  logic        m_go1, m_go2;
  alu_op_t     m_op1, m_op2;
  logic signed [32:0] ma, mb;
  logic signed [65:0] prod;

  always_ff @(posedge clk) begin
    if (rst) begin
      m_go1 <= 1'b0;
      m_go2 <= 1'b0;
    end else begin
      m_go1 <= start && (op == OP_MUL || op == OP_MULH || op == OP_MULHU);
      m_go2 <= m_go1;
    end
    if (start) begin
      m_op1 <= op;
      ma    <= {(op != OP_MULHU) & x[31], x};
      mb    <= {(op != OP_MULHU) & y[31], y};
    end
    prod  <= ma * mb;
    m_op2 <= m_op1;
  end

  wire [31:0] m_res = (m_op2 == OP_MUL) ? prod[31:0] : prod[63:32];
  wire        m_ov  = (prod[63:32] != {32{prod[31]}});

  // --------------------------------------------------------------- divide
  logic        d_busy, d_fin, d_neg, d_special;
  logic [5:0]  d_cnt;
  logic [31:0] d_q, d_div, d_spec;
  logic [32:0] d_r;

  wire        d_start  = start && (op == OP_DIV || op == OP_DIVU);
  wire        d_signed = (op == OP_DIV);
  wire        x_neg    = d_signed && x[31];
  wire        y_neg    = d_signed && y[31];
  wire [31:0] x_mag    = x_neg ? -x : x;
  wire [31:0] y_mag    = y_neg ? -y : y;
  wire        by_zero  = (y == 32'd0);
  wire        ovf_case = d_signed && (x == 32'h8000_0000) && (y == 32'hFFFF_FFFF);
  wire [32:0] d_r2     = {d_r[31:0], d_q[31]};
  wire        d_ge     = d_r2 >= {1'b0, d_div};

  always_ff @(posedge clk) begin
    d_fin <= 1'b0;
    if (rst) begin
      d_busy <= 1'b0;
    end else if (d_start) begin
      d_special <= by_zero || ovf_case;
      d_spec    <= (by_zero && !x_neg) ? 32'd0 : 32'hFFFF_FFFF;
      d_neg     <= x_neg ^ y_neg;
      d_q       <= x_mag;
      d_div     <= y_mag;
      d_r       <= 33'd0;
      d_cnt     <= 6'd0;
      d_busy    <= 1'b1;
    end else if (d_busy) begin
      if (d_special || d_cnt == 6'd32) begin
        d_busy <= 1'b0;
        d_fin  <= 1'b1;
      end else begin
        d_r   <= d_ge ? (d_r2 - {1'b0, d_div}) : d_r2;
        d_q   <= {d_q[30:0], d_ge};
        d_cnt <= d_cnt + 6'd1;
      end
    end
  end

  wire [31:0] d_res = d_special ? d_spec : (d_neg ? -d_q : d_q);

  assign done   = m_go2 || d_fin;
  assign result = d_fin ? d_res : m_res;
  assign ov     = d_fin ? d_special : m_ov;

endmodule
