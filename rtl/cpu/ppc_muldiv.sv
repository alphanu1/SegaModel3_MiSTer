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
// Multiply: 33 x 33 signed on the DSPs, result three cycles after start (the
// CPI model's figure, study R22). mullw, mulhw, mulhwu and mulli share the one
// product. The operands are the core's EX operand registers, stable while the
// multiply is in EX, so the DSPs take them in the start cycle with no
// register of their own (and are kept from making one, below); the partial products are registered (at the DSP
// outputs) and summed in the next cycle into the product register. With the
// operands registered here instead, Quartus put the product register at the
// DSP outputs and the 64-bit sum of the partial products fell in the result
// cycle, in front of OV and the core's result select (R23 timing).
//
// Divide: radix-2 restoring on magnitudes, one quotient bit a cycle, 35 cycles to the result.
// divw / divwu are 0.2% of VF3's divide-free instruction stream (R12); area
// matters more than speed here.
//
// The manual leaves divide by zero and 0x80000000 / -1 undefined, the one place
// an emulator is followed over it (study D-17). This follows Supermodel, the
// emulator the games are known to run on: divw by zero gives 0
// for a non-negative dividend and -1 for a negative one, 0x80000000 / -1 gives
// -1, divwu by zero gives 0, and each sets OV.

module ppc_muldiv
  import ppc_pkg::*;
(
  input  logic        clk,
  input  logic        rst,
  input  logic        start,
  input  logic        en,          // EX holds a multiply or divide (a register)
  input  alu_op_t     op,
  input  logic [31:0] x,           // rA
  input  logic [31:0] y,           // rB or the mulli immediate
  output logic        done,        // result valid this cycle
  output logic [31:0] result,
  output logic        ov
);

  // ------------------------------------------------------------- multiply
  logic        m_go1, m_go2, m_lo2;
  alu_op_t     m_op1;
  logic signed [65:0] prod_p, prod;
  // The operands reach the DSPs through a gate (en), so the fitter cannot
  // pack a copy of the core's operand registers into the DSPs' input
  // registers: it did, and the forwarding mux in front of those registers
  // then ended in the DSP column, 1.3 to 1.7 ns away (the leading paths of
  // every seed). The DSPs take the operands as inputs and register only
  // their outputs.
  wire  [31:0]        xg = x & {32{en}};
  wire  [31:0]        yg = y & {32{en}};
  wire  signed [32:0] ma = {(op != OP_MULHU) & xg[31], xg};
  wire  signed [32:0] mb = {(op != OP_MULHU) & yg[31], yg};

  always_ff @(posedge clk) begin
    if (rst) begin
      m_go1 <= 1'b0;
      m_go2 <= 1'b0;
    end else begin
      m_go1 <= start && (op == OP_MUL || op == OP_MULH || op == OP_MULHU);
      m_go2 <= m_go1;
    end
    if (start) m_op1 <= op;
    prod_p <= ma * mb;
    prod   <= prod_p;
    m_lo2 <= (m_op1 == OP_MUL);                        // mullw: the low word
  end

  wire        m_ov  = (prod[63:32] != {32{prod[31]}});

  // --------------------------------------------------------------- divide
  logic        d_busy, d_fin, d_neg, d_special, d_first, d_sgn;
  logic [31:0] d_x, d_y;
  logic [5:0]  d_cnt;
  logic [31:0] d_q, d_div, d_spec, d_res_q;
  logic [32:0] d_r;

  wire        d_start  = start && (op == OP_DIV || op == OP_DIVU);
  // operands are registered raw on start; magnitudes, signs and the special
  // cases come from the registers one cycle later (R23 timing)
  wire        x_neg    = d_sgn && d_x[31];
  wire        y_neg    = d_sgn && d_y[31];
  wire [31:0] x_mag    = x_neg ? -d_x : d_x;
  wire [31:0] y_mag    = y_neg ? -d_y : d_y;
  wire        d_zero   = (d_y == 32'd0);
  wire        d_ovf    = d_sgn && (d_x == 32'h8000_0000) && (d_y == 32'hFFFF_FFFF);
  // the special cases are checked on the second cycle, from registered compares,
  // not on the forwarded operands of the start cycle (R23 timing)
  wire [32:0] d_r2     = {d_r[31:0], d_q[31]};
  // the trial subtract's borrow is the compare (area pass 4: a separate >= was a second 33-bit carry chain)
  wire [33:0] d_sub    = {1'b0, d_r2} - {2'b00, d_div};
  wire        d_ge     = !d_sub[33];
  wire [31:0] d_res    = d_special ? d_spec : (d_neg ? -d_q : d_q);

  always_ff @(posedge clk) begin
    d_fin <= 1'b0;
    if (rst) begin
      d_busy <= 1'b0;
    end else if (d_start) begin
      d_first   <= 1'b1;
      d_special <= 1'b0;
      d_sgn     <= (op == OP_DIV);
      d_x       <= x;
      d_y       <= y;
      d_r       <= 33'd0;
      d_cnt     <= 6'd0;
      d_busy    <= 1'b1;
    end else if (d_busy && d_first) begin
      d_first   <= 1'b0;
      d_special <= d_zero || d_ovf;
      d_spec    <= (d_zero && !x_neg) ? 32'd0 : 32'hFFFF_FFFF;
      d_neg     <= x_neg ^ y_neg;
      d_q       <= x_mag;
      d_div     <= y_mag;
    end else if (d_busy) begin
      if (d_special || d_cnt == 6'd32) begin
        d_busy  <= 1'b0;
        d_fin   <= 1'b1;
        d_res_q <= d_res;                              // the quotient is final here
      end else begin
        d_r   <= d_ge ? d_sub[32:0] : d_r2;
        d_q   <= {d_q[30:0], d_ge};
        d_cnt <= d_cnt + 6'd1;
      end
    end
  end

  assign done   = m_go2 || d_fin;
  // The result's sign is applied as the divide finishes, a cycle before d_fin,
  // so the negation's carry chain is off the core's result path (R23 timing).
  // The result is an AND-OR of one-hot selects, zero when not done, so the
  // core can OR its held copy in with no mux level (the MMU work's timing).
  assign result = ({32{d_fin}} & d_res_q) | ({32{m_go2 && m_lo2}} & prod[31:0]) |
                  ({32{m_go2 && !m_lo2}} & prod[63:32]);
  assign ov     = d_fin ? d_special : m_ov;

endmodule
