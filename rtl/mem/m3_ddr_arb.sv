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
// NP masters on m3_ddr3p's port L (m2_ddr3's contract). The viewer's
// m3v_ddr_arb with byte enables passed through (its masters wrote whole
// beats; the Real3D's windows write 32-bit words and texels). Model 2's
// m2_ddr3_arb rules: port 0 (the scan-out reader, which has a beam deadline)
// always wins; the rest are granted in turn. A grant is held to the
// acknowledge; responses go by the registered owner, never by a live request;
// and no grant in the cycle after an acknowledge, while the finished master's
// request is still high (Model 2 R360-R375).
//
// Every output to the master comes from the registered owner: a grant is made
// in one cycle and presented the next, so no master's request reaches the
// DDR3 master's command queue or the f2sdram pins combinationally (the first
// stage B fit missed 100 MHz by 2 ns on that path). It costs a cycle a
// transaction.

`timescale 1ns/1ps

module m3_ddr_arb #(
  parameter int NP = 5
) (
  input  logic                 clk,
  input  logic                 rst,

  input  logic [NP-1:0]        req,
  input  logic [NP-1:0]        we,
  input  logic [NP-1:0][24:0]  addr,
  input  logic [NP-1:0][7:0]   blen,
  input  logic [NP-1:0][63:0]  din,
  input  logic [NP-1:0][7:0]   be,
  output logic [NP-1:0]        wacc,
  output logic [NP-1:0]        rvalid,
  output logic [NP-1:0]        ack,

  output logic        m_req,
  output logic        m_we,
  output logic [24:0] m_addr,
  output logic [7:0]  m_blen,
  output logic [63:0] m_din,
  output logic [7:0]  m_be,
  input  logic        m_wacc,
  input  logic        m_rvalid,
  input  logic        m_ack
);

  localparam int PB = $clog2(NP);
  logic          busy, cool;
  logic [PB-1:0] owner, last, win;
  logic          any;

  // port 0 first, then the others in turn after the last granted: the first requester above last, else the
  // first from port 1 up. hm, the ports above last, is a register kept with last, so the choice is two
  // lowest-set-bit searches over the requests and not seven wrapped adds in a chain from last (the arbiter's
  // loop through win into owner and last was the DDR side's worst at 120 MHz; tclose3)
  logic [NP-1:0] hm;
  // (no parameter-width casts in this module: Quartus 17 rejects them, Error 10170)
  function automatic logic [PB-1:0] ffs(input logic [NP-1:0] v);
    logic [31:0] kk;
    ffs = '0;
    for (int k = NP - 1; k >= 0; k--) if (v[k]) begin kk = k; ffs = kk[PB-1:0]; end
  endfunction
  function automatic logic [NP-1:0] above(input logic [PB-1:0] l);
    above = {NP{1'b1}} << l;
    above = above << 1;
  endfunction
  wire [NP-1:0] rq_hi = req & hm, rq_lo = req & ~hm & {{(NP-1){1'b1}}, 1'b0};
  always_comb begin
    any = |req;
    win = req[0] ? '0 : (rq_hi != '0) ? ffs(rq_hi) : ffs(rq_lo);
  end
`ifdef VERILATOR
  logic [PB:0] cand;
  logic [PB-1:0] win_r;
  logic          any_r;
  always_comb begin
    win_r = '0; any_r = 1'b0; cand = '0;
    if (req[0]) begin win_r = '0; any_r = 1'b1; end
    else begin
      for (int k = 1; k < NP; k++) begin
        cand = (PB+1)'(last) + (PB+1)'(k);
        if (cand >= (PB+1)'(NP)) cand = cand - (PB+1)'(NP - 1);
        if (!any_r && req[cand[PB-1:0]]) begin win_r = cand[PB-1:0]; any_r = 1'b1; end
      end
    end
  end
  always_ff @(posedge clk) if (!rst && (any != any_r || (any && win != win_r) || hm != above(last)))
    $fatal(1, "m3_ddr_arb: win against the turn's loop");
`endif

  assign m_req  = busy && req[owner];
  assign m_we   = we[owner];
  assign m_addr = addr[owner];
  assign m_blen = blen[owner];
  assign m_din  = din[owner];
  assign m_be   = be[owner];

  always_comb begin
    wacc = '0; rvalid = '0; ack = '0;
    wacc[owner]   = m_wacc;
    rvalid[owner] = m_rvalid;
    ack[owner]    = m_ack;
  end

  always_ff @(posedge clk) begin
    if (rst) begin
      busy <= 1'b0; cool <= 1'b0; owner <= '0; last <= '0; hm <= above('0);
    end else begin
      cool <= 1'b0;
      if (!busy) begin
        if (!cool && any) begin
          busy <= 1'b1; owner <= win;
          if (win != '0) begin last <= win; hm <= above(win); end
        end
      end else if (m_ack) begin
        busy <= 1'b0; cool <= 1'b1;
      end
    end
  end

endmodule
