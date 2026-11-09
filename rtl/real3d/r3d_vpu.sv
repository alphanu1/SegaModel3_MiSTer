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
// Real3D geometry engine: the one FP32 pool every stage runs on. Four lanes,
// each one r3d_fp_mul feeding one r3d_fp_add (4 multipliers, 4 adders), plus
// the shared correctly rounded divide / reciprocal unit. Every lane runs the
// same micro-instruction on its own data (a vertex, a matrix column, a clip
// component). Transform, light, projection, clip, the matrix stack, the model
// set-up and the frustum set-up's arithmetic all run here; no stage has an
// FP unit of its own.
//
// MAC: r = (a * b) + c, rounded twice exactly as two IEEE ops. A pure product
// uses c = -0, which IEEE addition leaves unchanged for every value, so MAME's
// separate multiplies and adds are reproduced bit for bit in its order. A pure
// add is (a * 1) + c. Post-ops, std::max / std::min / std::clamp semantics
// (NaN passes): SM_LIGHT = 0 (MAME) 1 max(r, 0), 3 clamp(r, 0, 255);
// SM_LIGHT = 1 (Supermodel's shader, sun clamp on) 1 clamp(r, 0, 1), 2 min(r, 1).
// A word with the tr bit reads broadcast port 0 transposed under SM_LIGHT
// (bank and the entry's low two bits swapped): the normal transform.
//   a  lane register file port A, or a side memory word (T&L inputs,
//      projection queue) read by the caller
//   b  lane register file port B | broadcast port 0
//   c  none (-0) | lane register file port B | broadcast port 1, optionally
//      negated
// The micro-code's constants are broadcast entries (bank 1, R_BC_K..), which
// the controller loads at the start of a frame through the external port.
// DIV: r = b / a, correctly rounded, optionally |r|; RECIP is DIV with b = 1.
// The four lanes go through r3d_recip one a cycle.
//
// The matrix stack is the upper half of each lane's register file block (256
// words, the registers use 128): an operand flagged astk reads word
// {1, sp_a, ra[1:0]}, a result flagged dstk writes {1, sp_d, dst[1:0]}; both
// latched at kernel start.
//
// A word may write both its lane register and the side memory at the same
// address (wrf and wside set): the set-up's copies of its inputs.
//
// Results: lane register, broadcast entry (lane j writes bank j: a matrix
// product lands column j in bank j, immediately readable as a broadcast
// operand), or a side memory (vertex store, projection output) written by the
// caller from sw_*.
//
// Timing (cycles from issue, the cycle the micro-word is out of the ROM):
// register files and side memories are read synchronously (+1), operands are
// registered at +1, product at +5, sum at +11 (the adder's align stage is
// split, PIPE_A2, and its add from its byte normalise, PIPE_B2: study D-50),
// chosen for write-back at +12 and written from a register at the end of +13,
// readable by an instruction issued at +14. DIV lane k is
// written at the end of +15+k. The micro-code generator (sim/real3d/gen_ucode.py) schedules
// around these figures, so the engine has no interlocks. A word with valid = 0
// holds the sequencer for ra more cycles. The external port loads and reads
// the register files while the engine is idle; a read returns the cycle after
// the address.
//
// SUBNORM: 0 (default) flush-to-zero FP units; 1 the subnormal-keeping ones.
// No subnormal reaches the geometry in 800 frames; under Verilator, subn_cnt
// counts every subnormal operand or result, and every flushed result, at a
// unit input or output.

module r3d_vpu
  import r3d_ucode_pkg::*;
#(
  parameter bit SUBNORM  = 1'b0,
  parameter bit SM_LIGHT = 1'b1,
  // measurement (m2-notes section 14): a third register-file read, port C,
  // a third block RAM copy per lane; csel = 3 takes c from it at the bc1
  // field's address
  parameter bit SETUP_3R = 1'b0,
  // the broadcast file in block RAM (8 x 32 x 32, read registered either way)
  parameter bit BC_M10K  = 1'b0,
  // the ROM with the set-up kernels (r3d_ucode_su: the default ROM's kernels at
  // the same addresses, then SETUP4 and SETUP2; r3d_geo SETUP_OUT)
  parameter bit SU_ROM   = 1'b0,
  // the adders' shifts in DSP blocks (r3d_fp_add DSP_SHIFT, study R98)
  parameter bit FP_DSP   = 1'b0
) (
  input  logic        clk,
  input  logic        rst,

  input  logic        start,
  input  logic [9:0]  entry,
  input  logic [4:0]  sp_a,
  input  logic [4:0]  sp_d,
  output logic        busy,

  // external access, idle only
  input  logic [3:0]  ext_we,
  input  logic [7:0]  ext_waddr,
  input  logic [31:0] ext_wdata [4],
  input  logic        ext_bwe,
  input  logic [1:0]  ext_bbank,
  input  logic [4:0]  ext_baddr,
  input  logic [31:0] ext_bdata,
  input  logic [6:0]  ext_raddr,
  output logic [31:0] ext_rdata [4],

  // side memories: read address in the issue cycle, data the cycle after
  output logic [6:0]  sa_addr,
  input  logic [31:0] sa_data [4],
  output logic [3:0]  sw_we,
  output logic [6:0]  sw_addr,
  output logic [31:0] sw_data [4]
);

  // ---- micro-instruction ---------------------------------------------------
  logic [UW-1:0] uw;
  logic [9:0]  pc;
  logic        running, fetch;
  logic [6:0]  wcnt;

  // explicit port connections: with implicit ones (.clk) Quartus 17 wants
  // r3d_ucode_su declared even when SU_ROM = 0, and the default build's file
  // lists do not carry it
  generate if (SU_ROM) begin : g_rom_su
    r3d_ucode_su u_rom (.clk(clk), .addr(pc), .q(uw));
  end else begin : g_rom
    r3d_ucode u_rom (.clk(clk), .addr(pc), .q(uw));
  end endgenerate

  function automatic logic [31:0] fld(input logic [UW-1:0] w, input int lo, input int n);
    fld = 32'((w >> lo) & ((UW'(1) << n) - UW'(1)));
  endfunction

  logic       u_valid, u_end, u_div, u_absr, u_aside, u_negc, u_wrf, u_wbc, u_wside, u_astk, u_dstk, u_tr;
  logic [4:0] spa_q, spd_q;
  logic [3:0] u_lanes;
  logic [6:0] u_ra, u_rb, u_dst, u_bc0, u_bc1;
  logic       u_bsel;
  logic [1:0] u_csel, u_post;
  logic [4:0] u_dstb;
  logic       live;
  assign live    = running && !fetch && wcnt == 7'd0;
  assign u_valid = uw[U_VALID] && live;
  assign u_end   = !uw[U_VALID] && uw[U_RB] && live;   // the kernel's last word
  assign u_div   = uw[U_DIV];
  assign u_absr  = uw[U_ABSR];
  assign u_lanes = 4'(fld(uw, U_LANES, UW_LANES));
  assign u_aside = uw[U_ASIDE];
  assign u_ra    = 7'(fld(uw, U_RA, UW_RA));
  assign u_rb    = 7'(fld(uw, U_RB, UW_RB));
  assign u_bsel  = uw[U_BSEL];
  assign u_csel  = 2'(fld(uw, U_CSEL, UW_CSEL));
  assign u_negc  = uw[U_NEGC];
  assign u_tr    = uw[U_TR];
  assign u_bc0   = (SM_LIGHT && u_tr) ? {uw[U_BC0 +: 2], uw[U_BC0 + 2 +: 3], uw[U_BC0 + 5 +: 2]} : 7'(fld(uw, U_BC0, UW_BC0));
  assign u_bc1   = 7'(fld(uw, U_BC1, UW_BC1));
  assign u_post  = 2'(fld(uw, U_POST, UW_POST));
  assign u_dst   = 7'(fld(uw, U_DST, UW_DST));
  assign u_wrf   = uw[U_WRF];
  assign u_wbc   = uw[U_WBC];
  assign u_dstb  = 5'(fld(uw, U_DSTB, UW_DSTB));
  assign u_wside = uw[U_WSIDE];
  assign u_astk  = uw[U_ASTK];
  assign u_dstk  = uw[U_DSTK];

  // program counter: a wait word (valid = 0, ra = n) holds it n more cycles
  always_ff @(posedge clk) begin
    if (rst) begin
      running <= 1'b0; fetch <= 1'b0; pc <= '0; wcnt <= '0;
    end else if (!running) begin
      wcnt <= '0;
      if (start) begin
        running <= 1'b1;
        fetch <= 1'b1;          // first word comes out of the ROM next cycle
        pc <= entry;
        spa_q <= sp_a; spd_q <= sp_d;
      end
    end else if (fetch) begin
      fetch <= 1'b0;
      pc <= pc + 10'd1;
    end else if (wcnt != 7'd0) begin
      wcnt <= wcnt - 7'd1;
      if (wcnt == 7'd1) pc <= pc + 10'd1;
    end else begin
      if (u_end) running <= 1'b0;
      else if (!uw[U_VALID] && u_ra != 7'd0) wcnt <= u_ra;
      else pc <= pc + 10'd1;
    end
  end
  assign busy = running | start;

  // ---- issue-cycle fields registered for the read cycle (+1) ---------------
  logic       r_aside, r_negc, r_bsel;
  logic [1:0] r_csel;
  logic [1:0] r_bk0, r_bk1;
  always_ff @(posedge clk) begin
    r_aside <= u_aside; r_negc <= u_negc; r_bsel <= u_bsel; r_csel <= u_csel;
    r_bk0 <= u_bc0[6:5]; r_bk1 <= u_bc1[6:5];
  end
  assign sa_addr = u_ra;

  // ---- broadcast banks (lane j writes bank j), read synchronously ----------
  logic [31:0] bc_wdata [4];
  logic [3:0]  bc_we;
  logic [4:0]  bc_waddr [4];
  logic [31:0] bc0_q [4], bc1_q [4];

  genvar j, l;
  generate
  for (j = 0; j < 4; j++) begin : g_bank
    r3d_ram #(.W(32), .A(5), .SYNC(1'b1), .BLOCK(BC_M10K)) u_m0 (.clk, .we(bc_we[j]), .wa(bc_waddr[j]), .wd(bc_wdata[j]), .ra(u_bc0[4:0]), .rq(bc0_q[j]));
    r3d_ram #(.W(32), .A(5), .SYNC(1'b1), .BLOCK(BC_M10K)) u_m1 (.clk, .we(bc_we[j]), .wa(bc_waddr[j]), .wd(bc_wdata[j]), .ra(u_bc1[4:0]), .rq(bc1_q[j]));
  end
  endgenerate
  logic [31:0] bc0_v, bc1_v;
  assign bc0_v = bc0_q[r_bk0];
  assign bc1_v = bc1_q[r_bk1];

  // ---- control pipeline to write-back (+12) --------------------------------
  typedef struct packed {
    logic       v;
    logic       div;
    logic       absr;
    logic [3:0] lanes;
    logic [1:0] post;
    logic [7:0] dst;
    logic       wrf, wbc, wside;
    logic [4:0] dstb;
  } wctl_t;
  wctl_t wq [1:12];
  always_ff @(posedge clk) begin
    wq[1].v     <= u_valid;
    wq[1].div   <= u_div;
    wq[1].absr  <= u_absr;
    wq[1].lanes <= u_lanes;
    wq[1].post  <= u_post;
    wq[1].dst   <= u_dstk ? {1'b1, spd_q, u_dst[1:0]} : {1'b0, u_dst};
    wq[1].wrf   <= u_wrf;
    wq[1].wbc   <= u_wbc;
    wq[1].wside <= u_wside;
    wq[1].dstb  <= u_dstb;
    for (int q = 2; q <= 12; q++) wq[q] <= wq[q-1];
  end
  logic mac10;
  assign mac10 = wq[12].v && !wq[12].div;

  // ---- divide / reciprocal: four lanes through one unit ---------------------
  logic [31:0] opA [4], opB [4];
  logic [31:0] dq_x [1:3], dq_a [1:3];
  logic [3:0]  dq_v;
  logic        div_go;
  logic [1:0]  dlane;
  logic [31:0] d_x, d_a;
  logic        d_v;
  logic [31:0] d_out;
  logic        d_ov, d_tiny;
  // Beside r3d_recip (latency 12) travel only {valid, lane}; a DIV's
  // destination waits in a 4-entry ring, written when it issues and retired
  // after its lane 3 comes out. The schedule keeps DIVs four cycles apart, so
  // at most four are in flight.
  typedef struct packed {
    logic [3:0] lanes;
    logic       absr, wrf, wbc;
    logic [7:0] dst;
    logic [4:0] dstb;
  } dtag_t;
  dtag_t      dring [4];
  logic [1:0] dwp, drp;
  logic       dv [0:11];
  logic [1:0] dl [0:11];
  dtag_t      dcur;              // the ring entry at the write-back, registered

  assign div_go = wq[2].v && wq[2].div;           // operands registered: +2
  always_ff @(posedge clk) begin
    if (rst) begin
      dq_v <= '0;
    end else if (div_go) begin
      for (int k = 1; k < 4; k++) begin dq_x[k] <= opA[k]; dq_a[k] <= opB[k]; end
      dq_v <= 4'b1110;
      dlane <= 2'd1;
    end else if (dq_v != '0) begin
      dq_v <= {1'b0, dq_v[3:1]};
      dlane <= dlane + 2'd1;
      // lanes 2 and 3 move down, so the unit's input is always dq_[1] (timing, study R104)
      dq_x[1] <= dq_x[2]; dq_a[1] <= dq_a[2];
      dq_x[2] <= dq_x[3]; dq_a[2] <= dq_a[3];
    end
  end
  always_ff @(posedge clk) begin
    if (rst) begin
      dwp <= '0; drp <= '0;
    end else begin
      if (div_go) begin
        dring[dwp].lanes <= wq[2].lanes; dring[dwp].absr <= wq[2].absr; dring[dwp].wrf <= wq[2].wrf;
        dring[dwp].wbc <= wq[2].wbc; dring[dwp].dst <= wq[2].dst; dring[dwp].dstb <= wq[2].dstb;
        dwp <= dwp + 2'd1;
      end
      if (dv[11] && dl[11] == 2'd3) drp <= drp + 2'd1;
    end
  end
  // the entry is written twelve cycles or more before it is needed and only
  // changes when the last lane retires, so it can be read a cycle early
  always_ff @(posedge clk)
    dcur <= dring[(dv[11] && dl[11] == 2'd3) ? drp + 2'd1 : drp];

  always_comb begin
    if (div_go) begin d_v = 1'b1; d_x = opA[0]; d_a = opB[0]; end
    else begin d_v = dq_v[1]; d_x = dq_x[1]; d_a = dq_a[1]; end
  end

  r3d_recip i_div (.clk, .in_valid(d_v), .div(1'b1), .a(d_a), .x(d_x), .out_valid(d_ov), .result(d_out), .tiny(d_tiny));

  always_ff @(posedge clk) begin
    dv[0] <= d_v && !rst;
    dl[0] <= div_go ? 2'd0 : dlane;
    for (int q = 1; q <= 11; q++) begin dv[q] <= dv[q-1] && !rst; dl[q] <= dl[q-1]; end
  end
  // the result of the value entered at cycle n is out at n+12, beside dv[11]
  logic [31:0] d_res;
  assign d_res = dcur.absr ? {1'b0, d_out[30:0]} : d_out;

  // ---- lanes -----------------------------------------------------------------
  logic [7:0] ra_mux;
  assign ra_mux = running ? (u_astk ? {1'b1, spa_q, u_ra[1:0]} : {1'b0, u_ra}) : {1'b0, ext_raddr};

`ifdef VERILATOR
  logic [31:0] subn_cnt /* verilator public_flat_rd */;
  logic [3:0]  sn_ev;
  function automatic logic issub(input logic [31:0] x);
    issub = (x[30:23] == 8'd0) && (x[22:0] != 23'd0);
  endfunction
`endif

  generate
  for (l = 0; l < 4; l++) begin : g_lane
    logic        we;
    logic [7:0]  wa;
    logic [31:0] wd;
    logic [31:0] a_q, b_q;
    // register file: two copies, one per read port, in block RAM
    r3d_ram #(.W(32), .A(8), .SYNC(1'b1), .BLOCK(1'b1)) u_rfa (.clk, .we, .wa, .wd, .ra(ra_mux), .rq(a_q));
    r3d_ram #(.W(32), .A(8), .SYNC(1'b1), .BLOCK(1'b1)) u_rfb (.clk, .we, .wa, .wd, .ra({1'b0, u_rb}), .rq(b_q));
    logic [31:0] c_q;
    if (SETUP_3R) begin : g_rfc
      r3d_ram #(.W(32), .A(8), .SYNC(1'b1), .BLOCK(1'b1)) u_rfc (.clk, .we, .wa, .wd, .ra({1'b0, u_bc1}), .rq(c_q));
    end else begin : g_norfc
      assign c_q = 32'd0;
    end
    assign ext_rdata[l] = a_q;

    // operands, registered at +1
    logic [31:0] b_op, c_op;
    always_ff @(posedge clk) begin
      opA[l] <= r_aside ? sa_data[l] : a_q;
      b_op <= r_bsel ? bc0_v : b_q;
      case (r_csel)
        2'd0: c_op <= 32'h8000_0000;                     // -0: a pure product
        2'd1: c_op <= {b_q[31] ^ r_negc, b_q[30:0]};
        2'd3: c_op <= SETUP_3R ? {c_q[31] ^ r_negc, c_q[30:0]} : {bc1_v[31] ^ r_negc, bc1_v[30:0]};
        default: c_op <= {bc1_v[31] ^ r_negc, bc1_v[30:0]};
      endcase
    end
    assign opB[l] = b_op;

    logic [31:0] prod, c_d [1:4];
    logic        pv, p_unf, s_unf;
    always_ff @(posedge clk) begin
      c_d[1] <= c_op;
      for (int q = 2; q <= 4; q++) c_d[q] <= c_d[q-1];
    end
    r3d_fp_mul #(.SUBNORM(SUBNORM)) u_mul (
      .clk, .rst_n(!rst), .in_valid(1'b1), .a(opA[l]), .b(b_op),
      .out_valid(pv), .result(prod), .overflow(), .underflow(p_unf), .invalid());
    logic [31:0] sum;
    logic        sv;
    r3d_fp_add #(.SUBNORM(SUBNORM), .PIPE_A2(1'b1), .DSP_SHIFT(FP_DSP), .NORM_B(1'b1), .PIPE_B2(1'b1)) u_add (
      .clk, .rst_n(!rst), .in_valid(1'b1), .a(prod), .b(c_d[4]), .sub(1'b0),
      .out_valid(sv), .result(sum), .overflow(), .underflow(s_unf), .invalid());

    // post-op at +12: which constant replaces the sum, pz +0 or pk the clamp
    // (1.0 Supermodel, 255 MAME). Only the flags are decided at +12; they are
    // registered beside the raw result and applied on the way into each
    // memory at +13, the cycle that otherwise only routes the register to the
    // memory's input (the clamp's compare and its 32-bit select in front of
    // the write-back register missed 10 ns at -40C, R73's lateack_s3)
    logic        pz, pk;
    logic        neg, isnan, iszero;
    localparam logic [31:0] PK = SM_LIGHT ? 32'h3f80_0000 : 32'h437f_0000;
    always_comb begin
      isnan  = (sum[30:23] == 8'hff) && (sum[22:0] != 23'd0);
      iszero = (sum[30:0] == 31'd0);
      neg    = sum[31] && !isnan && !iszero;
      pz = 1'b0; pk = 1'b0;
      if (SM_LIGHT) begin
        if (wq[12].post == 2'd1 && neg) pz = 1'b1;
        else if ((wq[12].post == 2'd1 || wq[12].post == 2'd2) && !sum[31] && !isnan && sum[30:0] > 31'h3f80_0000)
          pk = 1'b1;
      end else begin
        if ((wq[12].post == 2'd1 || wq[12].post == 2'd3) && neg) pz = 1'b1;
        else if (wq[12].post == 2'd3 && !sum[31] && !isnan && sum[30:0] > 31'h437f_0000) pk = 1'b1;
      end
    end

    logic div_wb;
    assign div_wb = dv[11] && d_ov && dl[11] == l && dcur.lanes[l];

    // write-back: the result, the divide's or the external port's, chosen at
    // +12 and registered with its post-op flags; each memory is written at
    // +13 from the register, the flags' constant put in on the way
    logic        we_n, bwe_n;
    logic [7:0]  wa_n;
    logic [4:0]  bwa_n;
    logic [31:0] wd_n, bwd_n;
    // the lane's result, the MAC's or the divide's (the schedule never lets
    // both write in one cycle), chosen once for both files
    logic        mac_l;
    logic [31:0] rr;
    assign mac_l = mac10 && wq[12].lanes[l];
    assign rr = mac_l ? sum : d_res;
    always_comb begin
      we_n = 1'b0; wa_n = ext_waddr; wd_n = ext_wdata[l];
      if (mac_l && wq[12].wrf) begin
        we_n = 1'b1; wa_n = wq[12].dst; wd_n = rr;
      end else if (div_wb && dcur.wrf) begin
        we_n = 1'b1; wa_n = dcur.dst; wd_n = rr;
      end else if (!running && ext_we[l]) begin
        we_n = 1'b1;
      end
      bwe_n = 1'b0; bwa_n = ext_baddr; bwd_n = ext_bdata;
      if (mac_l && wq[12].wbc) begin
        bwe_n = 1'b1; bwa_n = wq[12].dstb; bwd_n = rr;
      end else if (div_wb && dcur.wbc) begin
        bwe_n = 1'b1; bwa_n = dcur.dstb; bwd_n = rr;
      end else if (!running && ext_bwe && ext_bbank == l) begin
        bwe_n = 1'b1;
      end
    end
    logic [31:0] wd_r, bwd_r, swd_r;
    logic        wpz, wpk, spz, spk;
    always_ff @(posedge clk) begin
      we <= we_n && !rst; wa <= wa_n; wd_r <= wd_n;
      wpz <= mac_l && wq[12].wrf && pz; wpk <= mac_l && wq[12].wrf && pk;
      bc_we[l] <= bwe_n && !rst; bc_waddr[l] <= bwa_n; bwd_r <= bwd_n;
      sw_we[l] <= mac10 && wq[12].wside && wq[12].lanes[l] && !rst;
      swd_r <= sum; spz <= pz; spk <= pk;
    end
    // the micro-code puts post-op results only in the lane registers, and, with
    // MAME's rows, the shade clamp (post 3) into a side memory; never into the
    // broadcast file (gen_ucode.py asserts it), so only those carry the select
    assign wd = wpz ? 32'd0 : wpk ? PK : wd_r;
    assign bc_wdata[l] = bwd_r;
    assign sw_data[l] = SM_LIGHT ? swd_r : spz ? 32'd0 : spk ? PK : swd_r;
`ifdef VERILATOR
    always_ff @(posedge clk)
      if (mac10 && wq[12].lanes[l] && (pz || pk) && (wq[12].wbc || (SM_LIGHT && wq[12].wside)))
        $error("r3d_vpu: a post-op result into a memory without the post-op select");
`endif

`ifdef VERILATOR
    // subnormal monitor: operands at +2 (multiplier and divider inputs), the
    // product and the addend at +6, the sum at +12; flushed results by flag
    always_comb begin
      sn_ev[l] = 1'b0;
      if (wq[2].v && wq[2].lanes[l] && (issub(opA[l]) || issub(b_op))) sn_ev[l] = 1'b1;
      if (wq[6].v && !wq[6].div && wq[6].lanes[l] && (issub(prod) || issub(c_d[4]) || p_unf)) sn_ev[l] = 1'b1;
      if (mac10 && wq[12].lanes[l] && (issub(sum) || s_unf)) sn_ev[l] = 1'b1;
    end
`endif
  end
  endgenerate
  always_ff @(posedge clk) sw_addr <= wq[12].dst[6:0];

`ifdef VERILATOR
  // pool occupancy: instructions issued, and lane operations (issue x lanes)
  logic [31:0] iss_cnt /* verilator public_flat_rd */;
  logic [31:0] lop_cnt /* verilator public_flat_rd */;
  // study R99: MAC words using lane 2 or 3 (a second pass on a two-lane pool)
  logic [31:0] hi_cnt /* verilator public_flat_rd */;
  always_ff @(posedge clk) begin
    if (rst) hi_cnt <= '0;
    else if (u_valid && !u_div && (u_lanes[3:2] != 2'b00)) hi_cnt <= hi_cnt + 32'd1;
  end
  always_ff @(posedge clk) begin
    if (rst) begin iss_cnt <= '0; lop_cnt <= '0; end
    else if (u_valid) begin
      iss_cnt <= iss_cnt + 32'd1;
      lop_cnt <= lop_cnt + 32'(u_lanes[0]) + 32'(u_lanes[1]) + 32'(u_lanes[2]) + 32'(u_lanes[3]);
    end
  end
  always_ff @(posedge clk) begin
    if (rst) subn_cnt <= '0;
    else subn_cnt <= subn_cnt + 32'(sn_ev[0]) + 32'(sn_ev[1]) + 32'(sn_ev[2]) + 32'(sn_ev[3])
                   + 32'(dv[11] && dcur.lanes[dl[11]] && d_tiny);
  end
`endif

endmodule
