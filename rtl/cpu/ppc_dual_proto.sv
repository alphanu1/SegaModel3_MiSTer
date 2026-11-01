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
// Dual-issue prototype (study D-24): the integer datapath skeleton of
// ppc_core.sv, ID to WB, with one issue slot (DUAL = 0, the reference) or two,
// to measure in a standalone fit what a second slot adds in area and delay.
// Not a core: decode, hazards, special registers and memory are outside;
// each slot's decoded uop comes in from registers, as the core's predecode
// and decoder give it, and the load word comes in at WB.
//
// What is the core's (ppc_core.sv), kept as built:
//   - operands x (rA or rS), y (rB or immediate), a (rlwimi's rA), s (rS)
//     captured as the uop enters EX; EX starts from registers;
//   - the EX producer one step ahead through the last mux before the operand
//     register; MEM and WB producers (WB: a load's live word, else the
//     result) through a 4:1 with the register file or immediate as the base;
//   - the selects from register-field compares made a cycle early and
//     registered (here the next uop's fields against the producers it will
//     meet, without the core's hold muxes);
//   - GPRs written as a uop leaves WB; the ALU is ppc_alu.sv.
//
// Slot B (DUAL = 1):
//   BOPS  2: x, y (a simple integer op); 4: x, y, a, s (any unit's operands)
//   BW2   1: B's result has its own MEM / WB registers and GPR write port
//            (two writes a cycle); 0: a pair writes one GPR at most, so B's
//            result takes the slot's single result path (one write port)
//   BLD   1: B may be a load (its WB word is live as A's is)
//   BCR   1: B writes CR0 (Rc) and XER[CA]: merged with A's at MEM in program
//            order, as the core merges CR writes into m_cr
//   PMUX  1: three GPR read ports shared by the pair (ppc_regfile2w.sv)
// Every forward has both slots' producers as sources. Within a pair the
// second op never reads the first's result (the pairing rule), and the two
// never write one register.

module ppc_dual_proto
  import ppc_pkg::*;
#(
  parameter int DUAL = 1,
  parameter int BOPS = 2,
  parameter int BW2  = 1,
  parameter int BLD  = 0,
  parameter int BCR  = 1,
  parameter int PMUX = 0
)(
  input  logic        clk,
  // the uop each slot's ID holds next cycle (from the queue and predecode)
  input  logic [1:0]        n_v, n_rdw, n_ld, n_xs, n_yreg, n_rc,
  input  logic [1:0][4:0]   n_rd, n_ra, n_rb, n_rs,
  input  logic [1:0][31:0]  n_imm,
  input  logic [1:0][5:0]   n_op,
  input  logic [1:0][6:0]   n_sel,
  input  logic [1:0][3:0]   n_ltt,
  input  logic [1:0][31:0]  n_mask,
  input  logic [2:0][2:0]   n_pmap,       // PMUX: the read ports' fields
  input  logic [3:0][1:0]   n_pidx,       // PMUX: the port each of A's x, y and B's x, y reads
  input  logic              b_older,      // B's op is the older of the pair
  input  logic [31:0]       ld_word,      // the load word arriving in WB (slot A, and B with BLD)
  input  logic [31:0]       ld_word_b,
  output logic [31:0]       wb_a, wb_b,   // results leaving WB
  output logic [3:0]        cr0_out,
  output logic              ca_out
);

  localparam int NS  = (DUAL != 0) ? 2 : 1;
  localparam int NW  = (DUAL != 0 && BW2 != 0) ? 2 : 1;
  localparam int NRB = (DUAL == 0) ? 0 : (PMUX != 0) ? 0 : BOPS == 4 ? 3 : 2;
  localparam int NR  = 3 + NRB;
  localparam int I3  = (NR > 3) ? 3 : 0;   // indexes that exist only with B's ports
  localparam int I4  = (NR > 4) ? 4 : 0;
  localparam int I5  = (NR > 5) ? 5 : 0;

  // ---------------------------------------------------------------- ID
  logic [1:0]       id_v, id_rdw, id_ld, id_xs, id_yreg, id_rc;
  logic [1:0][4:0]  id_rd, id_ra, id_rb, id_rs;
  logic [1:0][31:0] id_imm;
  logic [1:0][5:0]  id_op;
  logic [1:0][6:0]  id_sel;
  logic [1:0][3:0]  id_ltt;
  logic [1:0][31:0] id_mask;
  logic [2:0][2:0]  id_pmap;
  logic [3:0][1:0]  id_pidx;
  logic             id_bo;
  always_ff @(posedge clk) begin
    id_v <= n_v; id_rdw <= n_rdw; id_ld <= n_ld; id_xs <= n_xs; id_yreg <= n_yreg; id_rc <= n_rc;
    id_rd <= n_rd; id_ra <= n_ra; id_rb <= n_rb; id_rs <= n_rs; id_imm <= n_imm; id_op <= n_op;
    id_sel <= n_sel; id_ltt <= n_ltt; id_mask <= n_mask; id_pmap <= n_pmap; id_pidx <= n_pidx; id_bo <= b_older;
  end

  // ---------------------------------------------------------------- stage registers
  logic [1:0]       ex_v, ex_rdw, ex_ld, ex_rc;
  logic [1:0][4:0]  ex_rd;
  logic [1:0][5:0]  ex_op;
  logic [1:0][6:0]  ex_sel;
  logic [1:0][3:0]  ex_ltt;
  logic [1:0][31:0] ex_mask;
  logic             ex_bo;
  logic [1:0][31:0] op_x, op_y, op_a, op_s;
  logic [1:0]       m_v, m_rdw, m_ld, m_rc, m_ca;
  logic [1:0][4:0]  m_rd;
  logic [1:0][31:0] m_res;
  logic             m_bo;
  logic [1:0]       w_v, w_rdw, w_ld;
  logic [1:0][4:0]  w_rd;
  logic [1:0][31:0] w_res;

  // ---------------------------------------------------------------- register file
  logic [NR-1:0][5:0]  rf_a;
  logic [NR-1:0][31:0] rf_q;
  logic [5:0][5:0]     rf_f;
  always_comb begin
    rf_a = '0;
    rf_a[0] = {1'b0, id_ra[0]}; rf_a[1] = {1'b0, id_rb[0]}; rf_a[2] = {1'b0, id_rs[0]};
    if (NRB >= 2) begin rf_a[I3] = {1'b0, id_xs[1] ? id_rs[1] : id_ra[1]}; rf_a[I4] = {1'b0, id_rb[1]}; end
    if (NRB >= 3) rf_a[I5] = {1'b0, id_rs[1]};
    rf_f = {{1'b0, id_rs[1]}, {1'b0, id_rb[1]}, {1'b0, id_ra[1]}, {1'b0, id_rs[0]}, {1'b0, id_rb[0]}, {1'b0, id_ra[0]}};
  end
  logic [NW-1:0]       rf_we;
  logic [NW-1:0][5:0]  rf_wa;
  logic [NW-1:0][31:0] rf_wd;
  wire  [1:0][31:0]    w_val;                                  // WB's value per slot
  assign w_val[0] = w_ld[0] ? ld_word : w_res[0];
  assign w_val[1] = (BLD != 0 && w_ld[1]) ? ld_word_b : w_res[1];
  always_comb begin
    rf_we[0] = w_v[0] && w_rdw[0]; rf_wa[0] = {1'b0, w_rd[0]}; rf_wd[0] = w_val[0];
    if (NW == 2) begin rf_we[NW-1] = w_v[1] && w_rdw[1]; rf_wa[NW-1] = {1'b0, w_rd[1]}; rf_wd[NW-1] = w_val[1]; end
  end
  ppc_regfile2w #(.NW (NW), .NR (NR), .PMUX (PMUX)) u_rf (
    .clk (clk), .ra (rf_a), .fa (rf_f), .pmap (id_pmap), .q (rf_q), .we (rf_we), .wa (rf_wa), .wd (rf_wd));

  // ---------------------------------------------------------------- forward selects
  // Registered compares: the uop ID holds next cycle (n_*) against the
  // producers it will meet next cycle: ID's uops (then in EX), EX's (MEM),
  // MEM's (WB). [consumer slot][field: 0 x, 1 y, 2 a, 3 s][producer slot]
  logic [1:0][3:0][1:0] fe, fm, fw;
  logic [1:0][3:0][4:0] nf;
  always_comb
    for (int c = 0; c < 2; c++) nf[c] = {n_rs[c], n_ra[c], n_rb[c], n_xs[c] ? n_rs[c] : n_ra[c]};
  always_ff @(posedge clk)
    for (int c = 0; c < NS; c++)
      for (int f = 0; f < 4; f++)
        for (int p = 0; p < NS; p++) begin
          fe[c][f][p] <= id_v[p] && id_rdw[p] && id_rd[p] == nf[c][f] && !(f == 1 && !n_yreg[c]);
          fm[c][f][p] <= ex_v[p] && ex_rdw[p] && !ex_ld[p] && ex_rd[p] == nf[c][f] && !(f == 1 && !n_yreg[c]);
          fw[c][f][p] <= m_v[p] && m_rdw[p] && m_rd[p] == nf[c][f] && !(f == 1 && !n_yreg[c]);
        end

  // ---------------------------------------------------------------- operand capture
  // base: register file (its port) or the immediate; then MEM / WB of either
  // slot; the EX producers come last
  logic [1:0][3:0][31:0] base, cap;
  always_comb begin
    base = '0;
    base[0][0] = id_xs[0] ? rf_q[2] : rf_q[0];
    base[0][1] = id_yreg[0] ? rf_q[1] : id_imm[0];
    base[0][2] = rf_q[0];
    base[0][3] = rf_q[2];
    if (NS == 2) begin
      if (PMUX != 0) begin
        base[0][0] = rf_q[id_pidx[0]];
        base[0][1] = id_yreg[0] ? rf_q[id_pidx[1]] : id_imm[0];
        base[1][0] = rf_q[id_pidx[2]];
        base[1][1] = id_yreg[1] ? rf_q[id_pidx[3]] : id_imm[1];
      end else begin
        base[1][0] = rf_q[I3];
        base[1][1] = id_yreg[1] ? rf_q[I4] : id_imm[1];
        if (NRB >= 3) begin base[1][2] = rf_q[I3]; base[1][3] = rf_q[I5]; end
      end
    end
    for (int c = 0; c < NS; c++)
      for (int f = 0; f < 4; f++) begin
        cap[c][f] = base[c][f];
        // one of these at most: no two producers in a stage write one register
        if (fw[c][f][0]) cap[c][f] = w_val[0];
        if (NS == 2 && fw[c][f][1]) cap[c][f] = (NW == 2) ? w_val[1] : w_val[0];
        if (fm[c][f][0]) cap[c][f] = m_res[0];
        if (NS == 2 && fm[c][f][1]) cap[c][f] = (NW == 2) ? m_res[1] : m_res[0];
      end
  end

  // ---------------------------------------------------------------- EX
  logic [1:0][31:0] ex_res;
  logic [1:0]       ex_ca;
  genvar gs;
  generate
    for (gs = 0; gs < NS; gs++) begin : g_alu
      logic [31:0] r_sum, r_rl, r_rr, r_clz, r_mid;
      logic        ca_o, ov_o;
      logic [3:0]  cmp_o;
      ppc_alu u_alu (
        .op (alu_op_t'(ex_op[gs])), .x (op_x[gs]), .y (op_y[gs]), .a_ins (op_a[gs]), .cin (CIN_ZERO),
        .ca_in (1'b0), .so_in (1'b0), .mask (ex_mask[gs]), .rlw_insert (ex_sel[gs][4] && ex_op[gs][0]),
        .ltt (ex_ltt[gs]), .ext_h (ex_op[gs][1]), .sel (ex_sel[gs]),
        .r_sum (r_sum), .r_rl (r_rl), .r_rr (r_rr), .r_clz (r_clz), .r_mid (r_mid),
        .ca_out (ca_o), .ov_out (ov_o), .cmp_out (cmp_o));
      assign ex_res[gs] = r_sum | r_rl | r_rr | r_clz | r_mid | (op_s[gs] & {32{ex_ld[gs]}});
      assign ex_ca[gs]  = ca_o;
    end
    if (NS == 1) begin : g_one
      assign ex_res[1] = '0;
      assign ex_ca[1]  = 1'b0;
    end
  endgenerate

  // operand registers: the EX producer one step ahead, else the capture
  logic [1:0][3:0][31:0] nxo;
  always_comb
    for (int c = 0; c < 2; c++)
      for (int f = 0; f < 4; f++) begin
        nxo[c][f] = cap[c][f];
        if (fe[c][f][0]) nxo[c][f] = ex_res[0];
        if (NS == 2 && fe[c][f][1]) nxo[c][f] = ex_res[1];
      end
  always_ff @(posedge clk) begin
    op_x[0] <= nxo[0][0]; op_y[0] <= nxo[0][1]; op_a[0] <= nxo[0][2]; op_s[0] <= nxo[0][3];
    if (NS == 2) begin
      op_x[1] <= nxo[1][0]; op_y[1] <= nxo[1][1];
      if (BOPS == 4) begin op_a[1] <= nxo[1][2]; op_s[1] <= nxo[1][3]; end
    end
  end

  always_ff @(posedge clk) begin
    ex_v <= id_v; ex_rdw <= id_rdw; ex_ld <= id_ld; ex_rc <= id_rc; ex_rd <= id_rd; ex_op <= id_op;
    ex_sel <= id_sel; ex_ltt <= id_ltt; ex_mask <= id_mask; ex_bo <= id_bo;
    if (NS == 1) begin ex_v[1] <= 1'b0; ex_rdw[1] <= 1'b0; end
    if (BLD == 0) ex_ld[1] <= 1'b0;
    // MEM: with one result path the pair's GPR result is whichever slot writes
    m_v <= ex_v; m_rdw <= ex_rdw; m_ld <= ex_ld; m_rd <= ex_rd; m_rc <= ex_rc; m_ca <= ex_ca; m_bo <= ex_bo;
    m_res[0] <= ex_res[0];
    m_res[1] <= ex_res[1];
    if (NS == 2 && NW == 1) begin
      m_res[0] <= (ex_v[1] && ex_rdw[1]) ? ex_res[1] : ex_res[0];
      m_rd[0]  <= (ex_v[1] && ex_rdw[1]) ? ex_rd[1]  : ex_rd[0];
      m_rdw[0] <= (ex_v[0] && ex_rdw[0]) || (ex_v[1] && ex_rdw[1]);
      m_v[0]   <= ex_v[0] || ex_v[1];
      m_v[1]   <= 1'b0;
    end
    w_v <= m_v; w_rdw <= m_rdw; w_ld <= m_ld; w_rd <= m_rd; w_res <= m_res;
  end
  assign wb_a = w_val[0];
  assign wb_b = w_val[1];

  // ---------------------------------------------------------------- CR0 / CA at MEM
  // CR0 from each slot's result, merged in program order (the younger wins),
  // as the core forms m_cr0 from MEM's registered result
  logic [3:0] cr0_a, cr0_b;
  always_comb begin
    cr0_a = {m_res[0][31], !m_res[0][31] && m_res[0] != 32'd0, m_res[0] == 32'd0, 1'b0};
    cr0_b = {m_res[1][31], !m_res[1][31] && m_res[1] != 32'd0, m_res[1] == 32'd0, 1'b0};
  end
  logic [3:0] cr0_q;
  logic       ca_q;
  always_ff @(posedge clk) begin
    if (DUAL != 0 && BCR != 0 && m_v[1] && m_rc[1] && (!m_bo || !(m_v[0] && m_rc[0]))) cr0_q <= cr0_b;
    else if (m_v[0] && m_rc[0]) cr0_q <= cr0_a;
    if (DUAL != 0 && BCR != 0 && m_v[1] && (!m_bo || !m_v[0])) ca_q <= m_ca[1];
    else if (m_v[0]) ca_q <= m_ca[0];
  end
  assign cr0_out = cr0_q;
  assign ca_out  = ca_q;

endmodule
