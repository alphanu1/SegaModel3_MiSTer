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
// PowerPC 603e integer pipeline (M0-A2, first slice). Design target: study D-15.
//
//   F   fetch: one request outstanding, two-entry queue
//   ID  decode, crack, register read addresses, load-use hazard
//   EX  ALU / branch / multiply-divide / address; all special registers are
//       computed here from forwarded values
//   MEM data access; special registers commit as the instruction leaves MEM
//   WB  GPR write
//
// Forwarding goes into the operand registers as a uop enters EX (the MEM or WB
// producer's value, else the register file read in ID); EX itself only takes
// MEM's result, for a producer one step ahead. A load is followed by one
// bubble when its consumer is next in line.
//
// Branches are predicted at fetch (static, the 603e's rule) and resolved in EX,
// which redirects only on a mispredict. sc, rfi, mtmsr and illegal
// instructions redirect from EX and commit their special-register writes at
// MEM exit; a fired trap commits and redirects from WB. Every redirect is
// registered before it reaches fetch.
//
// Not yet here: address translation, caches, floating point, external and
// decrementer interrupts, misaligned accesses (forced aligned for now).
//
// Memory ports: a request is accepted on the edge where *_req is high; the
// response (*_valid) comes one or more cycles later. One request outstanding
// per port.

module ppc_core
  import ppc_pkg::*;
#(
  parameter logic [31:0] RESET_PC = 32'hFFF0_0100,
  parameter logic [31:0] PVR      = 32'h0006_0101
)(
  input  logic        clk,
  input  logic        rst,

  output logic        i_req,
  output logic [31:0] i_addr,
  input  logic        i_valid,
  input  logic [31:0] i_data,

  output logic        d_req,
  output logic        d_we,
  output logic [31:0] d_addr,
  output logic [3:0]  d_be,
  output logic [31:0] d_wdata,
  input  logic        d_valid,
  input  logic [31:0] d_rdata,

  input  logic        tb_tick,      // timebase / decrementer enable (bus clock / 4)

  // performance events, one bit each per cycle, for rtl/dbg/m3_cpu_mon.sv:
  //   0 instruction retired   1 uop retired     2 decode starved (fetch)
  //   3 load-use bubble       4 mul/div wait    5 data memory wait
  //   6 redirect (branch, exception)            7 fetch waiting on memory
  output logic [7:0]  ev,

  // retire trace: one record per uop leaving WB, with the special registers it
  // committed. Unconnected in the core build, so it costs nothing there.
  output logic        rt_valid,
  output logic        rt_last,
  output logic [31:0] rt_pc,
  output logic [31:0] rt_insn,
  output logic        rt_gpr_we,
  output logic [4:0]  rt_gpr,
  output logic [31:0] rt_gpr_val,
  output logic [31:0] rt_cr,
  output logic [31:0] rt_xer,
  output logic [31:0] rt_lr,
  output logic [31:0] rt_ctr,
  output logic [31:0] rt_msr,
  output logic [31:0] rt_srr0,
  output logic [31:0] rt_srr1
);

  localparam logic [31:0] MSR_MASK   = 32'h0007_FF73;  // implemented MSR bits
  localparam logic [31:0] SRR1_KEEP  = 32'h0000_FF73;  // MSR bits saved in SRR1
  localparam logic [31:0] XER_MASK   = 32'hE000_007F;  // SO OV CA, byte count
  localparam int          MSR_IP     = 6;
  localparam int          MSR_ILE    = 16;

  // ===================================================================== state
  logic [31:0] cr_q, xer_q, lr_q, ctr_q, msr_q;
  logic        res_q;                                  // lwarx reservation
  logic [63:0] tb_q;
  logic [31:0] dec_q;

  // ============================================================== stage regs
  // EX
  logic        ex_valid;
  uop_t        ex_u;
  logic        ex_md_busy;                             // multiply/divide in flight

  // MEM
  typedef struct packed {
    logic        valid;
    logic [31:0] pc, insn;
    logic        last;
    logic        rd_we;
    logic [4:0]  rd;
    logic [31:0] result;
    logic        is_load, req;
    logic [1:0]  msize, boff;
    logic        msigned, brev;
    logic        cr_we, xer_we, lr_we, ctr_we, msr_we, spr_we, tb_we, dec_we;
    logic [31:0] cr, xer, lr, ctr, msr, sprv;
    logic [5:0]  spr;
    logic        tbu;
    logic        exc;                                  // sc / program: SRR0, SRR1
    logic [31:0] srr0, srr1;
    logic        res_set, res_clr;
    logic        rc0;                                  // CR0 from the result, formed in MEM
    logic [31:0] crm;                                  // CR bits this uop writes (cr holds their values)
    logic        cmp_we;                               // cmp / cmpl: the field goes in at MEM
    logic [3:0]  cmpv;
    logic [2:0]  crf;
    logic        is_trap;                              // tw / twi: the compare runs in MEM
    logic [4:0]  to;
    logic [31:0] ta, tb;
  } mem_t;
  mem_t m;

  // WB
  logic        wb_valid, wb_last, wb_rd_we;
  logic [4:0]  wb_rd;
  logic [31:0] wb_result, wb_pc, wb_insn;

  // ==================================================================== fetch
  // Four-entry queue, packed so it stays in flip-flops (Quartus made an indexed
  // array into a RAM block). A request is issued whenever four slots would hold
  // it without counting this cycle's pop, so decode and hazards are off the
  // request path (R23 timing).
  //
  // Branches are predicted here, as each word arrives: b is taken; bc is taken
  // when its condition is always true, otherwise backward-taken with the y bit
  // reversing it (the 603e's static rule); bclr and bcctr are predicted not
  // taken for now. A predicted-taken branch is a soft redirect: fetch moves to
  // its target and anything in flight is discarded, but the queue keeps the
  // branch and what came before it. The prediction travels with the word to EX,
  // which redirects only on a mispredict.
  //
  // Hard redirects (WB: a fired trap; EX: mispredict, sc, rfi, mtmsr, illegal)
  // also clear the queue and hold ID for a cycle. Every redirect is registered
  // (redir_q); the next request takes its address from redir_pc_q, and that
  // request was always going out the cycle after the decision, so the register
  // costs no fetch latency.
  logic [31:0] f_pc;
  logic        f_out, f_discard;
  logic [2:0]  q_cnt;

  logic        redirect;                               // hard, decided this cycle
  logic [31:0] redirect_pc;
  logic        redir_q, redir_hard_q;
  logic [31:0] redir_pc_q;
  // A redirect's address is chosen a cycle late from two registered candidates,
  // so a branch's taken bit lands in one flip-flop, not on 32 (R23 timing).
  logic [31:0] redir_a_q, redir_b_q, redir_s_q;     // hard taken / fall-through / soft
  logic [1:0]  redir_k_q;                            // which: 0 a, 1 b, 2 soft
  logic [31:0] redirect_pc_b;
  logic        redirect_sel;
  logic        id_take;                                // ID consumed the queue head

  wire  rsp     = f_out && i_valid;
  wire  rsp_use = rsp && !f_discard && !redir_q;
  assign redir_pc_q = (redir_k_q == 2'd0) ? redir_a_q : (redir_k_q == 2'd1) ? redir_b_q : redir_s_q;
  wire  [31:0] fetch_pc = redir_q ? redir_pc_q : f_pc;

  // prediction from the arriving word
  logic [31:0] f_rsp_pc;
  wire  [5:0]  pw_op   = i_data[31:26];
  wire  [4:0]  pw_bo   = i_data[25:21];
  wire  [31:0] pw_disp = (pw_op == 6'd18) ? {{6{i_data[25]}}, i_data[25:2], 2'b00}
                                          : {{16{i_data[15]}}, i_data[15:2], 2'b00};
  wire         pw_pred = (pw_op == 6'd18) ||
                         (pw_op == 6'd16 && ((pw_bo[4] && pw_bo[2]) || (i_data[15] ^ pw_bo[0])));
  wire  [31:0] pw_tgt  = (i_data[1] ? 32'd0 : f_rsp_pc) + pw_disp;
  wire         f_soft  = rsp_use && pw_pred;

  assign i_addr = fetch_pc;
  assign i_req  = !rst && (!f_out || rsp) && (redir_hard_q || ({1'b0, q_cnt} + {3'd0, rsp_use} < 4'd4));

  always_ff @(posedge clk) begin
    if (rst) begin
      f_pc         <= RESET_PC;
      f_out        <= 1'b0;
      f_discard    <= 1'b0;
      redir_q      <= 1'b0;
      redir_hard_q <= 1'b0;
    end else begin
      redir_q      <= redirect || f_soft;
      redir_hard_q <= redirect;
      // the candidates load every cycle; only the 2-bit choice waits on the
      // redirect decision (R23 timing)
      redir_a_q    <= redirect_pc;
      redir_b_q    <= redirect_pc_b;
      redir_s_q    <= pw_tgt;
      redir_k_q    <= !redirect ? 2'd2 : redirect_sel ? 2'd0 : 2'd1;
      if (i_req)        f_pc <= fetch_pc + 32'd4;
      else if (redir_q) f_pc <= redir_pc_q;
      if (redir_q) begin
        f_out     <= i_req || (f_out && !i_valid);
        f_discard <= f_out && !i_valid;              // an older request still in flight
      end else begin
        if (rsp) begin
          f_out     <= 1'b0;
          f_discard <= 1'b0;
        end
        if (i_req) f_out <= 1'b1;
      end
    end
  end

  // Queue writes go to the slot after the live entries, counted before any pop.
  logic [31:0] q_hi, q_hp;
  logic        q_hd;
  always_ff @(posedge clk) if (i_req) f_rsp_pc <= fetch_pc;

  // Predecode as the word arrives, so the queue's pop needs no decoder (R23
  // timing): which of rA, rB, rS the first uop reads, and how it cracks
  // (0 single uop, 1 update form, 2 lmw / stmw). A second decoder instance;
  // all but these few outputs are trimmed.
  uop_t        pd_u;
  logic [4:0]  pd_w, q_pd;
  ppc_decode u_pdec (.insn (i_data), .pc (32'd0), .step (5'd0), .u (pd_u));
  wire  [5:0]  pd_op = i_data[31:26];
  wire  [9:0]  pd_x  = i_data[10:1];
  wire  [1:0]  pd_crack =
          (pd_op == 6'd46 || pd_op == 6'd47) ? 2'd2 :
          (pd_op == 6'd33 || pd_op == 6'd35 || pd_op == 6'd41 || pd_op == 6'd43 ||
           (pd_op == 6'd31 && (pd_x == 10'd55 || pd_x == 10'd119 || pd_x == 10'd311 || pd_x == 10'd375))) ? 2'd1 : 2'd0;
  assign pd_w = {pd_crack, pd_u.a_used, pd_u.b_used, pd_u.s_used};

  ppc_fetchq u_q (
    .clk (clk), .clear (rst || redir_hard_q), .we (rsp_use && !redir_hard_q),
    .w_insn (i_data), .w_pc (f_rsp_pc), .w_pred (pw_pred), .w_pd (pd_w),
    .pop (id_take && !redir_hard_q), .count (q_cnt),
    .r_insn (q_hi), .r_pc (q_hp), .r_pred (q_hd), .r_pd (q_pd)
  );

  // ==================================================================== decode
  logic [4:0]  id_step;
  uop_t        id_u;
  wire         id_valid = (q_cnt != 3'd0) && !redir_hard_q;
  wire         id_pred  = q_hd;
  logic        ex_pred;

  ppc_decode u_dec (
    .insn (q_hi),
    .pc   (q_hp),
    .step (id_step),
    .u    (id_u)
  );

  // load-use: the load in EX writes a register this uop reads. From the
  // predecode and the raw fields, not the decoder; a cracked instruction
  // (update forms, lmw, stmw) waits out any load ahead of it.
  logic ex_ld, ex_md;                                  // EX is a load / multiply-divide (registered in ID)
  wire ex_is_load = ex_valid && ex_ld;
  wire load_use   = ex_is_load && ex_u.rd_we && (
                      (q_pd[4:3] != 2'd0) ||
                      (q_pd[2] && q_hi[20:16] == ex_u.rd) ||
                      (q_pd[1] && q_hi[15:11] == ex_u.rd) ||
                      (q_pd[0] && q_hi[25:21] == ex_u.rd));

  // the uop that finishes the instruction, from the predecode and the step
  wire id_last = (q_pd[4:3] == 2'd0) ||
                 (q_pd[4:3] == 2'd1 && id_step == 5'd1) ||
                 (q_pd[4:3] == 2'd2 && (q_hi[25:21] + id_step) == 5'd31);

  logic ex_hold, mem_hold, mem_adv;
  wire  id_adv = id_valid && !ex_hold && !load_use && !redirect;
  // the pop does not wait on a redirect: a redirect clears the queue next cycle
  assign id_take = id_valid && !ex_hold && !load_use && id_last;

  always_ff @(posedge clk) begin
    if (rst || redirect || redir_hard_q) id_step <= 5'd0;
    else if (id_adv)     id_step <= id_u.last ? 5'd0 : id_step + 5'd1;
  end

  // result group, logical truth table, shift and misc kinds, from decode
  logic [6:0] id_rsel;
  logic [3:0] id_ltt;
  logic [1:0] id_shk, id_mk;
  always_comb begin
    id_rsel = 7'd0;
    id_ltt  = 4'b0000;
    id_shk  = 2'd0;
    id_mk   = 2'd0;
    case (id_u.op)
      OP_AND:  begin id_rsel[1] = 1'b1; id_ltt = 4'b1000; end
      OP_OR:   begin id_rsel[1] = 1'b1; id_ltt = 4'b1110; end
      OP_XOR:  begin id_rsel[1] = 1'b1; id_ltt = 4'b0110; end
      OP_NAND: begin id_rsel[1] = 1'b1; id_ltt = 4'b0111; end
      OP_NOR:  begin id_rsel[1] = 1'b1; id_ltt = 4'b0001; end
      OP_EQV:  begin id_rsel[1] = 1'b1; id_ltt = 4'b1001; end
      OP_ANDC: begin id_rsel[1] = 1'b1; id_ltt = 4'b0100; end
      OP_ORC:  begin id_rsel[1] = 1'b1; id_ltt = 4'b1101; end
      OP_SLW:  begin id_rsel[2] = 1'b1; id_shk = 2'd0; end
      OP_SRW:  begin id_rsel[2] = 1'b1; id_shk = 2'd1; end
      OP_SRAW: begin id_rsel[2] = 1'b1; id_shk = 2'd2; end
      OP_RLW:  id_rsel[3] = 1'b1;
      OP_CNTLZW: begin id_rsel[4] = 1'b1; id_mk = 2'd0; end
      OP_EXTSB:  begin id_rsel[4] = 1'b1; id_mk = 2'd1; end
      OP_EXTSH:  begin id_rsel[4] = 1'b1; id_mk = 2'd2; end
      OP_MUL, OP_MULH, OP_MULHU, OP_DIV, OP_DIVU: id_rsel[5] = 1'b1;
      OP_MFSPR, OP_MFCR, OP_MFMSR:               id_rsel[6] = 1'b1;
      default: id_rsel[0] = 1'b1;                      // sum: add, subtract, EA
    endcase
  end
  logic [6:0] ex_rsel;
  logic [3:0] ex_ltt;
  logic [1:0] ex_shk, ex_mk;

  // the special registers this uop writes, from its decode alone
  wire id_w_cr  = id_u.rc || id_u.op == OP_CMP || id_u.op == OP_CMPL || id_u.op == OP_CRLOG ||
                  id_u.op == OP_MTCRF || id_u.op == OP_MCRXR || (id_u.op == OP_STORE && id_u.cond);
  wire id_w_xer = id_u.set_ca || id_u.oe || id_u.op == OP_MCRXR || (id_u.op == OP_MTSPR && id_u.spr == 10'd1);
  wire id_w_lr  = (id_u.op == OP_BRANCH && id_u.lk) || (id_u.op == OP_MTSPR && id_u.spr == 10'd8);
  wire id_w_ctr = (id_u.op == OP_BRANCH && id_u.bkind != BR_B && id_u.bkind != BR_BCCTR && !id_u.bo[2]) ||
                  (id_u.op == OP_MTSPR && id_u.spr == 10'd9);

  // ================================================================ registers
  logic [31:0] ld_val;                                 // load data, formed in MEM

  logic [31:0] rf_ra, rf_rb, rf_rs;                   // read in ID

  ppc_regfile u_rf (
    .clk (clk),
    .ra  (id_u.ra), .rb (id_u.rb), .rs (id_u.rs),
    .qa  (rf_ra),   .qb (rf_rb),   .qs (rf_rs),
    .we  (wb_valid && wb_rd_we),
    .wa  (wb_rd),
    .wd  (wb_result)
  );

  // Forwarding into the operand registers. As a uop moves from ID to EX each
  // operand register captures its final value: the MEM producer's result (or
  // its load data), the WB producer's result, else the register file, which ID
  // has already read; immediates, zero and -1 are captured the same way, and
  // the ALU's first operand (rA or rS) is chosen here too. Only a producer one
  // step ahead -- in EX now, in MEM next cycle -- is left for EX, which takes
  // MEM's result through a single 2:1 mux (R23 timing: EX had three mux levels
  // before its adder). While EX is held the registers re-latch their forwarded
  // values and the selects clear.
  //
  // x: the ALU's first operand (rA, (rA|0) or rS); y: rB, the immediate, 0 or -1;
  // a: rA for rlwimi's insert; s: rS, the store and move-to source.
  logic [31:0] op_x, op_y, op_a, op_s;
  logic        mx_x, mx_y, mx_a, mx_s;                 // take MEM's result in EX

  wire        ex_w   = ex_valid && ex_u.rd_we;
  wire        mem_w  = m.valid && m.rd_we;
  wire        wb_w   = wb_valid && wb_rd_we;
  wire [31:0] m_val  = m.is_load ? ld_val : m.result;

  wire [4:0]  id_xr    = id_u.x_from_s ? id_u.rs : id_u.ra;
  wire [31:0] id_xrf   = id_u.x_from_s ? rf_rs : rf_ra;
  wire        id_xzero = !id_u.x_from_s && id_u.a_zero && id_u.ra == 5'd0;
  wire        id_yreg  = id_u.bsel == BSEL_RB;

  wire        x_ex  = !id_xzero && ex_w && ex_u.rd == id_xr;
  wire        y_ex  = id_yreg && ex_w && ex_u.rd == id_u.rb;
  wire        a_ex  = ex_w && ex_u.rd == id_u.ra;
  wire        s_ex  = ex_w && ex_u.rd == id_u.rs;

  wire [31:0] x_cap = id_xzero                          ? 32'd0     :
                      (mem_w && m.rd == id_xr)          ? m_val     :
                      (wb_w && wb_rd == id_xr)          ? wb_result : id_xrf;
  wire [31:0] y_cap = (id_u.bsel == BSEL_IMM)           ? id_u.imm  :
                      (id_u.bsel == BSEL_ZERO)          ? 32'd0     :
                      (id_u.bsel == BSEL_MINUS1)        ? 32'hFFFF_FFFF :
                      (mem_w && m.rd == id_u.rb)        ? m_val     :
                      (wb_w && wb_rd == id_u.rb)        ? wb_result : rf_rb;
  wire [31:0] a_cap = (mem_w && m.rd == id_u.ra)        ? m_val     :
                      (wb_w && wb_rd == id_u.ra)        ? wb_result : rf_ra;
  wire [31:0] s_cap = (mem_w && m.rd == id_u.rs)        ? m_val     :
                      (wb_w && wb_rd == id_u.rs)        ? wb_result : rf_rs;

  wire [31:0] x_val = mx_x ? m.result : op_x;
  wire [31:0] y_val = mx_y ? m.result : op_y;
  wire [31:0] a_val = mx_a ? m.result : op_a;
  wire [31:0] s_val = mx_s ? m.result : op_s;

  // special registers as EX sees them: the MEM stage's pending write, else committed
  // CR0 from Rc is formed here, from MEM's registered result, not after EX's adder
  // CR0 from Rc and a compare's field are both put into CR here, in MEM, from
  // registered values, not after EX's adder and comparator (R23 timing)
  // A CR writer carries only the bits it writes (crm) and their values; they
  // are merged into the committed CR here, so no uop copies the whole
  // forwarded CR into MEM (R23 timing). m_cr is CR as it will be once this
  // uop commits: what EX's readers see while it is in MEM.
  logic [31:0] m_cr;
  always_comb begin
    m_cr = (cr_q & ~m.crm) | (m.cr & m.crm);
    if (m.rc0)
      m_cr[31:28] = {m.result[31], !m.result[31] && (m.result != 32'd0), m.result == 32'd0, m.xer[31]};
    if (m.cmp_we)
      m_cr[5'd31 - {m.crf, 2'b00} -: 4] = m.cmpv;
  end
  // Special registers forward from MEM only (they commit as MEM is left). As
  // for the GPRs, ID decides from the producer's decoded write flags and hands
  // EX a registered select, so MEM's own flags are off EX's paths (R23 timing).
  // sc, illegal and fired traps are never forwarded: each redirects, so
  // nothing that could read SRR0/SRR1/MSR is in EX before they commit.
  logic fs_cr, fs_xer, fs_lr, fs_ctr, fs_spr, fs_srr0, fs_srr1;
  logic ex_w_cr, ex_w_xer, ex_w_lr, ex_w_ctr, ex_w_spr;
  wire [31:0] cr_in  = fs_cr  ? m_cr  : cr_q;
  wire [31:0] xer_in = fs_xer ? m.xer : xer_q;
  wire [31:0] lr_in  = fs_lr  ? m.lr  : lr_q;
  wire [31:0] ctr_in = fs_ctr ? m.ctr : ctr_q;
  wire        res_in = (m.valid && m.res_set) ? 1'b1 : (m.valid && m.res_clr) ? 1'b0 : res_q;

  // Generic SPRs in MLAB (study 8.4): SPRGs, BATs, DSISR, DAR, SDR1, HID0, the
  // 603e TLB-miss registers. SRR0 and SRR1 are flip-flops: an exception writes
  // both on one commit, and a second array write port would cost a copy.
  logic [31:0] srr0_q, srr1_q;
  (* ramstyle = "MLAB, no_rw_check" *) logic [31:0] spr_mem [0:63];

  // The SPR number is decoded in ID and registered into EX with the uop, so the
  // decode is off EX's result path (R23 timing). mf_sel picks mfspr's source:
  // 0 generic/SRR, 1 XER, 2 LR, 3 CTR, 4 DEC, 5 TBL, 6 TBU, 7 PVR.
  logic       id_spr_hit, ex_spr_hit;
  logic [5:0] id_spr_idx, ex_spr_idx;
  logic [2:0] id_mf_sel, ex_mf_sel;
  always_comb begin
    case (id_u.spr)
      10'd1:   id_mf_sel = 3'd1;  10'd8:   id_mf_sel = 3'd2;  10'd9:   id_mf_sel = 3'd3;
      10'd22:  id_mf_sel = 3'd4;  10'd268: id_mf_sel = 3'd5;  10'd269: id_mf_sel = 3'd6;
      10'd287: id_mf_sel = 3'd7;  default: id_mf_sel = 3'd0;
    endcase
  end
  always_comb begin
    id_spr_hit = 1'b1;
    case (id_u.spr)
      10'd26:   id_spr_idx = 6'd0;    // SRR0 (flip-flops, not the array)
      10'd27:   id_spr_idx = 6'd1;    // SRR1
      10'd272:  id_spr_idx = 6'd2;    10'd273: id_spr_idx = 6'd3;      // SPRG0-3
      10'd274:  id_spr_idx = 6'd4;    10'd275: id_spr_idx = 6'd5;
      10'd18:   id_spr_idx = 6'd6;    // DSISR
      10'd19:   id_spr_idx = 6'd7;    // DAR
      10'd25:   id_spr_idx = 6'd8;    // SDR1
      10'd1008: id_spr_idx = 6'd9;    // HID0
      10'd1009: id_spr_idx = 6'd10;   // HID1
      10'd1010: id_spr_idx = 6'd11;   // IABR
      10'd282:  id_spr_idx = 6'd12;   // EAR
      10'd976:  id_spr_idx = 6'd13;   10'd977: id_spr_idx = 6'd14;     // DMISS DCMP
      10'd978:  id_spr_idx = 6'd15;   10'd979: id_spr_idx = 6'd16;     // HASH1 HASH2
      10'd980:  id_spr_idx = 6'd17;   10'd981: id_spr_idx = 6'd18;     // IMISS ICMP
      10'd982:  id_spr_idx = 6'd19;   // RPA
      default: begin
        if (id_u.spr >= 10'd528 && id_u.spr <= 10'd543)
          id_spr_idx = 6'd20 + {2'b00, id_u.spr[3:0]};                 // IBAT0U .. DBAT3L
        else begin
          id_spr_idx = 6'd63;
          id_spr_hit = 1'b0;
        end
      end
    endcase
  end

  wire [31:0] srr0_in  = fs_srr0 ? m.sprv : srr0_q;
  wire [31:0] srr1_in  = fs_srr1 ? m.sprv : srr1_q;
  // The SPR array is read in ID and the value captured into EX, with MEM's
  // committing write folded in, as for the GPRs; only a mtspr one step ahead
  // is taken from MEM in EX (R23 timing: the MLAB read was on EX's result path).
  logic [31:0] ex_spr_v;
  wire  [31:0] id_spr_v = (m.valid && m.spr_we && m.spr == id_spr_idx) ? m.sprv : spr_mem[id_spr_idx];
  wire [31:0] spr_rd   = (ex_spr_idx == 6'd0) ? srr0_in :
                         (ex_spr_idx == 6'd1) ? srr1_in :
                         fs_spr ? m.sprv : ex_spr_v;

  // ALU

  logic [31:0] alu_res, r_sum, r_logic, r_shift, r_rot, r_misc;
  logic        alu_ca, alu_ov;
  logic [3:0]  alu_cmp;

  ppc_alu u_alu (
    .op (ex_u.op), .x (x_val), .y (y_val), .a_ins (a_val),
    .cin (ex_u.cin), .ca_in (xer_in[29]), .so_in (xer_in[31]),
    .sh_imm (ex_u.sh), .sh_from_rb (ex_u.sh_from_rb), .mb (ex_u.mb), .me (ex_u.me),
    .rlw_insert (ex_u.rlw_insert),
    .ltt (ex_ltt), .shk (ex_shk), .mk (ex_mk),
    .r_sum (r_sum), .r_logic (r_logic), .r_shift (r_shift), .r_rot (r_rot), .r_misc (r_misc),
    .ca_out (alu_ca), .ov_out (alu_ov), .cmp_out (alu_cmp)
  );

  // multiply / divide
  wire ex_is_md = ex_valid && ex_md;
  logic        md_done, md_ov;
  logic [31:0] md_res;
  logic        md_have;                                // result captured, waiting on MEM
  logic [31:0] md_res_q;
  logic        md_ov_q;

  ppc_muldiv u_md (
    .clk (clk), .rst (rst),        // a redirect never meets a divide in EX
    .start (ex_is_md && !ex_md_busy && !md_have),
    .op (ex_u.op), .x (x_val), .y (y_val),
    .done (md_done), .result (md_res), .ov (md_ov)
  );

  wire        md_ready = md_done || md_have;
  wire [31:0] md_val   = md_done ? md_res : md_res_q;
  wire        md_ovv   = md_done ? md_ov  : md_ov_q;

  // branch
  wire        bo_noctr  = ex_u.bo[2];
  wire        bo_ctrz   = ex_u.bo[1];
  wire        bo_nocond = ex_u.bo[4];
  wire        bo_cond   = ex_u.bo[3];
  wire [31:0] ctr_dec   = ctr_in - 32'd1;
  wire        cr_bit    = cr_in[5'd31 - ex_u.bi];
  wire        br_ctr_ok = bo_noctr || ((ctr_in != 32'd1) ^ bo_ctrz);   // CTR-1 != 0, without the adder
  wire        br_cnd_ok = bo_nocond || (cr_bit == bo_cond);
  logic       br_taken;
  logic [31:0] br_target;
  always_comb begin
    case (ex_u.bkind)
      BR_B:     begin br_taken = 1'b1;                    br_target = (ex_u.aa ? 32'd0 : ex_u.pc) + ex_u.imm; end
      BR_BC:    begin br_taken = br_ctr_ok && br_cnd_ok;  br_target = (ex_u.aa ? 32'd0 : ex_u.pc) + ex_u.imm; end
      BR_BCLR:  begin br_taken = br_ctr_ok && br_cnd_ok;  br_target = {lr_in[31:2], 2'b00};  end
      default:  begin br_taken = br_cnd_ok;               br_target = {ctr_in[31:2], 2'b00}; end
    endcase
  end

  // memory access
  assign alu_res = r_sum;
  wire [31:0] ea      = r_sum;
  wire        is_ld   = ex_u.op == OP_LOAD;
  wire        is_st   = ex_u.op == OP_STORE;
  wire        st_ok   = !ex_u.cond || res_in;           // stwcx. stores only when reserved
  wire [1:0]  boff    = (ex_u.msize == 2'd2) ? 2'd0 : (ex_u.msize == 2'd1) ? {ea[1], 1'b0} : ea[1:0];
  logic [31:0] st_data, sv;
  logic [3:0]  st_be;
  always_comb begin
    sv = s_val;
    case (ex_u.msize)
      2'd2: begin
        st_data = ex_u.brev ? {sv[7:0], sv[15:8], sv[23:16], sv[31:24]} : sv;
        st_be   = 4'b1111;
      end
      2'd1: begin
        st_data = ex_u.brev ? {2{sv[7:0], sv[15:8]}} : {2{sv[15:0]}};
        st_be   = boff[1] ? 4'b0011 : 4'b1100;
      end
      default: begin
        st_data = {4{sv[7:0]}};
        st_be   = 4'b1000 >> boff;
      end
    endcase
  end

  // EX holds for multiply/divide and whenever MEM holds
  wire md_wait = ex_is_md && !md_ready;
  assign ex_hold = mem_hold || md_wait;
  wire  ex_adv  = ex_valid && !ex_hold;

  assign d_req   = ex_adv && (is_ld || (is_st && st_ok));
  assign d_we    = is_st;
  assign d_addr  = {ea[31:2], 2'b00};
  assign d_be    = is_st ? st_be : 4'b1111;
  assign d_wdata = st_data;

  // results and special-register updates
  // the move-from value: mfcr, mfmsr, mfspr
  logic [31:0] mf_val;
  always_comb begin
    case (ex_mf_sel)
      3'd1:    mf_val = xer_in;
      3'd2:    mf_val = lr_in;
      3'd3:    mf_val = ctr_in;
      3'd4:    mf_val = dec_q;
      3'd5:    mf_val = tb_q[31:0];
      3'd6:    mf_val = tb_q[63:32];
      3'd7:    mf_val = PVR;
      default: mf_val = ex_spr_hit ? spr_rd : 32'd0;
    endcase
    if (ex_u.op == OP_MFCR)  mf_val = cr_in;
    if (ex_u.op == OP_MFMSR) mf_val = msr_q;
  end

  mem_t mx;
  logic [3:0]  c0;
  logic [31:0] res, xer_n, cr_n, msk;
  logic        so_n;
  always_comb begin
    msk        = 32'd0;
    mx         = '0;
    mx.valid   = ex_valid;
    mx.pc      = ex_u.pc;
    mx.insn    = ex_u.insn;
    mx.last    = ex_u.last;
    mx.rd      = ex_u.rd;
    mx.rd_we   = ex_u.rd_we;
    mx.is_load = is_ld;
    mx.req     = is_ld || (is_st && st_ok);
    mx.msize   = ex_u.msize;
    mx.boff    = boff;
    mx.msigned = ex_u.msigned;
    mx.brev    = ex_u.brev;
    mx.spr     = ex_spr_idx;

    xer_n = xer_in;
    cr_n  = 32'd0;                                       // values of the bits written (mx.crm)
    mx.crm = 32'd0;

    // one-hot result select, registered in decode (R23 timing); the update
    // form of a store writes its EA (the sum) to rA
    res = ({32{ex_rsel[0]}} & r_sum)   | ({32{ex_rsel[1]}} & r_logic) |
          ({32{ex_rsel[2]}} & r_shift) | ({32{ex_rsel[3]}} & r_rot)   |
          ({32{ex_rsel[4]}} & r_misc)  | ({32{ex_rsel[5]}} & md_val)  |
          ({32{ex_rsel[6]}} & mf_val);

    // XER: carry, overflow, summary overflow
    if (ex_u.set_ca) xer_n[29] = alu_ca;
    if (ex_u.oe) begin
      xer_n[30] = (ex_u.op == OP_MUL || ex_u.op == OP_DIV || ex_u.op == OP_DIVU) ? md_ovv : alu_ov;
      if (xer_n[30]) xer_n[31] = 1'b1;
    end
    so_n = xer_n[31];
    mx.xer_we = ex_u.set_ca || ex_u.oe;

    // CR
    c0 = 4'd0;
    if (ex_u.rc) begin
      mx.rc0   = 1'b1;                                   // CR0 formed in MEM (m_cr)
      mx.crm[31:28] = 4'hF;
      mx.cr_we = 1'b1;
    end
    case (ex_u.op)
      OP_CMP, OP_CMPL: begin
        mx.cmp_we = 1'b1;                                // field inserted in MEM (m_cr)
        mx.crm[5'd31 - {ex_u.crf, 2'b00} -: 4] = 4'hF;
        mx.cmpv   = alu_cmp;
        mx.crf    = ex_u.crf;
        mx.cr_we  = 1'b1;
      end
      OP_CRLOG: begin
        if (ex_u.insn[10:1] == 10'd0)                        // mcrf
          begin
            cr_n[5'd31 - {ex_u.crf, 2'b00} -: 4]   = cr_in[5'd31 - {ex_u.crf_src, 2'b00} -: 4];
            mx.crm[5'd31 - {ex_u.crf, 2'b00} -: 4] = 4'hF;
          end
        else
          begin
            cr_n[5'd31 - ex_u.crbd]   = ex_u.crop[{cr_in[5'd31 - ex_u.crba], cr_in[5'd31 - ex_u.crbb]}];
            mx.crm[5'd31 - ex_u.crbd] = 1'b1;
          end
        mx.cr_we = 1'b1;
      end
      OP_MTCRF: begin
        for (int f = 0; f < 8; f++) msk[31 - 4*f -: 4] = {4{ex_u.crm[7 - f]}};
        cr_n   = s_val;
        mx.crm = msk;
        mx.cr_we = 1'b1;
      end
      OP_MCRXR: begin
        cr_n[5'd31 - {ex_u.crf, 2'b00} -: 4]   = xer_in[31:28];
        mx.crm[5'd31 - {ex_u.crf, 2'b00} -: 4] = 4'hF;
        xer_n[31:28] = 4'd0;
        mx.cr_we = 1'b1;
        mx.xer_we = 1'b1;
      end
      OP_STORE: if (ex_u.cond) begin                       // stwcx.
        cr_n[31:28]   = {2'b00, res_in, xer_in[31]};
        mx.crm[31:28] = 4'hF;
        mx.cr_we = 1'b1;
      end
      default: ;
    endcase

    mx.result = res;
    mx.cr     = cr_n;
    mx.xer    = xer_n & XER_MASK;
    mx.lr     = lr_in;
    mx.ctr    = ctr_in;
    mx.msr    = msr_q;

    // branch side effects
    if (ex_u.op == OP_BRANCH) begin
      if (ex_u.lk) begin mx.lr = ex_u.pc + 32'd4; mx.lr_we = 1'b1; end
      if (ex_u.bkind != BR_B && ex_u.bkind != BR_BCCTR && !bo_noctr) begin mx.ctr = ctr_dec; mx.ctr_we = 1'b1; end
    end

    // mtspr
    if (ex_u.op == OP_MTSPR) begin
      case (ex_u.spr)
        10'd1:   begin mx.xer = s_val & XER_MASK; mx.xer_we = 1'b1; end
        10'd8:   begin mx.lr  = s_val;           mx.lr_we  = 1'b1; end
        10'd9:   begin mx.ctr = s_val;           mx.ctr_we = 1'b1; end
        10'd22:  begin mx.dec_we = 1'b1;         mx.sprv = s_val; end
        10'd284: begin mx.tb_we  = 1'b1;         mx.sprv = s_val; mx.tbu = 1'b0; end
        10'd285: begin mx.tb_we  = 1'b1;         mx.sprv = s_val; mx.tbu = 1'b1; end
        default: begin mx.spr_we = ex_spr_hit;   mx.sprv = s_val; end
      endcase
    end
    // The new MSR's value follows from the opcode alone; only its write enable
    // waits on a trap's compare (R23 timing).
    case (ex_u.op)
      OP_MTMSR:                      mx.msr = s_val & MSR_MASK;
      OP_RFI:                        mx.msr = (srr1_in & SRR1_KEEP) | (msr_q & 32'h0001_0000);
      OP_SC, OP_ILLEGAL, OP_TRAP:    mx.msr = (msr_q & ~32'h0006_EF33) | {31'd0, msr_q[MSR_ILE]};
      default: ;
    endcase
    if (ex_u.op == OP_MTMSR) mx.msr_we = 1'b1;

    // reservation
    mx.res_set = ex_u.reserve;
    mx.res_clr = ex_u.cond;

    // exceptions: sc and program (illegal, trap); rfi restores MSR
    // SRR0/SRR1 values do not depend on whether the trap fires; only `exc` does,
    // so they are not held behind the trap compare (R23 timing)
    mx.srr0 = (ex_u.op == OP_SC) ? ex_u.pc + 32'd4 : ex_u.pc;
    mx.srr1 = (msr_q & SRR1_KEEP) |
              ((ex_u.op == OP_ILLEGAL) ? 32'h0008_0000 : (ex_u.op == OP_TRAP) ? 32'h0002_0000 : 32'd0);
    // tw / twi: the operands go to MEM, which compares them (R23 timing)
    mx.is_trap = (ex_u.op == OP_TRAP);
    mx.to      = ex_u.bo;
    mx.ta      = x_val;
    mx.tb      = y_val;
    if (ex_u.op == OP_SC || ex_u.op == OP_ILLEGAL) begin
      mx.exc    = 1'b1;
      mx.msr_we = 1'b1;
      mx.res_clr = 1'b0;
    end
    if (ex_u.op == OP_RFI) mx.msr_we = 1'b1;
  end

  // redirect from EX (younger instructions in ID and fetch are discarded)
  logic [31:0] exc_vec;
  always_comb begin
    exc_vec = msr_q[MSR_IP] ? 32'hFFF0_0000 : 32'd0;
    redirect      = 1'b0;
    redirect_pc   = 32'd0;
    redirect_pc_b = ex_u.pc + 32'd4;                   // a branch that falls through
    redirect_sel  = 1'b1;
    if (ex_adv) begin
      case (ex_u.op)
        OP_BRANCH:  begin
                      redirect_pc  = br_target;          // not behind the taken decision
                      redirect_sel = br_taken;
                      if (br_taken != ex_pred) redirect = 1'b1;
                    end
        OP_SC:      begin redirect = 1'b1; redirect_pc = exc_vec | 32'h0C00; end
        OP_ILLEGAL: begin redirect = 1'b1; redirect_pc = exc_vec | 32'h0700; end
        // A trap redirects to the next instruction from EX, decided by the
        // opcode alone; if it fired, MEM redirects again to the vector one cycle
        // later (below). The operand compare is then off the fetch path entirely
        // (R23 timing). Nothing younger can be in EX or ID by then: the first
        // redirect flushed them. Traps do not appear in VF3's census (R12).
        OP_TRAP:    begin redirect = 1'b1; redirect_pc = ex_u.pc + 32'd4; end
        OP_RFI:     begin redirect = 1'b1; redirect_pc = {srr0_in[31:2], 2'b00}; end
        OP_MTMSR:   begin redirect = 1'b1; redirect_pc = ex_u.pc + 32'd4; end
        default: ;
      endcase
    end
    if (wb_trap) begin
      redirect     = 1'b1;
      redirect_pc  = exc_vec | 32'h0700;
      redirect_sel = 1'b1;
    end
  end


  always_ff @(posedge clk) begin
    if (rst) begin
      ex_valid   <= 1'b0;
      ex_md_busy <= 1'b0;
      md_have    <= 1'b0;
    end else begin
      if (ex_is_md && !ex_md_busy && !md_have) ex_md_busy <= 1'b1;
      if (md_done) begin
        ex_md_busy <= 1'b0;
        md_have    <= 1'b1;
        md_res_q   <= md_res;
        md_ov_q    <= md_ov;
      end
      if (!ex_hold) begin
        md_have  <= 1'b0;
        ex_valid <= id_adv;
        // the uop's data loads whenever EX is free; only ex_valid waits on the
        // redirect decision (R23 timing)
        ex_u       <= id_u;
        ex_pred    <= id_pred;
        ex_rsel    <= id_rsel;
        ex_ld      <= id_u.op == OP_LOAD;
        ex_md      <= id_rsel[5];
        ex_spr_v   <= id_spr_v;
        ex_ltt     <= id_ltt;
        ex_shk     <= id_shk;
        ex_mk      <= id_mk;
        op_x       <= x_cap;
        op_y       <= y_cap;
        op_a       <= a_cap;
        op_s       <= s_cap;
        mx_x       <= x_ex;
        mx_y       <= y_ex;
        mx_a       <= a_ex;
        mx_s       <= s_ex;
        ex_spr_idx <= id_spr_idx;
        ex_spr_hit <= id_spr_hit;
        ex_mf_sel  <= id_mf_sel;
        ex_w_cr    <= id_w_cr;
        ex_w_xer   <= id_w_xer;
        ex_w_lr    <= id_w_lr;
        ex_w_ctr   <= id_w_ctr;
        ex_w_spr   <= id_u.op == OP_MTSPR && id_spr_hit;
        fs_cr      <= ex_valid && ex_w_cr;
        fs_xer     <= ex_valid && ex_w_xer;
        fs_lr      <= ex_valid && ex_w_lr;
        fs_ctr     <= ex_valid && ex_w_ctr;
        fs_spr     <= ex_valid && ex_w_spr && ex_spr_idx == id_spr_idx;
        fs_srr0    <= ex_valid && ex_w_spr && ex_spr_idx == 6'd0;
        fs_srr1    <= ex_valid && ex_w_spr && ex_spr_idx == 6'd1;
      end else begin
        // Held: keep the operands as forwarded now and drop the selects; every
        // producer they could need is in MEM or WB this cycle.
        ex_spr_v <= fs_spr ? m.sprv : ex_spr_v;
        op_x <= x_val;
        op_y <= y_val;
        op_a <= a_val;
        op_s <= s_val;
        mx_x <= 1'b0;
        mx_y <= 1'b0;
        mx_a <= 1'b0;
        mx_s <= 1'b0;
        // the producer in MEM commits when MEM advances; then read the register
        fs_cr   <= fs_cr   && !mem_adv;
        fs_xer  <= fs_xer  && !mem_adv;
        fs_lr   <= fs_lr   && !mem_adv;
        fs_ctr  <= fs_ctr  && !mem_adv;
        fs_spr  <= fs_spr  && !mem_adv;
        fs_srr0 <= fs_srr0 && !mem_adv;
        fs_srr1 <= fs_srr1 && !mem_adv;
      end
    end
  end

  // ====================================================================== MEM
  assign mem_hold = m.valid && m.req && !d_valid;

  // tw / twi decided here, on registered operands; a fired trap redirects to
  // 0x700 from MEM and commits like any other exception
  wire m_trap = m.is_trap && (
                  (m.to[4] && $signed(m.ta) <  $signed(m.tb)) || (m.to[3] && $signed(m.ta) > $signed(m.tb)) ||
                  (m.to[2] && m.ta == m.tb) || (m.to[1] && m.ta < m.tb) || (m.to[0] && m.ta > m.tb));
  wire m_exc  = m.exc;                                  // sc / illegal commit at MEM exit

  // A fired trap commits and redirects from WB, one cycle on, from registered
  // state (R23 timing): nothing younger can have issued, since EX already
  // redirected the trap to its next instruction.
  logic        wb_trap;
  logic [31:0] wb_tsrr0, wb_tsrr1, wb_tmsr;
  always_ff @(posedge clk) begin
    if (rst) wb_trap <= 1'b0;
    else     wb_trap <= mem_adv && m_trap;
    wb_tsrr0 <= m.srr0;
    wb_tsrr1 <= m.srr1;
    wb_tmsr  <= m.msr;
  end
  assign mem_adv  = m.valid && !mem_hold;

  logic [31:0] ld_w;
  logic [15:0] ld_h;
  logic [7:0]  ld_b;
  always_comb begin
    ld_w = d_rdata;
    ld_h = m.boff[1] ? ld_w[15:0] : ld_w[31:16];
    if (m.brev) ld_h = {ld_h[7:0], ld_h[15:8]};
    ld_b = ld_w[5'd31 - {m.boff, 3'b000} -: 8];
    case (m.msize)
      2'd2:    ld_val = m.brev ? {ld_w[7:0], ld_w[15:8], ld_w[23:16], ld_w[31:24]} : ld_w;
      2'd1:    ld_val = m.msigned ? {{16{ld_h[15]}}, ld_h} : {16'd0, ld_h};
      default: ld_val = {24'd0, ld_b};
    endcase
  end

  always_ff @(posedge clk) begin
    if (rst) begin
      m.valid <= 1'b0;
    end else if (!mem_hold) begin
      m <= mx;
      m.valid <= ex_adv;
    end
  end

  // commit of special registers as the instruction leaves MEM
  always_ff @(posedge clk) begin
    if (rst) begin
      cr_q  <= 32'd0;
      xer_q <= 32'd0;
      lr_q  <= 32'd0;
      ctr_q <= 32'd0;
      msr_q <= 32'h0000_0040;                            // IP: vectors at 0xFFF00000
      res_q <= 1'b0;
      tb_q  <= 64'd0;
      dec_q <= 32'hFFFF_FFFF;
    end else begin
      if (tb_tick) begin
        tb_q  <= tb_q + 64'd1;
        dec_q <= dec_q - 32'd1;
      end
      if (wb_trap) msr_q <= wb_tmsr;
      if (mem_adv) begin
        if (m.cr_we)  cr_q  <= m_cr;
        if (m.xer_we) xer_q <= m.xer;
        if (m.lr_we)  lr_q  <= m.lr;
        if (m.ctr_we) ctr_q <= m.ctr;
        if (m.msr_we) msr_q <= m.msr;
        if (m.res_set) res_q <= 1'b1;
        if (m.res_clr) res_q <= 1'b0;
        if (m.dec_we) dec_q <= m.sprv;
        if (m.tb_we) begin
          if (m.tbu) tb_q[63:32] <= m.sprv;
          else       tb_q[31:0]  <= m.sprv;
        end
      end
    end
  end

  always_ff @(posedge clk) begin
    if (mem_adv && m.spr_we && m.spr >= 6'd2) spr_mem[m.spr] <= m.sprv;
    if (rst) begin
      srr0_q <= 32'd0;
      srr1_q <= 32'd0;
    end else if (wb_trap) begin                        // a fired trap, one cycle on
      srr0_q <= wb_tsrr0;
      srr1_q <= wb_tsrr1;
    end else if (mem_adv) begin
      if (m_exc) begin
        srr0_q <= m.srr0;
        srr1_q <= m.srr1;
      end else if (m.spr_we && m.spr == 6'd0) srr0_q <= m.sprv;
      else if (m.spr_we && m.spr == 6'd1)     srr1_q <= m.sprv;
    end
  end

  // ======================================================================= WB
  always_ff @(posedge clk) begin
    if (rst) begin
      wb_valid <= 1'b0;
    end else begin
      wb_valid  <= mem_adv;
      wb_last   <= m.last;
      wb_rd_we  <= m.rd_we;
      wb_rd     <= m.rd;
      wb_result <= m.is_load ? ld_val : m.result;
      wb_pc     <= m.pc;
      wb_insn   <= m.insn;
    end
  end

  // =================================================================== events
  assign ev[0] = wb_valid && wb_last;
  assign ev[1] = wb_valid;
  assign ev[2] = !id_valid && !ex_hold && !redirect;
  assign ev[3] = id_valid && load_use && !ex_hold;
  assign ev[4] = md_wait && !mem_hold;
  assign ev[5] = mem_hold;
  assign ev[6] = redirect;
  assign ev[7] = f_out && !i_valid;

  // ==================================================================== trace
  logic [31:0] tr_cr, tr_xer, tr_lr, tr_ctr, tr_msr, tr_srr0, tr_srr1;
  always_ff @(posedge clk) begin
    if (mem_adv) begin
      tr_cr   <= m.cr_we  ? m_cr  : cr_q;
      tr_xer  <= m.xer_we ? m.xer : xer_q;
      tr_lr   <= m.lr_we  ? m.lr  : lr_q;
      tr_ctr  <= m.ctr_we ? m.ctr : ctr_q;
      tr_msr  <= (m.msr_we || m_trap) ? m.msr : msr_q;
      tr_srr0 <= (m_exc || m_trap) ? m.srr0 : (m.spr_we && m.spr == 6'd0) ? m.sprv : srr0_q;
      tr_srr1 <= (m_exc || m_trap) ? m.srr1 : (m.spr_we && m.spr == 6'd1) ? m.sprv : srr1_q;
    end
  end

  assign rt_valid   = wb_valid;
  assign rt_last    = wb_last;
  assign rt_pc      = wb_pc;
  assign rt_insn    = wb_insn;
  assign rt_gpr_we  = wb_rd_we;
  assign rt_gpr     = wb_rd;
  assign rt_gpr_val = wb_result;
  assign rt_cr      = tr_cr;
  assign rt_xer     = tr_xer;
  assign rt_lr      = tr_lr;
  assign rt_ctr     = tr_ctr;
  assign rt_msr     = tr_msr;
  assign rt_srr0    = tr_srr0;
  assign rt_srr1    = tr_srr1;

endmodule
