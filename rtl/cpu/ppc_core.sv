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
// Forwarding into EX: the MEM stage's ALU result, the WB stage's result (ALU or
// load), and wb2, the value written to the register file on the previous edge.
// A load is followed by one bubble when its consumer is next in line.
//
// Branches resolve in EX and redirect fetch; prediction is a later step (the
// CPI model assumes static prediction in ID, study R22). sc, rfi, mtmsr, traps
// and illegal instructions redirect from EX the same way and commit their
// special-register writes at MEM exit, so nothing younger has issued by then.
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
    logic        is_trap;                              // tw / twi: the compare runs in MEM
    logic [4:0]  to;
    logic [31:0] ta, tb;
  } mem_t;
  mem_t m;

  // WB
  logic        wb_valid, wb_last, wb_rd_we;
  logic [4:0]  wb_rd;
  logic [31:0] wb_result, wb_pc, wb_insn;
  logic        wb2_we;
  logic [4:0]  wb2_rd;
  logic [31:0] wb2_val;

  // ==================================================================== fetch
  logic [31:0] f_pc;
  logic        f_out, f_discard;
  logic [31:0] q_insn [0:1];
  logic [31:0] q_pc   [0:1];
  logic        q_head;
  logic [1:0]  q_cnt;

  logic        redirect;
  logic [31:0] redirect_pc;
  logic        id_take;                                // ID consumed the queue head

  // A redirect is decided late in EX, so it is kept off the queue's write
  // enables and the memory request (study R23 timing): a response is always
  // written, and a redirect clears the count, which makes the write dead; a
  // request issued in a redirect cycle is marked to be discarded.
  wire  rsp     = f_out && i_valid;
  wire  rsp_use = rsp && !f_discard;
  wire  [1:0] q_cnt_after = q_cnt + {1'b0, rsp_use} - {1'b0, id_take};

  assign i_addr = f_pc;
  assign i_req  = !rst && (!f_out || rsp) && (q_cnt_after < 2'd2);

  always_ff @(posedge clk) begin
    if (rst) begin
      f_pc      <= RESET_PC;
      f_out     <= 1'b0;
      f_discard <= 1'b0;
      q_cnt     <= 2'd0;
      q_head    <= 1'b0;
    end else begin
      if (redirect) begin
        f_pc      <= redirect_pc;
        q_cnt     <= 2'd0;
        f_out     <= (f_out && !i_valid) || i_req;
        f_discard <= (f_out && !i_valid) || i_req;       // anything still in flight is wrong-path
      end else begin
        if (rsp) begin
          f_out     <= 1'b0;
          f_discard <= 1'b0;
        end
        if (i_req) begin
          f_out <= 1'b1;
          f_pc  <= f_pc + 32'd4;
        end
        q_cnt <= q_cnt_after;
        if (id_take) q_head <= !q_head;
      end
    end
  end

  // Queue writes go to the slot after the live entries, counted before any pop.
  logic [31:0] f_rsp_pc;
  always_ff @(posedge clk) begin
    if (i_req) f_rsp_pc <= f_pc;
    if (rsp_use) begin
      q_insn[q_head ^ q_cnt[0]] <= i_data;
      q_pc  [q_head ^ q_cnt[0]] <= f_rsp_pc;
    end
  end

  // ==================================================================== decode
  logic [4:0]  id_step;
  uop_t        id_u;
  wire         id_valid = (q_cnt != 2'd0);

  ppc_decode u_dec (
    .insn (q_insn[q_head]),
    .pc   (q_pc[q_head]),
    .step (id_step),
    .u    (id_u)
  );

  // load-use: the load in EX writes a register this uop reads
  wire ex_is_load = ex_valid && ex_u.op == OP_LOAD;
  wire load_use   = ex_is_load && ex_u.rd_we && (
                      (id_u.a_used && id_u.ra == ex_u.rd) ||
                      (id_u.b_used && id_u.rb == ex_u.rd) ||
                      (id_u.s_used && id_u.rs == ex_u.rd));

  logic ex_hold, mem_hold, mem_adv;
  wire  id_adv = id_valid && !ex_hold && !load_use && !redirect;
  assign id_take = id_adv && id_u.last;

  always_ff @(posedge clk) begin
    if (rst || redirect) id_step <= 5'd0;
    else if (id_adv)     id_step <= id_u.last ? 5'd0 : id_step + 5'd1;
  end

  // the special registers this uop writes, from its decode alone
  wire id_w_cr  = id_u.rc || id_u.op == OP_CMP || id_u.op == OP_CMPL || id_u.op == OP_CRLOG ||
                  id_u.op == OP_MTCRF || id_u.op == OP_MCRXR || (id_u.op == OP_STORE && id_u.cond);
  wire id_w_xer = id_u.set_ca || id_u.oe || id_u.op == OP_MCRXR || (id_u.op == OP_MTSPR && id_u.spr == 10'd1);
  wire id_w_lr  = (id_u.op == OP_BRANCH && id_u.lk) || (id_u.op == OP_MTSPR && id_u.spr == 10'd8);
  wire id_w_ctr = (id_u.op == OP_BRANCH && id_u.bkind != BR_B && id_u.bkind != BR_BCCTR && !id_u.bo[2]) ||
                  (id_u.op == OP_MTSPR && id_u.spr == 10'd9);

  // ================================================================ registers
  logic [31:0] rf_a, rf_b, rf_s;

  ppc_regfile u_rf (
    .clk (clk),
    .ren (!ex_hold),
    .ra  (id_u.ra), .rb (id_u.rb), .rs (id_u.rs),
    .qa  (rf_a),    .qb (rf_b),    .qs (rf_s),
    .we  (wb_valid && wb_rd_we),
    .wa  (wb_rd),
    .wd  (wb_result)
  );

  // Forwarding. Which older instruction holds each operand is decided in ID,
  // where the producers' register numbers are already in their stage registers,
  // and carried into EX as a one-hot select: {MEM, WB, wb2}, else the register
  // file. EX then has a 4:1 mux and no comparators on its critical path (study
  // R23's worst path ran through them). While EX is held the selects follow the
  // producer down the pipe: MEM -> WB when MEM advances, WB -> wb2, wb2 -> file.
  //
  // (Written out, not as a function: Quartus 17 dropped the module-level
  // signals a forwarding function read without taking them as arguments.)
  logic [2:0] fa_sel, fb_sel, fs_sel;                  // {wb2, WB, MEM}

  function automatic logic [2:0] fwd_pick(input logic [4:0] r,
                                          input logic ex_w, input logic [4:0] ex_rd,
                                          input logic m_w,  input logic [4:0] m_rd,
                                          input logic w_w,  input logic [4:0] w_rd);
    if (ex_w && ex_rd == r)     fwd_pick = 3'b001;     // EX now: in MEM next cycle
    else if (m_w && m_rd == r)  fwd_pick = 3'b010;     // MEM now: in WB next cycle
    else if (w_w && w_rd == r)  fwd_pick = 3'b100;     // WB now: wb2 next cycle
    else                        fwd_pick = 3'b000;
  endfunction

  function automatic logic [2:0] fwd_follow(input logic [2:0] sel, input logic m_adv);
    fwd_follow = {sel[1], sel[0] && m_adv, sel[0] && !m_adv};
  endfunction

  // ======================================================================= EX
  wire [31:0] a_fwd = fa_sel[0] ? m.result : fa_sel[1] ? wb_result : fa_sel[2] ? wb2_val : rf_a;
  wire [31:0] b_val = fb_sel[0] ? m.result : fb_sel[1] ? wb_result : fb_sel[2] ? wb2_val : rf_b;
  wire [31:0] s_val = fs_sel[0] ? m.result : fs_sel[1] ? wb_result : fs_sel[2] ? wb2_val : rf_s;
  wire [31:0] a_val = (ex_u.a_zero && ex_u.ra == 5'd0) ? 32'd0 : a_fwd;

  // special registers as EX sees them: the MEM stage's pending write, else committed
  // CR0 from Rc is formed here, from MEM's registered result, not after EX's adder
  wire [31:0] m_cr   = m.rc0 ? {m.result[31], !m.result[31] && (m.result != 32'd0), m.result == 32'd0, m.xer[31], m.cr[27:0]} : m.cr;
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
  wire [31:0] spr_rd   = (ex_spr_idx == 6'd0) ? srr0_in :
                         (ex_spr_idx == 6'd1) ? srr1_in :
                         fs_spr ? m.sprv : spr_mem[ex_spr_idx];

  // ALU
  logic [31:0] y_val;
  always_comb begin
    case (ex_u.bsel)
      BSEL_RB:     y_val = b_val;
      BSEL_IMM:    y_val = ex_u.imm;
      BSEL_ZERO:   y_val = 32'd0;
      default:     y_val = 32'hFFFF_FFFF;
    endcase
  end
  wire [31:0] x_val = ex_u.x_from_s ? s_val : a_val;

  logic [31:0] alu_res;
  logic        alu_ca, alu_ov;
  logic [3:0]  alu_cmp;

  ppc_alu u_alu (
    .op (ex_u.op), .x (x_val), .y (y_val), .a_ins (a_val),
    .cin (ex_u.cin), .ca_in (xer_in[29]), .so_in (xer_in[31]),
    .sh_imm (ex_u.sh), .sh_from_rb (ex_u.sh_from_rb), .mb (ex_u.mb), .me (ex_u.me),
    .rlw_insert (ex_u.rlw_insert),
    .result (alu_res), .ca_out (alu_ca), .ov_out (alu_ov), .cmp_out (alu_cmp)
  );

  // multiply / divide
  wire ex_is_md = ex_valid && (ex_u.op == OP_MUL || ex_u.op == OP_MULH || ex_u.op == OP_MULHU ||
                               ex_u.op == OP_DIV || ex_u.op == OP_DIVU);
  logic        md_done, md_ov;
  logic [31:0] md_res;
  logic        md_have;                                // result captured, waiting on MEM
  logic [31:0] md_res_q;
  logic        md_ov_q;

  ppc_muldiv u_md (
    .clk (clk), .rst (rst || redirect),
    .start (ex_is_md && !ex_md_busy && !md_have),
    .op (ex_u.op), .x (a_val), .y (y_val),
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
  wire [31:0] ea      = alu_res;
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

    res   = alu_res;
    xer_n = xer_in;
    cr_n  = cr_in;

    case (ex_u.op)
      OP_MUL, OP_MULH, OP_MULHU, OP_DIV, OP_DIVU: res = md_val;
      OP_STORE:  res = ea;                                   // update form writes EA to rA
      OP_MFCR:   res = cr_in;
      OP_MFMSR:  res = msr_q;
      OP_MFSPR: begin
        case (ex_mf_sel)
          3'd1:    res = xer_in;
          3'd2:    res = lr_in;
          3'd3:    res = ctr_in;
          3'd4:    res = dec_q;
          3'd5:    res = tb_q[31:0];
          3'd6:    res = tb_q[63:32];
          3'd7:    res = PVR;
          default: res = ex_spr_hit ? spr_rd : 32'd0;
        endcase
      end
      default: ;
    endcase

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
      mx.cr_we = 1'b1;
    end
    case (ex_u.op)
      OP_CMP, OP_CMPL: begin
        cr_n[5'd31 - {ex_u.crf, 2'b00} -: 4] = alu_cmp;
        mx.cr_we = 1'b1;
      end
      OP_CRLOG: begin
        if (ex_u.insn[10:1] == 10'd0)                        // mcrf
          cr_n[5'd31 - {ex_u.crf, 2'b00} -: 4] = cr_in[5'd31 - {ex_u.crf_src, 2'b00} -: 4];
        else
          cr_n[5'd31 - ex_u.crbd] = ex_u.crop[{cr_in[5'd31 - ex_u.crba], cr_in[5'd31 - ex_u.crbb]}];
        mx.cr_we = 1'b1;
      end
      OP_MTCRF: begin
        for (int f = 0; f < 8; f++) msk[31 - 4*f -: 4] = {4{ex_u.crm[7 - f]}};
        cr_n = (cr_in & ~msk) | (s_val & msk);
        mx.cr_we = 1'b1;
      end
      OP_MCRXR: begin
        cr_n[5'd31 - {ex_u.crf, 2'b00} -: 4] = xer_in[31:28];
        xer_n[31:28] = 4'd0;
        mx.cr_we = 1'b1;
        mx.xer_we = 1'b1;
      end
      OP_STORE: if (ex_u.cond) begin                       // stwcx.
        cr_n[31:28] = {2'b00, res_in, xer_in[31]};
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
    mx.ta      = a_val;
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
    redirect    = 1'b0;
    redirect_pc = 32'd0;
    if (ex_adv) begin
      case (ex_u.op)
        OP_BRANCH:  if (br_taken) begin redirect = 1'b1; redirect_pc = br_target; end
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
    if (m.valid && m_trap) begin
      redirect    = 1'b1;
      redirect_pc = exc_vec | 32'h0700;
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
        fa_sel <= fwd_pick(id_u.ra, ex_valid && ex_u.rd_we, ex_u.rd, m.valid && m.rd_we, m.rd, wb_valid && wb_rd_we, wb_rd);
        fb_sel <= fwd_pick(id_u.rb, ex_valid && ex_u.rd_we, ex_u.rd, m.valid && m.rd_we, m.rd, wb_valid && wb_rd_we, wb_rd);
        fs_sel <= fwd_pick(id_u.rs, ex_valid && ex_u.rd_we, ex_u.rd, m.valid && m.rd_we, m.rd, wb_valid && wb_rd_we, wb_rd);
      end else begin
        // the producer in MEM commits when MEM advances; then read the register
        fs_cr   <= fs_cr   && !mem_adv;
        fs_xer  <= fs_xer  && !mem_adv;
        fs_lr   <= fs_lr   && !mem_adv;
        fs_ctr  <= fs_ctr  && !mem_adv;
        fs_spr  <= fs_spr  && !mem_adv;
        fs_srr0 <= fs_srr0 && !mem_adv;
        fs_srr1 <= fs_srr1 && !mem_adv;
        fa_sel <= fwd_follow(fa_sel, mem_adv);
        fb_sel <= fwd_follow(fb_sel, mem_adv);
        fs_sel <= fwd_follow(fs_sel, mem_adv);
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
  wire m_exc  = m.exc || m_trap;
  assign mem_adv  = m.valid && !mem_hold;

  logic [31:0] ld_val, ld_w;
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
      if (mem_adv) begin
        if (m.cr_we)  cr_q  <= m_cr;
        if (m.xer_we) xer_q <= m.xer;
        if (m.lr_we)  lr_q  <= m.lr;
        if (m.ctr_we) ctr_q <= m.ctr;
        if (m.msr_we || m_trap) msr_q <= m.msr;
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
      wb2_we   <= 1'b0;
    end else begin
      wb_valid  <= mem_adv;
      wb_last   <= m.last;
      wb_rd_we  <= m.rd_we;
      wb_rd     <= m.rd;
      wb_result <= m.is_load ? ld_val : m.result;
      wb_pc     <= m.pc;
      wb_insn   <= m.insn;
      wb2_we    <= wb_valid && wb_rd_we;
      wb2_rd    <= wb_rd;
      wb2_val   <= wb_result;
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
      tr_srr0 <= m_exc ? m.srr0 : (m.spr_we && m.spr == 6'd0) ? m.sprv : srr0_q;
      tr_srr1 <= m_exc ? m.srr1 : (m.spr_we && m.spr == 6'd1) ? m.sprv : srr1_q;
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
