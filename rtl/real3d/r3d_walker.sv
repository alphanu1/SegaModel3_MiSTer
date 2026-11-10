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
// Real3D display-list walker (M2), Step 1.0 node layout, or Step 1.5's with
// step15 (words 1 and 2 unused: word 0, then words 3-9 read into the same
// places, MAME's offset of 2).
//
// Walks the culling RAM exactly as MAME 0.289's model3_v.cpp does (the M2 exit
// test is MAME's triangle list; behaviour, not code, is taken from it, BSD-3-
// Clause, R. Belmont, Ville Linde) and emits one event per matrix-stack or
// model action. The FP work happens downstream; this block only reads words
// and follows links.
//
//   VP     a = viewport address, b = matrix base (node word 22), c = pass
//   PUSH   copy the top of the matrix stack
//   TRANS  a, b, c = translation x, y, z (FP32 bits, node words 2-4)
//   MAT    a = address of a 12-word matrix (matrix base + 12 * index)
//   MODEL  a = model address (polygon RAM below 0x100000, VROM above)
//   POP
//   END    the frame's four passes are done
//
// MAME's order is kept: the viewport chain (node word 1) is walked to its end
// first and drawn last-to-first, once per priority pass; a node pushes, applies
// its translation or matrix, follows its child (word 5), pops, then follows
// its sibling (word 6) unless node[0] & 7 == 6; node[0] & 8 makes the child a
// model list whose entry 0 is drawn; pointer lists are scanned to their end and
// walked backwards, and nest at most three deep. Addresses MAME treats as
// invalid read as zero (the census patch's behaviour). The recursion is an
// explicit return stack: block frames hold the pending sibling link, list frames
// the list base and the next index.
//
// Memory: word addresses in the Real3D culling space (0x000000-0x0FFFFF is the
// 8C RAM, 0x800000-0x83FFFF the 8E RAM). A request holds until mem_ack; its
// mem_len words come back in order on mem_rvalid. One request at a time.
//
// Checked against sim/real3d's event trace over 400 VF3 and 400 Bass Fishing
// frames (sim/real3d/tb_r3d_walker.cpp).

module r3d_walker #(
  // return stack: 32 frames, one MLAB deep (MAME's matrix stack depth reached
  // 11; the geometry's matrix stack holds 32 levels, list frames nest at most 3)
  parameter int RS_LOG2 = 5,
  parameter int VP_LOG2 = 6,   // viewport chain: 64 entries (VF3 reaches 28)
  // Supermodel-style node culling: after a node's matrix, emit CULLQ (type 0,
  // a = node word 7, the radius) and wait for the geometry's verdict; an
  // outside node keeps its push/pop and sibling but its child is skipped.
  parameter bit CULL = 1'b0,
  // Supermodel's colour-table base (study D-19, m2-notes row 12): a node with
  // node[0] & 4 sets it from node words 3, 7, 8; it stays until the next such
  // node, and starts each frame at 0x400. MODEL events carry it in b
  // (CTAB = 0: always 0x400, MAME's fixed table).
  parameter bit CTAB = 1'b1,
  // VP_M10K (area pass, study R164): the viewport chain in one M10K, read registered at vp_idx's next value, in
  // place of four MLABs read asynchronously at vp_idx. The word is the same in every clock that uses it: those
  // are S_VP_SEL to S_VP_EMIT, all after the S_VP_NEXT that sets vp_idx, and no write comes after the chain is
  // collected. 0 keeps the MLAB.
  parameter bit VP_M10K = 1'b0
) (
  input  logic        clk,
  input  logic        rst,

  input  logic        start,          // walk one frame
  input  logic        step15,         // Step 1.5 and later: ten-word nodes
  output logic        busy,
  output logic        overflow,       // a return-stack or chain limit was hit (sticky until start)

  output logic        mem_req,
  output logic [23:0] mem_addr,
  output logic [2:0]  mem_len,        // words - 1, 0..6
  input  logic        mem_ack,
  input  logic        mem_rvalid,
  input  logic [31:0] mem_rdata,

  output logic        ev_valid,
  input  logic        ev_ready,
  output logic [2:0]  ev_type,
  output logic [31:0] ev_a,
  output logic [31:0] ev_b,
  output logic [31:0] ev_c,

  input  logic        cull_valid,
  input  logic        cull_out
);

  localparam logic [2:0] EV_VP = 3'd1, EV_PUSH = 3'd2, EV_TRANS = 3'd3, EV_MAT = 3'd4,
                         EV_MODEL = 3'd5, EV_POP = 3'd6, EV_END = 3'd7, EV_CULLQ = 3'd0;

  localparam int RS_N = 1 << RS_LOG2;
  localparam int VP_N = 1 << VP_LOG2;

  typedef enum logic [4:0] {
    S_IDLE, S_RD, S_RD_DATA,
    S_VPC_RD, S_VPC_GOT, S_VP_NEXT, S_VP_SEL, S_VP_W22, S_VP_W2, S_VP_EMIT,
    S_LINK, S_BLK_RD, S_BLK_RD2, S_BLK_GOT, S_BLK_XF, S_BLK_FRAME, S_BLK_L4, S_BLK_CQ, S_BLK_CW, S_BLK_SKIP,
    S_LST_START, S_LST_SCAN, S_LST_SCAN_GOT, S_LST_PUSH, S_LST_ENT,
    S_RET, S_RET_POP, S_END
  } state_t;

  // ---- read helper -------------------------------------------------------
  logic [31:0] rd_addr;
  logic [2:0]  rd_len;          // words - 1
  logic [2:0]  rd_beat;
  logic [2:0]  rd_base;         // first rbuf word of the read
  logic [31:0] rbuf [0:7];
  logic        rd_hi, rd_lo;
  logic        rd_past;         // beat rd_beat is past the region's end

  // MAME get_memory_pointer: bit 23 set -> 8E if below 0x840000, else invalid;
  // bit 23 clear -> 8C if below 0x100000, else invalid. The regions are 2^18
  // and 2^20 words, so offset + beat runs past the end only from the region's
  // last eight words: a 4-bit sum, not a 21-bit one
  always_comb begin
    rd_hi   = (rd_addr[31:18] == 14'h0020);
    rd_lo   = (rd_addr[31:20] == 12'h000);
    rd_past = (rd_hi ? &rd_addr[17:3] : &rd_addr[19:3]) && ({1'b0, rd_addr[2:0]} + {1'b0, rd_beat} >= 4'd8);
  end

  // ---- walker state -------------------------------------------------------
  logic [1:0]  pass;
  // Written only while the chain is collected, read only afterwards (no_rw_check).
  // {priority, address} in one memory: VP_M10K one M10K, else MLAB (M10K was the binding resource)
  logic [31:0] vp_addr_q;
  logic [1:0]  vp_pri_q;
  logic [VP_LOG2:0] vp_cnt, vp_idx;
  state_t st, rd_ret;
  logic        vp_we;
  logic [33:0] vp_wd;
  generate if (VP_M10K) begin : g_vpm
    (* ramstyle = "M10K, no_rw_check" *) logic [33:0] vp_mem [0:VP_N-1];
    logic [33:0] vp_q;
    // vp_idx's next value where S_VP_NEXT moves it to S_VP_SEL (vp_idx - 1); elsewhere vp_idx itself, which
    // the states that read the word hold. S_VP_NEXT's wrap (vp_idx 0 to vp_cnt) is followed by S_VP_NEXT
    // again, which reads nothing, so its address does not matter
`ifdef WALKER_FAULT_VPRA
    wire [VP_LOG2:0] vp_ra = vp_idx;
`else
    wire [VP_LOG2:0] vp_ra = (st == S_VP_NEXT) ? vp_idx - 1'b1 : vp_idx;
`endif
    always_ff @(posedge clk) begin
`ifdef WALKER_FAULT_WE63
      if (vp_we && vp_cnt[VP_LOG2-1:0] != '1) vp_mem[vp_cnt[VP_LOG2-1:0]] <= vp_wd;
`else
      if (vp_we) vp_mem[vp_cnt[VP_LOG2-1:0]] <= vp_wd;
`endif
      vp_q <= vp_mem[vp_ra[VP_LOG2-1:0]];
    end
    assign {vp_pri_q, vp_addr_q} = vp_q;
`ifdef VERILATOR
`ifndef WALKER_NOCHK
    // the MLAB's word beside it: every state that reads the word must see the same
    logic [33:0] vp_sh [0:VP_N-1];
    always_ff @(posedge clk) if (vp_we) vp_sh[vp_cnt[VP_LOG2-1:0]] <= vp_wd;
    always_ff @(posedge clk)
      if (!rst && (st == S_VP_SEL || st == S_VP_W22 || st == S_VP_W2 || st == S_VP_EMIT) &&
          vp_q != vp_sh[vp_idx[VP_LOG2-1:0]])
        $fatal(1, "walker: vp_mem %09X against %09X at %0d", vp_q, vp_sh[vp_idx[VP_LOG2-1:0]], vp_idx);
`endif
`endif
  end else begin : g_vpl
    (* ramstyle = "MLAB, no_rw_check" *) logic [33:0] vp_mem [0:VP_N-1];
    always_ff @(posedge clk) if (vp_we) vp_mem[vp_cnt[VP_LOG2-1:0]] <= vp_wd;
    assign {vp_pri_q, vp_addr_q} = vp_mem[vp_idx[VP_LOG2-1:0]];
  end endgenerate
  logic [31:0] vaddr;           // chain walk
  // S_VPC_GOT's write (outside the reset, as before)
  assign vp_we = !rst && st == S_VPC_GOT;
  assign vp_wd = {rbuf[0][4:3], vaddr};
  logic [31:0] mbase;           // matrix base of the current viewport
  logic [31:0] link;            // link being processed
  // a node's words stay in rbuf from S_BLK_GOT to S_BLK_FRAME:
  // 0 flags, 1 matrix index, 2-4 translation, 5 child link, 6 sibling link
  logic [23:0] baddr, lbase;
  logic [19:0] lidx;
  logic [1:0]  ldepth;
  logic [19:0] ctab;

  // return stack: {is_list, has_sibling, payload[43:0]}
  //   block: payload[31:0] = sibling link
  //   list:  payload[43:20] = list base, payload[19:0] = next index (entries left)
  // One write port, so it maps to MLAB (an unregistered read of the top). S_RET
  // reads the top and rewrites it in the same cycle; it uses the value from
  // before the edge, so read-during-write is don't-care (no_rw_check). Without
  // it Quartus 17 leaves the stack in registers: 3,750 of them, 3,080 ALM.
  (* ramstyle = "MLAB, no_rw_check" *) logic [45:0] rs_mem [0:RS_N-1];
  logic [RS_LOG2:0] rs_sp;
  logic [45:0] rs_top;
  logic        rs_we;
  logic [RS_LOG2-1:0] rs_wa;
  logic [45:0] rs_wd;
  // the top's index rs_sp - 1 kept as a register beside rs_sp (study R109: the decrement was in front of
  // the MLAB's read address, the frame's add and rd_addr)
  logic [RS_LOG2:0] rs_spm1;
  assign rs_top = rs_mem[rs_spm1[RS_LOG2-1:0]];
`ifdef VERILATOR
  always_ff @(posedge clk) if (!rst && rs_spm1 != rs_sp - 1'b1) $fatal(1, "walker: rs_spm1");
`endif

  // the FSM's three writes: push a block frame, push a list frame, step a list
  always_comb begin
    rs_we = 1'b0;
    rs_wa = rs_sp[RS_LOG2-1:0];
    rs_wd = {1'b0, (rbuf[0][2:0] != 3'd6), 12'd0, rbuf[6]};
    case (st)
      S_BLK_FRAME, S_BLK_SKIP: rs_we = 1'b1;
      S_LST_PUSH: begin
        rs_we = 1'b1;
        rs_wd = {1'b1, 1'b0, lbase, lidx};
      end
      S_RET: if (rs_sp != '0 && rs_top[45] && rs_top[19:0] != 20'd0) begin
        rs_we = 1'b1;
        rs_wa = rs_spm1[RS_LOG2-1:0];
        rs_wd = {rs_top[45:20], rs_top[19:0] - 20'd1};
      end
      default: ;
    endcase
  end

  always_ff @(posedge clk)
    if (rs_we) rs_mem[rs_wa] <= rs_wd;

  // the viewport's word 22, then word 2: one adder
  logic [31:0] vp_wa;
  assign vp_wa = vp_addr_q + ((st == S_VP_SEL) ? 32'd22 : 32'd2);

  logic        ev_free;
  assign ev_free = !ev_valid || ev_ready;
  assign busy = (st != S_IDLE);

  always_ff @(posedge clk) begin
    if (rst) begin
      st <= S_IDLE;
      mem_req <= 1'b0;
      ev_valid <= 1'b0;
      overflow <= 1'b0;
      rs_sp <= '0; rs_spm1 <= '1;
      ldepth <= '0;
    end else begin
      if (ev_valid && ev_ready) ev_valid <= 1'b0;

      case (st)
        S_IDLE: if (start) begin
          overflow <= 1'b0;
          rd_base <= 3'd0;
          rs_sp <= '0; rs_spm1 <= '1;
          ldepth <= '0;
          pass <= 2'd0;
          ctab <= 20'h400;
          vp_cnt <= '0;
          vaddr <= 32'h0080_0000;
          st <= S_VPC_RD;
        end

        // ---- read rd_len+1 words at rd_addr into rbuf, then go to rd_ret
        S_RD: begin
          if (!(rd_hi || rd_lo)) begin
            for (int k = 0; k < 8; k++) rbuf[k] <= 32'd0;
            rd_base <= 3'd0;
            st <= rd_ret;
          end else if (!mem_req) begin
            mem_req  <= 1'b1;
            mem_addr <= rd_addr[23:0];
            mem_len  <= rd_len;
            rd_beat  <= 3'd0;
          end else if (mem_ack) begin
            mem_req <= 1'b0;
            st <= S_RD_DATA;
          end
        end
        S_RD_DATA: if (mem_rvalid) begin
          // a word past the end of its region reads as zero
          rbuf[rd_base + rd_beat] <= rd_past ? 32'd0 : mem_rdata;
          rd_beat <= rd_beat + 3'd1;
          if (rd_beat == rd_len) begin st <= rd_ret; rd_base <= 3'd0; end
        end

        // ---- viewport chain: collect, then draw last to first per pass
        S_VPC_RD: begin
          rd_addr <= vaddr; rd_len <= 3'd1; rd_ret <= S_VPC_GOT; st <= S_RD;
        end
        S_VPC_GOT: begin
          vp_cnt <= vp_cnt + 1'b1;
          if (rbuf[1] != 32'h0100_0000 && rbuf[1] != 32'd0) begin
            if (vp_cnt + 1'b1 == VP_N[VP_LOG2:0]) begin   // chain longer than the buffer
              overflow <= 1'b1;
              vp_idx <= vp_cnt + 1'b1;
              st <= S_VP_NEXT;
            end else begin
              vaddr <= rbuf[1];
              st <= S_VPC_RD;
            end
          end else begin
            vp_idx <= vp_cnt + 1'b1;
            st <= S_VP_NEXT;
          end
        end
        S_VP_NEXT: begin
          if (vp_idx == '0) begin
            if (pass == 2'd3) st <= S_END;
            else begin
              pass <= pass + 2'd1;
              vp_idx <= vp_cnt;
            end
          end else begin
            vp_idx <= vp_idx - 1'b1;
            st <= S_VP_SEL;
          end
        end
        S_VP_SEL: begin
          if (vp_pri_q != pass) st <= S_VP_NEXT;
          else begin
            rd_addr <= vp_wa; rd_len <= 3'd0; rd_ret <= S_VP_W22; st <= S_RD;
          end
        end
        S_VP_W22: begin
          mbase <= rbuf[0];
          rd_addr <= vp_wa; rd_len <= 3'd0; rd_ret <= S_VP_W2; st <= S_RD;
        end
        S_VP_W2: begin
          link <= rbuf[0];
          st <= S_VP_EMIT;
        end
        S_VP_EMIT: if (ev_free) begin
          ev_valid <= 1'b1; ev_type <= EV_VP;
          ev_a <= vp_addr_q; ev_b <= mbase; ev_c <= {30'd0, pass};
          st <= S_LINK;
        end

        // ---- process_link
        S_LINK: begin
          if (link == 32'd0 || link == 32'h0FFF_FFFF || link == 32'h0080_0800 || link == 32'h0100_0000)
            st <= S_RET;
          else case (link[31:24])
            8'h00: begin baddr <= link[23:0]; st <= S_BLK_RD; end
            8'h01, 8'h03: if (ev_free) begin
              ev_valid <= 1'b1; ev_type <= EV_MODEL; ev_a <= {8'd0, link[23:0]}; ev_b <= {12'd0, ctab};
              st <= S_RET;
            end
            8'h04: begin lbase <= link[23:0]; st <= S_LST_START; end
            default: st <= S_RET;
          endcase
        end

        // ---- draw_block
        S_BLK_RD: begin
          if (rs_sp[RS_LOG2]) begin           // no room for its frame: the node is skipped
            overflow <= 1'b1;
            st <= S_RET;
          end else begin
            rd_addr <= {8'd0, baddr}; rd_len <= step15 ? 3'd0 : CULL ? 3'd7 : 3'd6; rd_base <= 3'd0;
            rd_ret <= step15 ? S_BLK_RD2 : S_BLK_GOT; st <= S_RD;
          end
        end
        S_BLK_RD2: begin                               // Step 1.5: words 3-9 into rbuf 1-7
          rd_addr <= {8'd0, baddr} + 32'd3; rd_len <= CULL ? 3'd6 : 3'd5; rd_base <= 3'd1;
          rd_ret <= S_BLK_GOT; st <= S_RD;
        end
        S_BLK_GOT: if (ev_free) begin
          ev_valid <= 1'b1; ev_type <= EV_PUSH;
          // (n3 >> 19) | (n7 >> 28) << 13 | (n8 >> 25) << 17, 20 bits
          if (CTAB && rbuf[0][2]) ctab <= {rbuf[6][27:25], rbuf[5][31:28], rbuf[1][31:19]};
          st <= S_BLK_XF;
        end
        S_BLK_XF: begin
          if (rbuf[0][4]) begin
            if (ev_free) begin
              ev_valid <= 1'b1; ev_type <= EV_TRANS; ev_a <= rbuf[2]; ev_b <= rbuf[3]; ev_c <= rbuf[4];
              st <= CULL ? S_BLK_CQ : S_BLK_FRAME;
            end
          end else if (rbuf[1][11:0] != 12'd0) begin
            if (ev_free) begin
              ev_valid <= 1'b1; ev_type <= EV_MAT;
              ev_a <= mbase + {17'd0, rbuf[1][11:0], 3'b000} + {18'd0, rbuf[1][11:0], 2'b00};  // + 12 * index
              st <= CULL ? S_BLK_CQ : S_BLK_FRAME;
            end
          end else st <= CULL ? S_BLK_CQ : S_BLK_FRAME;
        end
        S_BLK_CQ: if (ev_free) begin
          ev_valid <= 1'b1; ev_type <= EV_CULLQ; ev_a <= {16'd0, rbuf[7][15:0]};
          st <= S_BLK_CW;
        end
        S_BLK_CW: if (cull_valid) st <= cull_out ? S_BLK_SKIP : S_BLK_FRAME;
        S_BLK_SKIP: begin                              // frame only: POP and sibling still follow
          rs_sp <= rs_sp + 1'b1; rs_spm1 <= rs_sp;
          st <= S_RET;
        end
        S_BLK_FRAME: begin
          rs_sp <= rs_sp + 1'b1; rs_spm1 <= rs_sp;                       // frame written by the port above
          if (rbuf[0][3]) begin
            rd_addr <= {8'd0, rbuf[5][23:0]}; rd_len <= 3'd0; rd_ret <= S_BLK_L4; st <= S_RD;
          end else begin
            link <= rbuf[5];
            st <= S_LINK;
          end
        end
        S_BLK_L4: if (ev_free) begin
          ev_valid <= 1'b1; ev_type <= EV_MODEL; ev_a <= {8'd0, rbuf[0][23:0]}; ev_b <= {12'd0, ctab};
          st <= S_RET;
        end

        // ---- traverse_list: scan to the end, then walk back from the return stack
        S_LST_START: begin
          if (ldepth == 2'd3) st <= S_RET;    // MAME: m_list_depth > 2
          else if (rs_sp[RS_LOG2]) begin
            overflow <= 1'b1;
            st <= S_RET;
          end else begin
            ldepth <= ldepth + 2'd1;
            lidx <= 20'd0;
            st <= S_LST_SCAN;
          end
        end
        S_LST_SCAN: begin
          rd_addr <= {8'd0, lbase} + {12'd0, lidx}; rd_len <= 3'd0; rd_ret <= S_LST_SCAN_GOT; st <= S_RD;
        end
        S_LST_SCAN_GOT: begin
          if (rbuf[0][25]) begin                        // end flag: the entry is in the list
            lidx <= lidx + 20'd1;
            st <= S_LST_PUSH;
          end else if (rbuf[0] == 32'd0 || rbuf[0][31:24] != 8'd0) begin
            st <= S_LST_PUSH;
          end else if (lidx == 20'hF_FFFF) begin
            overflow <= 1'b1;
            st <= S_LST_PUSH;
          end else begin
            lidx <= lidx + 20'd1;
            st <= S_LST_SCAN;
          end
        end
        S_LST_PUSH: begin
          rs_sp <= rs_sp + 1'b1; rs_spm1 <= rs_sp;
          st <= S_RET;
        end
        S_LST_ENT: begin
          if (rbuf[0][23:0] != 24'd0 && rbuf[0][23:0] != 24'h80_0800) begin
            baddr <= rbuf[0][23:0];
            st <= S_BLK_RD;
          end else st <= S_RET;
        end

        // ---- return: pop a block (POP, then its sibling) or step a list
        S_RET: begin
          if (rs_sp == '0) st <= S_VP_NEXT;
          else if (!rs_top[45]) st <= S_RET_POP;
          else if (rs_top[19:0] == 20'd0) begin
            rs_sp <= rs_sp - 1'b1; rs_spm1 <= rs_spm1 - 1'b1;
            ldepth <= ldepth - 2'd1;
          end else begin                               // the port above steps the index
            rd_addr <= {8'd0, rs_top[43:20]} + {12'd0, rs_top[19:0] - 20'd1};
            rd_len <= 3'd0; rd_ret <= S_LST_ENT; st <= S_RD;
          end
        end
        S_RET_POP: if (ev_free) begin
          ev_valid <= 1'b1; ev_type <= EV_POP;
          link <= rs_top[31:0];
          rs_sp <= rs_sp - 1'b1; rs_spm1 <= rs_spm1 - 1'b1;
          st <= rs_top[44] ? S_LINK : S_RET;
        end

        S_END: if (ev_free) begin
          ev_valid <= 1'b1; ev_type <= EV_END;
          st <= S_IDLE;
        end

        default: st <= S_IDLE;
      endcase
    end
  end

endmodule
