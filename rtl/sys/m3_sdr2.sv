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
// The second SDRAM (study D-66, D-68): the CPU's work RAM on a memory of its
// own, so its fills and cast-outs no longer queue behind the Real3D's ROM and
// texture traffic on the first SDRAM. The bus's work-RAM transactions (m3_bus
// s_w2: both masters', the 53C810's included) come here instead of to the
// first SDRAM's port; everything else stays where it was.
//
//   m3_bus s_* (s_w2) --> m3_sdr_port --> m2_sdram port 0 --> SDRAM2 pins
//
// The port and the controller are the first SDRAM's, with port 0's options
// (write bursts, the line fill as one sixteen-column read from the critical
// word, posted line writes, fast dispatch, the bank hash), on the same clock,
// so a work-RAM access costs what it did, less the waiting for other ports.
// Port 0 is in m2_sdram's priority class (PRI / PRI_TOP): today it is alone,
// and when the sound board's clients come onto this memory (D-63, port 1 on)
// it stays ahead of them. Port 1 is that place, tied off.
//
// The work RAM keeps its SDRAM byte address, SDR_RAM (0x6400000-0x6BFFFFF), so
// the bus's s_sa needs no other base: on the 128 MB module that is the second
// chip, as snd2's map reserves for the CPU (R178: free from 0x4800000).
//
// The start-up probe (snd2's, R178): once the controller is up, four words at
// the work RAM's base are written (the controller's write port, which nothing
// else uses here) with four patterns and read back as their group on port 0,
// then the same with the patterns inverted. With no module the DQ lines float
// (weak pull-ups; a floating bus can hold the last value driven, so the four
// words differ from each other and from their inverses). SDRAM2_EN low counts
// as missing without a try. ok rises when it passed and m3_system holds the
// game in reset until then; missing is sticky, and the OSD says so every 4.5 s
// (info_req: hps_io's info 1, Model3.sv's "I," line). The probe owns port 0
// only before ok, while the bus is held: its address is a mux on port 0's
// (a register select), its request an OR (the port's is low then).
//
// The pins are always driven (d130, cl3/NOTES.md: a floating second SDRAM
// cost the 130 MHz build its margin): straight from m2_sdram's registers, so
// they pack into the I/O cells (sys_dual_sdram.tcl's FAST_OUTPUT_REGISTER).
// Idle is m2_sdram's NOP; on the 128 MB module nCS picks one chip or the
// other, so NOP is how both are left alone, as on the first SDRAM.

module m3_sdr2
  import ppc_cache_pkg::*;
#(
  parameter int          AW       = 26,
  parameter int          CLK_MHZ  = 120,
  parameter int          T_REFI   = 924,
  parameter int          T_RC     = 8,
  parameter int          T_RAS    = 6,
  parameter logic [26:0] PROBE_SA = 27'h640_0000,     // the probe's four words: the work RAM's base (SDR_RAM)
  parameter bit          L16      = 1'b1,
  parameter bit          LW1      = 1'b1,
  parameter bit          PAIR     = 1'b1,
  parameter bit          PW       = 1'b1,
  parameter bit          FWD      = 1'b1,
  parameter bit          FREQ     = 1'b1,
  parameter int          PW_DLY   = 8,
  parameter bit          BXOR     = 1'b1,
  parameter bit          FDSP     = 1'b1,
  parameter bit          HITRD    = 1'b0,
  parameter int          INFO_BIT = 28                // the OSD message's period: 2^(INFO_BIT+1) clocks (4.1 s at 130 MHz)
) (
  input  logic          clk,
  input  logic          mem_rst_n,     // the memory's reset (PLL lock)
  input  logic          ra_clr,        // m3_sdr_port's (the game's reset)
  input  logic          en,            // SDRAM2_EN (static)
  output logic          ok,            // the probe passed: the game may run
  output logic          missing,       // the probe failed (sticky)
  output logic          info_req,      // hps_io info_req: the OSD's message while missing

  // the bus's work-RAM transactions (m3_bus s_*, s_w2)
  input  logic          s_req,
  input  logic [1:0]    s_cmd,
  input  logic [AW:0]   s_sa,
  input  logic          s_swap,
  input  logic          s_zero,
  input  logic [3:0]    s_be,
  input  logic [31:0]   s_wdata,
  input  logic [3:0]    s_wbe,
  output logic          s_ack,
  output logic          s_wnext,
  output logic          s_rvalid,
  output logic [31:0]   s_rdata,

  // the pins
  input  logic [2:0]    rd_lat_sel,
  output logic          sd_cs_n, sd_ras_n, sd_cas_n, sd_we_n,
  output logic [1:0]    sd_ba,
  output logic [12:0]   sd_a,
  output logic [15:0]   sd_dq_o,
  output logic          sd_dq_oe,
  input  logic [15:0]   sd_dq_i
);

  logic        mem_ready;
  logic        en_s1, en_s2;
  always_ff @(posedge clk) begin en_s1 <= en; en_s2 <= en_s1; end

  // ---- the bus's port
  logic        sp_req, sp_we, sp_long, sp_lo, sp_ack;
  logic [AW:1] sp_addr;
  logic [15:0] sp_din;
  logic [1:0]  sp_be;
  logic [63:0] sp_dout;
  logic [3:0]  wb_col;
  logic [15:0] wb_din;
  logic [1:0]  wb_be;
  m3_sdr_port #(.AW(AW), .L16(L16), .LW1(LW1), .PAIR(PAIR), .RA(1'b0), .PW(PW), .FWD(FWD), .FREQ(FREQ), .PW_DLY(PW_DLY)) u_port (
    .clk, .rst(!mem_rst_n), .ra_clr,
    .s_req, .s_cmd, .s_sa, .s_swap, .s_zero, .s_be, .s_wdata, .s_wbe, .s_ack, .s_wnext, .s_rvalid, .s_rdata, .s_idle(),
    .p_req(sp_req), .p_we(sp_we), .p_addr(sp_addr), .p_din(sp_din), .p_be(sp_be), .p_long(sp_long),
    .p_dout(sp_dout), .p_lo(sp_lo), .p_ack(sp_ack), .wb_col, .wb_din, .wb_be);

  // ---- the start-up probe
  typedef enum logic [1:0] { PR_IDLE, PR_W, PR_R, PR_DONE } pr_t;
  pr_t         pr_s;
  logic        pr_act;          // the probe owns port 0 (register: the address mux's select)
  logic        pr_wreq, pr_rreq, pr_pass;
  logic  [1:0] pr_i;
  logic        pr_inv;
  logic [15:0] pr_d;
  logic        wr_ack;
  localparam logic [AW:1] PA = PROBE_SA[AW:1];      // 4-word aligned: the four words are {PA[AW:3], i}
  function automatic logic [15:0] ppat(input logic [1:0] i, input logic inv);
    logic [15:0] v;
    case (i) 2'd0: v = 16'h5AA5; 2'd1: v = 16'h3CC3; 2'd2: v = 16'h0FF0; default: v = 16'h9669; endcase
    ppat = inv ? ~v : v;
  endfunction
  always_ff @(posedge clk) begin
    if (!mem_rst_n) begin
      pr_s <= PR_IDLE; pr_act <= 1'b1; pr_wreq <= 1'b0; pr_rreq <= 1'b0; pr_i <= '0; pr_inv <= 1'b0;
      pr_pass <= 1'b1; pr_d <= '0; ok <= 1'b0; missing <= 1'b0;
    end else case (pr_s)
      PR_IDLE: if (mem_ready) begin
        if (!en_s2) begin pr_s <= PR_DONE; pr_pass <= 1'b0; end
        else begin pr_s <= PR_W; pr_i <= '0; end
      end
      PR_W: begin
        if (!pr_wreq && !wr_ack) begin pr_wreq <= 1'b1; pr_d <= ppat(pr_i, pr_inv); end
        else if (pr_wreq && wr_ack) begin
          pr_wreq <= 1'b0; pr_i <= pr_i + 2'd1;
          if (pr_i == 2'd3) pr_s <= PR_R;
        end
      end
      PR_R: begin
        if (!pr_rreq && !sp_ack) pr_rreq <= 1'b1;
        else if (pr_rreq && sp_ack) begin
          pr_rreq <= 1'b0;
          if (sp_dout != {ppat(2'd3, pr_inv), ppat(2'd2, pr_inv), ppat(2'd1, pr_inv), ppat(2'd0, pr_inv)}) pr_pass <= 1'b0;
          pr_inv <= ~pr_inv; pr_i <= '0;
          pr_s <= pr_inv ? PR_DONE : PR_W;
        end
      end
      default: begin
        pr_act <= 1'b0;
        ok <= pr_pass; missing <= !pr_pass;
      end
    endcase
  end
  logic [INFO_BIT:0] inf_cnt;
  always_ff @(posedge clk) begin
    if (!missing) begin inf_cnt <= '0; info_req <= 1'b0; end
    else begin inf_cnt <= inf_cnt + 1'd1; info_req <= inf_cnt[INFO_BIT:INFO_BIT-1] == 2'b01; end
  end

  // ---- the controller: port 0 the bus's (the CPU's work RAM; the probe's reads before ok), port 1 reserved
  localparam int NP = 2;
  logic [NP-1:0]         c_req, c_we, c_ack, c_lo, c_long;
  logic [NP-1:0][AW:1]   c_addr;
  logic [NP-1:0][15:0]   c_din;
  logic [NP-1:0][1:0]    c_be;
  logic [NP-1:0][63:0]   c_dout;
  assign c_req  = {1'b0, sp_req | pr_rreq};
  assign c_we   = {1'b0, sp_we && !pr_act};
  assign c_addr = {{AW{1'b0}}, pr_act ? PA : sp_addr};
  assign c_din  = {16'd0, sp_din};
  assign c_be   = {2'b00, sp_be};
  assign c_long = {1'b0, sp_long && !pr_act};
  assign sp_ack = c_ack[0]; assign sp_lo = c_lo[0]; assign sp_dout = c_dout[0];
  localparam logic [NP-1:0] P0 = NP'(1);
  // (area130) every address here is the work RAM's (8 MiB from PROBE_SA, SDR_RAM) or the probe's, so on the
  // 128 MB module all on the second chip (byte address bit 26): m2_sdram keeps row state for that chip alone
  // (DCHIP1). Port 1's clients (D-63) would end it.
  localparam bit W2_C1 = (PROBE_SA >= 27'h400_0000) && ({1'b0, PROBE_SA} + 28'h80_0000 <= 28'h800_0000);
  logic [1:0] sd_dqm_unused;
  logic       sd_cke_unused;
  m2_sdram #(.COL_BITS(10), .NP(NP), .T_REFI(T_REFI), .T_RC(T_RC), .T_RAS(T_RAS), .ACK_HOLD(1),
             .INIT_NOP(CLK_MHZ * 110), .DUAL(1'b1), .MASK_ON_A(1'b1),
             .PRI(P0), .PRI_TOP(P0), .BXOR(BXOR),
             .WB(P0), .L16(L16 ? P0 : NP'(0)), .PAIR(PAIR ? P0 : NP'(0)), .FDSP(FDSP), .HITRD(HITRD),
             .SHD(P0), .DCHIP1(W2_C1)) u_sdram (
    .clk, .rst_n(mem_rst_n), .ready(mem_ready), .rd_lat_sel,
    .sd_cke(sd_cke_unused), .sd_cs_n, .sd_ras_n, .sd_cas_n, .sd_we_n, .sd_ba, .sd_a, .sd_dqm(sd_dqm_unused),
    .sd_dq_o, .sd_dq_oe, .sd_dq_i,
    .wr_req(pr_wreq), .wr_addr({PA[AW:3], pr_i}), .wr_din(pr_d), .wr_be(2'b11), .wr_ack,
    .p_req(c_req), .p_we(c_we), .p_addr(c_addr), .p_din(c_din), .p_be(c_be), .p_dout(c_dout), .p_ack(c_ack),
    .p_long(c_long), .p_lo(c_lo), .wb_col, .wb_din, .wb_be,
`ifdef M3_R3B
    .wb_grant(),
`endif
    .dbg_req(), .dbg_grant());

  /* verilator lint_off UNUSEDSIGNAL */
  wire _unused = &{1'b0, sd_dqm_unused, sd_cke_unused, c_ack[1], c_lo[1], c_dout[1]};
  /* verilator lint_on UNUSEDSIGNAL */

`ifdef VERILATOR
  // the probe owns port 0 only while the bus has nothing there
  always_ff @(posedge clk)
    if (mem_rst_n && pr_act && (s_req || sp_req)) $fatal(1, "m3_sdr2: a bus request while the probe owns port 0");
`endif

endmodule
