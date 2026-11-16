// SPDX-License-Identifier: GPL-3.0-or-later
//
// Sega Model 3 core for MiSTer FPGA
// Copyright (C) 2026 alphanu1
//
// Bench top for m3_sdr2 (the second SDRAM: the CPU's work RAM, D-66 / D-68), as m3_system builds it, with the
// port bench's pins (tb_m3_sdr_port.cpp built with -DSDR2, --prefix Vm3_sdr_tb): ready is the start-up probe's
// ok. There is no second reader: m3_sdr2's port 1 (the sound board's place) is tied off inside, so q_ack stays
// low and the bench runs with --noq.
module m3_sdr2_tb #(parameter bit L16 = 1'b1, parameter bit BXOR = 1'b1, parameter bit LW1 = 1'b1, parameter bit PAIR = 1'b1,
  parameter bit PW = 1'b1, parameter bit FWD = 1'b1, parameter bit FREQ = 1'b1, parameter bit FDSP = 1'b1, parameter bit HITRD = 1'b0,
  parameter int PW_DLY = 8, parameter int CLK_MHZ = 2) (   // CLK_MHZ 2: the probe's power-up wait 220 clocks
  input  logic        clk,
  input  logic        rst_n,
  output logic        ready,
  input  logic        en,
  output logic        missing,
  output logic        info_req,
  input  logic        s_req,
  input  logic [1:0]  s_cmd,
  input  logic [26:0] s_sa,
  input  logic        s_swap,
  input  logic        s_zero,
  input  logic [3:0]  s_be,
  input  logic [31:0] s_wdata,
  input  logic [3:0]  s_wbe,
  output logic        s_ack,
  output logic        s_wnext,
  output logic        s_rvalid,
  output logic [31:0] s_rdata,
  output logic        s_idle,
  input  logic        q_req,
  input  logic [26:1] q_addr,
  output logic        q_ack,
  output logic [63:0] q_dout,
  output logic        sd_cke, sd_cs_n, sd_ras_n, sd_cas_n, sd_we_n,
  output logic [1:0]  sd_ba,
  output logic [12:0] sd_a,
  output logic [15:0] sd_dq_o,
  output logic        sd_dq_oe,
  input  logic [15:0] sd_dq_i
);
  m3_sdr2 #(.AW(26), .CLK_MHZ(CLK_MHZ), .T_REFI(770), .T_RC(7), .T_RAS(5), .L16(L16), .LW1(LW1), .PAIR(PAIR), .PW(PW),
            .FWD(FWD), .FREQ(FREQ), .PW_DLY(PW_DLY), .BXOR(BXOR), .FDSP(FDSP), .HITRD(HITRD), .INFO_BIT(10)) u_sdr2 (
    .clk, .mem_rst_n(rst_n), .ra_clr(1'b0), .en, .ok(ready), .missing, .info_req,
    .s_req, .s_cmd, .s_sa, .s_swap, .s_zero, .s_be, .s_wdata, .s_wbe, .s_ack, .s_wnext, .s_rvalid, .s_rdata,
    .rd_lat_sel(3'd2), .sd_cs_n, .sd_ras_n, .sd_cas_n, .sd_we_n, .sd_ba, .sd_a, .sd_dq_o, .sd_dq_oe, .sd_dq_i);
  assign sd_cke = 1'b1;
  // the port's idle, as the port bench waits on it: the port sees nothing between transactions here
  assign s_idle = u_sdr2.u_port.s_idle;
  assign q_ack = 1'b0;
  assign q_dout = '0;
  /* verilator lint_off UNUSEDSIGNAL */
  wire _unused = &{1'b0, q_req, q_addr};
  /* verilator lint_on UNUSEDSIGNAL */
endmodule
