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
// The fetch predecode's fields that came from a second ppc_decode instance
// (FPA = 0, step 0), as flat tables of the raw word: opcode alone, or the
// extended opcode for opcodes 19 and 31, and two fields that also read rA = 0
// (a_used) and Rc (rc). The decoder's case structure was five or six LUT
// levels from i_data into the fetch queue (the R63 full builds' predecode
// path); a table of the opcode or the extended opcode, chosen by the opcode,
// is fewer. Generated from that decoder by an exhaustive sweep (every opcode,
// every extended opcode, both values of rA = 0 and Rc), and checked against
// it on every fetched word in simulation (ppc_core.sv). Regenerate it if
// ppc_decode changes. ROM recognition is off: the tables are logic.
//
// Each table word: [0] a_used, [1] b_used, [2] s_used, [3] x_from_s,
// [4] a_zero, [5] bsel = rB, [6] rc, [7] set_ca or oe, [8] fpw, [9] fhi,
// [10] fuc, [11] fdbl, [12] op = illegal, [15:13] y's immediate form,
// [20:16] result kind, [26:21] op, [27] a_used when rA != 0, [28] rc when
// Rc = 1 (ppc_core.sv's predecode, pd_t).

(* altera_attribute = "-name AUTO_ROM_RECOGNITION OFF" *)
module ppc_pdec (
  input  logic [31:0] w,
  output logic        ua, ub, us, xs, az, yreg, rc, xer, fpw, fhi, fuc, fdbl, ill,
  output logic [2:0]  immk,
  output logic [4:0]  rk,
  output logic [5:0]  op
);

  wire  [5:0]  opc = w[31:26];
  wire  [9:0]  x   = w[10:1];
  logic [28:0] vo, v19, v31, v;

  always_comb begin
      case (opc)
        6'd3: vo = 29'h04C00001;
        6'd7: vo = 29'h02680001;
        6'd8: vo = 29'h00200081;
        6'd10: vo = 29'h02402001;
        6'd11: vo = 29'h02200001;
        6'd12: vo = 29'h00000081;
        6'd13: vo = 29'h000000C1;
        6'd14: vo = 29'h08000010;
        6'd15: vo = 29'h08004010;
        6'd16, 6'd18: vo = 29'h04200020;
        6'd17: vo = 29'h04800020;
        6'd20: vo = 29'h11A4600D;
        6'd21: vo = 29'h11A4600C;
        6'd23: vo = 29'h11A4002E;
        6'd24: vo = 29'h0071200C;
        6'd25: vo = 29'h0071400C;
        6'd26: vo = 29'h0092200C;
        6'd27: vo = 29'h0092400C;
        6'd28: vo = 29'h0050204C;
        6'd29: vo = 29'h0050404C;
        6'd32, 6'd34, 6'd40, 6'd42: vo = 29'h0C400010;
        6'd33, 6'd35, 6'd41, 6'd43: vo = 29'h0C400000;
        6'd36, 6'd38, 6'd44: vo = 29'h0C600014;
        6'd37, 6'd39, 6'd45: vo = 29'h04600005;
        6'd46: vo = 29'h0C408010;
        6'd47: vo = 29'h0C608014;
        6'd48: vo = 29'h0C400110;
        6'd49: vo = 29'h04400101;
        6'd50: vo = 29'h0C400A10;
        6'd51: vo = 29'h04400A01;
        6'd52: vo = 29'h0C600410;
        6'd53: vo = 29'h04600401;
        6'd54: vo = 29'h0C600C10;
        6'd55: vo = 29'h04600C01;
        default: vo = 29'h05C01020;
      endcase
      case (x)
        10'd0, 10'd33, 10'd129, 10'd193, 10'd225, 10'd257, 10'd289, 10'd417, 10'd449: v19 = 29'h03E00020;
        10'd16, 10'd528: v19 = 29'h04200020;
        10'd50: v19 = 29'h04A00020;
        10'd150: v19 = 29'h05600020;
        default: v19 = 29'h05C01020;
      endcase
      case (x)
        10'd0: v31 = 29'h02200023;
        10'd4: v31 = 29'h04C00023;
        10'd8, 10'd136, 10'd520, 10'd552, 10'd648: v31 = 29'h102000A3;
        10'd10, 10'd138, 10'd522, 10'd650, 10'd778: v31 = 29'h100000A3;
        10'd11, 10'd523: v31 = 29'h12A80023;
        10'd19: v31 = 29'h03690020;
        10'd20, 10'd23, 10'd54, 10'd86, 10'd87, 10'd246, 10'd278, 10'd279, 10'd343, 10'd534, 10'd790: v31 = 29'h0C400032;
        10'd24: v31 = 29'h1141002E;
        10'd26: v31 = 29'h11C5002C;
        10'd28: v31 = 29'h1050002E;
        10'd32: v31 = 29'h02400023;
        10'd40: v31 = 29'h10200023;
        10'd55, 10'd119, 10'd311, 10'd375: v31 = 29'h0C400022;
        10'd60: v31 = 29'h1116002E;
        10'd75, 10'd587: v31 = 29'h12880023;
        10'd83: v31 = 29'h03A90020;
        10'd104: v31 = 29'h1020A001;
        10'd124: v31 = 29'h10D4002E;
        10'd144: v31 = 29'h03800024;
        10'd146: v31 = 29'h03C00024;
        10'd150, 10'd151, 10'd215, 10'd407, 10'd662, 10'd918: v31 = 29'h0C600036;
        10'd183, 10'd247, 10'd439: v31 = 29'h04600027;
        10'd200, 10'd616, 10'd712: v31 = 29'h1020A081;
        10'd202, 10'd714: v31 = 29'h1000A081;
        10'd210: v31 = 29'h04EAE004;
        10'd232, 10'd744: v31 = 29'h1020C081;
        10'd234, 10'd746: v31 = 29'h1000C081;
        10'd235: v31 = 29'h12680023;
        10'd242: v31 = 29'h04EA0026;
        10'd266: v31 = 29'h10000023;
        10'd284: v31 = 29'h10F5002E;
        10'd306: v31 = 29'h052A0022;
        10'd316: v31 = 29'h1092002E;
        10'd339, 10'd371: v31 = 29'h03290020;
        10'd412: v31 = 29'h1137002E;
        10'd444: v31 = 29'h1071002E;
        10'd459: v31 = 29'h12E80023;
        10'd467: v31 = 29'h03400024;
        10'd470, 10'd1014: v31 = 29'h0C600032;
        10'd476: v31 = 29'h10B3002E;
        10'd491: v31 = 29'h12C80023;
        10'd512: v31 = 29'h04000020;
        10'd535: v31 = 29'h0C400132;
        10'd536: v31 = 29'h1162002E;
        10'd566, 10'd598, 10'd854: v31 = 29'h05A00020;
        10'd567: v31 = 29'h04400123;
        10'd595: v31 = 29'h050AE000;
        10'd599: v31 = 29'h0C400A32;
        10'd631: v31 = 29'h04400A23;
        10'd659: v31 = 29'h050A0022;
        10'd663, 10'd983: v31 = 29'h0C600432;
        10'd695: v31 = 29'h04600423;
        10'd727: v31 = 29'h0C600C32;
        10'd747: v31 = 29'h126800A3;
        10'd759: v31 = 29'h04600C23;
        10'd792: v31 = 29'h118300AE;
        10'd824: v31 = 29'h1183608C;
        10'd922: v31 = 29'h1207002C;
        10'd954: v31 = 29'h11E6002C;
        10'd971: v31 = 29'h12E800A3;
        10'd978, 10'd1010: v31 = 29'h054A0022;
        10'd982: v31 = 29'h0DA00032;
        10'd1003: v31 = 29'h12C800A3;
        default: v31 = 29'h05C01020;
      endcase

  end
  assign v = opc == 6'd31 ? v31 : opc == 6'd19 ? v19 : vo;

  assign ua   = v[0] || (v[27] && w[20:16] != 5'd0);
  assign rc   = v[6] || (v[28] && w[0]);
  assign ub   = v[1];
  assign us   = v[2];
  assign xs   = v[3];
  assign az   = v[4];
  assign yreg = v[5];
  assign xer  = v[7];
  assign fpw  = v[8];
  assign fhi  = v[9];
  assign fuc  = v[10];
  assign fdbl = v[11];
  assign ill  = v[12];
  assign immk = v[15:13];
  assign rk   = v[20:16];
  assign op   = v[26:21];

endmodule
