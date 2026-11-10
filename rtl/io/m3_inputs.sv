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
// The cabinet: what the 315-5649 (m3_io) reads -- port B's two banks, ports C
// and D and the ADC's eight channels -- from MiSTer's joysticks, sticks,
// triggers and paddle, by the input PROFILE the MRA's game configuration
// gives (index 4, byte 20; m3_loader), so a new game adds an MRA and not
// mapping logic.
//
// Copied from my Model 2 core's rtl/io/m2_2a_inputs.sv (4d79cd47; THIRD_PARTY.md):
// the profile scheme (each profile wires MiSTer's joystick bits to the ports
// in one fixed order, which the MRA's <buttons names> follows), the stick's
// directions ORed into the d-pad (R879) and the analog cabinet block, here
// m3_cab_inputs. Where Model 3 differs: the ports are Model 3's (MAME 0.289
// model3.cpp INPUT_PORTS, Supermodel 036af1c Model3.cpp ReadInputs):
//   port B, bank 0 (IN0): 0 coin 1, 1 coin 2, 2 test A, 3 service A,
//     4 start 1, 5 start 2, 7:6 high; bank 1: 7 test B, 6 service B (m3_io
//     adds the EEPROM's DO); all active low;
//   port C (IN2) player 1, port D (IN3) player 2; ADC channels 0-7.
// The profiles (MAME and Supermodel agree unless said):
//   0 GENERIC (VF3; any MRA without byte 20): MAME's model3 ports, which are
//     Supermodel's joystick1/2 + fighting. C / D: 0 B1 1 B2 2 B3 3 B4
//     4 down 5 up 6 right 7 left, active low. VF3: Punch Kick Guard Escape.
//     J 4 B1 5 B2 6 B3 7 B4 8 Start 9 Coin 10 Test 11 Service (P2 the same on
//     joystick 1: coin 2, start 2, test B, service B). The MRA's idle bytes
//     (bytes 18-19) are ANDed in, so an old MRA reads as it did.
//   1 DRIVING (Scud Race, Daytona USA 2; DRIVE): m3_cab_inputs -- C the VR
//     buttons and the four-speed shifter, D FF, ADC 0 steering, 1 accelerator,
//     2 brake. J as Daytona's on Model 2 (m3_cab_inputs); coin 2 is joystick
//     1's Coin.
//   2 GUN (The Lost World; GUN): Supermodel's analog guns, the path the game
//     takes with no serial gun board (study R133, m3_io SER_ST 0x00). ADC 0
//     P1 X, 1 P1 Y, 2 P2 X, 3 P2 Y, each 0x80 + the stick's axis (left 0x00,
//     top 0x00: Supermodel's Lost World values after its Y inversion); C / D
//     bit 0 the trigger, bit 1 Supermodel's second trigger, active low (MAME:
//     bit 0 only). A USB or Sinden gun on MiSTer is a joystick with absolute
//     axes, so the left stick is the gun. J 4 Trigger 5 Trigger 2 6 Start
//     7 Coin 8 Test 9 Service, player 2 on joystick 1.
//   3 FISHING (Sega Bass Fishing; FISH): ADC 0 rod Y, 1 rod X (left stick,
//     0x80 + axis), 2 tension (0: Get Bass only), 3 reel (RT's 0..255, the
//     Reel button 0xFF), 4 stick Y, 5 stick X (the right stick, the d-pad a
//     full push). Cast and Select: Supermodel's bassdx wiring, C bits 4 / 5
//     ACTIVE HIGH (port C rests at CF; Supermodel's controls fix, 6627b1e),
//     or with flags bit 0 MAME's, bits 0 / 1 active low. They disagree; the
//     board decides. J 4 Cast 5 Select 6 Reel 7 Start 8 Coin 9 Test 10 Service.
// Channels a profile does not drive read 0x00 (Supermodel's).
// With a parameter off, its profile reads as GENERIC with nothing pressed on
// the ADC: the core before this block, channel for channel.

module m3_inputs #(
  parameter int unsigned SYS_MHZ = 110,
  parameter bit          DRIVE   = 1'b1,  // profile 1, m3_cab_inputs
  parameter bit          GUN     = 1'b1,  // profile 2
  parameter bit          FISH    = 1'b1,  // profile 3
  parameter bit          LEAN    = 1'b0   // m3_cab_inputs without Model 2's sensitivity, smoothing and wheel modes
) (
  input  logic        clk,          // clk_sys
  input  logic        rst_n,        // the game reset
  input  logic  [7:0] prof,         // the profile (configuration byte 20)
  input  logic  [7:0] pflg,         // its flags (byte 21)
  input  logic  [7:0] idle_c,       // the MRA's idle ports C and D (bytes 18-19)
  input  logic  [7:0] idle_d,
  input  logic [15:0] joystick_0,
  input  logic [15:0] joystick_1,
  input  logic [15:0] joy_analog,   // player 1's left stick (registered by the caller)
  input  logic [15:0] joy_ranalog,  // player 1's right stick
  input  logic [15:0] joy_analog_1, // player 2's left stick
  input  logic  [7:0] paddle,
  input  logic  [7:0] trig_rt,      // player 1's analogue triggers (hps_io 0x46)
  input  logic  [7:0] trig_lt,
  input  logic        st_pedswap,   // the driving OSD lines (m3_cab_inputs)
  input  logic  [1:0] st_ssens,
  input  logic  [1:0] st_ssm,
  input  logic        st_rsteer,
  input  logic  [1:0] st_wheel,
  output logic  [7:0] in0,          // port B bank 0
  output logic        test_b_n,     // port B bank 1
  output logic        service_b_n,
  output logic  [7:0] in2,          // port C
  output logic  [7:0] in3,          // port D
  output logic  [7:0][7:0] an,      // the ADC
  output logic        drive         // the driving profile (the OSD's lines)
);

  wire [15:0] j0 = joystick_0;
  wire [15:0] j1 = joystick_1;

  // ---- the profile (quasi-static: set once per load, the game in reset)
  wire p_drv  = DRIVE && (prof == 8'd1);
  wire p_gun  = GUN   && (prof == 8'd2);
  wire p_fish = FISH  && (prof == 8'd3);
  assign drive = p_drv;

  // Model 2 R879: a pad's left stick drives the d-pad's directions too, past
  // a dead zone of 48 of 127 (X 7:0, Y 15:8, signed; d-pad bits 3:0 up,
  // down, left, right).
  function automatic logic [3:0] adir(input logic [15:0] a);
    logic signed [7:0] x, y;
    x = a[7:0]; y = a[15:8];
    adir = {y < -8'sd48, y > 8'sd48, x < -8'sd48, x > 8'sd48};
  endfunction
  wire [3:0] d0 = j0[3:0] | adir(joy_analog);
  wire [3:0] d1 = j1[3:0] | adir(joy_analog_1);

  // ---- GENERIC: MAME's model3 ports
  wire [7:0] gn_in0 = ~{2'b00, j1[8], j0[8], j0[11], j0[10], j1[9], j0[9]};
  wire [7:0] gn_in2 = idle_c & ~{d0[1], d0[0], d0[3], d0[2], j0[7], j0[6], j0[5], j0[4]};
  wire [7:0] gn_in3 = idle_d & ~{d1[1], d1[0], d1[3], d1[2], j1[7], j1[6], j1[5], j1[4]};

  // ---- DRIVING
  wire [7:0] dr_in0 = ~{2'b00, j1[5], j0[5], j0[7], j0[6], j1[4], j0[4]};
  wire [7:0] dr_c, dr_st, dr_ac, dr_br;
  generate if (DRIVE) begin : g_drive
    m3_cab_inputs #(.SYS_MHZ(SYS_MHZ), .SENS(!LEAN), .SMOOTH(!LEAN), .WHEEL(!LEAN)) u_cab (
      .clk, .rst_n, .joystick_0(j0), .joy_analog, .joy_ranalog, .paddle,
      .st_pedswap, .st_ssens, .st_ssm, .st_rsteer, .st_wheel,
      .trig_rt, .trig_lt,
      .pc(dr_c), .steer(dr_st), .accel(dr_ac), .brake(dr_br));
  end else begin : g_nodrive
    assign dr_c = 8'hFF; assign dr_st = 8'h80; assign dr_ac = 8'h00; assign dr_br = 8'h00;
  end endgenerate

  // ---- GUN: offset binary, the sign bit inverted (0x80 + v)
  wire [7:0] gu_in0 = ~{2'b00, j1[6], j0[6], j0[9], j0[8], j1[7], j0[7]};
  wire [7:0] gu_in2 = ~{6'b000000, j0[5], j0[4]};
  wire [7:0] gu_in3 = ~{6'b000000, j1[5], j1[4]};
  function automatic logic [7:0] ob(input logic [7:0] v);
    return {~v[7], v[6:0]};
  endfunction

  // ---- FISHING
  wire [7:0] fi_in0 = ~{2'b00, j1[7], j0[7], j0[10], j0[9], j1[8], j0[8]};
  wire [7:0] fi_in2 = pflg[0] ? ~{6'b000000, j0[5], j0[4]}             // MAME: bits 0 / 1, active low
                              : ~{2'b00, ~j0[5], ~j0[4], 4'b0000};     // Supermodel bassdx: 4 / 5, active high
  // the cabinet's stick: the right stick, the d-pad a full push
  wire [7:0] fi_sx = j0[0] ? 8'hFF : j0[1] ? 8'h00 : ob(joy_ranalog[7:0]);
  wire [7:0] fi_sy = j0[2] ? 8'hFF : j0[3] ? 8'h00 : ob(joy_ranalog[15:8]);
  wire [7:0] fi_reel = j0[6] ? 8'hFF : trig_rt;

  always_ff @(posedge clk) begin
    an <= '0;
    test_b_n <= 1'b1; service_b_n <= 1'b1;
    if (p_drv) begin
      in0 <= dr_in0; in2 <= dr_c; in3 <= idle_d;
      an[0] <= dr_st; an[1] <= dr_ac; an[2] <= dr_br;
    end else if (p_gun) begin
      in0 <= gu_in0; in2 <= gu_in2; in3 <= gu_in3;
      an[0] <= ob(joy_analog[7:0]);   an[1] <= ob(joy_analog[15:8]);
      an[2] <= ob(joy_analog_1[7:0]); an[3] <= ob(joy_analog_1[15:8]);
    end else if (p_fish) begin
      in0 <= fi_in0; in2 <= fi_in2; in3 <= idle_d;
      an[0] <= ob(joy_analog[15:8]); an[1] <= ob(joy_analog[7:0]);
      an[3] <= fi_reel; an[4] <= fi_sy; an[5] <= fi_sx;
    end else begin
      in0 <= gn_in0; in2 <= gn_in2; in3 <= gn_in3;
      test_b_n <= ~j1[10]; service_b_n <= ~j1[11];
    end
  end

  /* verilator lint_off UNUSEDSIGNAL */
  wire _unused = &{1'b0, pflg[7:1], j1[15:12]};
  /* verilator lint_on UNUSEDSIGNAL */

endmodule
