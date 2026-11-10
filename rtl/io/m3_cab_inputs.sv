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
// The driving cabinet (Scud Race, Daytona USA 2): the 315-5649's port C and
// its ADC channels 0-2 from MiSTer's joystick, sticks, triggers and paddle.
//
// Copied from my Model 2 core's rtl/io/m2_cab_inputs.sv (4d79cd47; THIRD_PARTY.md),
// Daytona's block, which the board has run since R816-R981. The steering
// sources (left stick past a dead zone, right stick on "Right stick:
// Pedals+steer", the paddle, the d-pad's ramp), "Steering", "Steering
// smoothing", "Pedals", "Analog control" (Wheel A / B) and the analogue
// triggers are Model 2's, line for line in structure. Where Model 3 differs:
//   - the ADC's range is the whole byte. MAME 0.289 model3.cpp (scud,
//     daytona2) declares steering 0x80 at centre and the pedals 0x00 at rest
//     with no PORT_MINMAX, and Supermodel's axes are 0x00-0xFF; Model 2's
//     ports are PORT_MINMAX(0x20, 0xE0). So the steering is 0x80 + the axis
//     (full stick is full lock, "Steering: Full" is entry 0 here, where Model
//     2's 3/4 was), the pedals 0x00 at rest and 0xFF fully pressed (a
//     seven-bit magnitude m is {m, m[6]}), a trigger its own 0..255;
//   - the smoothing's rates are scaled by 4/3 (29 and 15 counts a frame, not
//     22 and 11), so lock to lock takes the frames it did on Model 2's
//     three-quarter range;
//   - the shifter is Model 3's four-speed H (Supermodel's shift4 encoding on
//     port C bits 7:4, active low: 1 = 0x50 low, 2 = 0x60, 3 = 0x10, 4 =
//     0x20, neutral none; MAME 0.289 gives each gear one bit, 0x40 / 0x80 /
//     0x10 / 0x20, and Supermodel is the emulator that drives these games),
//     stepped by Gear Up / Gear Down as Model 2's, starting in first gear
//     (Supermodel's start, and the AF the core's idle port C has read since
//     R100);
//   - the four VR (view) buttons are port C bits 3:0, active low (both
//     emulators); coins, test and service are m3_inputs', on port B;
//   - Desert Tank's map (is_desert, the turret, the lever) is Model 2 only and
//     is not taken;
//   - the OSD bits are used as hps_io gives them: hps_io runs on clk_sys here,
//     so Model 2's three-flop synchronisers do nothing but cost registers;
//   - SENS, SMOOTH and WHEEL build "Steering", "Steering smoothing" and
//     "Analog control" (on: Model 2's block); off, each reads as its default
//     (Full, Off, Standard), output for output, and its logic is not built.
// The button order is Daytona's on Model 2, so its MRAs' names carry over:
//   J 4 Coin 5 Start 6 Test 7 Service 8 VR1 9 VR4 10 Accel 11 Brake
//     12 Gear Up 13 Gear Down 14 VR2 15 VR3

module m3_cab_inputs #(
  parameter int unsigned SYS_MHZ = 110,
  parameter bit          SENS    = 1'b1,  // "Steering" (sensitivity)
  parameter bit          SMOOTH  = 1'b1,  // "Steering smoothing"
  parameter bit          WHEEL   = 1'b1   // "Analog control": Wheel A / Wheel B
) (
  input  logic        clk,          // clk_sys
  input  logic        rst_n,        // the game reset
  input  logic [15:0] joystick_0,
  input  logic [15:0] joy_analog,   // left stick (registered by the caller)
  input  logic [15:0] joy_ranalog,  // right stick
  input  logic  [7:0] paddle,
  input  logic        st_pedswap,   // status[28]  Pedals
  input  logic  [1:0] st_ssens,     // status[30:29] Steering: 0 Full, 1 5/4, 2 3/4, 3 Half
  input  logic  [1:0] st_ssm,       // status[51:50] Steering smoothing
  input  logic        st_rsteer,    // status[54]  Right stick
  input  logic  [1:0] st_wheel,     // status[71:70] Analog control: 0 Standard, 1 Wheel A, 2 Wheel B
  input  logic  [7:0] trig_rt,      // player 1's right trigger, 0 released .. 255 (hps_io 0x46)
  input  logic  [7:0] trig_lt,      // player 1's left trigger
  output logic  [7:0] pc,           // port C: {shifter, VR4..VR1}, active low
  output logic  [7:0] steer,        // ADC 0, 0x80 at centre
  output logic  [7:0] accel,        // ADC 1, 0x00 at rest
  output logic  [7:0] brake         // ADC 2
);

// ---------------------------------------------------------- port C
// The four-speed shifter, a state stepped by the buttons, encoded as
// Supermodel's shift4 reads the H gate's microswitches.
logic [2:0] gear;                    // 0 = N, 1..4
wire        gup   = joystick_0[12];
wire        gdn   = joystick_0[13];
logic       gup_d, gdn_d;
always_ff @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		gear <= 3'd1; gup_d <= 1'b0; gdn_d <= 1'b0;
	end else begin
		gup_d <= gup; gdn_d <= gdn;
		if (gup && !gup_d && gear != 3'd4) gear <= gear + 3'd1;
		if (gdn && !gdn_d && gear != 3'd0) gear <= gear - 3'd1;
	end
end
logic [3:0] gearval;                 // port C bits 7:4
always_comb begin
	case (gear)
		3'd1:    gearval = 4'b1010;  // 0x50 low
		3'd2:    gearval = 4'b1001;  // 0x60 low
		3'd3:    gearval = 4'b1110;  // 0x10 low
		3'd4:    gearval = 4'b1101;  // 0x20 low
		default: gearval = 4'b1111;  // neutral
	endcase
end
assign pc = {gearval, ~joystick_0[9], ~joystick_0[15], ~joystick_0[14], ~joystick_0[8]};

// ---------------------------------------------------------- steering
// Model 2's sources (R816, R831, R958): the left stick past a dead zone of
// 12, else the right stick's X on "Pedals+steer", the paddle when it moves,
// the d-pad's ramp otherwise; on Wheel A / B the left stick's X always.
wire signed [7:0] ax_l  = joy_analog[7:0];
wire signed [7:0] ax_r  = joy_ranalog[7:0];
wire              use_l = (ax_l > 8'sd12) || (ax_l < -8'sd12);
wire              use_r = st_rsteer && ((ax_r > 8'sd12) || (ax_r < -8'sd12));
wire        [1:0] wheel = WHEEL ? st_wheel : 2'd0;
wire              whl_a = wheel[0] & ~wheel[1];          // 1: Wheel A
wire              whl_b = wheel[1] & ~wheel[0];          // 2: Wheel B
wire              whl   = whl_a | whl_b;
wire signed [7:0] ax    = (use_l || whl) ? ax_l : ax_r;

// Sensitivity: Full (entry 0: full stick is full lock on Model 3's whole-byte
// range), 5/4 (clamps before the stick's edge), 3/4, 1/2.
wire [1:0] ssens = SENS ? st_ssens : 2'd0;
wire signed [9:0] ax_x  = {{2{ax[7]}}, ax};                 // sign-extended
wire signed [9:0] ax_h  = {{3{ax[7]}}, ax[7:1]};            // x/2
wire signed [9:0] ax_q  = {{4{ax[7]}}, ax[7:2]};            // x/4
wire signed [9:0] ax_sc = (ssens == 2'd0) ? ax_x            // 1, the default
                        : (ssens == 2'd1) ? (ax_x + ax_q)   // 5/4
                        : (ssens == 2'd2) ? (ax_x - ax_q)   // 3/4
                        :                   ax_h;           // 1/2
// the ADC's rails, 0x00 and 0xFF
wire signed [10:0] ax_sum = 11'sh080 + {ax_sc[9], ax_sc};
wire        [7:0]  steer_an = ax_sum[10] ? 8'h00 : ax_sum[8] ? 8'hFF : ax_sum[7:0];
logic [7:0] steer_dig;
logic [9:0] steer_div;
always_ff @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		steer_dig <= 8'h80; steer_div <= 10'd0;
	end else begin
		steer_div <= steer_div + 10'd1;
		if (steer_div == 10'd0) begin
			if (joystick_0[1]) begin                       // left
				if (steer_dig != 8'h00) steer_dig <= steer_dig - 8'd1;
			end else if (joystick_0[0]) begin              // right
				if (steer_dig != 8'hFF) steer_dig <= steer_dig + 8'd1;
			end else if (steer_dig > 8'h80) steer_dig <= steer_dig - 8'd1;
			else if (steer_dig < 8'h80) steer_dig <= steer_dig + 8'd1;
		end
	end
end
wire use_stick = whl ? !(joystick_0[1] || joystick_0[0])   // R958: the wheel unless the d-pad steers
                    : (use_l || use_r);                    // R816
wire [7:0] steer_raw = |paddle ? paddle
                 : use_stick ? steer_an
                 : steer_dig;
reg [7:0] steer_rq;                  // Model 2 R843: registered before the limiter
always_ff @(posedge clk) steer_rq <= steer_raw;

// Steering smoothing (Model 2 R830): Light ~29 counts a frame, Wheel ~15.
wire [1:0] ssm = SMOOTH ? st_ssm : 2'd0;
localparam int unsigned STEER_FRAME = SYS_MHZ * 17391;          // clk_sys cycles in a 57.5 Hz frame
localparam int unsigned STEER_K_LT  = STEER_FRAME / 29;
localparam int unsigned STEER_K_WH  = STEER_FRAME / 15;
reg  [7:0]  steer_lim;
reg  [17:0] steer_slew;
always_ff @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		steer_lim <= 8'h80; steer_slew <= '0;
	end else if (ssm == 2'd0) begin
		steer_lim <= steer_rq; steer_slew <= '0;
	end else if (steer_slew == 18'd0) begin
		steer_slew <= (ssm == 2'd1) ? 18'(STEER_K_LT - 1) : 18'(STEER_K_WH - 1);
		if (steer_lim < steer_rq)       steer_lim <= steer_lim + 8'd1;
		else if (steer_lim > steer_rq)  steer_lim <= steer_lim - 8'd1;
	end else
		steer_slew <= steer_slew - 18'd1;
end
assign steer = (ssm == 2'd0) ? steer_rq : steer_lim;

// ---------------------------------------------------------- pedals
// Model 2's arrangement (Model 1's, R981): both pedals on the right stick's
// Y split by sign ("Pedals" swaps the halves), Wheel A / B one axis each, the
// analogue triggers (RT accelerator, LT brake) beside them, the Accel / Brake
// buttons a full press; each pedal reads whichever is pressed furthest. On
// Model 3's range: 0x00 at rest, 0xFF fully pressed.
wire signed [7:0] pedal_ax  = joy_ranalog[15:8];        // right stick Y
wire        [6:0] pedal_mag = (joy_ranalog[15:8] == 8'h80) ? 7'd127          // -128 (Model 2's -8'sd128, which Quartus 17 calls an overflow)
                            : pedal_ax[7] ? (~pedal_ax[6:0] + 7'd1) : pedal_ax[6:0];
wire              pedal_up  = st_pedswap ? ~pedal_ax[7] : pedal_ax[7];
function automatic logic [7:0] ped_an(input logic [6:0] m);    // 0..127 onto 0x00..0xFF
	return {m, m[6]};
endfunction
wire        [7:0] accel_an  = pedal_up ? ped_an(pedal_mag) : 8'h00;
wire        [7:0] brake_an  = pedal_up ? 8'h00 : ped_an(pedal_mag);

function automatic logic [6:0] whl_up(input logic [7:0] v);   // Wheel A: up half
	return (v == 8'h80) ? 7'd127 : v[7] ? (~v[6:0] + 7'd1) : 7'd0;
endfunction
/* verilator lint_off UNUSEDSIGNAL */
function automatic logic [6:0] whl_full(input logic [7:0] v);  // Wheel B: whole axis
	logic [7:0] u;                      // (u[0]: the half step dropped)
	u = v + 8'd127;                     // -127 -> 0, +127 -> 254; -128 wraps to 255
	return (v == 8'h80) ? 7'd0 : u[7:1];
endfunction
/* verilator lint_on UNUSEDSIGNAL */
wire        [6:0] whl_acc_c = whl_b ? whl_full(joy_analog[15:8])  : whl_up(joy_analog[15:8]);
wire        [6:0] whl_brk_c = whl_b ? whl_full(joy_ranalog[7:0])  : whl_up(joy_ranalog[15:8]);
reg         [7:0] whl_acc_an, whl_brk_an;
always_ff @(posedge clk) begin
	whl_acc_an <= ped_an(st_pedswap ? whl_brk_c : whl_acc_c);
	whl_brk_an <= ped_an(st_pedswap ? whl_acc_c : whl_brk_c);
end
// The triggers: their own 0..255 (registered, as the wheel pedals are). A
// Main without UIO 0x46 never writes them, so they read 0 and the stick rules.
reg         [7:0] trg_acc_an, trg_brk_an;
always_ff @(posedge clk) begin
	trg_acc_an <= trig_rt;
	trg_brk_an <= trig_lt;
end
wire        [7:0] stk_accel = whl ? whl_acc_an : accel_an;
wire        [7:0] stk_brake = whl ? whl_brk_an : brake_an;
assign accel = joystick_0[10] ? 8'hFF : (trg_acc_an > stk_accel) ? trg_acc_an : stk_accel;
assign brake = joystick_0[11] ? 8'hFF : (trg_brk_an > stk_brake) ? trg_brk_an : stk_brake;

/* verilator lint_off UNUSEDSIGNAL */
wire _unused = &{1'b0, joystick_0[7:2], ax_sum[9], st_wheel, st_ssens, st_ssm};   // (the OSD bits: with SENS / SMOOTH / WHEEL off)   // (ax_sum: -32..288, bit 10 the sign, bit 8 over 255)
/* verilator lint_on UNUSEDSIGNAL */

endmodule
