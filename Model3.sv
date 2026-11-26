//============================================================================
//
//  Sega Model 3 core for MiSTer FPGA.
//
//  Stage A of study D-37: the 603e, the MPC105 bus and memory map, board RAM
//  and the ROMs in SDRAM from the MRA, interrupts, the 53C810, the tile
//  generator and video out; the Real3D is a status-handshake stub, and there
//  is no sound. The cabinet's controls come from hps_io's joysticks, sticks,
//  triggers and paddle through m3_inputs, by the MRA's input profile
//  (VERILOG_MACRO "M3_NO_INPUTS=1" leaves them out: the core before them,
//  CONF_STR included). The work is in rtl/sys/m3_system.sv; this
//  is the framework's emu around it. The debug overlay (hex) is on by
//  default; sim/system/notes.md has its layout. The CPU's performance counters
//  go out on UART_TXD once a second (m3_cpu_mon's "P nn value" lines).
//
//  With the macro M3_CPUTEST (VERILOG_MACRO "M3_CPUTEST=1") this is the CPU
//  bring-up build instead: rtl/dbg/m3_cputest.sv, the 603e running a
//  generated program and reporting on UART_TXD (tools/cputest-capture.sh),
//  with the template's test picture.
//
//  Based on Template_MiSTer e901a9f (THIRD_PARTY.md).
//
//============================================================================
//
//  This program is free software; you can redistribute it and/or modify it
//  under the terms of the GNU General Public License as published by the Free
//  Software Foundation; either version 2 of the License, or (at your option)
//  any later version.
//
//  This program is distributed in the hope that it will be useful, but WITHOUT
//  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
//  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
//  more details.
//
//  You should have received a copy of the GNU General Public License along
//  with this program; if not, write to the Free Software Foundation, Inc.,
//  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//
//============================================================================

module emu
(
	`include "sys/emu_ports.vh"
);

///////// Default values for ports not used in this core /////////

assign ADC_BUS  = 'Z;
assign USER_OUT = '1;
assign {UART_RTS, UART_DTR} = 0;
assign {SD_SCK, SD_MOSI, SD_CS} = 'Z;
// D-66 / D-68: every dual-SDRAM build puts the CPU's work RAM on the second SDRAM (m3_system W2, m3_sdr2), so
// M3_CPU_SDR2 follows MISTER_DUAL_SDRAM (sys/sys_dual_sdram.tcl, the build script's --dual). Set by hand without it,
// the build stops here (the module below does not exist): a single-SDRAM build has no second SDRAM's pins (R150).
`ifdef MISTER_DUAL_SDRAM
`ifndef M3_CPUTEST
`ifndef M3_CPU_SDR2
`define M3_CPU_SDR2
`endif
`endif
`endif
`ifdef M3_CPU_SDR2
`ifndef MISTER_DUAL_SDRAM
M3_CPU_SDR2_needs_MISTER_DUAL_SDRAM_R150 refuse();
initial $error("M3_CPU_SDR2 needs MISTER_DUAL_SDRAM (sys/sys_dual_sdram.tcl)");
`endif
`elsif MISTER_DUAL_SDRAM
// the second SDRAM fitted but unused (the CPU test build): held deselected (nCS high, NOP, clock low) rather than
// left floating (d130, cl3/NOTES.md)
assign SDRAM2_DQ = 'Z;
assign {SDRAM2_A, SDRAM2_BA} = '0;
assign SDRAM2_CLK = 1'b0;
assign {SDRAM2_nCS, SDRAM2_nRAS, SDRAM2_nCAS, SDRAM2_nWE} = 4'b1111;
`endif
`ifdef M3_CPUTEST
assign {DDRAM_CLK, DDRAM_BURSTCNT, DDRAM_ADDR, DDRAM_DIN, DDRAM_BE, DDRAM_RD, DDRAM_WE} = '0;
`endif

assign VGA_SL = 0;
assign VGA_F1 = 0;
assign VGA_SCALER  = 0;
assign VGA_DISABLE = 0;
assign HDMI_FREEZE = 0;
assign HDMI_BLACKOUT = 0;
assign HDMI_BOB_DEINT = 0;

// the sound link's two channels, signed, 44.1 kHz held (m3_system's audio_l / audio_r)
wire signed [15:0] snd_l, snd_r;
assign AUDIO_S = 1;
assign AUDIO_L = snd_l;
assign AUDIO_R = snd_r;
assign AUDIO_MIX = 0;

assign LED_DISK = 0;
assign LED_POWER = 0;
assign BUTTONS = 0;

//////////////////////////////////////////////////////////////////

wire [1:0] ar = status[122:121];

assign VIDEO_ARX = (!ar) ? 12'd4 : (ar - 1'd1);
assign VIDEO_ARY = (!ar) ? 12'd3 : 12'd0;

`include "build_id.v"

`ifndef M3_CPUTEST
// ===========================================================================
// the Model 3 system (stage A)
// ===========================================================================
localparam CONF_STR = {
	"Model3;;",
	"-;",
	"DIP;",                                  // the MRA's <switches>: the CPU board's DIP bank (index 254, m3_loader)
	"-;",
	"O[4],Debug overlay,On,Off;",
	"O[5],FPS counter,Off,On;",
	"O[9:8],FPS average,1 s,5 s,10 s;",      // the OSD's default is the first entry (m3_fps avg)
	"O[7:6],Volume,Normal,+6 dB,+12 dB;",    // on the sound link's fixed gain (m3_sndbridge GAIN), saturating
	"O[122:121],Aspect ratio,Original,Full Screen,[ARC1],[ARC2];",
	// the SDRAM read capture (m2_sdram rd_lat_sel, n = CL + n), per controller, for the board's margin above 120 MHz
	// (R184): change, then Reset
	"O[74:73],SDRAM read capture,CL+2,CL+1,CL+3;",
	// sys140: the DQ capture clock's phase in PLL steps of 149 ps (outclk_4 / outclk_5 stepped at run time: the
	// read eye moved without moving the pin clock); change, then Reset
	// sys140: the read capture's phase: Auto (each SDRAM calibrated at power-up) or a fixed offset in PLL steps of
	// 149 ps from the generated phase, single steps over the band the timing covers (-8 to +14); change, then
	// Reset. "SDRAM calibration: Show" puts the result in the FPS box. (t6 had O[79:77] in steps of 2.)
	"O[88:84],SDRAM capture phase,Auto,0,+1,+2,+3,+4,+5,+6,+7,+8,+9,+10,+11,+12,+13,+14,-8,-7,-6,-5,-4,-3,-2,-1;",
	"O[83],SDRAM calibration,Hide,Show;",
`ifdef M3_CPU_SDR2
	"O[76:75],SDRAM2 read capture,CL+2,CL+1,CL+3;",
	"O[93:89],SDRAM2 capture phase,Auto,0,+1,+2,+3,+4,+5,+6,+7,+8,+9,+10,+11,+12,+13,+14,-8,-7,-6,-5,-4,-3,-2,-1;",
`endif
`ifndef M3_NO_INPUTS
	// the driving profile's lines (Scud Race, Daytona USA 2), shown only for it
	// (status_menumask bit 0): Model 2's lines and status bits (its R816-R958),
	// on Model 3's whole-byte ADC, where full stick is full lock (m3_cab_inputs)
	"h0O[28],Pedals,Normal,Swapped;",
`ifndef M3_INP_LEAN
	"h0O[30:29],Steering,Full,5/4,3/4,Half;",
	"h0O[51:50],Steering smoothing,Off,Light,Wheel;",
`endif
	"h0O[54],Right stick,Pedals only,Pedals+steer;",
`ifndef M3_INP_LEAN
	"h0O[71:70],Analog control,Standard,Wheel A,Wheel B;",
`endif
`endif
	"-;",
	"T[0],Reset;",
	"R[0],Reset and close OSD;",
`ifndef M3_NO_INPUTS
	// the buttons, in the generic profile's order (VF3's); each MRA's <buttons
	// names> gives its profile's (rtl/io/m3_inputs.sv). Last in the menu block,
	// as on Model 2 (between the R items it broke the OSD's Reset there)
	"J1,B1,B2,B3,B4,Start,Coin,Test,Service,B9,B10,B11,B12;",
`endif
`ifdef M3_CPU_SDR2
	"I,This build needs the second SDRAM;",  // hps_io info 1: m3_sdr2's start-up probe found none (D-67, D-68)
`endif
	"V,v",`BUILD_DATE
};

wire   [1:0] buttons;
wire [127:0] status;
wire         ioctl_download, ioctl_wr, ioctl_wait;
wire  [15:0] ioctl_index, ioctl_dout;
wire  [26:0] ioctl_addr;
wire  [31:0] joystick_0, joystick_1;
wire  [15:0] joy_l0, joy_l1, joy_r0;
wire   [7:0] joy_lt0, joy_rt0, paddle0;
wire         inp_drive;
// the second SDRAM (M3_CPU_SDR2: m3_system W2)
`ifdef M3_CPU_SDR2
localparam bit CPU_SDR2 = 1'b1;
wire        sd2_en = SDRAM2_EN;
wire [15:0] sd2_dq_i = SDRAM2_DQ;
`else
localparam bit CPU_SDR2 = 1'b0;
wire        sd2_en = 1'b0;
wire [15:0] sd2_dq_i = 16'd0;
`endif
wire        sd2_cs_n, sd2_ras_n, sd2_cas_n, sd2_we_n, sd2_dq_oe, sd2_info_req;
wire  [1:0] sd2_ba;
wire [12:0] sd2_a;
wire [15:0] sd2_dq_o;

// WIDE: 16-bit ioctl words, the address rising by 2 (m3_loader)
// sys140: hps_io runs on clk_hps (outclk_6, 70 MHz: clk_sys / 2 from the same VCO, rising with every other edge of
// clk_sys), so the framework's command side (sys_top's configuration, both OSDs' HPS side, video_calc) runs there,
// as it does in cores whose clk_sys is slower: at 140 MHz those paths failed by up to 2.7 ns, and sys/ stays stock.
// Every path between clk_hps and clk_sys is timed (one PLL, a whole ratio); the download's words are taken into
// clk_sys below.
wire        hio_download, hio_wr;
wire [15:0] hio_index, hio_dout;
wire [26:0] hio_addr;
hps_io #(.CONF_STR(CONF_STR), .WIDE(1)) hps_io
(
	.clk_sys(clk_hps),
	.HPS_BUS(HPS_BUS),
	.EXT_BUS(),
	.gamma_bus(),
	.buttons(buttons),
	.status(status),
	.status_menumask({15'd0, inp_drive}),
`ifdef M3_CPU_SDR2
	.info_req(sd2_info_req), .info(8'd1),
`endif
`ifndef M3_NO_INPUTS
	.joystick_0(joystick_0),
	.joystick_1(joystick_1),
	.joystick_l_analog_0(joy_l0),
	.joystick_l_analog_1(joy_l1),
	.joystick_r_analog_0(joy_r0),
	.joystick_lt_analog_0(joy_lt0), .joystick_rt_analog_0(joy_rt0),   // Main's 0x46 (Template e901a9f)
	.paddle_0(paddle0),
`endif
	// the force feedback request (Template 76dd8f5; FFB 0): no drive board yet
	.joystick_0_ffb(56'd0), .joystick_1_ffb(56'd0), .joystick_2_ffb(56'd0),
	.joystick_3_ffb(56'd0), .joystick_4_ffb(56'd0), .joystick_5_ffb(56'd0),
	.ioctl_download(hio_download),
	.ioctl_index(hio_index),
	.ioctl_wr(hio_wr),
	.ioctl_addr(hio_addr),
	.ioctl_dout(hio_dout),
	.ioctl_wait(ioctl_wait)
);

// the download into clk_sys: each word's fields registered beside its strobe, and the strobe (one clk_hps cycle, so
// two of clk_sys) taken as one clk_sys pulse at its first clock; hps_io's strobes are far apart (a word every few
// HPS register writes), and its fields hold from before the strobe to the next. ioctl_wait goes back unregistered
// (m3_loader gates it on ioctl_download, a clock later than hps_io's own)
reg        io_wr_q = 1'b0, io_wr_d = 1'b0;
reg        io_dl_q = 1'b0;
reg [15:0] io_ix_q, io_do_q;
reg [26:0] io_a_q;
always @(posedge clk_sys) begin
	io_wr_q <= hio_wr; io_wr_d <= io_wr_q;
	io_dl_q <= hio_download; io_ix_q <= hio_index; io_a_q <= hio_addr; io_do_q <= hio_dout;
end
assign ioctl_wr       = io_wr_q & ~io_wr_d;
assign ioctl_download = io_dl_q;
assign ioctl_index    = io_ix_q;
assign ioctl_addr     = io_a_q;
assign ioctl_dout     = io_do_q;

///////////////////////   CLOCKS   ///////////////////////////////

// The PLL is named `pll`: sys_top.sdc's clock groups match it by that name.
// sys140 (D-81): every output from one VCO at 840 MHz (the phase step 148.8 ps, 48 to a 140 MHz period). The IP
// tool's physical parameters are on (M 84, N 5, C 42 / 6 x5 / 12) to hold it there: with them off the tool picks the
// VCO from the phases asked for, and 1488 / 1190 / 3273 ps (all multiples of 297.6 ps) gave 420 MHz, steps of
// 297.6 ps and a period of 24 of them, which m3_sdr_cal's NSTEP 48 does not describe (the board's 99 / 18, R198).
// The calibration's NSTEP / BAND and the stepper's modulo below follow this VCO: change them together.
wire clk_sys;        // outclk_1: 140 MHz, everything (outclk_0 is the CPU test's 20 MHz)
wire clk_sdr;        // outclk_2: 140 MHz at 3273 ps: SDRAM_CLK's DDIO, set for the commands and writes
wire clk_sdr2;       // outclk_3: 140 MHz at 3273 ps: SDRAM2_CLK's DDIO, likewise (m3_sdr_clk below; R187)
wire clk_cap1;       // outclk_4: 140 MHz at 1488 ps: the first SDRAM's DQ input registers (m2_sdram CAPCLK)
wire clk_cap2;       // outclk_5: 140 MHz at 1190 ps: the second's
wire clk_hps;        // outclk_6: 70 MHz: hps_io and the framework's command side, and CLK_VIDEO
wire pll_locked;
wire       ps_done;
reg        ps_en = 1'b0, ps_up = 1'b0;
reg  [4:0] ps_cnt = 5'd4;
reg  [5:0] ps_off1 = 6'd0, ps_off2 = 6'd0;      // the offsets applied, modulo 48
reg  [1:0] ps_st = 2'd0;
reg  [2:0] ps_lk = 3'b000;
reg  [5:0] ps_to = 6'd0;                        // a step's time-out (phase_done never seen low)
pll pll
(
	.refclk(CLK_50M),
	.rst(0),
	.outclk_0(),
	.outclk_1(clk_sys),
	.outclk_2(clk_sdr),
	.outclk_3(clk_sdr2),
	.outclk_4(clk_cap1),
	.outclk_5(clk_cap2),
	.outclk_6(clk_hps),
	.locked(pll_locked),
	.phase_en(ps_en), .scanclk(clk_hps), .updn(ps_up), .cntsel(ps_cnt), .phase_done(ps_done)
);

// sys140: the capture clocks' phases, one PLL step at a time through the dynamic phase port (altera_pll: cntsel the
// counter, updn 1 later, phase_en a pulse, phase_done low until the step is done), on clk_hps (70 MHz, under
// scanclk's 100 MHz). The targets are m3_system's (cap1_off / cap2_off: 0-47 steps from the generated phase, a
// period being 48 steps of 148.8 ps at 140 MHz from 840 MHz): the calibration's sweep and choice, or the OSD's
// manual offset. Each counter's applied offset is kept modulo 48 and reset to 0 when the PLL locks (a relock
// restores the generated phases); each step goes the shorter way round. capN_settled: the counter is at its target
// and the stepper is idle (registered on clk_hps; m3_system waits 16 clocks after a change before it reads it).
wire [5:0] ps_t1, ps_t2;                         // m3_system's targets (clk_sys registers)
reg        ps_set1 = 1'b0, ps_set2 = 1'b0;
// the shorter way from o to t, modulo 48: up when (t - o) mod 48 is 1-24
function automatic logic ps_upw(input logic [5:0] t, input logic [5:0] o);
	logic [6:0] d;
	d = (t >= o) ? {1'b0, t - o} : 7'({1'b0, t} + 7'd48 - {1'b0, o});
	ps_upw = d <= 7'd24;
endfunction
function automatic logic [5:0] ps_inc(input logic [5:0] o, input logic up);
	ps_inc = up ? ((o == 6'd47) ? 6'd0 : o + 6'd1) : ((o == 6'd0) ? 6'd47 : o - 6'd1);
endfunction
always @(posedge clk_hps) begin
	ps_lk <= {ps_lk[1:0], pll_locked};
	ps_en <= 1'b0;
	ps_set1 <= (ps_st == 2'd0) && (ps_off1 == ps_t1);
	ps_set2 <= (ps_st == 2'd0) && (ps_off2 == ps_t2);
	if (!ps_lk[2]) begin ps_st <= 2'd0; ps_off1 <= 6'd0; ps_off2 <= 6'd0; ps_set1 <= 1'b0; ps_set2 <= 1'b0; end
	else case (ps_st)
		2'd0: if (ps_off1 != ps_t1) begin
				ps_cnt <= 5'd4; ps_up <= ps_upw(ps_t1, ps_off1); ps_en <= 1'b1; ps_st <= 2'd1;
				ps_off1 <= ps_inc(ps_off1, ps_upw(ps_t1, ps_off1)); ps_set1 <= 1'b0;
			end else if (ps_off2 != ps_t2) begin
				ps_cnt <= 5'd5; ps_up <= ps_upw(ps_t2, ps_off2); ps_en <= 1'b1; ps_st <= 2'd1;
				ps_off2 <= ps_inc(ps_off2, ps_upw(ps_t2, ps_off2)); ps_set2 <= 1'b0;
			end
		2'd1: begin ps_en <= 1'b1; ps_st <= 2'd2; ps_to <= 6'd0; end   // phase_en two scanclk cycles
		2'd2: begin ps_to <= ps_to + 6'd1; if (!ps_done || &ps_to) ps_st <= 2'd3; end   // the step under way
		2'd3: if (ps_done) ps_st <= 2'd0;                           // done
	endcase
end

// The system clock (outclk_1). A diagnostic build at another rate changes this
// and the PLL's outclk_1 / outclk_2 / outclk_3 together; m3_system derives every
// real-time count from it (pixel enable, timebase, SDRAM refresh, EEPROM, MIDI,
// UART). outclk_2 / outclk_3 keep their leads in time, not in degrees: T / 2 -
// 500 ps and T / 2 - 750 ps (3500 / 3250 ps at 125 MHz; at 130 the PLL's 240 ps
// steps make it 3365 / 3125, leads of 0.48 / 0.72 ns).
// Two outputs because one lead cannot keep both SDRAMs on board-proven ground
// (R187: the second SDRAM's read setup wants its clock earlier than the first's
// fast-corner read hold allows).
// D-69 as decided on 2026-11-14: 125 MHz everywhere, c125's clock (outclk_1 125 MHz).
// sys140 (D-81): 140 MHz everywhere. The pin clocks are no longer set for the reads: each SDRAM's DQ is taken in
// by a clock of its own (clk_cap1 / clk_cap2), so the pin clock is set for the commands and writes alone (their
// setup at least c130ddpa_s1's, the board build at 130) and the capture clock puts the read eye where the board
// builds at 130 and 135 read (sm3-builds/sys140/NOTES.md). tRCD / tRP 3 clocks (15 ns), one more register after
// each DQ input register, one register stage on the DDR3 port's commands (m3_system SDR_CAPCLK, SDR_CAPPIPE,
// DDR_OREG).
localparam int SYS_HZ = 140_000_000;

// memory: out of reset on PLL lock, and stays out (study rules)
reg [2:0] mem_rst_s = 3'b000;
always @(posedge clk_sys) mem_rst_s <= {mem_rst_s[1:0], pll_locked};
wire mem_rst_n = mem_rst_s[2];

wire [15:0] sd_dq_o;
wire        sd_dq_oe;
wire  [1:0] sdr_dqm;
wire        ce_pix, v_hs, v_vs, v_hb, v_vb, run_led;
wire  [7:0] v_r, v_g, v_b;

// DDR3 may be touched once Main has finished the load (R71, the viewer's
// gate): Main releases the FPGA-to-SDRAM ports only after the RBF is in, and
// hps_io's first command (io_enable) comes after that. RESET must then be low
// for 100 ms without a break; RESET high closes the gate again. A command sent
// before the release wedged the port until a power cycle (R72).
reg [2:0]  ioe_s = 3'b000, rs_s = 3'b000;
reg        hps_alive = 1'b0, ddr_ok = 1'b0;
reg [23:0] ok_cnt = 24'd0;
always @(posedge clk_sys) begin
	ioe_s <= {ioe_s[1:0], HPS_BUS[34]};
	rs_s  <= {rs_s[1:0], RESET};
	if (!mem_rst_n) begin hps_alive <= 1'b0; ddr_ok <= 1'b0; ok_cnt <= 24'd0; end
	else begin
		if (ioe_s[2]) hps_alive <= 1'b1;
		if (rs_s[2] || !hps_alive) begin ok_cnt <= 24'd0; ddr_ok <= 1'b0; end
		else if (!ddr_ok) begin ok_cnt <= ok_cnt + 24'd1; if (ok_cnt == 24'(SYS_HZ / 10)) ddr_ok <= 1'b1; end
	end
end

`ifdef M3_NO_INPUTS
localparam bit INPUTS = 1'b0;
assign {joystick_0, joystick_1, joy_l0, joy_l1, joy_r0, joy_lt0, joy_rt0, paddle0} = '0;
`else
localparam bit INPUTS = 1'b1;
`endif
// VERILOG_MACRO "M3_INP_LEAN=1": the driving block without Model 2's "Steering", "Steering smoothing" and
// "Analog control" (Wheel A / B), their OSD lines with them; each reads as its default (m3_cab_inputs)
`ifdef M3_INP_LEAN
localparam bit INP_LEAN = 1'b1;
`else
localparam bit INP_LEAN = 1'b0;
`endif

// the OSD's order to m2_sdram's rd_lat_sel (index n = CL + n): CL+2 (2, the default entry), CL+1 (1), CL+3 (3)
wire [2:0] sdr_cap1 = (status[74:73] == 2'd1) ? 3'd1 : (status[74:73] == 2'd2) ? 3'd3 : 3'd2;
`ifdef M3_CPU_SDR2
wire [2:0] sdr_cap2 = (status[76:75] == 2'd1) ? 3'd1 : (status[76:75] == 2'd2) ? 3'd3 : 3'd2;
`else
wire [2:0] sdr_cap2 = 3'd2;
`endif
m3_system #(.CLK_HZ(SYS_HZ), .CPI_MON(1'b0), .INPUTS(INPUTS), .INP_LEAN(INP_LEAN), .SDR_BXOR(1'b1), .SDR_L16(1'b1), .SDR_LW1(1'b1),
            .SDR_PAIR(1'b1), .SDR_PW(1'b1),
            .BUS_FAST(1'b1), .BUS_R3P(1'b1), .SDR_FREQ(1'b1), .SDR_FWD(1'b1), .SDR_FDSP(1'b1),
            .SDR_PW_DLY(8), .ISB(1'b1), .W2(CPU_SDR2), .SDR_CAPCLK(1'b1), .SDR_CAPPIPE(1'b1), .DDR_OREG(1'b1), .SDR_OPIPE(1'b1), .SDR2_CAPX(1),
            .SDR_CAL(1'b1)) sys3
(
	.clk(clk_sys), .clk_cap1(clk_cap1), .clk_cap2(clk_cap2), .mem_rst_n(mem_rst_n), .reset(RESET | status[0] | buttons[1]),
	.ioctl_download(ioctl_download), .ioctl_index(ioctl_index), .ioctl_wr(ioctl_wr),
	.ioctl_addr(ioctl_addr), .ioctl_dout(ioctl_dout), .ioctl_wait(ioctl_wait),
	.rd_lat_sel(sdr_cap1),                   // OSD: CL+2 (the default: Model 2's board value, its R411), CL+1, CL+3
	.sd2_rd_lat_sel(sdr_cap2),               // the second SDRAM's, the same choices
	.sd_cke(SDRAM_CKE), .sd_cs_n(SDRAM_nCS), .sd_ras_n(SDRAM_nRAS), .sd_cas_n(SDRAM_nCAS), .sd_we_n(SDRAM_nWE),
	.sd_ba(SDRAM_BA), .sd_a(SDRAM_A), .sd_dqm(sdr_dqm), .sd_dq_o(sd_dq_o), .sd_dq_oe(sd_dq_oe), .sd_dq_i(SDRAM_DQ),
	.sd2_en(sd2_en), .sd2_cs_n(sd2_cs_n), .sd2_ras_n(sd2_ras_n), .sd2_cas_n(sd2_cas_n), .sd2_we_n(sd2_we_n),
	.sd2_ba(sd2_ba), .sd2_a(sd2_a), .sd2_dq_o(sd2_dq_o), .sd2_dq_oe(sd2_dq_oe), .sd2_dq_i(sd2_dq_i),
	.sd2_missing(), .sd2_info_req(sd2_info_req),
	.ddr_ok(ddr_ok), .DDRAM_CLK(DDRAM_CLK), .DDRAM_BUSY(DDRAM_BUSY), .DDRAM_BURSTCNT(DDRAM_BURSTCNT), .DDRAM_ADDR(DDRAM_ADDR),
	.DDRAM_DIN(DDRAM_DIN), .DDRAM_BE(DDRAM_BE), .DDRAM_WE(DDRAM_WE), .DDRAM_RD(DDRAM_RD),
	.DDRAM_DOUT(DDRAM_DOUT), .DDRAM_DOUT_READY(DDRAM_DOUT_READY),
	.ce_pix(ce_pix), .v_hs(v_hs), .v_vs(v_vs), .v_hblank(v_hb), .v_vblank(v_vb),
	.v_r(v_r), .v_g(v_g), .v_b(v_b), .overlay_en(~status[4]), .fps_en(status[5]), .fps_avg(status[9:8]),
	.joystick_0(joystick_0[15:0]), .joystick_1(joystick_1[15:0]), .joy_l0(joy_l0), .joy_l1(joy_l1), .joy_r0(joy_r0),
	.joy_lt0(joy_lt0), .joy_rt0(joy_rt0), .paddle0(paddle0),
	.st_pedswap(status[28]), .st_ssens(status[30:29]), .st_ssm(status[51:50]), .st_rsteer(status[54]),
	.st_wheel(status[71:70]), .inp_drive(inp_drive),
	.audio_l(snd_l), .audio_r(snd_r), .snd_vol(status[7:6]),
	.led(run_led), .uart_tx(UART_TXD), .tb_fast(1'b0),
	.cap1_off(ps_t1), .cap2_off(ps_t2), .cap1_settled(ps_set1), .cap2_settled(ps_set2), .cap1_osd(status[88:84]),
`ifdef M3_CPU_SDR2
	.cap2_osd(status[93:89]),
`else
	.cap2_osd(5'd0),
`endif
	.cal_show(status[83]),
	.rom_loaded(), .frame(), .tr_v(), .tr_master(), .tr_we(), .tr_addr(), .tr_be(), .tr_data(),
	.ev_v(), .ev_kind(), .ev_data(),
	.rt_valid_o(), .rt_pc_o(), .rt_insn_o(), .rt_gpr_we_o(), .rt_gpr_o(), .rt_gpr_sh_o(), .rt_gpr_val_o()
);

assign SDRAM_DQ  = sd_dq_oe ? sd_dq_o : 16'bZ;
// the 128 MB module takes its byte mask on A12:A11 (Model 2 R891)
assign {SDRAM_DQMH, SDRAM_DQML} = SDRAM_A[12:11];
// R187: clk_sdr from the pin's DDIO output register (after MemTest256), so it leaves its I/O cell as the address,
// command and DQ registers leave theirs, 0.5 ns ahead of clk_sys's 180 degrees; Model3.sdc's generated clock sd1_pin
m3_sdr_clk #(.INV(1'b0)) u_sdclk1 (.clk(clk_sdr), .pin(SDRAM_CLK));

`ifdef M3_CPU_SDR2
// the second SDRAM (D-66, D-68): always driven, straight from its controller's registers (they pack into the I/O
// cells); idle is a NOP. Its clock is clk_sdr2 (outclk_3, 0.25 ns ahead of the first's) from its own pin's DDIO
// output register (R187; Model3.sdc's sd2_pin); its byte mask is on A12:A11 (the header has no DQM pins), nCS the
// 128 MB module's chip select
assign SDRAM2_DQ   = sd2_dq_oe ? sd2_dq_o : 16'bZ;
assign SDRAM2_A    = sd2_a;
assign SDRAM2_BA   = sd2_ba;
assign SDRAM2_nCS  = sd2_cs_n;
assign SDRAM2_nRAS = sd2_ras_n;
assign SDRAM2_nCAS = sd2_cas_n;
assign SDRAM2_nWE  = sd2_we_n;
m3_sdr_clk #(.INV(1'b0)) u_sdclk2 (.clk(clk_sdr2), .pin(SDRAM2_CLK));
`endif

// sys140: the video into clk_hps (70 MHz), so the framework's video side (ascal's input, both OSDs, the scanlines,
// video_calc) runs there: at 140 MHz ascal's input side alone failed by 2.1 ns. Each pixel's fields are taken at
// its ce_pix (pixels are 8.75 clk_sys clocks apart at 16 MHz) and its enable stretched over the next two clk_sys
// clocks, of which exactly one ends at an edge of clk_hps, so clk_hps sees each pixel once, with its fields held.
// 1.5 to 2.5 clk_sys clocks (11-18 ns) more on the way out, no frame.
reg  [27:0] vq;
reg  [1:0]  vce = 2'b00;
always @(posedge clk_sys) begin
	vce <= {vce[0], ce_pix};
	if (ce_pix) vq <= {v_r, v_g, v_b, v_hs, v_vs, ~(v_hb | v_vb), 1'b0};
end
reg  [27:0] vh;
reg         vh_ce = 1'b0;
always @(posedge clk_hps) begin
	vh_ce <= |vce;
	vh    <= vq;
end
assign CLK_VIDEO = clk_hps;
assign CE_PIXEL  = vh_ce;
assign {VGA_R, VGA_G, VGA_B, VGA_HS, VGA_VS, VGA_DE} = vh[27:1];

assign LED_USER = run_led;

`else
// ===========================================================================
// the CPU bring-up build (M0-A2)
// ===========================================================================
localparam CONF_STR = {
	"Model3;;",
	"-;",
	"O[122:121],Aspect ratio,Original,Full Screen,[ARC1],[ARC2];",
	"O[2],TV Mode,NTSC,PAL;",
	"O[4:3],Noise,White,Red,Green,Blue;",
	"-;",
	"T[0],Reset;",
	"R[0],Reset and close OSD;",
	"v,0;",
	"V,v",`BUILD_DATE
};

wire forced_scandoubler;
wire   [1:0] buttons;
wire [127:0] status;
wire  [10:0] ps2_key;

hps_io #(.CONF_STR(CONF_STR)) hps_io
(
	.clk_sys(clk_sys),
	.HPS_BUS(HPS_BUS),
	.EXT_BUS(),
	.gamma_bus(),

	.forced_scandoubler(forced_scandoubler),

	.buttons(buttons),
	.status(status),
	.status_menumask({status[5]}),

	.ps2_key(ps2_key)
);

///////////////////////   CLOCKS   ///////////////////////////////

wire clk_sys;
wire clk_cpu, pll_locked;
pll pll
(
	.refclk(CLK_50M),
	.rst(0),
	.outclk_0(clk_sys),
	.outclk_1(clk_cpu),
	.outclk_2(),
	.outclk_3(),
	.outclk_4(),
	.outclk_5(),
	.outclk_6(),
	.locked(pll_locked)
);

wire reset = RESET | status[0] | buttons[1];

// ---------------------------------------------------------------- CPU test
// 110 MHz (study D-3, D-44); the UART divisor follows the clock.
localparam int CPU_HZ = 100_000_000;

reg [2:0] cpu_rst_sync = 3'b111;
always @(posedge clk_cpu) cpu_rst_sync <= {cpu_rst_sync[1:0], reset | ~pll_locked};

m3_cputest #(
	.CODE_HEX("build/cputest/cputest_code.hex"),
	.DATA_HEX("build/cputest/cputest_data.hex"),
	.DIVISOR((CPU_HZ + 57_600) / 115_200)
) cputest (
	.clk     (clk_cpu),
	.rst     (cpu_rst_sync[2]),
	.tb_tick (1'b0),
	.uart_tx (UART_TXD),
	.running ()
);

assign {SDRAM_DQ, SDRAM_A, SDRAM_BA, SDRAM_CLK, SDRAM_CKE, SDRAM_DQML, SDRAM_DQMH, SDRAM_nWE, SDRAM_nCAS, SDRAM_nRAS, SDRAM_nCS} = 'Z;

wire [1:0] col = status[4:3];

wire HBlank;
wire HSync;
wire VBlank;
wire VSync;
wire ce_pix;
wire [7:0] video;

mycore mycore
(
	.clk(clk_sys),
	.reset(reset),

	.pal(status[2]),
	.scandouble(forced_scandoubler),

	.ce_pix(ce_pix),

	.HBlank(HBlank),
	.HSync(HSync),
	.VBlank(VBlank),
	.VSync(VSync),

	.video(video)
);

assign CLK_VIDEO = clk_sys;
assign CE_PIXEL = ce_pix;

assign VGA_DE = ~(HBlank | VBlank);
assign VGA_HS = HSync;
assign VGA_VS = VSync;
assign VGA_G  = (!col || col == 2) ? video : 8'd0;
assign VGA_R  = (!col || col == 1) ? video : 8'd0;
assign VGA_B  = (!col || col == 3) ? video : 8'd0;

reg  [26:0] act_cnt;
always @(posedge clk_sys) act_cnt <= act_cnt + 1'd1;
assign LED_USER    = act_cnt[26]  ? act_cnt[25:18]  > act_cnt[7:0]  : act_cnt[25:18]  <= act_cnt[7:0];
`endif

endmodule
