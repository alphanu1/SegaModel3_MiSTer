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
`ifdef MISTER_DUAL_SDRAM
assign {SDRAM2_DQ, SDRAM2_A, SDRAM2_BA, SDRAM2_CLK, SDRAM2_nWE, SDRAM2_nCAS, SDRAM2_nRAS, SDRAM2_nCS} = 'Z;
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

assign AUDIO_S = 0;
assign AUDIO_L = 0;
assign AUDIO_R = 0;
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
	"O[122:121],Aspect ratio,Original,Full Screen,[ARC1],[ARC2];",
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

// WIDE: 16-bit ioctl words, the address rising by 2 (m3_loader)
hps_io #(.CONF_STR(CONF_STR), .WIDE(1)) hps_io
(
	.clk_sys(clk_sys),
	.HPS_BUS(HPS_BUS),
	.EXT_BUS(),
	.gamma_bus(),
	.buttons(buttons),
	.status(status),
	.status_menumask({15'd0, inp_drive}),
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
	.ioctl_download(ioctl_download),
	.ioctl_index(ioctl_index),
	.ioctl_wr(ioctl_wr),
	.ioctl_addr(ioctl_addr),
	.ioctl_dout(ioctl_dout),
	.ioctl_wait(ioctl_wait)
);

///////////////////////   CLOCKS   ///////////////////////////////

// The PLL is named `pll`: sys_top.sdc's clock groups match it by that name.
wire clk_sys;        // 110 MHz (D-44), everything (outclk_1; outclk_0 is the CPU test's 20 MHz)
wire clk_sdram_pin;  // 110 MHz at 180 degrees, SDRAM_CLK (Model 2's and the viewer's arrangement)
wire pll_locked;
pll pll
(
	.refclk(CLK_50M),
	.rst(0),
	.outclk_0(),
	.outclk_1(clk_sys),
	.outclk_2(clk_sdram_pin),
	.locked(pll_locked)
);

// The system clock (outclk_1). A diagnostic build at another rate changes this
// and the PLL's outclk_1 / outclk_2 together; m3_system derives every real-time
// count from it (pixel enable, timebase, SDRAM refresh, EEPROM, MIDI, UART).
localparam int SYS_HZ = 120_000_000;

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

m3_system #(.CLK_HZ(SYS_HZ), .CPI_MON(1'b0), .INPUTS(INPUTS), .INP_LEAN(INP_LEAN), .SDR_BXOR(1'b1), .SDR_L16(1'b1), .SDR_LW1(1'b1),
            .SDR_PAIR(1'b1), .SDR_PW(1'b1),
            .BUS_FAST(1'b1), .BUS_R3P(1'b1), .SDR_FREQ(1'b1), .SDR_FWD(1'b1), .SDR_FDSP(1'b1),
            .SDR_PW_DLY(8), .ISB(1'b1)) sys3
(
	.clk(clk_sys), .mem_rst_n(mem_rst_n), .reset(RESET | status[0] | buttons[1]),
	.ioctl_download(ioctl_download), .ioctl_index(ioctl_index), .ioctl_wr(ioctl_wr),
	.ioctl_addr(ioctl_addr), .ioctl_dout(ioctl_dout), .ioctl_wait(ioctl_wait),
	.rd_lat_sel(3'd2),                       // CL + 2, Model 2's board value (its R411), as the viewer
	.sd_cke(SDRAM_CKE), .sd_cs_n(SDRAM_nCS), .sd_ras_n(SDRAM_nRAS), .sd_cas_n(SDRAM_nCAS), .sd_we_n(SDRAM_nWE),
	.sd_ba(SDRAM_BA), .sd_a(SDRAM_A), .sd_dqm(sdr_dqm), .sd_dq_o(sd_dq_o), .sd_dq_oe(sd_dq_oe), .sd_dq_i(SDRAM_DQ),
	.ddr_ok(ddr_ok), .DDRAM_CLK(DDRAM_CLK), .DDRAM_BUSY(DDRAM_BUSY), .DDRAM_BURSTCNT(DDRAM_BURSTCNT), .DDRAM_ADDR(DDRAM_ADDR),
	.DDRAM_DIN(DDRAM_DIN), .DDRAM_BE(DDRAM_BE), .DDRAM_WE(DDRAM_WE), .DDRAM_RD(DDRAM_RD),
	.DDRAM_DOUT(DDRAM_DOUT), .DDRAM_DOUT_READY(DDRAM_DOUT_READY),
	.ce_pix(ce_pix), .v_hs(v_hs), .v_vs(v_vs), .v_hblank(v_hb), .v_vblank(v_vb),
	.v_r(v_r), .v_g(v_g), .v_b(v_b), .overlay_en(~status[4]), .fps_en(status[5]),
	.joystick_0(joystick_0[15:0]), .joystick_1(joystick_1[15:0]), .joy_l0(joy_l0), .joy_l1(joy_l1), .joy_r0(joy_r0),
	.joy_lt0(joy_lt0), .joy_rt0(joy_rt0), .paddle0(paddle0),
	.st_pedswap(status[28]), .st_ssens(status[30:29]), .st_ssm(status[51:50]), .st_rsteer(status[54]),
	.st_wheel(status[71:70]), .inp_drive(inp_drive),
	.led(run_led), .uart_tx(UART_TXD), .tb_fast(1'b0),
	.rom_loaded(), .frame(), .tr_v(), .tr_master(), .tr_we(), .tr_addr(), .tr_be(), .tr_data(),
	.ev_v(), .ev_kind(), .ev_data(),
	.rt_valid_o(), .rt_pc_o(), .rt_insn_o(), .rt_gpr_we_o(), .rt_gpr_o(), .rt_gpr_sh_o(), .rt_gpr_val_o()
);

assign SDRAM_DQ  = sd_dq_oe ? sd_dq_o : 16'bZ;
// the 128 MB module takes its byte mask on A12:A11 (Model 2 R891)
assign {SDRAM_DQMH, SDRAM_DQML} = SDRAM_A[12:11];
assign SDRAM_CLK = clk_sdram_pin;

assign CLK_VIDEO = clk_sys;
assign CE_PIXEL  = ce_pix;
assign VGA_R  = v_r;
assign VGA_G  = v_g;
assign VGA_B  = v_b;
assign VGA_HS = v_hs;
assign VGA_VS = v_vs;
assign VGA_DE = ~(v_hb | v_vb);

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
