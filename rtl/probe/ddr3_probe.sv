//============================================================================
//
//  Sega Model 3 core for MiSTer FPGA -- the DDR3 probe (study D-33, D-34).
//
//  A test core: the pipelined DDR3 read master against texture-like reads,
//  with and without the layer's and the store's traffic, results as a table
//  on screen. No OSD input; it cycles through its cases for ever. The work is
//  in rtl/probe/dp_core.sv; this is the framework's emu around it.
//
//  Based on Template_MiSTer 0874acd (THIRD_PARTY.md), as Model3.sv.
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

assign ADC_BUS  = 'Z;
assign USER_OUT = '1;
assign {UART_RTS, UART_DTR} = 0;
assign {SD_SCK, SD_MOSI, SD_CS} = 'Z;
assign {SDRAM_DQ, SDRAM_A, SDRAM_BA, SDRAM_CLK, SDRAM_CKE, SDRAM_DQML, SDRAM_DQMH, SDRAM_nWE, SDRAM_nCAS, SDRAM_nRAS, SDRAM_nCS} = 'Z;
`ifdef MISTER_DUAL_SDRAM
assign {SDRAM2_DQ, SDRAM2_A, SDRAM2_BA, SDRAM2_CLK, SDRAM2_nWE, SDRAM2_nCAS, SDRAM2_nRAS, SDRAM2_nCS} = 'Z;
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

assign VIDEO_ARX = 12'd4;
assign VIDEO_ARY = 12'd3;

`include "build_id.v"
// Status bit map (the CFG file's bits; 0 is the default everywhere):
//   0      reset
//   1      mode: 0 automatic sweep, 1 manual
//   2      manual direction: 0 read, 1 write
//   3      manual addresses: 0 random, 1 sequential
//   6:4    manual burst, beats: 0..7 = 1, 2, 4, 8, 16, 32, 64, 128
//   9:7    manual reads in flight: 0..5 = 1, 2, 4, 8, 16, 32 (6, 7 = 32)
//   11:10  manual background: 0 off, 1 the layer (22 + 22 MB/s), 2 the layer and the store (+100 MB/s)
//   13:12  the DDR3 clock: 0 100 MHz, 1 125, 2 85, 3 62.5 (VARCLK builds only)
//   14     diagnostic: 1 skips the pattern (reads whatever is there; errors not counted)
//   15     diagnostic: 1 sends the probe's reads through port L, one at a time
//   16     diagnostic: 1 writes the pattern a beat at a time
//   18:17  diagnostic: a further start delay: 0 none, 1 1 s, 2 4 s
//
// No DDR3 access until Main has finished the load: Main releases the
// FPGA-to-SDRAM ports (fpgaportrst) only after the RBF is in, when this core
// is already running, and a command sent before that is lost or wedges the
// port for good. hps_io's first command (io_enable on HPS_BUS) comes after
// the release; the probe waits for it, then 100 ms, then the delay above.
//
// VARCLK = 0 runs the DDR3 side on the fixed clock (no reconfiguration);
// macro PROBE_VARCLK builds the variable clock in.
`ifdef PROBE_VARCLK
localparam bit VARCLK = 1'b1;
`else
localparam bit VARCLK = 1'b0;
`endif
localparam CONF_STR = {
	"DDR3_probe;;",
	"-;",
	"O[1],Mode,Auto sweep,Manual;",
	"-;",
	"O[2],Direction,Read,Write;",
	"O[3],Address,Random,Sequential;",
	"O[6:4],Burst beats,1,2,4,8,16,32,64,128;",
	"O[9:7],In flight,1,2,4,8,16,32;",
	"O[11:10],Background,Off,Layer 22+22,Layer+Store +100;",
	"-;",
	"O[13:12],DDR3 clock,100 MHz,125 MHz,85 MHz,62.5 MHz;",
	"-;",
	"O[14],Pattern init,On,Skip;",
	"O[15],Probe reads via,Pipelined port,Port L;",
	"O[16],Init writes,32-beat bursts,Single beats;",
	"O[18:17],Start delay,0 s,1 s,4 s;",
	"-;",
	"R[0],Reset;",
	"V,v",`BUILD_DATE
};

wire   [1:0] buttons;
wire [127:0] status;

hps_io #(.CONF_STR(CONF_STR)) hps_io
(
	.clk_sys(clk_sys),
	.HPS_BUS(HPS_BUS),
	.EXT_BUS(),
	.gamma_bus(),
	.buttons(buttons),
	.status(status),
	.status_menumask(16'd0)
);

// The fixed clock, from the PLL named `pll` (sys_top.sdc's clock groups match
// it by that name): the settings, the screen, the UART, the video.
wire clk_sys;        // 100 MHz
wire pll_locked;
pll pll
(
	.refclk(CLK_50M),
	.rst(0),
	.outclk_0(clk_sys),
	.outclk_1(),
	.locked(pll_locked)
);

// The DDR3 clock: Peip's reconfigurable PLL (rtl/probe/pll_var, THIRD_PARTY.md),
// compiled at 125 MHz (the timing it is checked at), set at run time through the
// framework's pll_cfg, as his MiSTerDDR3Test does: M, C0 and the fractional K.
wire clk_ddr, ddr_locked;
wire [63:0] reconfig_to_pll, reconfig_from_pll;
generate if (VARCLK) begin : g_var
pll_var pll_var
(
	.refclk(CLK_50M),
	.rst(0),
	.outclk_0(clk_ddr),
	.reconfig_to_pll(reconfig_to_pll),
	.reconfig_from_pll(reconfig_from_pll),
	.locked(ddr_locked)
);
end else begin : g_fix
assign clk_ddr = clk_sys;
assign ddr_locked = pll_locked;
assign reconfig_from_pll = '0;
end endgenerate

wire        cfg_waitrequest;
reg         cfg_write = 0;
reg   [5:0] cfg_address;
reg  [31:0] cfg_data;
pll_cfg pll_cfg
(
	.mgmt_clk(CLK_50M),
	.mgmt_reset(0),
	.mgmt_waitrequest(cfg_waitrequest),
	.mgmt_read(0),
	.mgmt_readdata(),
	.mgmt_write(cfg_write),
	.mgmt_address(cfg_address),
	.mgmt_writedata(cfg_data),
	.reconfig_to_pll(reconfig_to_pll),
	.reconfig_from_pll(reconfig_from_pll)
);

wire [1:0] clk_sel;
wire       cfg_tgl;
always @(posedge CLK_50M) begin : cfg_block
	reg [4:0] state = 0;
	reg [2:0] tg = 0;
	reg [1:0] sel1, sel;
	tg <= {tg[1:0], cfg_tgl};
	sel1 <= clk_sel; sel <= sel1;          // steady long before the toggle arrives
	if (tg[2] != tg[1]) state <= 1;
	cfg_write <= 0;
	if (!cfg_waitrequest) begin
		if (state) state <= state + 1'd1;
		case (state)
			1: begin cfg_address <= 0; cfg_data <= 0; cfg_write <= 1; end
			3: begin
					cfg_address <= 4;     // M
					cfg_data <= (sel == 1) ? 32'h00505 : 32'h00404;
					cfg_write <= 1;
				end
			5: begin
					cfg_address <= 5;     // C0
					cfg_data <= (sel == 2) ? 32'h20302 : (sel == 3) ? 32'h20304 : 32'h00202;
					cfg_write <= 1;
				end
			7: begin
					cfg_address <= 7;     // K, M's fraction
					cfg_data <= (sel == 2) ? 32'h80000000 : (sel == 3) ? 32'hC0000000 : 32'h00000001;
					cfg_write <= 1;
				end
			9: begin cfg_address <= 2; cfg_data <= 0; cfg_write <= 1; end
			default: ;
		endcase
		if (state == 10) state <= 0;
	end
end

// the probe: out of reset on the fixed clock's lock, RESET or the OSD's reset;
// its DDR3 master only on its own clock's lock (the memory rule)
reg [2:0] rst_s = 3'b000, por_s = 3'b000;
always @(posedge clk_sys) begin
	rst_s <= {rst_s[1:0], pll_locked & ~RESET & ~status[0] & ~buttons[1]};
	por_s <= {por_s[1:0], pll_locked};
end

// DDR3 allowed: hps_io has been spoken to (the ports are released by then),
// RESET (sysmem's reset_out, which also drives the f2sdram terminator) has
// been low for 100 ms without a break, and the start delay has passed. RESET
// high stops it again.
reg [2:0]  ioe_s = 3'b000, rs_s = 3'b000;
reg        hps_alive = 1'b0, ddr_ok = 1'b0;
reg [28:0] ok_cnt = 29'd0;
always @(posedge clk_sys) begin
	ioe_s <= {ioe_s[1:0], HPS_BUS[34]};
	rs_s  <= {rs_s[1:0], RESET};
	if (!por_s[2]) begin hps_alive <= 1'b0; ddr_ok <= 1'b0; ok_cnt <= 29'd0; end
	else begin
		if (ioe_s[2]) hps_alive <= 1'b1;
		if (rs_s[2] || !hps_alive) begin ok_cnt <= 29'd0; ddr_ok <= 1'b0; end
		else if (!ddr_ok) begin
			ok_cnt <= ok_cnt + 29'd1;
			if (ok_cnt >= 29'd10_000_000 + ((status[18:17] == 2'd1) ? 29'd100_000_000 : status[18] ? 29'd400_000_000 : 29'd0)) ddr_ok <= 1'b1;
		end
	end
end

// debug at the pins: RESET's rising edges and the cycles since the last edge;
// RESET's level when the first read was taken (2: none yet); the first read's
// cycles from RD presented to its take and to its first beat
reg [31:0] dbg_rise = 0, dbg_since = 0, dbg_w = 32'd2, dbg_p = 0, dbg_v = 0;
reg        rd_seen = 0, rd_taken = 0, beat_seen = 0;
always @(posedge clk_sys) begin
	if (!por_s[2]) begin
		dbg_rise <= 0; dbg_since <= 0; dbg_w <= 32'd2; dbg_p <= 0; dbg_v <= 0; rd_seen <= 0; rd_taken <= 0; beat_seen <= 0;
	end else begin
		if (rs_s[1] && !rs_s[2]) begin dbg_rise <= dbg_rise + 1'd1; dbg_since <= 0; end
		else if (rs_s[1] != rs_s[2]) dbg_since <= 0;
		else if (dbg_since != 32'hFFFFFFFF) dbg_since <= dbg_since + 1'd1;
		if (DDRAM_RD && !rd_seen) rd_seen <= 1'b1;
		if (rd_seen && !rd_taken) dbg_p <= dbg_p + 1'd1;
		if (rd_seen && !beat_seen) dbg_v <= dbg_v + 1'd1;
		if (DDRAM_RD && !DDRAM_BUSY && !rd_taken) begin rd_taken <= 1'b1; dbg_w <= {31'd0, rs_s[2]}; end
		if (DDRAM_DOUT_READY && rd_seen) beat_seen <= 1'b1;
	end
end

wire ce_pix, v_hs, v_vs, v_hb, v_vb;
wire [7:0] v_r, v_g, v_b;

dp_core #(.BUILD(`BUILD_DATE)) core
(
	.clk(clk_sys), .por_n(por_s[2]), .rst_n(rst_s[2]), .ddr_ok(ddr_ok),
	.dbg_emu({dbg_v, dbg_p, dbg_w, dbg_since, dbg_rise}),
	.status({status[16:14], VARCLK ? status[13:12] : 2'b00, status[11:0]}),
	.clk_d(clk_ddr), .d_locked(ddr_locked), .clk_sel(clk_sel), .cfg_tgl(cfg_tgl),
	.DDRAM_CLK(DDRAM_CLK), .DDRAM_BUSY(DDRAM_BUSY), .DDRAM_BURSTCNT(DDRAM_BURSTCNT), .DDRAM_ADDR(DDRAM_ADDR),
	.DDRAM_DIN(DDRAM_DIN), .DDRAM_BE(DDRAM_BE), .DDRAM_WE(DDRAM_WE), .DDRAM_RD(DDRAM_RD),
	.DDRAM_DOUT(DDRAM_DOUT), .DDRAM_DOUT_READY(DDRAM_DOUT_READY),
	.ce_pix(ce_pix), .v_hs(v_hs), .v_vs(v_vs), .v_hblank(v_hb), .v_vblank(v_vb),
	.v_r(v_r), .v_g(v_g), .v_b(v_b),
	.uart_tx(UART_TXD)
);

assign CLK_VIDEO = clk_sys;
assign CE_PIXEL  = ce_pix;
assign VGA_R  = v_r;
assign VGA_G  = v_g;
assign VGA_B  = v_b;
assign VGA_HS = v_hs;
assign VGA_VS = v_vs;
assign VGA_DE = ~(v_hb | v_vb);

assign LED_USER = 0;

endmodule
