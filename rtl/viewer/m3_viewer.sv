//============================================================================
//
//  Sega Model 3 core for MiSTer FPGA -- M2.5 viewer (study D-31).
//
//  One Real3D frame, drawn by the Real3D RTL from a Supermodel dump of the
//  Real3D memories, with no CPU. Load the VROM file (F1) and the frame file
//  (F2) made by tools/viewer/m3v_pack.py; the frame is drawn once both are in.
//  "Next frame" steps through the frames in the file. The work is in
//  rtl/viewer/m3v_core.sv; this is the framework's emu around it.
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
assign {UART_RTS, UART_TXD, UART_DTR} = 0;
assign {SD_SCK, SD_MOSI, SD_CS} = 'Z;
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

wire [1:0] ar = status[122:121];
assign VIDEO_ARX = (!ar) ? 12'd4 : (ar - 1'd1);
assign VIDEO_ARY = (!ar) ? 12'd3 : 12'd0;

`include "build_id.v"
localparam CONF_STR = {
	"Model3_viewer;;",
	"F1,V3V,Load VROM;",
	"F2,V3F,Load frames;",
	"-;",
	"T[1],Next frame;",
	"T[2],Redraw;",
	"O[3],Redraw loop,Off,On;",
	"O[4],Overlay,On,Off;",
	"O[6:5],DDR3 probe,On,Off,On + scan-out paused;",
	"O[122:121],Aspect ratio,Original,Full Screen,[ARC1],[ARC2];",
	"-;",
	"R[0],Reset;",
	"V,v",`BUILD_DATE
};

wire   [1:0] buttons;
wire [127:0] status;
wire         ioctl_download, ioctl_wr, ioctl_wait;
wire  [15:0] ioctl_index, ioctl_dout;
wire  [26:0] ioctl_addr;

// WIDE: 16-bit ioctl words, the address rising by 2 (m3v_loader)
hps_io #(.CONF_STR(CONF_STR), .WIDE(1)) hps_io
(
	.clk_sys(clk_sys),
	.HPS_BUS(HPS_BUS),
	.EXT_BUS(),
	.gamma_bus(),
	.buttons(buttons),
	.status(status),
	.status_menumask(16'd0),
	.ioctl_download(ioctl_download),
	.ioctl_index(ioctl_index),
	.ioctl_wr(ioctl_wr),
	.ioctl_addr(ioctl_addr),
	.ioctl_dout(ioctl_dout),
	.ioctl_wait(ioctl_wait)
);

///////////////////////   CLOCKS   ///////////////////////////////

// The PLL is named `pll`: sys_top.sdc's clock groups match it by that name.
wire clk_sys;        // 100 MHz, everything
wire clk_sdram_pin;  // 100 MHz at 180 degrees, SDRAM_CLK (Model 2's arrangement)
wire pll_locked;
pll pll
(
	.refclk(CLK_50M),
	.rst(0),
	.outclk_0(clk_sys),
	.outclk_1(clk_sdram_pin),
	.locked(pll_locked)
);

// memory: out of reset on PLL lock and stays out (study rules)
reg [2:0] mem_rst_s = 3'b000;
always @(posedge clk_sys) mem_rst_s <= {mem_rst_s[1:0], pll_locked};
wire mem_rst_n = mem_rst_s[2];

// the viewer: reset, OSD reset, or a download in progress
reg [2:0] rst_s = 3'b111;
always @(posedge clk_sys) rst_s <= {rst_s[1:0], RESET | status[0] | buttons[1] | ioctl_download | ~mem_rst_n};
wire rst = rst_s[2];

// DDR3 may be touched once Main has finished the load: it releases the
// FPGA-to-SDRAM ports (fpgaportrst) only after the RBF is in, when this core
// already runs, and hps_io's first command (io_enable) comes after that. RESET
// (sysmem's reset_out, which also resets the f2sdram terminator) must then be
// low for 100 ms without a break; RESET high closes the gate again. A command
// sent to the port before its release wedged it on the board, through core
// reloads, until a power cycle (the DDR3 probe core's finding).
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
		else if (!ddr_ok) begin ok_cnt <= ok_cnt + 24'd1; if (ok_cnt == 24'd10_000_000) ddr_ok <= 1'b1; end
	end
end

// OSD buttons into single pulses
reg [2:0] t1_s, t2_s;
always @(posedge clk_sys) begin t1_s <= {t1_s[1:0], status[1]}; t2_s <= {t2_s[1:0], status[2]}; end
wire next_frame = t1_s[1] & ~t1_s[2];
wire redraw     = t2_s[1] & ~t2_s[2];

///////////////////////   THE VIEWER   ///////////////////////////////

wire [15:0] sd_dq_o;
wire        sd_dq_oe;
wire  [1:0] sdr_dqm;
wire        ce_pix, v_hs, v_vs, v_hb, v_vb, drawing;
wire  [7:0] v_r, v_g, v_b;

m3v_core core
(
	.clk(clk_sys), .mem_rst_n(mem_rst_n), .rst(rst),
	.ioctl_download(ioctl_download), .ioctl_index(ioctl_index), .ioctl_wr(ioctl_wr),
	.ioctl_addr(ioctl_addr), .ioctl_dout(ioctl_dout), .ioctl_wait(ioctl_wait),
	.rd_lat_sel(3'd2),                       // CL + 2, Model 2's board value (its R411)
	.sd_cke(SDRAM_CKE), .sd_cs_n(SDRAM_nCS), .sd_ras_n(SDRAM_nRAS), .sd_cas_n(SDRAM_nCAS), .sd_we_n(SDRAM_nWE),
	.sd_ba(SDRAM_BA), .sd_a(SDRAM_A), .sd_dqm(sdr_dqm), .sd_dq_o(sd_dq_o), .sd_dq_oe(sd_dq_oe), .sd_dq_i(SDRAM_DQ),
	.DDRAM_CLK(DDRAM_CLK), .DDRAM_BUSY(DDRAM_BUSY), .DDRAM_BURSTCNT(DDRAM_BURSTCNT), .DDRAM_ADDR(DDRAM_ADDR),
	.DDRAM_DIN(DDRAM_DIN), .DDRAM_BE(DDRAM_BE), .DDRAM_WE(DDRAM_WE), .DDRAM_RD(DDRAM_RD),
	.DDRAM_DOUT(DDRAM_DOUT), .DDRAM_DOUT_READY(DDRAM_DOUT_READY),
	.ce_pix(ce_pix), .v_hs(v_hs), .v_vs(v_vs), .v_hblank(v_hb), .v_vblank(v_vb),
	.v_r(v_r), .v_g(v_g), .v_b(v_b),
	.next_frame(next_frame), .redraw(redraw), .loop(status[3]), .overlay_en(~status[4]),
	.probe_en(status[6:5] != 2'd1), .probe_pause(status[6:5] == 2'd2), .ddr_ok(ddr_ok),
	.drawing(drawing),
	.frames_done(), .fb_show_o()
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

assign LED_USER = drawing;

endmodule
