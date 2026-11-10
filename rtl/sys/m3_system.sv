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
// The Model 3 Step 1.0 system, study D-37 stages A and B: everything below
// the MiSTer framework, so the Verilator bench runs this module whole (as the
// viewer's m3v_core). One 100 MHz clock (D-3, D-28).
//
//   ppc_cpu (603e, caches) --+
//   m3_scripts (53C810 DMA) -+-> m3_bus --> m3_sdr_port --> m2_sdram port 0
//                                     |--> m3_scripts registers, PCI config
//                                     |--> m3_io (315-5649, 93C46), m3_jtag
//                                     |--> m3_tilegen registers and palette
//                                     `--> r3d_cmd (the Real3D's status and windows)
//   r3d_cmd -> r3d_mem (8C, 8E: SDRAM port 4; polygon RAM: DDR3, two copies)
//           -> r3d_texup (texture FIFO, VROM port: SDRAM port 5, DDR3)
//           -> r3d_geo (culling RAM: port 2, VROM: port 3, polygon RAM: DDR3)
//              -> r3d_rast (MIP; store, layer, texture cache: DDR3)
//   m3_tilegen (VRAM in SDRAM, port 1) + m3_compose (the layer from DDR3) -> video
//   m3_loader: the MRA's downloads -> SDRAM (download port), the extra memory,
//              the 93C46, the game configuration (study R100)
//   m3_inputs: MiSTer's joysticks, sticks, triggers and paddle -> m3_io's ports
//              and ADC, by the configuration's input profile (INPUTS)
//
// R100 (Daytona 2, Step 2.1): the game configuration (MRA index 4) picks the
// step at run time; STEP2 builds the Step 2.x hardware (the Real3D DMA front
// end on m3_scripts, the 17-device JTAG chain, the bank map, the security
// board m3_sec with SEC5881), XMEM where the MRA's index 1 (VROM) lives:
// 1 DDR3 (m3_xmem on m3_ddr3p's second pipelined port; the loader on DDR3
// arbiter port 6), 2 the second SDRAM (not built: its m2_sdram goes in the
// g_xmem branch). The Step 2 map in m3_bus is a patch held outside the tree
// while m3_bus is being changed elsewhere; its ports are connected only with
// M3_STEP2_BUS defined (sm3-builds/dt2/m3_bus_step2.patch).
//
// DDR3 (64-bit words from DDR_BASE; sim/system/notes.md, Stage B): layer
// buffers 0-3 at 0x20000 k, polygon RAM copies at 0x100000 and 0x180000, the
// texture ring at 0x200000, the rasteriser's store's records at 0x400000 (two
// 32 MiB banks to 0xBFFFFF) and its run lists at 0x1800000 (32 MiB, in the
// top 64 MiB above the extra memory; study R136), the texture
// sheets at 0xC00000, the extra memory (VROM, R100) at 0x1000000 (64 MiB). Nothing touches DDR3 before ddr_ok (R71: hps_io's first
// command and RESET low for 100 ms): the arbiter's request is gated on it.
//
// SDRAM layout (bytes; study 7.3, sim/system/notes.md, the MRA):
//   0x0000000  banked CROM, 64 MiB (CROM0-CROM3, 8 MiB a bank)
//   0x4000000  VROM, 32 MiB (stage B; four 8 MiB blocks of four ROMs)
//   0x6000000  fixed CROM, 2 MiB (FFE00000-FFFFFFFF)
//   0x6200000  backup RAM, 128 KiB (the MRA fills it with FF)
//   0x6400000  work RAM, 8 MiB
//   0x6C00000  tile VRAM, 1 MiB
//   0x6D00000  culling RAM 8C, 4 MiB
//   0x7100000  8E, two copies of 1 MiB (r3d_mem)
//   0x7300000  a Step 2 set's fixed CROM, 8 MiB (R100; MRA index 3)
//
// Reset: the memories come out of reset on PLL lock and stay out
// (mem_rst_n); the game (CPU, bus, devices) is held in reset until the ROMs
// are in (rom_loaded) and during any download, OSD reset or RESET.

module m3_system
  import ppc_cache_pkg::*;
#(
  parameter int          COL_BITS = 10,
  parameter bit          DUAL     = 1'b1,        // the 128 MB module
  parameter logic [4:0]  CROM_BANKS = 5'd8,
  parameter int          EE_TWP   = 0,           // the EEPROM's write time, clocks (0: 2 ms at CLK_HZ)
  parameter int          MIDI_GAP = 0,           // a MIDI byte, clocks (0: 320 us at CLK_HZ)
  parameter bit          OVERLAY  = 1'b1,
  // CPU performance on the board (stage A): m3_cpu_mon's counters, latched and
  // reported once a second on the debug UART ("P nn value", tools/uart_capture.py)
  // and shown on the overlay with CPI x 256. 0 drops it (the release).
  parameter bit          CPI_MON  = 1'b1,
  parameter bit          FPS      = 1'b1,         // the frame counter, top right, OSD-switched; kept in release builds
  // ISB = 1: a 32 KiB I-cache, twice the 603e's 16 KiB (study R121, fetch side):
  // +12 M10K, Daytona 2 work CPI -8.7%, VF3 -1.9%. 0 builds the 603e's 16 KiB.
  parameter bit          ISB      = 1'b1,
  parameter bit          FBUSY    = 1'b1,         // the CPU's busy time per frame on overlay row 7 (study R104)
  // RELEASE = 1 drops every debug block together: the overlay and its capture
  // logic, CPI_MON's counters and UART, FBUSY (study R105). Release builds
  // set it with the Verilog macro M3_RELEASE (Model3.qsf); test builds keep
  // the debug.
`ifdef M3_RELEASE
  parameter bit          RELEASE  = 1'b1,
`else
  parameter bit          RELEASE  = 1'b0,
`endif
  // XCAP = 0 (macro M3_NO_XCAP) leaves the overlay's exception rows 8-11 at zero, so their capture logic
  // is not built: a smaller test build (study R105)
`ifdef M3_NO_XCAP
  parameter bit          XCAP     = 1'b0,
`else
  parameter bit          XCAP     = 1'b1,
`endif
  parameter int          CLK_HZ   = 100_000_000,
  // D-41: the 603e revision the game reads (Step 1.0: rev 1.3, as MAME). VF3's
  // boot code and TLB-miss handler set W in every BAT and RPA on revisions below
  // 1.3; at 1.3 RAM is copy-back (D-14: per step)
  parameter logic [31:0] PVR      = 32'h0006_0103,  // 603e rev 1.3: RAM copy-back, the D-cache snoops the 53C810 (D-41, R101)
  parameter logic [28:0] DDR_BASE = 29'h0600_0000,  // byte 0x30000000, the cores' DDR3 (Model 2 R654)
  // R100 (Daytona 2): Step 2.x, and where the MRA's index 1 (VROM) lives
  parameter bit          STEP2    = 1'b1,         // the Step 2.x hardware, chosen at run time by the step
  parameter bit          SEC5881  = 1'b1,         // the security board's 315-5881 (with STEP2)
  parameter int          XMEM     = 1,            // index 1: 1 DDR3, 2 the second SDRAM (not built), 0 none
  // ---- R100: where a Step 1.x set's VROM lives (the MRA's index-0 image at 0x4000000):
  //   0 SDRAM, as before; 1 DDR3 and 2 the second SDRAM, i.e. the extra memory, which
  //   must then be that memory (VROM_LOC == XMEM). The loader routes that range of
  //   index 0 there and the VROM readers (geometry, texture upload) read it there.
  //   A Step 2 set's VROM is always in the extra memory (its index 1).
  parameter int          VROM_LOC = 1,
  // ---- R127: VROM in the extra memory in pair order (VROM word {g, b, h} at beat {g, b}, not the MRA's
  //   block-major b * BLK + g), so a geometry request's pairs are one DDR3 burst; the loader's writes and
  //   m3_xmem's reads both follow it. 0: the MRA layout, a beat a read, as before
  parameter bit          VROM_LIN = 1'b1,
  parameter int          DDR_MAXQ = 8,               // m3_ddr3p: DDR3 reads queued or in flight at most (1-32; R101)
  parameter bit          SND_DSP  = 1'b1,            // the sound link (D-53): the SCSP DSPs and the HPS bridge on DDR3 port 7; 0 silence
  // ---- SDRAM scheduling for the CPU (study R107; all 0: m2_sdram as it was)
  //   SDR_PRI      ports granted ahead of the round-robin (m2_sdram PRI; bit 0 the bus, i.e. the CPU)
  //   SDR_PRI_CAP  PRI grants in a row while others wait, before one round-robin grant
  //   SDR_AGE      ports taken into the class after waiting SDR_AGE_T clocks (m2_sdram AGE)
  //   SDR_BXOR     bank bits XOR the row's low bits (m2_sdram BXOR): each region spread over the four banks
  //   SDR_L16      the bus's line fill as one 16-column read (m2_sdram L16, m3_sdr_port L16)
  //   SDR_LW1      a cast-out's beats one a clock into m3_sdr_port, not one in four (m3_sdr_port LW1)
  //   SDR_PAIR     with SDR_L16: the line fill from its critical word, delivered a 32-bit word at a time (m2_sdram PAIR)
  //   SDR_RA       with SDR_L16 and SDR_PAIR: m3_sdr_port reads the next line ahead into a two-line buffer (m3_sdr_port RA)
  //   SDR_PW       m3_sdr_port posts a line write's bursts from a buffer of their own; reads of other lines pass them (PW)
  parameter logic [5:0]  SDR_PRI     = 6'b000000,
  parameter int          SDR_PRI_CAP = 2,
  parameter logic [5:0]  SDR_AGE     = 6'b000000,
  parameter int          SDR_AGE_T   = 48,
  parameter bit          SDR_BXOR    = 1'b0,
  parameter bit          SDR_L16     = 1'b0,
  parameter bit          SDR_LW1     = 1'b0,
  parameter bit          SDR_PAIR    = 1'b0,
  parameter bit          SDR_RA      = 1'b0,
  parameter bit          SDR_PW      = 1'b0,
  parameter bit          SDR_RA_CONF = 1'b0,            // SDR_RA: arm only on a stream seen (m3_sdr_port RA_CONF)
  // ---- the CPU's miss path, clocks taken out (study R112; all 0: as it was)
  //   BUS_FAST     m3_bus: the CPU's work-RAM reads go to the port from B_IDLE (m3_bus FAST)
  //   SDR_FREQ     m3_sdr_port: a read's request to m2_sdram in the clock the port takes it (m3_sdr_port FREQ)
  //   SDR_FWD      with SDR_L16 and SDR_PAIR: each fill beat to the bus as it arrives (m3_sdr_port FWD)
  //   SDR_FDSP     m2_sdram: the bus port's read straight to S_DISPATCH when the controller is empty (FDSP)
  //   SDR_HITRD    m2_sdram: a read with its row open issues its first READ from S_DISPATCH (HITRD)
  parameter bit          BUS_FAST    = 1'b0,
  parameter bit          SDR_FREQ    = 1'b0,
  parameter bit          SDR_FWD     = 1'b0,
  parameter bit          SDR_FDSP    = 1'b0,
  parameter bit          SDR_HITRD   = 1'b0,
  //   SDR_PW_DLY   with SDR_PW: a posted line write waits this many clocks before its bursts unless needed (m3_sdr_port PW_DLY)
  parameter int          SDR_PW_DLY  = 0,
  //   BUS_R3P      m3_bus: writes to the Real3D's host port posted in an eight-word buffer, the bus freed (m3_bus R3P)
  parameter bit          BUS_R3P     = 1'b0,
  // ---- the cabinet's controls (m3_inputs): INPUTS 0 ties m3_io's inputs idle (in0 FF, test and service high,
  //   ports C and D the configuration's idle bytes, the ADC 00), the core before them; INP_DRIVE, INP_GUN,
  //   INP_FISH build the driving, analog-gun and fishing profiles (off: those games read the generic ports)
  parameter bit          INPUTS      = 1'b1,
  parameter bit          INP_DRIVE   = 1'b1,
  parameter bit          INP_GUN     = 1'b1,
  parameter bit          INP_FISH    = 1'b1,
  parameter bit          INP_LEAN    = 1'b0,       // the driving block without "Steering", "Steering smoothing", "Analog control"
  //   TG_TIMING    m3_bus: the tile generator's bus timing, tile RAM and palette writes at the board's rate (R152)
  parameter bit          TG_TIMING   = 1'b1,
  // ---- the bridges pass (zero cycles, the same words at every port's strobe)
  //   SDR_SHD      m2_sdram: the read register shared by every port but r3d_mem's (port 4, which reads its word on
  //                through its write burst); the others take p_dout only with p_lo / p_ack (m2_sdram SHD)
  parameter bit          SDR_SHD     = 1'b1,
  //   BR_C106      m3_bus: the MPC106's 64 registers in one M10K, as they behaved (m3_bus C106_M10K)
  //   BR_SEC       one second counter for the RTC and the frame counter (m3_bus, m3_fps EXT_SEC)
  //   BR_BUS       m3_bus: the targets' fields wired, wd from the owner in B_DEC (m3_bus OWIRE, WD_DEC); m3_scripts:
  //                DSPS and TEMP load one shared value (m3_scripts LDV)
  parameter bit          BR_C106     = 1'b1,
  parameter bit          BR_SEC      = 1'b1,
  parameter bit          BR_BUS      = 1'b1,
  //   BR_LD        m3_loader: the destinations' fields wired from the drain's registers (m3_loader OWIRE)
  parameter bit          BR_LD       = 1'b1
) (
  input  logic        clk,
  input  logic        mem_rst_n,
  input  logic        reset,         // RESET | OSD reset | buttons

  input  logic        ioctl_download,
  input  logic [15:0] ioctl_index,
  input  logic        ioctl_wr,
  input  logic [26:0] ioctl_addr,
  input  logic [15:0] ioctl_dout,
  output logic        ioctl_wait,

  input  logic [2:0]  rd_lat_sel,
  output logic        sd_cke, sd_cs_n, sd_ras_n, sd_cas_n, sd_we_n,
  output logic [1:0]  sd_ba,
  output logic [12:0] sd_a,
  output logic [1:0]  sd_dqm,
  output logic [15:0] sd_dq_o,
  output logic        sd_dq_oe,
  input  logic [15:0] sd_dq_i,

  // DDR3 (the framework's DDRAM port), and the gate on it (R71)
  input  logic        ddr_ok,
  output logic        DDRAM_CLK,
  input  logic        DDRAM_BUSY,
  output logic [7:0]  DDRAM_BURSTCNT,
  output logic [28:0] DDRAM_ADDR,
  output logic [63:0] DDRAM_DIN,
  output logic [7:0]  DDRAM_BE,
  output logic        DDRAM_WE,
  output logic        DDRAM_RD,
  input  logic [63:0] DDRAM_DOUT,
  input  logic        DDRAM_DOUT_READY,

  output logic        ce_pix,
  output logic        v_hs, v_vs, v_hblank, v_vblank,
  output logic [7:0]  v_r, v_g, v_b,
  input  logic        overlay_en,
  input  logic        fps_en,        // the frame counter (FPS) shown
  input  logic  [1:0] fps_avg,       // its window: 0 5 s, 1 10 s, 2 1 s (m3_fps avg)

  // the cabinet's controls (hps_io, clk_sys; INPUTS)
  input  logic [15:0] joystick_0,    // player 1: 3:0 right, left, down, up; 15:4 the J1 buttons
  input  logic [15:0] joystick_1,    // player 2
  input  logic [15:0] joy_l0,        // player 1's left stick {Y, X}, signed
  input  logic [15:0] joy_l1,        // player 2's left stick
  input  logic [15:0] joy_r0,        // player 1's right stick
  input  logic  [7:0] joy_lt0,       // player 1's analogue triggers (hps_io 0x46)
  input  logic  [7:0] joy_rt0,
  input  logic  [7:0] paddle0,
  input  logic        st_pedswap,    // the driving OSD lines (m3_cab_inputs)
  input  logic  [1:0] st_ssens,
  input  logic  [1:0] st_ssm,
  input  logic        st_rsteer,
  input  logic  [1:0] st_wheel,
  output logic        inp_drive,     // the driving profile is loaded (the OSD shows its lines)

  output logic signed [15:0] audio_l,  // the sound link's (front + rear) / 2 a side with its gain, 44.1 kHz held (0 without SND_DSP)
  output logic signed [15:0] audio_r,
  input  logic  [1:0] snd_vol,         // the OSD's Volume: 0 Normal, 1 +6 dB, 2 +12 dB (m3_sndbridge)
  output logic        led,
  output logic        uart_tx,
  input  logic        tb_fast,       // bench only (0 on the board): the timebase at the clock rate

  // bench
  output logic        rom_loaded,
  output logic [31:0] frame,
  output logic        tr_v,
  output logic        tr_master,
  output logic        tr_we,
  output logic [31:0] tr_addr,
  output logic [3:0]  tr_be,
  output logic [31:0] tr_data,
  output logic        ev_v,          // events: 1 IRQ set (bits), 2 ping-pong flip, 3 frame start
  output logic [1:0]  ev_kind,
  output logic [7:0]  ev_data,
  output logic        rt_valid_o,
  output logic [31:0] rt_pc_o,
  output logic [31:0] rt_insn_o,
  output logic        rt_gpr_we_o,
  output logic [4:0]  rt_gpr_o,
  output logic        rt_gpr_sh_o,
  output logic [31:0] rt_gpr_val_o
);

  localparam int AW = (DUAL ? 1 : 0) + 2 + 13 + COL_BITS;
  // everything that counts real time follows CLK_HZ (100 MHz; 90 MHz for the
  // diagnostic build): the pixel enable (16 MHz), the timebase (16.5 MHz), the
  // SDRAM refresh (7.7 us), the EEPROM's write time, a MIDI byte, the UART
  localparam int CLK_MHZ = CLK_HZ / 1_000_000;
  localparam int EE_TWP_C   = (EE_TWP != 0)   ? EE_TWP   : CLK_HZ / 500;
  localparam int MIDI_GAP_C = (MIDI_GAP != 0) ? MIDI_GAP : CLK_HZ / 3125;
  localparam int T_REFI_C   = CLK_MHZ * 77 / 10;
  // SDRAM row timings for a -6 part (tRC 60 ns, tRAS 42 ns), rounded up to whole clocks, never below m2_sdram's
  // 7 / 5 (the counts proven on the board at 90-110 MHz): 7 / 5 to 116 MHz, 8 / 6 at 120. tRCD and tRP (15 ns)
  // stay at m2_sdram's 2 clocks to 133 MHz.
  localparam int T_RC_N     = (CLK_MHZ * 60 + 999) / 1000;
  localparam int T_RAS_N    = (CLK_MHZ * 42 + 999) / 1000;
  localparam int T_RC_C     = (T_RC_N  > 7) ? T_RC_N  : 7;
  localparam int T_RAS_C    = (T_RAS_N > 5) ? T_RAS_N : 5;
  localparam logic [26:0] SDR_CROM = 27'h000_0000, SDR_FIXED = 27'h600_0000, SDR_BACKUP = 27'h620_0000,
                          SDR_RAM = 27'h640_0000, SDR_VRAM = 27'h6C0_0000, SDR_FIX8 = 27'h730_0000;

  // ===========================================================================
  // reset
  // ===========================================================================
  logic loader_busy /* verilator public_flat_rd */;
  logic [2:0] rst_s = 3'b111;
  always_ff @(posedge clk) rst_s <= {rst_s[1:0], reset || !mem_rst_n || !rom_loaded || loader_busy};
  wire rst = rst_s[2];
  // copies of rst_s[2] for the geometry and the rasteriser, the same function and timing, kept apart
  // (preserve) so each block's reset fans out from a flip-flop of its own (rst_s[2] into the
  // rasteriser's registers was -0.52 ns at 100 MHz; study R104)
  (* preserve *) logic rst_geo = 1'b1, rst_rast = 1'b1;
  // and one for the CPU (rst_s[2] into the FP divider's registers, -1.11 ns in a 110 MHz merged fit)
  (* preserve *) logic rst_cpu = 1'b1;
  // and one for the sound link (SND_DSP), which spans its own corner of the device
  (* preserve *) logic rst_snd = 1'b1;
  always_ff @(posedge clk) begin
    rst_geo  <= rst_s[1];
    rst_rast <= rst_s[1];
    rst_cpu  <= rst_s[1];
    rst_snd  <= rst_s[1];
  end
`ifdef VERILATOR
  always_ff @(posedge clk) if ({rst_geo, rst_rast, rst_cpu, rst_snd} != {4{rst}}) $fatal(1, "m3_system: the reset copies");
`endif
  logic por_n;                       // power-on: the EEPROM and the I/O latches
  always_ff @(posedge clk) por_n <= mem_rst_n;

  // ===========================================================================
  // SDRAM: port 0 the bus, 1 the tile generator, 2 the geometry's culling
  // reads, 3 its VROM reads, 4 r3d_mem, 5 r3d_texup; the loader on wr_*
  // ===========================================================================
  localparam int NSP = 6;
  logic          mem_ready;
  logic          ld_req, ld_ack;
  logic [AW:1]   ld_addr;
  logic [15:0]   ld_din;
  logic [1:0]    ld_be;
  logic [NSP-1:0]          p_req, p_we, p_long, p_lo, p_ack;
  logic [NSP-1:0][AW:1]    p_addr;
  logic [NSP-1:0][15:0]    p_din;
  logic [NSP-1:0][1:0]     p_be;
  logic [NSP-1:0][63:0]    p_dout;

  logic [3:0]  wb_col;
  logic [15:0] wb_din;
  logic [1:0]  wb_be;
`ifdef M3_R3B
  // (study R144) two write-burst ports, the bus and r3d_mem: the columns come from the port the controller is
  // writing for
  logic [2:0]  wb_grant;
  logic [15:0] sp_wbd, r3_wbd;
  logic [1:0]  sp_wbb, r3_wbb;
  assign wb_din = (wb_grant == 3'd4) ? r3_wbd : sp_wbd;
  assign wb_be  = (wb_grant == 3'd4) ? r3_wbb : sp_wbb;
`endif
  m2_sdram #(.COL_BITS(COL_BITS), .NP(NSP), .T_REFI(T_REFI_C), .T_RC(T_RC_C), .T_RAS(T_RAS_C), .ACK_HOLD(1), .DUAL(DUAL), .MASK_ON_A(DUAL),
             .PRI(SDR_PRI), .PRI_CAP(SDR_PRI_CAP), .AGE(SDR_AGE), .AGE_T(SDR_AGE_T), .BXOR(SDR_BXOR),
             .L16({{(NSP-1){1'b0}}, SDR_L16}), .PAIR({{(NSP-1){1'b0}}, SDR_PAIR}),
`ifdef M3_R3B
             .WB(NSP'(6'b010001)), .FDSP(SDR_FDSP), .HITRD(SDR_HITRD),
             .SHD(SDR_SHD ? NSP'(6'b101111) : NSP'(0))) u_sdram (   // R101, R144: the bus (port 0) and r3d_mem (port 4) write in bursts
`else
             .WB({{(NSP-1){1'b0}}, 1'b1}), .FDSP(SDR_FDSP), .HITRD(SDR_HITRD),
             .SHD(SDR_SHD ? NSP'(6'b101111) : NSP'(0))) u_sdram (   // port 0 (the bus) writes in bursts (R101)
`endif
    .clk, .rst_n(mem_rst_n), .ready(mem_ready), .rd_lat_sel,
    .sd_cke, .sd_cs_n, .sd_ras_n, .sd_cas_n, .sd_we_n, .sd_ba, .sd_a, .sd_dqm, .sd_dq_o, .sd_dq_oe, .sd_dq_i,
    .wr_req(ld_req), .wr_addr(ld_addr), .wr_din(ld_din), .wr_be(ld_be), .wr_ack(ld_ack),
    .p_req, .p_we, .p_addr, .p_din, .p_be, .p_dout, .p_ack, .p_long, .p_lo, .wb_col, .wb_din, .wb_be,
`ifdef M3_R3B
    .wb_grant(wb_grant),
`endif
    .dbg_req(), .dbg_grant());

  logic [26:0] ld_bytes;
  logic        ld_ovf;
  // R100: the extra memory's writes, the 93C46's load, the game configuration
  logic        lx_req, lx_ack, ee_ld_we;
  logic [26:1] lx_addr;
  logic [15:0] lx_din, ee_ld_d;
  logic [5:0]  ee_ld_a;
  logic [7:0]  cfg_step /* verilator public_flat_rd */, cfg_flags /* verilator public_flat_rw */;   // (the bench may set bit 0: --vrom-x)
  logic [15:0][4:0] cfg_bank /* verilator public_flat_rd */;
  logic [7:0]  cfg_in2 /* verilator public_flat_rd */, cfg_in3 /* verilator public_flat_rd */;
  logic [7:0]  cfg_dsw /* verilator public_flat_rd */;   // the DIP bank (R133)
  logic [7:0]  cfg_prof /* verilator public_flat_rd */, cfg_pflg;   // the input profile and its flags
  m3_loader #(.AW(AW), .BACKUP(SDR_BACKUP), .FIX8(SDR_FIX8), .OWIRE(BR_LD),
              .VROM_X((VROM_LOC != 0) && (VROM_LOC == XMEM)), .VROM_AT(27'h400_0000)) u_ld (   // R100: VROM placement
    .clk, .rst(!mem_rst_n), .mem_ready, .x_ok(ddr_ok),
    .ioctl_download, .ioctl_index, .ioctl_wr, .ioctl_addr, .ioctl_dout, .ioctl_wait,
    .sdr_wr_req(ld_req), .sdr_wr_addr(ld_addr), .sdr_wr_din(ld_din), .sdr_wr_be(ld_be), .sdr_wr_ack(ld_ack),
    .x_req(lx_req), .x_addr(lx_addr), .x_din(lx_din), .x_ack(lx_ack),
    .ee_we(ee_ld_we), .ee_a(ee_ld_a), .ee_d(ee_ld_d),
    .cfg_step, .cfg_flags, .cfg_bank, .cfg_in2, .cfg_in3, .cfg_dsw, .cfg_prof, .cfg_pflg,
    .rom_loaded, .busy(loader_busy), .overflow(ld_ovf), .bytes(ld_bytes));

  // R100: the game configuration, static while the game runs (it changes only
  // during a download, with the game in reset)
  // The step's decodes are registered (c7_s1: cfg_step into m3_bus's read data and r3d_texup's FIFO, -1.1 ns): they
  // follow cfg_step a cycle later, while the loader is busy and the game in reset.
  logic       step2, step15;
  logic [1:0] jstep;
  always_ff @(posedge clk) begin
    step2  <= STEP2 && cfg_step >= 8'h20;                     // the Step 2.x devices
    step15 <= cfg_step >= 8'h15;                              // 13.11 vertices, full culling nodes
    jstep  <= {cfg_step >= 8'h20, cfg_step == 8'h15 || cfg_step == 8'h21};
  end
  // VROM in the extra memory: by placement (VROM_LOC), or by the set (Step 2: flags bit 0)
  wire        vrom_x  = (XMEM == 1) && (((VROM_LOC != 0) && (VROM_LOC == XMEM)) || cfg_flags[0]);
  wire        fixed8  = cfg_flags[1];                         // an 8 MiB fixed CROM at SDR_FIX8
  wire        vrom64  = cfg_flags[2];
  logic [29:0] sec_key;                                       // the 315-5881 key: configuration bytes 18-21
  assign sec_key = 30'h2925_0E16;                             // (daytona2; MAME ROM_PARAMETER ":315_5881:key")

  // ===========================================================================
  // the CPU
  // ===========================================================================
  // the timebase and decrementer count at the 66 MHz bus / 4 (D-14): 16.5 MHz
  logic [8:0] tb_acc;   // 9 bits: tb_acc + 33 reaches 2 x CLK_MHZ + 32, past 255 above 111 MHz
  logic       tb_tick;
  always_ff @(posedge clk) begin
    if (rst) begin tb_acc <= '0; tb_tick <= 1'b0; end
    else if (tb_fast) tb_tick <= 1'b1;
    else if (tb_acc + 9'd33 >= 9'(2 * CLK_MHZ)) begin tb_acc <= tb_acc + 9'd33 - 9'(2 * CLK_MHZ); tb_tick <= 1'b1; end
    else begin tb_acc <= tb_acc + 9'd33; tb_tick <= 1'b0; end
  end

  logic [1:0]       m_req, m_ack, m_wnext, m_rvalid;
  logic [1:0][1:0]  m_cmd;
  logic [1:0][31:0] m_addr, m_wdata;
  logic [1:0][3:0]  m_be;
  logic [31:0]      m_rdata;
  logic             sn_req, sn_inv, sn_ack;
  logic [31:5]      sn_addr;
  logic             ext_int;
  logic             rt_valid, rt_gpr_we, rt_gpr_sh;
  logic [4:0]       rt_gpr;
  logic [31:0]      rt_gpr_val;
  logic [7:0]       cpu_ev;
  logic [5:0]       cpu_ev_mmu;
  logic [1:0]       rt_irq;
  logic [31:0]      rt_lr, rt_msr, rt_srr0, rt_srr1;
  logic [31:0]      rt_pc;

  // the debug blocks as built (RELEASE turns them all off)
  localparam bit OV_ON  = OVERLAY && !RELEASE;
  localparam bit MON_ON = CPI_MON && !RELEASE;
  localparam bit FB_ON  = FBUSY   && !RELEASE;
  localparam bit XC_ON  = XCAP    && OV_ON;

  ppc_cpu #(.PVR(PVR), .ISB(ISB)) u_cpu (
    .clk, .rst(rst_cpu),
    .m_req(m_req[0]), .m_cmd(m_cmd[0]), .m_addr(m_addr[0]), .m_be(m_be[0]), .m_wdata(m_wdata[0]),
    .m_ack(m_ack[0]), .m_wnext(m_wnext[0]), .m_rvalid(m_rvalid[0]), .m_rdata,
    // the D-cache snoop (D-41): the 53C810's RAM accesses, from m3_bus
    .sn_req, .sn_addr, .sn_inv, .sn_ack,
    .tb_tick, .ext_int, .ev(cpu_ev), .ev_mmu(cpu_ev_mmu), .ev_btb(),
    .rt_valid, .rt_last(), .rt_pc, .rt_insn(rt_insn_o), .rt_gpr_we, .rt_gpr, .rt_gpr_sh,
    .rt_irq(rt_irq), .rt_gpr_val,
    .rt_cr(), .rt_xer(), .rt_lr(rt_lr), .rt_ctr(), .rt_msr(rt_msr), .rt_srr0(rt_srr0), .rt_srr1(rt_srr1), .rt_fpscr(), .rt_fpq(),
    .rt_fw_en(), .rt_fw_idx(), .rt_fw_val(), .rt_dly(),
    .t_dreq(), .t_dwe(), .t_daddr(), .t_dwimg(), .t_dop(), .t_dvalid(), .t_drdata(),
    .t_dbe(), .t_dwdata(), .t_drsv(), .t_dcnd());
  assign rt_valid_o = rt_valid;
  assign rt_gpr_we_o = rt_gpr_we; assign rt_gpr_o = rt_gpr; assign rt_gpr_sh_o = rt_gpr_sh; assign rt_gpr_val_o = rt_gpr_val;

  // ---- CPU performance: m3_cpu_mon, a report a second ----
  //   00 cycles  01 instructions  02 uops  03 decode starved  04 load-use
  //   05 mul/div wait  06 data memory wait  07 redirects  08 fetch waiting
  logic [8:0][31:0] perf;
  logic [15:0]      cpi_q8;          // CPI x 256 of the last report
  generate if (MON_ON) begin : g_mon
    logic [31:0] sec;
    logic        rep, clr;
    always_ff @(posedge clk) begin
      rep <= 1'b0; clr <= rep;
      if (rst) sec <= '0;
      else if (sec == 32'(CLK_HZ - 1)) begin sec <= '0; rep <= 1'b1; end
      else sec <= sec + 32'd1;
    end
    m3_cpu_mon #(.DIVISOR((CLK_HZ + 57_600) / 115_200), .SIG(1'b0)) u_mon (
      .clk, .rst, .clear(clr), .report(rep), .ev(cpu_ev),
      .rt_valid, .rt_gpr_we, .rt_gpr, .rt_gpr_val, .busy(), .tx(uart_tx), .counts(perf));
    // CPI x 256 = (cycles << 8) / instructions, a bit a clock after each report
    logic [39:0] num;
    logic [31:0] den;
    logic [40:0] rem;
    logic [5:0]  k;
    logic [23:0] quo;
    logic        div_go, div_run;
    wire  [40:0] r2 = {rem[39:0], num[39]};
    always_ff @(posedge clk) begin
      div_go <= clr;
      if (rst) begin div_run <= 1'b0; cpi_q8 <= '0; end
      else if (div_go) begin
        num <= {perf[0], 8'd0}; den <= perf[1]; rem <= '0; quo <= '0; k <= 6'd40; div_run <= 1'b1;
      end else if (div_run) begin
        num <= {num[38:0], 1'b0};
        if (r2 >= {9'd0, den}) begin rem <= r2 - {9'd0, den}; quo <= {quo[22:0], 1'b1}; end
        else begin rem <= r2; quo <= {quo[22:0], 1'b0}; end
        k <= k - 6'd1;
        if (k == 6'd1) begin
          div_run <= 1'b0;
          cpi_q8 <= (den == 32'd0) ? 16'hFFFF : (quo[22:15] != 8'd0) ? 16'hFFFF : {quo[14:0], (r2 >= {9'd0, den})};
        end
      end
    end
  end else begin : g_nomon
    assign perf = '0; assign cpi_q8 = '0; assign uart_tx = 1'b1;
  end endgenerate
  assign rt_pc_o = rt_pc;

  // ===========================================================================
  // the 53C810
  // ===========================================================================
  logic        sc_req, sc_we, sc_ack, sc_win_en, cf_req, cf_we, cf_ack, scsi_irq, scsi_busy;
  logic [6:2]  sc_addr;
  logic [3:0]  sc_be, cf_be;
  logic [31:0] sc_wdata, sc_rdata, cf_wdata, cf_rdata;
  logic [31:8] sc_win_base;
  logic [5:0]  cf_idx;

  m3_scripts #(.R3DMA(STEP2), .LDV(BR_BUS)) u_scsi (
    .clk, .rst, .step2,     // R100: Step 2.x, the Real3D DMA device on this move engine
    .c_req(cf_req), .c_we(cf_we), .c_idx(cf_idx), .c_be(cf_be), .c_wdata(cf_wdata), .c_ack(cf_ack), .c_rdata(cf_rdata),
    .win_en(sc_win_en), .win_base(sc_win_base),
    .s_req(sc_req), .s_we(sc_we), .s_addr(sc_addr), .s_be(sc_be), .s_wdata(sc_wdata), .s_ack(sc_ack), .s_rdata(sc_rdata),
    .m_req(m_req[1]), .m_cmd(m_cmd[1]), .m_addr(m_addr[1]), .m_be(m_be[1]), .m_wdata(m_wdata[1]),
    .m_ack(m_ack[1]), .m_wnext(m_wnext[1]), .m_rvalid(m_rvalid[1]), .m_rdata,
    .irq(scsi_irq), .busy(scsi_busy));

  // ===========================================================================
  // the bus and its devices
  // ===========================================================================
  logic        s_req, s_swap, s_zero, s_ack, s_wnext, s_rvalid, s_idle;
  logic [1:0]  s_cmd;
  logic [AW:0] s_sa;
  logic [3:0]  s_be;
  logic [31:0] s_wdata, s_rdata;
  logic        io_sel, io_we;
  logic [3:0]  io_off;
  logic [7:0]  io_wbyte, io_rbyte, io_pa;
  logic        tg_reg_we, tg_pal_we;
  logic [5:0]  tg_reg_addr;
  logic [31:0] tg_reg_wdata;
  logic [14:0] tg_pal_addr;
  logic [15:0] tg_pal_wdata;
  logic        ln_start;
  logic [9:0]  ln;
  logic [7:0]  irq_state, irq_enable, crom_bank;
  logic [23:0] n_submit, n_cull_lo, n_cull_hi, n_poly, n_tex, n_vrom;
  logic [15:0] n_unmapped;
  logic [31:0] last_unmapped;
  logic        pingpong;
  logic [18:0] r3d_cfg0, r3d_cfg1, r3d_cfg2, jt_cfg0, jt_cfg1, jt_cfg2;
  logic        jt_wr, jt_busy, jt_tdo, jt_cfg_we;
  logic [7:0]  jt_wdata;
  logic        h_go, h_rdy, h_we, h_rv, r3_pingpong, r3_flip;
  logic [31:0] h_addr, h_wdata, h_rdata;
  logic        mw_we;
  logic [3:0]  mw_dev;
  logic [31:0] mw_val;

  // R100: the security board, and the bank register's slot from the game's bank map
  logic        sec_req, sec_we, sec_ack, sec_cmp;
  logic [17:2] sec_addr;
  logic [3:0]  sec_be;
  logic [31:0] sec_wdata, sec_rdata;
  wire  [4:0]  crom_map = cfg_bank[~crom_bank[3:0]];
  generate if (STEP2 && SEC5881) begin : g_sec
    m3_sec u_sec (
      .clk, .rst, .game_key(sec_key),
      .req(sec_req), .we(sec_we), .addr(sec_addr), .be(sec_be), .wdata(sec_wdata), .ack(sec_ack), .rdata(sec_rdata),
      .cmp_seen(sec_cmp));
  end else begin : g_nosec
    always_ff @(posedge clk) sec_ack <= sec_req && !sec_ack;
    assign sec_rdata = 32'hFFFF_FFFF; assign sec_cmp = 1'b0;
  end endgenerate
`ifndef M3_STEP2_BUS
  assign sec_req = 1'b0; assign sec_we = 1'b0; assign sec_addr = '0; assign sec_be = '0; assign sec_wdata = '0;
`endif

  // one second of clk (the bridges pass): the RTC's (m3_bus EXT_SEC) and the frame counter's (m3_fps EXT_SEC), the
  // count each of them kept, from the game's reset
  localparam int SCW = $clog2(CLK_HZ);
  logic [SCW-1:0] sec_cnt;
  wire            sec_tick = BR_SEC && (sec_cnt == SCW'(CLK_HZ - 1));
  always_ff @(posedge clk) if (rst || sec_tick) sec_cnt <= '0; else sec_cnt <= sec_cnt + 1'd1;
`ifdef VERILATOR
  // the counter m3_bus's rtc_pre and m3_fps's tcnt each were (reset with rst, CLK_HZ - 1 then 0): sec_tick is its wrap
  logic [SCW-1:0] sec_chk;
  always_ff @(posedge clk) begin
    if (rst) sec_chk <= '0; else sec_chk <= (sec_chk == SCW'(CLK_HZ - 1)) ? '0 : sec_chk + 1'd1;
    if (!rst && BR_SEC && sec_tick != (sec_chk == SCW'(CLK_HZ - 1))) $fatal(1, "m3_system: sec_tick against the old prescaler");
  end
`endif

  m3_bus #(.AW(AW), .SDR_CROM(SDR_CROM), .SDR_FIXED(SDR_FIXED), .SDR_BACKUP(SDR_BACKUP), .SDR_RAM(SDR_RAM),
           .SDR_VRAM(SDR_VRAM), .CROM_BANKS(CROM_BANKS), .MIDI_GAP(MIDI_GAP_C), .FAST(BUS_FAST), .R3P(BUS_R3P), .TGT(TG_TIMING), .TG_MHZ(CLK_MHZ),
           .C106_M10K(BR_C106), .OWIRE(BR_BUS), .WD_DEC(BR_BUS)
`ifdef M3_STEP2_BUS
           , .STEP2(STEP2), .SDR_FIX8(SDR_FIX8), .RTC_HZ(CLK_HZ), .EXT_SEC(BR_SEC)
`endif
           ) u_bus (
    .clk, .rst, .sec_tick,
`ifdef M3_STEP2_BUS
    .step2, .fixed8, .crom_slot(crom_map[3:0]), .crom_zero(crom_map[4]),
    .sec_req, .sec_we, .sec_addr, .sec_be, .sec_wdata, .sec_ack, .sec_rdata,
`endif
    .m_req, .m_cmd, .m_addr, .m_be, .m_wdata, .m_ack, .m_wnext, .m_rvalid, .m_rdata,
    .sn_req, .sn_addr, .sn_inv, .sn_ack,
    .s_req, .s_cmd, .s_sa, .s_swap, .s_zero, .s_be, .s_wdata, .s_ack, .s_wnext, .s_rvalid, .s_rdata,
    .sc_req, .sc_we, .sc_addr, .sc_be, .sc_wdata, .sc_ack, .sc_rdata, .sc_win_en, .sc_win_base,
    .cf_req, .cf_we, .cf_idx, .cf_be, .cf_wdata, .cf_ack, .cf_rdata, .scsi_irq,
    .io_sel, .io_we, .io_off, .io_wbyte, .io_rbyte,
    .tg_reg_we, .tg_reg_addr, .tg_reg_wdata, .tg_pal_we, .tg_pal_addr, .tg_pal_wdata,
    .ln_start, .ln, .ext_int,
    .irq_state_o(irq_state), .irq_enable_o(irq_enable), .crom_bank_o(crom_bank),
    .n_submit, .n_cull_lo, .n_cull_hi, .n_poly, .n_tex, .n_vrom, .n_unmapped, .last_unmapped,
    .pingpong_o(pingpong), .pingpong_i(r3_pingpong), .h_go, .h_rdy, .h_we, .h_addr, .h_wdata, .h_rv, .h_rdata,
    .r3d_cfg0, .r3d_cfg1, .r3d_cfg2,
    .jt_wr, .jt_wdata, .jt_busy, .jt_tdo, .jt_cfg_we, .jt_cfg0, .jt_cfg1, .jt_cfg2,
    .snd_we, .snd_data, .snd_byte,
    .tr_v, .tr_master, .tr_we, .tr_addr, .tr_be, .tr_data);

  m3_sdr_port #(.AW(AW), .L16(SDR_L16), .LW1(SDR_LW1), .PAIR(SDR_PAIR), .RA(SDR_RA), .RA_TOP(SDR_VRAM), .RA_CONF(SDR_RA_CONF), .PW(SDR_PW), .FWD(SDR_FWD), .FREQ(SDR_FREQ), .PW_DLY(SDR_PW_DLY)) u_sport (
    .clk, .rst(!mem_rst_n), .ra_clr(rst),
    .s_req, .s_cmd, .s_sa, .s_swap, .s_zero, .s_be, .s_wdata, .s_ack, .s_wnext, .s_rvalid, .s_rdata, .s_idle,
    .p_req(p_req[0]), .p_we(p_we[0]), .p_addr(p_addr[0]), .p_din(p_din[0]), .p_be(p_be[0]), .p_long(p_long[0]),
`ifdef M3_R3B
    .p_dout(p_dout[0]), .p_lo(p_lo[0]), .p_ack(p_ack[0]), .wb_col, .wb_din(sp_wbd), .wb_be(sp_wbb));
`else
    .p_dout(p_dout[0]), .p_lo(p_lo[0]), .p_ack(p_ack[0]), .wb_col, .wb_din, .wb_be);
`endif

  // the cabinet (m3_inputs): the sticks, triggers and paddle registered once here, beside the logic they feed
  // (Model 2 R842: hps_io's analog outputs straight into the steering block failed clk_sys there)
  logic [7:0]      io_in0, io_in2, io_in3;
  logic            io_test_b_n, io_service_b_n;
  logic [7:0][7:0] io_an;
  generate if (INPUTS) begin : g_inputs
    logic [15:0] jl0_q, jl1_q, jr0_q;
    logic  [7:0] lt0_q, rt0_q, pad0_q;
    always_ff @(posedge clk) begin
      jl0_q <= joy_l0; jl1_q <= joy_l1; jr0_q <= joy_r0;
      lt0_q <= joy_lt0; rt0_q <= joy_rt0; pad0_q <= paddle0;
    end
    m3_inputs #(.SYS_MHZ(CLK_MHZ), .DRIVE(INP_DRIVE), .GUN(INP_GUN), .FISH(INP_FISH), .LEAN(INP_LEAN)) u_inputs (
      .clk, .rst_n(!rst), .prof(cfg_prof), .pflg(cfg_pflg), .idle_c(cfg_in2), .idle_d(cfg_in3),
      .joystick_0, .joystick_1, .joy_analog(jl0_q), .joy_ranalog(jr0_q), .joy_analog_1(jl1_q), .paddle(pad0_q),
      .trig_rt(rt0_q), .trig_lt(lt0_q),
      .st_pedswap, .st_ssens, .st_ssm, .st_rsteer, .st_wheel,
      .in0(io_in0), .test_b_n(io_test_b_n), .service_b_n(io_service_b_n), .in2(io_in2), .in3(io_in3), .an(io_an),
      .drive(inp_drive));
  end else begin : g_noinputs
    assign io_in0 = 8'hFF; assign io_test_b_n = 1'b1; assign io_service_b_n = 1'b1;
    assign io_in2 = cfg_in2; assign io_in3 = cfg_in3; assign io_an = '0; assign inp_drive = 1'b0;
    /* verilator lint_off UNUSEDSIGNAL */
    wire _unused_inp = &{1'b0, joystick_0, joystick_1, joy_l0, joy_l1, joy_r0, joy_lt0, joy_rt0, paddle0,
                         st_pedswap, st_ssens, st_ssm, st_rsteer, st_wheel, cfg_prof, cfg_pflg};
    /* verilator lint_on UNUSEDSIGNAL */
  end endgenerate

  m3_io #(.EE_TWP(EE_TWP_C), .EE_LOAD(1'b1)) u_io (
    .clk, .por_n, .rst_n(!rst), .sel(io_sel), .we(io_we), .off(io_off), .wbyte(io_wbyte), .rbyte(io_rbyte),
    .in0(io_in0), .test_b_n(io_test_b_n), .service_b_n(io_service_b_n), .in2(io_in2), .in3(io_in3), .an(io_an),
    .dsw(cfg_dsw), .pa_q(io_pa),
    .ee_ld_we, .ee_ld_a, .ee_ld_d);       // R100: the MRA's <nvram>

  m3_jtag u_jtag (
    .clk, .rst, .jstep(STEP2 ? jstep : 2'd0), .wr(jt_wr), .wdata(jt_wdata), .busy(jt_busy), .tdo(jt_tdo),
    .cfg0(r3d_cfg0), .cfg1(r3d_cfg1), .cfg2(r3d_cfg2),
    .cfg_we(jt_cfg_we), .cfg0_w(jt_cfg0), .cfg1_w(jt_cfg1), .cfg2_w(jt_cfg2),
    .mw_we, .mw_dev, .mw_val);

  // ===========================================================================
  // the Real3D (D-37 stage B; sim/system/notes.md, Stage B)
  // ===========================================================================
  localparam logic [24:0] D_POLY0 = 25'h10_0000, D_POLY1 = 25'h18_0000, D_STG = 25'h20_0000,
                          D_STORE = 25'h40_0000, D_TEX = 25'hC0_0000;
  localparam logic [26:0] SDR_8C = 27'h6D0_0000, SDR_8E0 = 27'h710_0000, SDR_8E1 = 27'h720_0000, SDR_VROM = 27'h400_0000;

  // ---- DDR3: m3_ddr3p (the texture cache on its pipelined port, R100's
  // extra memory reads on a second) behind m3_ddr_arb: 0 the layer's
  // scan-out, 1 the store, 2 the layer, 3 the geometry's polygon reads,
  // 4 r3d_mem, 5 r3d_texup, 6 the loader's extra-memory writes (R100), 7 the sound
  // link (SND_DSP)
  localparam int NDP = SND_DSP ? 8 : 7;
  localparam logic [24:0] D_XMEM = 25'h100_0000;                   // R100: VROM, 64 MiB
  logic        xp_req, xp_ack, xp_rvalid;
  logic [7:0]  xp_blen;
  logic [24:0] xp_addr;
  logic [NDP-1:0]       d_req, d_we, d_wacc, d_rvalid, d_ack;
  logic [NDP-1:0][24:0] d_addr;
  logic [NDP-1:0][7:0]  d_blen, d_be;
  logic [NDP-1:0][63:0] d_din;
  logic        dm_req, dm_we, dm_wacc, dm_rvalid, dm_ack;
  logic [24:0] dm_addr;
  logic [7:0]  dm_blen, dm_be;
  logic [63:0] dm_din, dm_dout;
  logic        tx_req, tx_ack, tx_rvalid;
  logic [24:0] tx_addr;
  m3_ddr_arb #(.NP(NDP)) u_darb (
    .clk, .rst(!mem_rst_n), .req(d_req), .we(d_we), .addr(d_addr), .blen(d_blen), .din(d_din), .be(d_be),
    .wacc(d_wacc), .rvalid(d_rvalid), .ack(d_ack),
    .m_req(dm_req), .m_we(dm_we), .m_addr(dm_addr), .m_blen(dm_blen), .m_din(dm_din), .m_be(dm_be),
    .m_wacc(dm_wacc), .m_rvalid(dm_rvalid), .m_ack(dm_ack));
  // ---- the sound link (D-53): the main CPU's sound port out to the HPS, its records and commands
  // in to the SCSP DSPs, their channels out at 44.1 kHz (sm3-builds/sound/DESIGN.md section 16)
  logic       snd_we, snd_data;
  logic [7:0] snd_byte;
  generate if (SND_DSP) begin : g_snd
    m3_sndbridge #(.CLK_HZ(CLK_HZ), .D(256), .BLOCK(25'h1E4_0000)) u_snd (
      .clk, .rst(rst_snd), .midi_we(snd_we), .midi_data(snd_data), .midi_byte(snd_byte), .vol(snd_vol),
      .d_req(d_req[7]), .d_we(d_we[7]), .d_addr(d_addr[7]), .d_blen(d_blen[7]), .d_din(d_din[7]), .d_be(d_be[7]),
      .d_wacc(d_wacc[7]), .d_rvalid(d_rvalid[7]), .d_ack(d_ack[7]), .d_dout(dm_dout),
      .audio_l, .audio_r,
      .sim_ff(1'b0), .dbg_idle(), .dbg_cyc(), .dbg_tick(), .dbg_fs(), .dbg_p(), .dbg_pop(), .dbg_ch(), .dbg_q(), .dbg_skip(),
      .dbg_epoch(), .dbg_d18());
  end else begin : g_nosnd
    assign audio_l = '0; assign audio_r = '0;
    wire _unused_vol = &{1'b0, snd_vol};
  end endgenerate

  m3_ddr3p #(.PIPE(1'b1), .NP(2), .MAXQ(DDR_MAXQ)) u_ddr3 (   // R101: reads in flight capped (the probe: 16 or more raise latency)
    .clk, .rst_n(mem_rst_n), .base(DDR_BASE),
    .req(dm_req && ddr_ok), .we(dm_we), .addr(dm_addr), .blen(dm_blen), .din(dm_din), .be(dm_be),
    .wnext(), .wacc(dm_wacc), .rvalid(dm_rvalid), .ack(dm_ack),
    .p_req({xp_req && ddr_ok, tx_req && ddr_ok}), .p_addr({xp_addr, tx_addr}), .p_blen({xp_blen, 8'd4}),
    .p_we(2'b00), .p_din('0), .p_ack({xp_ack, tx_ack}), .p_rvalid({xp_rvalid, tx_rvalid}), .p_wacc(),
    .dout(dm_dout),
    .DDRAM_CLK, .DDRAM_BUSY, .DDRAM_BURSTCNT, .DDRAM_ADDR, .DDRAM_DIN, .DDRAM_BE,
    .DDRAM_WE, .DDRAM_RD, .DDRAM_DOUT, .DDRAM_DOUT_READY,
    .dbg_lat_v(), .dbg_lat(), .dbg_lat_src(), .dbg_lat_max(), .dbg_acc(), .dbg_acc_max(),
    .dbg_take(), .dbg_take_src(), .dbg_done(), .dbg_done_src(), .dbg_idle(), .dbg_st(), .dbg_rdy(), .dbg_rq());

  // ---- command, status, the frame sequence
  logic        m3_valid, m3_ready, m3_can_swap, m3_rbusy;
  logic        m3_swap /* verilator public_flat_rd */, m3_bank /* verilator public_flat_rd */;
  logic [1:0]  m3_reg;
  logic [19:0] m3_addr;
  logic [31:0] m3_data;
  logic        t3_valid, t3_ready, t3_vrom, t3_flush, t3_pend, t3_mark, t3_mpend, t3_go, t3_busy;
  logic [31:0] t3_data;
  logic        g_start, g_busy, f_ready, f_begin, r_busy, show_3d;
  logic        f_end /* verilator public_flat_rd */, r_done /* verilator public_flat_rd */;
  logic [1:0]  f_buf /* verilator public_flat_rd */, show_buf;
  logic [15:0] n_draw, n_drop, n_runs, n_copy, n_tex_up, n_tex_skip;
  r3d_cmd u_r3cmd (
    .clk, .rst, .h_go, .h_rdy, .h_we, .h_addr, .h_wdata, .h_rv, .h_rdata,
    .tg_reg_we, .tg_reg_addr, .tg_reg_wdata, .ln_start, .ln, .mw_we, .mw_dev, .mw_val,
    .m_valid(m3_valid), .m_ready(m3_ready), .m_reg(m3_reg), .m_addr(m3_addr), .m_data(m3_data),
    .m_swap(m3_swap), .m_can_swap(m3_can_swap), .m_rbusy(m3_rbusy),
    .t_valid(t3_valid), .t_ready(t3_ready), .t_vrom(t3_vrom), .t_data(t3_data), .t_flush(t3_flush), .t_pend(t3_pend),
    .t_mark(t3_mark), .t_mpend(t3_mpend), .t_go(t3_go), .t_busy(t3_busy),
    .g_start, .g_busy, .f_ready, .f_begin, .f_end, .f_buf, .r_done, .r_busy,
    .show_buf, .show_3d, .pingpong(r3_pingpong), .flip(r3_flip), .n_draw, .n_drop);

  // ---- culling and polygon RAM windows
  r3d_mem #(.AW(AW), .SDR_8C(SDR_8C), .SDR_8E0(SDR_8E0), .SDR_8E1(SDR_8E1), .DDR_P0(D_POLY0), .DDR_P1(D_POLY1)) u_r3mem (
    .clk, .rst, .w_valid(m3_valid), .w_ready(m3_ready), .w_reg(m3_reg), .w_addr(m3_addr), .w_data(m3_data),
    .swap(m3_swap), .r_busy(m3_rbusy), .bank(m3_bank), .can_swap(m3_can_swap),
    .s_req(p_req[4]), .s_we(p_we[4]), .s_addr(p_addr[4]), .s_din(p_din[4]), .s_be(p_be[4]), .s_ack(p_ack[4]), .s_dout(p_dout[4]),
    .d_req(d_req[4]), .d_we(d_we[4]), .d_addr(d_addr[4]), .d_blen(d_blen[4]), .d_din(d_din[4]), .d_be(d_be[4]),
    .d_wacc(d_wacc[4]), .d_rvalid(d_rvalid[4]), .d_ack(d_ack[4]), .d_dout(dm_dout),
`ifdef M3_R3B
    .s_long(p_long[4]), .s_wbc(wb_col), .s_wbd(r3_wbd), .s_wbb(r3_wbb),
`endif
    .n_runs, .n_copy);
`ifndef M3_R3B
  assign p_long[4] = 1'b0;
`endif

  // ---- textures
  // R100: Step 2's texture port (90) is three words, an address, a header and a word
  // that starts the upload; without r3d_texup's STEP2_PORT they are dropped here, so a
  // Step 1.0 decode never reads them as VROM block addresses. D-46: the port is three
  // words from Step 1.5 on (Supermodel's singleWordTexturePort is Step 1.0 only), and
  // the VROM block size follows the set's VROM (vrom64), not the port. VROM in the
  // extra memory: r3d_texup's VROM reads go to m3_xmem.
`ifdef M3_STEP2_TEX
  // R100: with r3d_texup's STEP2_PORT (sm3-builds/dt2/handoff2) the Step 2 port is uploaded, not dropped
  wire         t3_drop = 1'b0;
`else
  wire         t3_drop = step15 && t3_vrom;
`endif
  logic        tu_ready, tu_sreq, tu_sack, xt_ack;
  logic [AW:1] tu_saddr;
  logic [63:0] tu_sdout, xt_dout;
  assign t3_ready = t3_drop || tu_ready;
  assign p_req[5] = tu_sreq && !vrom_x; assign p_addr[5] = tu_saddr;
  assign tu_sack  = vrom_x ? xt_ack : p_ack[5];
  assign tu_sdout = vrom_x ? xt_dout : p_dout[5];
  r3d_texup #(.AW(AW), .SDR_VROM(SDR_VROM), .DDR_STG(D_STG), .DDR_TEX(D_TEX)
`ifdef M3_STEP2_TEX
              , .STEP2_PORT(STEP2)
`endif
              ) u_r3tex (
    .clk, .rst,
`ifdef M3_STEP2_TEX
    .port3w(step15),
`else
    .port3w(1'b0),
`endif
    .vrom64,
    .t_valid(t3_valid && !t3_drop), .t_ready(tu_ready), .t_vrom(t3_vrom), .t_data(t3_data), .t_flush(t3_flush),
    .t_pend(t3_pend), .mark(t3_mark), .t_mpend(t3_mpend), .go(t3_go), .busy(t3_busy),
    .s_req(tu_sreq), .s_addr(tu_saddr), .s_ack(tu_sack), .s_dout(tu_sdout),
    .d_req(d_req[5]), .d_we(d_we[5]), .d_addr(d_addr[5]), .d_blen(d_blen[5]), .d_din(d_din[5]), .d_be(d_be[5]),
    .d_wacc(d_wacc[5]), .d_rvalid(d_rvalid[5]), .d_ack(d_ack[5]), .d_dout(dm_dout),
    .n_tex(n_tex_up), .n_skip(n_tex_skip));
  assign p_we[5] = 1'b0; assign p_din[5] = '0; assign p_be[5] = '0; assign p_long[5] = 1'b0;

  // ---- the geometry (set-up on its pool, the LOD word: R76, R75)
  logic        geo_busy /* verilator public_flat_rd */, cr_req, cr_ack, cr_rv, pr_req, pr_ack, pr_rv;
  logic        vr_req /* verilator public_flat_rd */, vr_ack /* verilator public_flat_rd */, vr_rv /* verilator public_flat_rd */;
  logic [7:0]  geo_err;
  logic [23:0] cr_addr, pr_addr, vr_addr;
  logic [2:0]  cr_len, pr_len, vr_len /* verilator public_flat_rd */;
  logic [31:0] cr_rd, pr_rd, vr_rd;
  logic        gq_valid, gq_ready, gq_last;
  logic [31:0] gq_data;
  r3d_geo #(.SETUP_OUT(1'b1), .LOD_WORD(1'b1)) u_geo (
    .clk, .rst(rst_geo), .start(g_start), .step15, .busy(geo_busy), .err(geo_err),
    .cull_req(cr_req), .cull_addr(cr_addr), .cull_len(cr_len), .cull_ack(cr_ack), .cull_rvalid(cr_rv), .cull_rdata(cr_rd),
    .pram_req(pr_req), .pram_addr(pr_addr), .pram_len(pr_len), .pram_ack(pr_ack), .pram_rvalid(pr_rv), .pram_rdata(pr_rd),
    .vrom_req(vr_req), .vrom_addr(vr_addr), .vrom_len(vr_len), .vrom_ack(vr_ack), .vrom_rvalid(vr_rv), .vrom_rdata(vr_rd),
    .out_valid(gq_valid), .out_ready(gq_ready), .out_data(gq_data), .out_last(gq_last));
  assign g_busy = geo_busy || gq_valid;

  // culling RAM: words 0-0xFFFFF are 8C, 0x800000-0x83FFFF 8E (bank R); else zero
  logic [23:0] cb_addr;
  logic [AW:1] cb_sdr;
  logic        cb_zero;
  always_comb begin
    cb_zero = 1'b0; cb_sdr = '0;
    if (cb_addr < 24'h10_0000) cb_sdr = AW'(SDR_8C[26:1]) + AW'({cb_addr[19:0], 1'b0});
    else if (cb_addr >= 24'h80_0000 && cb_addr < 24'h84_0000)
      cb_sdr = AW'((m3_bank ? SDR_8E1 : SDR_8E0) >> 1) + AW'({cb_addr[17:0], 1'b0});
    else cb_zero = 1'b1;
  end
  m3v_sdr_rd #(.AW(AW)) u_crd (
    .clk, .rst,
    .g_req(cr_req), .g_addr(cr_addr), .g_len(cr_len), .g_ack(cr_ack), .g_rvalid(cr_rv), .g_rdata(cr_rd),
    .blk_addr(cb_addr), .blk_sdr(cb_sdr), .blk_zero(cb_zero),
    .p_req(p_req[2]), .p_addr(p_addr[2]), .p_lo(p_lo[2]), .p_ack(p_ack[2]), .p_dout(p_dout[2]), .n_req());
  assign p_we[2] = 1'b0; assign p_din[2] = '0; assign p_be[2] = '0; assign p_long[2] = 1'b1;
  logic        vs_ack, vs_rv, xg_ack, xg_rv;
  logic [31:0] vs_rd, xg_rd;
  // the geometry's VROM reader on SDRAM port 3, built only when VROM can be in the SDRAM (study R131:
  // with VROM_LOC == XMEM its request is a constant 0, but its state machine and port 3's state were
  // still fitted, about 50 ALM)
  generate if ((XMEM == 1) && (VROM_LOC != 0) && (VROM_LOC == XMEM)) begin : g_novrd
    assign vs_ack = 1'b0; assign vs_rv = 1'b0; assign vs_rd = '0;
    assign p_req[3] = 1'b0; assign p_addr[3] = '0;
  end else begin : g_vrd
    r3d_vrom_rd #(.AW(AW), .SDR_VROM(SDR_VROM)) u_vrd (
      .clk, .rst,
      .g_req(vr_req && !vrom_x), .g_addr(vr_addr), .g_len(vr_len), .g_ack(vs_ack), .g_rvalid(vs_rv), .g_rdata(vs_rd),
      .p_req(p_req[3]), .p_addr(p_addr[3]), .p_ack(p_ack[3]), .p_dout(p_dout[3]), .n_req());
  end endgenerate
  assign p_we[3] = 1'b0; assign p_din[3] = '0; assign p_be[3] = '0; assign p_long[3] = 1'b0;
  // R100: VROM in the extra memory (m3_xmem), chosen by the game configuration
  assign vr_ack = vrom_x ? xg_ack : vs_ack;
  assign vr_rv  = vrom_x ? xg_rv  : vs_rv;
  assign vr_rd  = vrom_x ? xg_rd  : vs_rd;
  generate if (XMEM == 1) begin : g_xmem
    logic [23:0] xb_addr;
    m3_xmem #(.AW(AW), .SDR_VROM(SDR_VROM), .XLIN(VROM_LIN)) u_xmem (
      .clk, .rst, .vrom64,
      .g_req(vr_req && vrom_x), .g_addr(vr_addr), .g_len(vr_len), .g_ack(xg_ack), .g_rvalid(xg_rv), .g_rdata(xg_rd),
      .t_req(tu_sreq && vrom_x), .t_addr(tu_saddr), .t_ack(xt_ack), .t_dout(xt_dout),
      .b_req(xp_req), .b_addr(xb_addr), .b_blen(xp_blen), .b_ack(xp_ack), .b_rvalid(xp_rvalid), .b_dout(dm_dout));
    assign xp_addr = D_XMEM + 25'(xb_addr);
    // the loader's writes: one 16-bit word a beat, its lanes enabled; VROM_LIN: the MRA layout's byte
    // {b, g, h, 2'b00} (b at 25:24 for a 64 MiB VROM, 24:23 for 32 MiB) to beat {g, b}
    wire [22:0] lx_lin = vrom64 ? {lx_addr[23:3], lx_addr[25:24]} : {1'b0, lx_addr[22:3], lx_addr[24:23]};
    assign d_req[6] = lx_req; assign d_we[6] = 1'b1; assign d_addr[6] = D_XMEM + (VROM_LIN ? 25'(lx_lin) : 25'(lx_addr[26:3]));
    assign d_blen[6] = 8'd1; assign d_din[6] = {4{lx_din}}; assign d_be[6] = 8'b11 << {lx_addr[2:1], 1'b0};
    assign lx_ack = d_ack[6];
  end else begin : g_noxmem
    // XMEM 2, the second SDRAM: its m2_sdram instance (the SDRAM2 pins), a read
    // port for m3_xmem's b_* (a beat a read) and a write port for lx_*, go here.
    assign xg_ack = 1'b0; assign xg_rv = 1'b0; assign xg_rd = '0; assign xt_ack = 1'b0; assign xt_dout = '0;
    assign xp_req = 1'b0; assign xp_addr = '0; assign xp_blen = 8'd1;
    assign d_req[6] = 1'b0; assign d_we[6] = 1'b0; assign d_addr[6] = '0; assign d_blen[6] = 8'd1;
    assign d_din[6] = '0; assign d_be[6] = '0;
    assign lx_ack = lx_req;
  end endgenerate
  m3v_ddr_rd u_prd (
    .clk, .rst, .base(m3_bank ? D_POLY1 : D_POLY0),
    .g_req(pr_req), .g_addr(pr_addr), .g_len(pr_len), .g_ack(pr_ack), .g_rvalid(pr_rv), .g_rdata(pr_rd),
    .m_req(d_req[3]), .m_addr(d_addr[3]), .m_blen(d_blen[3]), .m_rvalid(d_rvalid[3]), .m_ack(d_ack[3]), .m_dout(dm_dout),
    .n_req());
  assign d_we[3] = 1'b0; assign d_din[3] = '0; assign d_be[3] = 8'hFF;

  // ---- the rasteriser (nearest mip level, R79/R88); scroll fog off for now
  logic [15:0] rast_novf;               // study R136: frames whose binning overflowed the store (overlay row 2)
  r3d_rast #(.MIP(1'b1), .RUNO(25'h140_0000)) u_rast (   // runs at D_STORE + 0x1400000 = 0x1800000
    .clk, .rst(rst_rast),
    .i_valid(gq_valid), .i_ready(gq_ready), .i_data(gq_data), .i_last(gq_last),
    .f_ready, .f_binning(), .f_begin, .f_end, .fb_base(25'({f_buf, 17'd0})),
    .sf_x0(9'd0), .sf_x1(9'd0), .sf_y0(9'd0), .sf_y1(9'd0), .sf_col(24'd0), .sf_out(24'd0),
    .done(r_done), .busy(r_busy),
    .store_base(D_STORE), .tex_base(D_TEX),
    .s_req(d_req[1]), .s_we(d_we[1]), .s_addr(d_addr[1]), .s_blen(d_blen[1]), .s_din(d_din[1]), .s_be(d_be[1]),
    .s_wacc(d_wacc[1]), .s_rvalid(d_rvalid[1]), .s_ack(d_ack[1]), .s_dout(dm_dout),
    .t_req(tx_req), .t_addr(tx_addr), .t_ack(tx_ack), .t_rvalid(tx_rvalid), .t_dout(dm_dout),
    .l_req(d_req[2]), .l_addr(d_addr[2]), .l_blen(d_blen[2]), .l_din(d_din[2]), .l_wacc(d_wacc[2]), .l_ack(d_ack[2]),
    .n_pix(), .n_frames(), .f_cycles(), .f_eobw(), .st_rec(), .st_drop(), .st_run(), .st_pair(), .st_ovf(), .st_novf(rast_novf), .vp_ovf());
  assign d_we[2] = 1'b1; assign d_be[2] = 8'hFF;

  // ===========================================================================
  // video: timing, the tile generator, the overlay
  // ===========================================================================
  logic [7:0] ce_acc;   // 8 bits: ce_acc + 16 reaches CLK_MHZ + 15, past 127 above 112 MHz
  always_ff @(posedge clk) begin
    if (!mem_rst_n) begin ce_acc <= '0; ce_pix <= 1'b0; end
    else if (ce_acc + 8'd16 >= 8'(CLK_MHZ)) begin ce_acc <= ce_acc + 8'd16 - 8'(CLK_MHZ); ce_pix <= 1'b1; end
    else begin ce_acc <= ce_acc + 8'd16; ce_pix <= 1'b0; end
  end

  logic [9:0] hcnt, vcnt;
  logic       hblank, vblank, hsync, vsync, vblank_start;
  m2_video_timing u_vt (
    .clk, .ce_pix, .rst_n(mem_rst_n), .interlace(1'b0), .ntsc_lines(1'b0), .mid_lines(1'b0), .narrow(1'b0),
    .hcnt, .vcnt, .hblank, .vblank, .hsync, .vsync, .visible(), .line_start(), .line_number(),
    .vblank_start, .field(), .ypos(), .vtotal());

  // line events for the bus: the start of each line (hcnt 0)
  always_ff @(posedge clk) begin
    ln_start <= ce_pix && hcnt == 10'd655;
    ln <= (vcnt == 10'd423) ? 10'd0 : vcnt + 10'd1;
  end

  // tile generator: at the start of the horizontal blank of line L, the line
  // built during L is shown next (swap), and line L + 2 is started (go)
  logic        tg_ready, tg_busy, tg_ovr, tg_swap, tg_go, tg_go_n;
  logic [8:0]  tg_go_y;
  logic        o_v, o_drawn, o_above;
  logic [7:0]  o_r, o_g, o_b;
  logic        t_req, t_ack;
  logic [AW:1] t_addr;
  logic [63:0] t_dout;
  wire  [9:0]  l1 = (vcnt == 10'd423) ? 10'd0 : vcnt + 10'd1;
  wire  [9:0]  l2 = (vcnt >= 10'd422) ? vcnt - 10'd422 : vcnt + 10'd2;
  always_ff @(posedge clk) begin
    tg_swap <= 1'b0; tg_go <= tg_go_n; tg_go_n <= 1'b0;
    if (ce_pix && hcnt == 10'd496) begin
      if (l1 < 10'd384) tg_swap <= 1'b1;
      if (l2 < 10'd384) begin tg_go_n <= 1'b1; tg_go_y <= l2[8:0]; end
    end
  end
  logic [15:0] tg_ovr_n;
  always_ff @(posedge clk) if (rst) tg_ovr_n <= '0; else if (tg_swap && tg_busy) tg_ovr_n <= tg_ovr_n + 16'd1;

  logic       t_long, tg_ninv, tg_nw;     // R103: the name row buffer's eight-word reads and its emptying
  logic [5:0] tg_ncnt;
  m3_tilegen #(.AW(AW), .VBASE(AW'(SDR_VRAM >> 1))) u_tg (
    .clk, .rst_n(!rst), .ready(tg_ready),
    .reg_we(tg_reg_we), .reg_addr(tg_reg_addr), .reg_wdata(tg_reg_wdata),
    .pal_we(tg_pal_we), .pal_addr(tg_pal_addr), .pal_wdata(tg_pal_wdata),
    .swap(tg_swap), .go(tg_go), .go_y(tg_go_y), .busy(tg_busy), .overrun(tg_ovr),
    .sc_rd(ce_pix && !hblank && !vblank), .sc_x(hcnt[8:0]),
    .o_v, .o_drawn, .o_above, .o_r, .o_g, .o_b,
    .p_req(t_req), .p_addr(t_addr), .p_long(t_long), .p_dout(t_dout), .p_lo(p_lo[1]), .p_ack(t_ack),
    .name_inv(tg_ninv),
    .st_cycles(), .st_reqs());
  assign p_req[1] = t_req; assign p_we[1] = 1'b0; assign p_addr[1] = t_addr; assign p_din[1] = '0;
  assign p_be[1] = '0; assign p_long[1] = t_long; assign t_ack = p_ack[1]; assign t_dout = p_dout[1];
  // R103: a write to the name tables (F10F8000-F10FFFFF, either master) empties the tile generator's name row
  // buffer, at the bus's acknowledge and again 64 clocks later, when the posted write has surely reached SDRAM
  // (a refetch in between could otherwise read the old word and keep it)
  assign tg_nw = tr_v && tr_we && (tr_addr[31:15] == {12'hF10, 5'h1F});
  always_ff @(posedge clk) begin
    if (rst) tg_ncnt <= '0;
    else if (tg_nw) tg_ncnt <= 6'd63;
    else if (tg_ncnt != 6'd0) tg_ncnt <= tg_ncnt - 6'd1;
  end
  assign tg_ninv = tg_nw || (tg_ncnt == 6'd1);

  // ---- the frame count and the bench's events ----
  always_ff @(posedge clk) begin
    ev_v <= 1'b0;
    if (rst) frame <= 32'd1;           // Supermodel's numbering: frame 1 is the first
    else if (ln_start && ln == 10'd384) begin frame <= frame + 32'd1; ev_v <= 1'b1; ev_kind <= 2'd3; ev_data <= 8'd0; end
    else if (r3_flip) begin ev_v <= 1'b1; ev_kind <= 2'd2; ev_data <= 8'd0; end
  end

  // ---- the overlay (hex; rows of four words) ----
  logic [31:0] pc_v, pc_last;
  logic [15:0] msr_v;                   // (R104) the MSR's low half, and the core's ext_int, at vblank
  logic        xint_v;
  logic [15:0] n_scsi_irq;
  logic        scsi_irq_d;
  logic [31:0] cpu_wd_cnt, cpu_wd_max;  // the CPU's bus wait: cycles since its request
  logic        wd_flag;
  always_ff @(posedge clk) begin
    scsi_irq_d <= scsi_irq;
    if (rt_valid) pc_last <= rt_pc;
    if (vblank_start) begin pc_v <= pc_last; msr_v <= rt_msr[15:0]; xint_v <= ext_int; end
    if (rst) begin n_scsi_irq <= '0; cpu_wd_cnt <= '0; cpu_wd_max <= '0; wd_flag <= 1'b0; end
    else begin
      if (scsi_irq && !scsi_irq_d) n_scsi_irq <= n_scsi_irq + 16'd1;
      if (m_req[0] && !m_ack[0]) cpu_wd_cnt <= cpu_wd_cnt + 32'd1; else cpu_wd_cnt <= '0;
      if (cpu_wd_cnt > cpu_wd_max) cpu_wd_max <= cpu_wd_cnt;
      if (cpu_wd_cnt == 32'd10_000_000) wd_flag <= 1'b1;
    end
  end

  // Exceptions, for the overlay (rows 8-11).
  // An exception entry is found from the retirement stream. The core retires
  // the instruction an exception replaces as a record of its own, and that
  // record already carries the new SRR0 / SRR1 (ppc_core tr_srr0), so the next
  // record is the vector's first instruction. (Build 5 compared each record's
  // SRR0 / SRR1 with the one before and asked for the change on the vector's
  // record: the change is on the record before, so it never fired. Row 8 was
  // zero on the board for that reason.)
  // The record is marked by the core's own taken-exception events: ev_mmu
  // [3:0] (an interrupt, DSI or ISI, ITLB miss, DTLB miss, in that record) and
  // rt_irq. The other kinds (program, alignment, sc, FP unavailable, trap)
  // have no exported strobe. For those the record is the one whose SRR0 or SRR1
  // differ from the previous record's, unless it is an mtspr to either.
  // A repeated TLB miss or DSI at one PC is still counted, because it comes
  // through ev_mmu. The vector is the PC of the record that follows.
  //   first slot (rows 8, 9): the first entry after reset
  //   last slots (rows 10, 11): the most recent entry and the one before it,
  //   frozen at the boot ROM's fatal handler (FFF00000). The fatal entry is then
  //   the most recent, and the one before it is in row 10.
  //   DAR / DSISR: the last values an mfspr read, frozen with them.
  logic [31:0] ex_vec, ex_srr0, ex_srr1, ex_msr, ex_pc, ex_ins;
  logic [31:0] lc_vec, lc_srr0, lc_srr1, lc_ins;     // the most recent entry
  logic [31:0] lp_vec, lp_srr0, lp_srr1, lp_ins;     // the one before it
  logic [31:0] xn_cnt, x_dar, x_dsisr;
  logic [31:0] fx_vec, fx_srr0, pc_prev;
  logic        ex_seen, fx_seen, x_arm;
  logic [31:0] x_srr0, x_srr1, x_msr, x_pc, x_ins;   // the armed record
  logic [31:0] rt_insn_q, srr0_prev, srr1_prev;
  assign rt_insn_q = rt_insn_o;
  // mtspr SRR0 / SRR1 (spr 26, 27): the split field is insn[20:16] = spr[4:0], insn[15:11] = spr[9:5]
  wire is_mtsrr = (rt_insn_q[31:26] == 6'd31) && (rt_insn_q[10:1] == 10'd467) &&
                  (rt_insn_q[20:17] == 4'b1101) && (rt_insn_q[15:11] == 5'd0);
  // mfspr DAR (19) / DSISR (18)
  wire is_mfdx  = (rt_insn_q[31:26] == 6'd31) && (rt_insn_q[10:1] == 10'd339) &&
                  (rt_insn_q[20:17] == 4'b1001) && (rt_insn_q[15:11] == 5'd0);
  wire x_take   = (cpu_ev_mmu[3:0] != 4'd0) || (rt_irq != 2'd0) ||
                  (((rt_srr0 != srr0_prev) || (rt_srr1 != srr1_prev)) && !is_mtsrr);
  // DAR / DSISR's capture a cycle after the record (its decode was a -0.54 ns path at 100 MHz into the
  // 64 enables; study R104): the decision is made at the record, with fx_seen as it was then
  logic        xd_we, xd_dar;
  logic [31:0] xd_val;
  always_ff @(posedge clk) begin
    xd_we  <= !rst && rt_valid && is_mfdx && rt_gpr_we && !fx_seen;
    xd_dar <= rt_insn_q[16];
    xd_val <= rt_gpr_val;
    if (rst) begin x_dar <= '0; x_dsisr <= '0; end
    else if (xd_we) begin
      if (xd_dar) x_dar <= xd_val; else x_dsisr <= xd_val;
    end
  end
  always_ff @(posedge clk) begin
    if (rst) begin
      ex_seen <= 1'b0; fx_seen <= 1'b0; x_arm <= 1'b0; xn_cnt <= '0;
      ex_vec <= '0; ex_srr0 <= '0; ex_srr1 <= '0; ex_msr <= '0; ex_pc <= '0; ex_ins <= '0;
      lc_vec <= '0; lc_srr0 <= '0; lc_srr1 <= '0; lc_ins <= '0;
      lp_vec <= '0; lp_srr0 <= '0; lp_srr1 <= '0; lp_ins <= '0;
      fx_vec <= '0; fx_srr0 <= '0;
      srr0_prev <= '0; srr1_prev <= '0;
    end else if (rt_valid) begin
      pc_prev <= rt_pc; srr0_prev <= rt_srr0; srr1_prev <= rt_srr1;
      x_arm <= x_take;
      if (x_take) begin
        x_srr0 <= rt_srr0; x_srr1 <= rt_srr1; x_msr <= rt_msr; x_pc <= rt_pc; x_ins <= rt_insn_q;
      end
      if (x_arm) begin                                // this record is the vector's first
        if (!ex_seen) begin
          ex_seen <= 1'b1; ex_vec <= rt_pc; ex_srr0 <= x_srr0; ex_srr1 <= x_srr1;
          ex_msr <= x_msr; ex_pc <= x_pc; ex_ins <= x_ins;
        end
        if (!fx_seen) begin
          xn_cnt <= xn_cnt + 32'd1;
          lp_vec <= lc_vec; lp_srr0 <= lc_srr0; lp_srr1 <= lc_srr1; lp_ins <= lc_ins;
          lc_vec <= rt_pc;  lc_srr0 <= x_srr0;  lc_srr1 <= x_srr1;  lc_ins <= x_ins;
        end
      end
      if (rt_pc == 32'hFFF0_0000 && !fx_seen && rom_loaded) begin
        fx_seen <= 1'b1; fx_vec <= pc_prev; fx_srr0 <= rt_srr0;
      end
`ifdef M3_XLOG
      if (x_arm && !fx_seen)
        $display("xlog n=%0d vec=%08x pc=%08x ins=%08x srr0=%08x srr1=%08x msr=%08x mmu=%x",
                 xn_cnt + 32'd1, rt_pc, x_pc, x_ins, x_srr0, x_srr1, x_msr, cpu_ev_mmu);
`endif
    end
  end

  logic [9:0]  x_d, y_d;
  logic        hb_d, vb_d, hs_d, vs_d;
  always_ff @(posedge clk) if (ce_pix) begin
    x_d <= hcnt; y_d <= vcnt; hb_d <= hblank; vb_d <= vblank; hs_d <= hsync; vs_d <= vsync;
  end

  // the tile pixel and the 3D layer (scan-out on DDR3 port 0)
  logic [23:0] px;
  logic [15:0] fb_late;
  m3_compose u_comp (
    .clk, .rst_n(mem_rst_n), .ddr_ok, .hcnt, .vcnt, .sc_rd(ce_pix && !hblank && !vblank),
    .show_buf, .show_3d, .o_v, .o_drawn, .o_above, .o_r, .o_g, .o_b, .px,
    .m_req(d_req[0]), .m_addr(d_addr[0]), .m_blen(d_blen[0]), .m_rvalid(d_rvalid[0]), .m_ack(d_ack[0]), .m_dout(dm_dout),
    .n_late(fb_late));
  assign d_we[0] = 1'b0; assign d_din[0] = '0; assign d_be[0] = 8'hFF;

  // the CPU's busy time per frame (R104): vblank to the first retirement in a wait loop
  logic [31:0] fb_mm, fb_lo, fb_hi, fb_last;
  generate if (FB_ON) begin : g_fbusy
    m3_frame_busy #(.CLK_HZ(CLK_HZ)) u_fbusy (.clk, .rst, .vblank_start, .step2, .rt_valid, .rt_pc,
      .o_minmax(fb_mm), .o_bins_lo(fb_lo), .o_bins_hi(fb_hi), .o_last(fb_last));
  end else begin : g_nofbusy
    assign fb_mm = '0; assign fb_lo = '0; assign fb_hi = '0; assign fb_last = '0;
  end endgenerate

  logic [5:0]  ov_idx;
  logic [31:0] ov_v;
  logic        ov_box, ov_ink;
  generate if (OV_ON) begin : g_ov
    logic [31:0] tgr20, tgr60, tgr64, tgr68, tgr6c, tgr08;
    always_ff @(posedge clk) if (tg_reg_we) begin
      case (tg_reg_addr)
        6'h02: tgr08 <= tg_reg_wdata;
        6'h08: tgr20 <= tg_reg_wdata;
        6'h18: tgr60 <= tg_reg_wdata;
        6'h19: tgr64 <= tg_reg_wdata;
        6'h1A: tgr68 <= tg_reg_wdata;
        6'h1B: tgr6c <= tg_reg_wdata;
        default: ;
      endcase
    end
    always_ff @(posedge clk) begin
      case (ov_idx)
        // row 0: CPU PC (at vblank), IRQ state/enable and CROM bank, frames, the 53C810 interrupts
        6'd0:  ov_v <= pc_v;
        6'd1:  ov_v <= {irq_state, irq_enable, crom_bank, 7'd0, pingpong};
        6'd2:  ov_v <= frame;
        6'd3:  ov_v <= {n_scsi_irq, cfg_step, cfg_flags[6:0], sec_cmp};   // R100: the step and flags
        // row 1: Real3D: frames submitted; frames drawn and dropped; the geometry's
        // error bits, layer lines late, textures uploaded; runs deferred, copies
        6'd4:  ov_v <= {8'h88, n_submit};
        6'd5:  ov_v <= {n_draw, n_drop};
        6'd6:  ov_v <= {geo_err, fb_late[7:0], n_tex_up};
        6'd7:  ov_v <= {n_runs, n_copy};
        // row 2: texture FIFO, VROM port, unmapped accesses (count) and store overflows (frames), last unmapped address
        6'd8:  ov_v <= {8'h94, n_tex};
        6'd9:  ov_v <= {8'h90, n_vrom};
        6'd10: ov_v <= {n_unmapped, rast_novf};   // study R136: frames the store overflowed
        6'd11: ov_v <= last_unmapped;
        // row 3: tile registers 20, 60, 64, 68
        6'd12: ov_v <= tgr20;
        6'd13: ov_v <= tgr60;
        6'd14: ov_v <= tgr64;
        6'd15: ov_v <= tgr68;
        // row 4: tile register 6C, 08; overruns; the I/O port A
        6'd16: ov_v <= tgr6c;
        6'd17: ov_v <= tgr08;
        6'd18: ov_v <= {tg_ovr_n, 8'd0, io_pa};
        6'd19: ov_v <= {r3d_cfg0[15:0], r3d_cfg2[15:0]};
        // row 5: SDRAM and loader: bytes loaded; flags; the CPU's longest bus wait
        6'd20: ov_v <= {5'd0, ld_bytes};
        6'd21: ov_v <= {3'd0, mem_ready, 3'd0, rom_loaded, 3'd0, ld_ovf, 3'd0, wd_flag, 16'd0};
        6'd22: ov_v <= cpu_wd_max;
        6'd23: ov_v <= {16'd0, cpi_q8};             // CPI x 256, the last second
        // row 6: CPU performance, the last second: cycles, instructions, data memory wait, fetch waiting
        6'd24: ov_v <= perf[0];
        6'd25: ov_v <= perf[1];
        6'd26: ov_v <= perf[6];
        6'd27: ov_v <= perf[8];
        // row 7: redirects, load-use, mul/div wait, decode starved
        // (with FBUSY, R104: the CPU's busy time a frame, the last second: {min, max} and the last frame in
        // units of 256 clocks; frames by vblanks needed {<0.9, 0.9-1, 1-1.1, 1.1-1.25}, {1.25-1.5, >=1.5, 0, frames})
        6'd28: ov_v <= FB_ON ? fb_mm : perf[7];
        6'd29: ov_v <= FB_ON ? fb_lo : perf[4];
        6'd30: ov_v <= FB_ON ? fb_hi : perf[5];
        6'd31: ov_v <= FB_ON ? {msr_v, xint_v, fb_last[14:0]} : perf[3];   // {MSR[15:0] at vblank, ext_int, the last frame}
        // row 8: the first exception entry: vector, SRR0, SRR1, MSR after entry
        6'd32: ov_v <= XC_ON ? ex_vec : 32'd0;
        6'd33: ov_v <= XC_ON ? ex_srr0 : 32'd0;
        6'd34: ov_v <= XC_ON ? ex_srr1 : 32'd0;
        6'd35: ov_v <= XC_ON ? ex_msr : 32'd0;
        // row 9: its record's PC and instruction; the boot ROM's fatal handler:
        // the PC before FFF00000 (the vector it came from), SRR0 then
        6'd36: ov_v <= XC_ON ? ex_pc : 32'd0;
        6'd37: ov_v <= XC_ON ? ex_ins : 32'd0;
        6'd38: ov_v <= XC_ON ? fx_vec : 32'd0;
        6'd39: ov_v <= XC_ON ? fx_srr0 : 32'd0;
        // row 10: the entry before the most recent one (frozen at the fatal
        // handler: the last entry before the fatal one): vector, SRR0, SRR1, instruction
        6'd40: ov_v <= XC_ON ? lp_vec : 32'd0;
        6'd41: ov_v <= XC_ON ? lp_srr0 : 32'd0;
        6'd42: ov_v <= XC_ON ? lp_srr1 : 32'd0;
        6'd43: ov_v <= XC_ON ? lp_ins : 32'd0;
        // row 11: entries counted; the last DAR and DSISR read by mfspr; the most recent entry's vector
        6'd44: ov_v <= XC_ON ? xn_cnt : 32'd0;
        6'd45: ov_v <= XC_ON ? x_dar : 32'd0;
        6'd46: ov_v <= XC_ON ? x_dsisr : 32'd0;
        6'd47: ov_v <= XC_ON ? lc_vec : 32'd0;
        default: ov_v <= 32'h3B00_0001;   // stage B, build 1
      endcase
    end
    m3v_overlay #(.NR(12)) u_ov (.clk, .en(overlay_en), .x(x_d), .y(y_d), .idx(ov_idx), .val(ov_v), .box(ov_box), .ink(ov_ink));
  end else begin : g_noov
    assign ov_box = 1'b0; assign ov_ink = 1'b0; assign ov_idx = '0; assign ov_v = '0;
  end endgenerate

  // the frame counter: Real3D frames begun a second, n_draw's low bit toggling once a frame
  logic fps_box, fps_ink;
  generate if (FPS) begin : g_fps
    logic nd0_q;
    always_ff @(posedge clk) nd0_q <= n_draw[0];
    m3_fps #(.CLK_HZ(CLK_HZ), .EXT_SEC(BR_SEC)) u_fps (.clk, .rst, .sec_tick, .en(fps_en), .avg(fps_avg), .draw(n_draw[0] ^ nd0_q), .x(x_d), .y(y_d),
                                     .box(fps_box), .ink(fps_ink));
  end else begin : g_nofps
    assign fps_box = 1'b0; assign fps_ink = 1'b0;
    wire _unused_avg = &{1'b0, fps_avg};
  end endgenerate

  always_ff @(posedge clk) if (ce_pix) begin
    logic [23:0] c;
    c = px;
    if (ov_box) c = ov_ink ? 24'hffffff : {1'b0, c[23:17], 1'b0, c[15:9], 1'b0, c[7:1]} & 24'h3f3f3f;
    if (fps_box) c = fps_ink ? 24'hffffff : {1'b0, c[23:17], 1'b0, c[15:9], 1'b0, c[7:1]} & 24'h3f3f3f;
    {v_r, v_g, v_b} <= (hb_d || vb_d) ? 24'd0 : c;
    v_hs <= hs_d; v_vs <= vs_d; v_hblank <= hb_d; v_vblank <= vb_d;
  end

  assign led = !rst;

  /* verilator lint_off UNUSEDSIGNAL */
  wire _unused = &{1'b0, tg_ready, tg_ovr, s_idle, scsi_busy, r3d_cfg1, x_d[9], y_d[9], n_cull_lo, n_cull_hi, n_poly,
                    n_tex_skip, p_lo[5:3], p_lo[1:0], d_wacc[3], d_wacc[0], d_rvalid[2], d_rvalid[5], fb_late[15:8],
                    d_wacc[6], d_rvalid[6], cfg_flags[7], crom_map, fixed8, sec_key};
  /* verilator lint_on UNUSEDSIGNAL */

endmodule
