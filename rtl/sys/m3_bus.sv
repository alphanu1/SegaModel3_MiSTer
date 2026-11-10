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
// The Step 1.0 system bus (study 4, 5.2, D-37 stage A): the PowerPC and the
// 53C810's DMA as masters, the MPC105's address map behind them. Both masters
// speak the CPU caches' protocol (ppc_cache_pkg.sv); one transaction at a
// time, granted in turn when both ask, owned by its master to the end.
//
// Map (VF3's, Supermodel and MAME agree unless noted; sim/system/notes.md):
//   00000000-007FFFFF  RAM, 8 MiB                     SDRAM, word form
//   84000000-8400003F  Real3D status                  r3d_cmd (the host port, below)
//   88000000           Real3D command port            r3d_cmd
//   8C/8E/90/94/98     Real3D windows                 r3d_cmd (writes counted here)
//   9C                 Real3D buffer configuration    9C0000x0-08 kept here, with the JTAG
//   C0000000-C00000FF  53C810 registers (BAR1)        m3_scripts, at the game's BAR
//   F0040000-F004003F  315-5649                       m3_io (byte n at 4n)
//   F0080000-F008FFFF  MIDI port (i8251)              stub, below
//   F00C0000-F00DFFFF  backup RAM, 128 KiB            SDRAM, word form
//   F0100000-F010003F  system registers               bank, JTAG, IRQ
//   F0140000-F014003F  RTC 72421                      stub: 0x00030000
//   F0800CF8 / F0C00CF8-CFF  PCI configuration address / data (MPC105 CONFIG_ADDR / _DATA)
//   F1000000-F10FFFFF  tile pattern RAM and names     SDRAM, byte form
//   F1100000-F111FFFF  palette                        m3_tilegen (writes)
//   F1180000-F11800FF  tile generator registers       m3_tilegen, and a shadow
//   F8FFF000-F8FFF0FF  MPC105 registers               writes taken, ID read
//   FF000000-FF7FFFFF  banked CROM (bank = ~F0100008 & 15)  SDRAM
//   FF800000-FFFFFFFF  fixed CROM: the low 6 MiB is CROM0 (MAME ROM_COPY), the top 2 MiB the fixed ROMs
// F0xxxxxx I/O mirrors every 2 MiB up to FExxxxxx (MAME mirror 0x0E000000).
//
// Step 2.x (STEP2, selected at run time by step2; study R100, Daytona 2):
//   FEC00000-FEDFFFFF  MPC106 CONFIG_ADDR; FEE00000-FEEFFFFF CONFIG_DATA (Supermodel's
//                      decode; the boot ROM uses FEC00CF8 / FEE00CFC); the bridge's ID
//                      0x00021057 (MPC106), device 13 (the Real3D) 0x178611DB
//   F0180000-F01BFFFF  the security board (315-5881): RAM 18-19, registers 1A (sec_*)
//   FF000000-FF7FFFFF  banked CROM: the slot crom_slot (from the game's bank map; the
//                      caller decodes crom_bank_o), empty banks (crom_zero) read zero
//   FF800000-FFFFFFFF  with fixed8, all 8 MiB fixed CROM at SDR_FIX8
//   C2000000-C20000FF  the Real3D DMA device: m3_scripts' window (win_base C20000)
//   The MPC106's own configuration registers (device 0) are kept: the boot ROM reads
//   back MCCR1 (F0) and PICR1 (A8) after writing them. Not yet written, a register reads
//   Supermodel's reset value (CMPC10x::Reset). As Supermodel: a byte read of device 0
//   reads FF (it has no 8-bit handler there), and device 16 reads 0x182711DB as a
//   word and 0 in halfwords (CModel3::ReadPCIConfigSpace, "used by Daytona 2").
//   The RTC 72421 runs: Daytona 2 waits for its seconds to change at boot (Supermodel with
//   the clock frozen stays on the system program's screen; with the time running and the
//   date fixed at 2000-01-01, a Saturday, it reaches the attract: study R100). Seconds,
//   minutes and hours count from 00:00:00 at RTC_HZ clocks a second; the date stays.
//   Registers D-F read 0 and writes are ignored, as Supermodel's. Step 1.0 keeps the
//   constant it had (VF3 reads only D and E).
// Anything else reads FFFFFFFF (Supermodel) and is counted.
//
// Interrupts (sim/system/notes.md): IRQ 0x02 at the start of line 383 and
// 0x0D (0x08 | 0x04 | 0x01) at the end of vertical blanking, as Supermodel
// raises them; MAME raises 0x02 at line 384 and 0x0C every line. Cleared by
// the byte written to F1180010 (both emulators) or to F0100018 (Supermodel;
// MAME only re-evaluates). The 53C810's interrupt is not maskable (R80).
// The CPU's external interrupt is (state & enable) != 0, or the 53C810's.
//
// MIDI (no sound board, D-8): a write to F0080004 sets the control byte;
// while its bit 5 (RTS) is set, IRQ 0x40 is raised whenever the transmitter
// is free: MIDI_GAP clocks (a byte at 31.25 kbaud, 320 us) after the last
// data write. A data write clears it. This is the i8251's TxRDY as the game
// uses it; Supermodel raises 0x40 in blanking only, MAME on a 1 ms timer.
//
// Real3D (D-37 stage B): the status reads and the window writes other than 9C
// go to r3d_cmd's host port, one word a strobe: h_go only in a cycle with
// h_rdy (combinational, so a flip that closes the port cannot lose a word);
// writes are posted there, a read answers on h_rv. The ping-pong bit and its
// flip line are r3d_cmd's (pingpong_i).
//
// Snoop (D-41, study R90/R101): the MPC105 shows a PCI master's cycle to the
// 603e. Every 53C810 access to work RAM is snooped first, one per access (a
// line, or the line of a single word): sn_req with the line and sn_inv (a
// write) is held to sn_ack, and the access is not granted before it. The CPU
// keeps the bus meanwhile, so the D-cache's push (a cast-out) goes through.
// After sn_ack the CPU is not granted until the 53C810's access is, so the
// write that invalidated a line lands before the CPU can fill it again, and
// a read follows the push. ROM, VRAM and the I/O space are not snooped (the
// game maps them cache-inhibited). The I-cache is not snooped (603e 3.1).
//
// Bench: every access outside RAM and CROM reads is offered on the tr_*
// port, for the bus trace (sim/system/bustrace.py).

module m3_bus
  import ppc_cache_pkg::*;
#(
  parameter int          AW         = 26,
  parameter logic [26:0] SDR_CROM   = 27'h000_0000,   // banked CROM, 8 MiB a bank
  parameter logic [26:0] SDR_FIXED  = 27'h600_0000,   // the 2 MiB of fixed CROM
  parameter logic [26:0] SDR_BACKUP = 27'h620_0000,
  parameter logic [26:0] SDR_RAM    = 27'h640_0000,
  parameter logic [26:0] SDR_VRAM   = 27'h6C0_0000,
  parameter logic [4:0]  CROM_BANKS = 5'd8,           // VF3: 64 MiB
  parameter logic [31:0] R3D_ID     = 32'h16C3_11DB,  // 315-5827 (Step 1.x)
  parameter int          MIDI_GAP   = 32000,
  parameter bit          STEP2      = 1'b0,           // the Step 2.x map (R100)
  parameter logic [26:0] SDR_FIX8   = 27'h730_0000,   // a Step 2 set's 8 MiB fixed CROM
  parameter int          RTC_HZ     = 100_000_000,    // clocks a second, for the RTC
  parameter bit          FAST       = 1'b0,           // the CPU's work-RAM reads to the SDRAM port from B_IDLE, past B_DEC (R112)
  parameter bit          R3P        = 1'b0,           // writes to the Real3D's host port posted in an eight-word buffer (R112)
  parameter bit          TGT        = 1'b0,           // the tile generator's bus timing (R152)
  parameter int          TG_MHZ     = 100,            // the clock, for TGT's 32 MHz VRAM clock
  // The bridges pass (all zero-cycle; 1 / 0 as before):
  //   C106_M10K the MPC106's 64 configuration registers in one M10K, read registered at pci_reg every clock (cq_q,
  //             as before), each holding its reset value until written: a sweep writes the reset values from each
  //             reset's start (64 clocks; any access waits in B_DEC while it runs, which the core's resets, all far
  //             longer than 64 clocks, never make it do), so no written flags. 0: the MLAB and flags
  //   EXT_SEC   the RTC's second from sec_tick (m3_system's one counter, m3_fps's too), not a prescaler of its own;
  //             sec_tick is high in the clock rtc_pre would have read RTC_HZ - 1
  //   OWIRE     the targets' fields (sc_*, sec_*, cf_* but the requests; tg_reg_wdata, tg_pal_wdata) wired from the
  //             transaction's registers, which hold while each is read, not copied into registers of their own
  //   WD_DEC    wd taken in B_DEC from the owner's m_wdata (who), the select s_wdata already has, not in B_IDLE
  //             through the grant's (the master holds its word until m_ack)
  parameter bit          C106_M10K  = 1'b0,
  parameter bit          EXT_SEC    = 1'b0,
  parameter bit          OWIRE      = 1'b0,
  parameter bit          WD_DEC     = 1'b0
) (
  input  logic          clk,
  input  logic          rst,
  input  logic          sec_tick,       // EXT_SEC: one clock a second (STEP2's RTC)

  // Step 2.x (STEP2), run time
  input  logic              step2,
  input  logic              fixed8,
  input  logic [3:0]        crom_slot,      // the bank register's slot (combinational from crom_bank_o)
  input  logic              crom_zero,
  output logic              sec_req,        // the security board: one access per request, to sec_ack
  output logic              sec_we,
  output logic [17:2]       sec_addr,
  output logic [3:0]        sec_be,
  output logic [31:0]       sec_wdata,
  input  logic              sec_ack,
  input  logic [31:0]       sec_rdata,

  // master 0: the CPU; master 1: the 53C810
  input  logic [1:0]        m_req,
  input  logic [1:0][1:0]   m_cmd,
  input  logic [1:0][31:0]  m_addr,
  input  logic [1:0][3:0]   m_be,
  input  logic [1:0][31:0]  m_wdata,
  output logic [1:0]        m_ack,
  output logic [1:0]        m_wnext,
  output logic [1:0]        m_rvalid,
  output logic [31:0]       m_rdata,

  // the CPU's D-cache snoop (ppc_cpu sn_*), for master 1's RAM accesses
  output logic              sn_req,
  output logic [31:5]       sn_addr,
  output logic              sn_inv,
  input  logic              sn_ack,

  // SDRAM (m3_sdr_port)
  output logic              s_req,
  output logic [1:0]        s_cmd,
  output logic [AW:0]       s_sa,
  output logic              s_swap,
  output logic              s_zero,
  output logic [3:0]        s_be,
  output logic [31:0]       s_wdata,
  input  logic              s_ack,
  input  logic              s_wnext,
  input  logic              s_rvalid,
  input  logic [31:0]       s_rdata,

  // 53C810: register window and PCI configuration (m3_scripts)
  output logic              sc_req,
  output logic              sc_we,
  output logic [6:2]        sc_addr,
  output logic [3:0]        sc_be,
  output logic [31:0]       sc_wdata,
  input  logic              sc_ack,
  input  logic [31:0]       sc_rdata,
  input  logic              sc_win_en,
  input  logic [31:8]       sc_win_base,
  output logic              cf_req,
  output logic              cf_we,
  output logic [5:0]        cf_idx,
  output logic [3:0]        cf_be,
  output logic [31:0]       cf_wdata,
  input  logic              cf_ack,
  input  logic [31:0]       cf_rdata,
  input  logic              scsi_irq,

  // 315-5649 (m3_io)
  output logic              io_sel,
  output logic              io_we,
  output logic [3:0]        io_off,
  output logic [7:0]        io_wbyte,
  input  logic [7:0]        io_rbyte,

  // tile generator
  output logic              tg_reg_we,
  output logic [5:0]        tg_reg_addr,
  output logic [31:0]       tg_reg_wdata,     // little-endian value (the bus word byte-reversed)
  output logic              tg_pal_we,
  output logic [14:0]       tg_pal_addr,
  output logic [15:0]       tg_pal_wdata,

  // video: a pulse at the start of each line, and the line (0-383 shown, 384-423 blanking)
  input  logic              ln_start,
  input  logic [9:0]        ln,

  output logic              ext_int,

  // telemetry (the overlay)
  output logic [7:0]        irq_state_o,
  output logic [7:0]        irq_enable_o,
  output logic [7:0]        crom_bank_o,
  output logic [23:0]       n_submit,         // Real3D command port writes (frames submitted)
  output logic [23:0]       n_cull_lo, n_cull_hi, n_poly, n_tex, n_vrom,   // words written to 8C, 8E, 98, 94, 90
  output logic [15:0]       n_unmapped,
  output logic [31:0]       last_unmapped,
  output logic              pingpong_o,
  input  logic              pingpong_i,

  // the Real3D's host port (r3d_cmd)
  output logic              h_go,
  input  logic              h_rdy,
  output logic              h_we,
  output logic [31:0]       h_addr,
  output logic [31:0]       h_wdata,          // little-endian value (the bus word byte-reversed)
  input  logic              h_rv,
  input  logic [31:0]       h_rdata,
  output logic [18:0]       r3d_cfg0, r3d_cfg1, r3d_cfg2,   // 9C000000/4/8 (JTAG's Mercury, stage B)

  // JTAG (m3_jtag)
  output logic              jt_wr,
  output logic [7:0]        jt_wdata,
  input  logic              jt_busy,
  input  logic              jt_tdo,
  input  logic              jt_cfg_we,
  input  logic [18:0]       jt_cfg0, jt_cfg1, jt_cfg2,

  // bench trace: every access outside RAM and CROM reads
  output logic              tr_v,
  output logic              tr_master,
  output logic              tr_we,
  output logic [31:0]       tr_addr,
  output logic [3:0]        tr_be,
  output logic [31:0]       tr_data
);

  function automatic logic [31:0] bswap(input logic [31:0] v);
    bswap = {v[7:0], v[15:8], v[23:16], v[31:24]};
  endfunction

  // ===========================================================================
  // arbitration and the transaction
  // ===========================================================================
  typedef enum logic [3:0] {
    B_IDLE, B_DEC, B_SDR, B_INT, B_INTW, B_SCSI, B_CFG, B_JTAG, B_BEAT, B_R3D, B_R3DR, B_SEC, B_R3P
  } bst_t;
  bst_t        bs;
  logic        who;               // the owning master
  logic        last;              // the master granted last (turns)
  logic [1:0]  cmd;
  logic [31:0] addr;
  logic [3:0]  be;
  logic [31:0] wd;
  logic [2:0]  beat;              // line transactions to internal targets: beats done
  logic [2:0]  crit;

  // target classes, decoded from the registered address in B_DEC
  typedef enum logic [3:0] {
    X_NONE, X_SDR, X_ROM_W, X_R3DH, X_R3D, X_SCSI, X_IO, X_MIDI, X_SYS, X_RTC,
    X_PCIA, X_PCID, X_PAL, X_TGREG, X_MPC, X_SEC
  } tgt_t;
  tgt_t        tgt;
  logic [AW:0] dsa;
  logic        dswap, dzero;

  // ---- the decode ----
  logic [7:0]  crom_bank;         // F0100008 as written (the bank number is its low nibble, inverted)
  logic [3:0]  bank_n;
  assign bank_n = ~crom_bank[3:0];
  wire  [7:0]  a_hi = addr[31:24];
  wire         io_win = (a_hi[7:4] == 4'hF) && !a_hi[0];       // F0, F2 ... FE (MPC105's F8FFF0xx decoded first)
  wire s2 = STEP2 && step2;
  always_comb begin
    tgt = X_NONE; dsa = '0; dswap = 1'b0; dzero = 1'b0;
    if (addr[31:23] == 9'd0) begin
      tgt = X_SDR; dsa = SDR_RAM + {4'd0, addr[22:0]};
    end else if (a_hi == 8'hFF) begin
      tgt = cmd[0] ? X_ROM_W : X_SDR;
      if (!addr[23]) begin
        if (STEP2) begin
          dsa = SDR_CROM + {crom_slot, addr[22:0]};
          dzero = crom_zero;
        end else begin
          dsa = SDR_CROM + {bank_n[2:0], addr[22:0]};
          dzero = ({1'b0, bank_n} >= CROM_BANKS);
        end
      end else if (STEP2 && fixed8) dsa = SDR_FIX8 + {4'd0, addr[22:0]};            // a Step 2 set's 8 MiB
      else if (addr[22:21] != 2'b11) dsa = SDR_CROM + {3'd0, addr[22:0]};       // CROM0's first 6 MiB
      else dsa = SDR_FIXED + {6'd0, addr[20:0]};
    end else if (addr[31:6] == 26'h210_0000) tgt = X_R3DH;                           // 84000000-3F
    else if (a_hi == 8'h88 || a_hi == 8'h8C || a_hi == 8'h8E || a_hi == 8'h90 || a_hi == 8'h94 ||
             a_hi == 8'h98) tgt = X_R3DH;
    else if (a_hi == 8'h9C) tgt = X_R3D;
    else if (sc_win_en && addr[31:8] == sc_win_base) tgt = X_SCSI;
    else if (a_hi == 8'hF1) begin
      if (addr[23:20] == 4'h0) begin tgt = X_SDR; dsa = SDR_VRAM + {7'd0, addr[19:0]}; dswap = 1'b1; end
      else if (addr[23:17] == 7'b0001000) tgt = X_PAL;                              // F1100000-F111FFFF
      else if (addr[23:8] == 16'h1800) tgt = X_TGREG;                               // F1180000-FF
    end else if (a_hi == 8'hF8 && addr[23:8] == 16'hFFF0) tgt = X_MPC;
    else if (io_win) begin
      if (addr[23:6] == 18'h0_1000) tgt = X_IO;                                     // xx040000-3F
      else if (addr[23:16] == 8'h08) tgt = X_MIDI;
      else if (addr[23:17] == 7'b0000110) begin                                     // xx0C0000-xx0DFFFF
        tgt = X_SDR; dsa = SDR_BACKUP + {10'd0, addr[16:0]};
      end else if (addr[23:6] == 18'h0_4000) tgt = X_SYS;                           // xx100000-3F
      else if (addr[23:6] == 18'h0_5000) tgt = X_RTC;                               // xx140000-3F
      else if (a_hi == 8'hF0 && addr[23:3] == 21'h10_019F) tgt = X_PCIA;            // F0800CF8-CFF
      else if (a_hi == 8'hF0 && addr[23:3] == 21'h18_019F) tgt = X_PCID;            // F0C00CF8-CFF
      else if (s2 && addr[23:18] == 6'b0001_10) tgt = X_SEC;                         // xx180000-xx1BFFFF
      else if (s2 && a_hi == 8'hFE && addr[23:21] == 3'b110) tgt = X_PCIA;           // FEC00000-FEDFFFFF
      else if (s2 && a_hi == 8'hFE && addr[23:20] == 4'hE) tgt = X_PCID;             // FEE00000-FEEFFFFF
    end
  end

  // ===========================================================================
  // registers
  // ===========================================================================
  logic [7:0]  irq_state, irq_enable;
  logic [31:0] pci_addr;          // CONFIG_ADDR, as the PCI side sees it (little-endian value)
  logic [7:0]  midi_ctrl;
  logic [15:0] midi_cnt;
  logic [7:0]  irq_set, irq_clr;
  (* ramstyle = "M10K, no_rw_check" *) logic [31:0] tg_shadow [0:63];   // M10K (area pass): read registered, as tg_sh_q
  logic [31:0] tg_sh_q;
  // power-up contents: zero, as Supermodel's registers read before any write
  // (VF3 reads F1180000 once, at boot, and gets 0)
  initial for (int i = 0; i < 64; i++) tg_shadow[i] = 32'd0;

  // Step 2: the RTC's time (BCD), the date fixed
  logic [26:0] rtc_pre;
  logic [3:0]  rtc_s1, rtc_m1, rtc_h1;
  logic [2:0]  rtc_s10, rtc_m10;
  logic [1:0]  rtc_h10;
  logic [3:0]  rtc_d;
  wire rtc_wrap = EXT_SEC ? sec_tick : (rtc_pre == 27'(RTC_HZ - 1));
  always_ff @(posedge clk) begin
    if (!STEP2 || rst) begin
      rtc_pre <= '0; rtc_s1 <= '0; rtc_s10 <= '0; rtc_m1 <= '0; rtc_m10 <= '0; rtc_h1 <= '0; rtc_h10 <= '0;
    end else if (!rtc_wrap) begin
      if (!EXT_SEC) rtc_pre <= rtc_pre + 27'd1;
    end else begin
      rtc_pre <= '0;
      if (rtc_s1 != 4'd9) rtc_s1 <= rtc_s1 + 4'd1;
      else begin
        rtc_s1 <= '0;
        if (rtc_s10 != 3'd5) rtc_s10 <= rtc_s10 + 3'd1;
        else begin
          rtc_s10 <= '0;
          if (rtc_m1 != 4'd9) rtc_m1 <= rtc_m1 + 4'd1;
          else begin
            rtc_m1 <= '0;
            if (rtc_m10 != 3'd5) rtc_m10 <= rtc_m10 + 3'd1;
            else begin
              rtc_m10 <= '0;
              if (rtc_h10 == 2'd2 && rtc_h1 == 4'd3) begin rtc_h10 <= '0; rtc_h1 <= '0; end
              else if (rtc_h1 != 4'd9) rtc_h1 <= rtc_h1 + 4'd1;
              else begin rtc_h1 <= '0; rtc_h10 <= rtc_h10 + 2'd1; end
            end
          end
        end
      end
    end
  end
  always_comb begin
    case (addr[5:2])
      4'h0: rtc_d = rtc_s1;           4'h1: rtc_d = {1'b0, rtc_s10};
      4'h2: rtc_d = rtc_m1;           4'h3: rtc_d = {1'b0, rtc_m10};
      4'h4: rtc_d = rtc_h1;           4'h5: rtc_d = {2'b00, rtc_h10};
      4'h6: rtc_d = 4'd1;             4'h8: rtc_d = 4'd1;              // the 1st of January
      4'hC: rtc_d = 4'd6;                                              // 2000, a Saturday
      default: rtc_d = 4'd0;
    endcase
  end

  wire [4:0]  pci_dev = pci_addr[15:11];
  wire [5:0]  pci_reg = pci_addr[7:2];

  // Step 2: the MPC106's configuration registers (device 0), CPU byte order
  (* ramstyle = "MLAB, no_rw_check" *) logic [31:0] c106 [0:63];
  logic [63:0] c106_w;               // written since reset
  function automatic logic [31:0] def106(input logic [5:0] r);
    case (r)                         // CMPC10x::Reset, MPC106 (the little-endian values byte-reversed)
      6'h01: def106 = bswap(32'h0080_0006);
      6'h02: def106 = bswap(32'h0006_0000);
      6'h03: def106 = bswap(32'h0000_0800);
      6'h1C: def106 = bswap(32'h00CD_0000);
      6'h2A: def106 = bswap(32'h0010_FF00);
      6'h2B: def106 = bswap(32'h060C_000C);
      6'h2E: def106 = bswap(32'h0400_0000);
      6'h30: def106 = bswap(32'h0000_0100);
      6'h38: def106 = bswap(32'h0042_0FFF);
      6'h3A: def106 = bswap(32'h0020_0000);
      6'h3C: def106 = bswap(32'h0000_FF02);
      6'h3D: def106 = bswap(32'h0003_0000);
      6'h3F: def106 = bswap(32'h0000_0010);
      default: def106 = 32'd0;
    endcase
  endfunction
  wire [31:0] c106_def = def106(pci_reg);
  wire [31:0] cq = c106_w[pci_reg] ? c106[pci_reg] : c106_def;     // (the MLAB form)
  // C106_M10K: the file in one M10K, always the written value or the reset value (the sweep, below)
  (* ramstyle = "M10K, no_rw_check" *) logic [31:0] c106m [0:63];
  logic [31:0] c106m_q;
  always_ff @(posedge clk) c106m_q <= c106m[pci_reg];
  // The internal targets' reads (B_INT) and writes (do_w) take the target and the MPC106 register registered:
  // both are used a cycle or more after addr, pci_addr and the registers settled (B_INT and the write cycle
  // follow B_BEAT, which does not move addr; consecutive accesses are B_IDLE and B_DEC apart), so they read
  // as before, from registers (c7_s1: pci_addr -> c106 select -> rd_int -> m_rdata -0.99 ns, addr -> the
  // target decode -> pci_addr's enable -0.70 ns)
  tgt_t        tgt_q;
  wire  [31:0] cq_q;
  always_ff @(posedge clk) tgt_q <= tgt;
  assign cq_q = C106_M10K ? c106m_q : cq_r;
  logic [31:0] cq_r;
  always_ff @(posedge clk) cq_r <= cq;
  wire        be_byte = (be == 4'h8) || (be == 4'h4) || (be == 4'h2) || (be == 4'h1);

  // the internal targets' read word (CPU byte order)
  logic [31:0] rd_int;
  always_comb begin
    rd_int = 32'hFFFF_FFFF;
    case (tgt_q)
      X_IO:    rd_int = {io_rbyte, 24'h00_0000};
      X_MIDI:  rd_int = addr[2] ? 32'h8300_0000 : 32'h0000_0000;
      X_SYS: begin
        case (addr[5:2])
          4'h2:    rd_int = {crom_bank, 24'hFF_FFFF};
          4'h4:    rd_int = {2'b00, jt_tdo, 5'd0, 24'hFF_FFFF};                   // 10: TDO in bit 5 (Supermodel)
          4'h5:    rd_int = {irq_enable, 24'hFF_FFFF};
          4'h6:    rd_int = {irq_state, 24'hFF_FFFF};
          default: rd_int = 32'hFFFF_FFFF;
        endcase
      end
      X_RTC:   rd_int = {4'd0, s2 ? rtc_d : 4'd0, 24'h03_0000};
      X_PCIA:  rd_int = bswap(pci_addr);
      X_PCID:  rd_int = (pci_dev == 5'd13) ? bswap((pci_reg == 6'd0) ? (s2 ? 32'h1786_11DB : R3D_ID) : 32'd0)
                      : (s2 && pci_dev == 5'd0) ? (be_byte ? 32'hFFFF_FFFF : (pci_reg == 6'd0) ? 32'h5710_0200
                                                   : cq_q)
                      : (pci_dev == 5'd0)  ? ((pci_reg == 6'd0) ? 32'h5710_0100 : 32'd0)
                      : (s2 && pci_dev == 5'd16) ? (((be == 4'hF) && pci_reg == 6'd0) ? bswap(32'h1827_11DB) : 32'd0)
                      : 32'hFFFF_FFFF;                                             // no device: master abort
      X_PAL:   rd_int = 32'd0;
      X_TGREG: rd_int = bswap(tg_sh_q);
      X_MPC:   rd_int = (addr[7:2] == 6'd0) ? (s2 ? 32'h5710_0200 : 32'h5710_0100) : 32'd0;
      default: rd_int = 32'hFFFF_FFFF;
    endcase
  end

  always_ff @(posedge clk) tg_sh_q <= tg_shadow[addr[7:2]];

  // ===========================================================================
  // the FSM
  // ===========================================================================
  wire        cur_w   = (cmd == MC_WR) || (cmd == MC_LINE_WR);
  wire        is_line = (cmd == MC_LINE_RD) || (cmd == MC_LINE_WR);
  wire [31:0] wd_le = bswap(wd);      // the word as a little-endian device sees it

  // OWIRE: the targets' fields. Each target reads them only while its request is up (B_SCSI, B_SEC, B_CFG, which
  // move none of cmd, addr, be, wd or pci_addr) or, for the tile generator's data, in the clock after do_w (wd moves
  // in B_IDLE or B_BEAT only, never in that clock; tg_pal_addr and tg_reg_addr stay registered: a line write's addr
  // has moved on by then). The registered copies are the old form
  logic        sc_we_q, sec_we_q, cf_we_q;
  logic [6:2]  sc_addr_q;
  logic [17:2] sec_addr_q;
  logic [5:0]  cf_idx_q;
  logic [3:0]  sc_be_q, sec_be_q, cf_be_q;
  logic [31:0] sc_wdata_q, sec_wdata_q, cf_wdata_q, tg_reg_wdata_q;
  logic [15:0] tg_pal_wdata_q;
  assign sc_we        = OWIRE ? cur_w      : sc_we_q;
  assign sc_addr      = OWIRE ? addr[6:2]  : sc_addr_q;
  assign sc_be        = OWIRE ? be         : sc_be_q;
  assign sc_wdata     = OWIRE ? wd         : sc_wdata_q;
  assign sec_we       = OWIRE ? cur_w      : sec_we_q;
  assign sec_addr     = OWIRE ? addr[17:2] : sec_addr_q;
  assign sec_be       = OWIRE ? be         : sec_be_q;
  assign sec_wdata    = OWIRE ? wd         : sec_wdata_q;
  assign cf_we        = OWIRE ? cur_w      : cf_we_q;
  assign cf_idx       = OWIRE ? pci_reg    : cf_idx_q;
  assign cf_be        = OWIRE ? {be[0], be[1], be[2], be[3]} : cf_be_q;
  assign cf_wdata     = OWIRE ? bswap(wd)  : cf_wdata_q;
  assign tg_reg_wdata = OWIRE ? wd_le      : tg_reg_wdata_q;
  assign tg_pal_wdata = OWIRE ? {wd[23:16], wd[31:24]} : tg_pal_wdata_q;

  // A master sees its acknowledge a cycle after the bus gives it, and drops
  // its request only then: a request whose acknowledge is out this cycle is
  // the one just served, not a new one (a posted SDRAM write returns to idle
  // at its acknowledge).
  // The snoop: master 1 waits for it (RAM only); the CPU waits from its
  // acknowledge to master 1's grant.
  logic        snp_ok;            // master 1's present RAM access has been snooped
  wire         ram1 = (m_addr[1][31:23] == 9'd0);
  wire         snp = snp_ok || (sn_req && sn_ack);       // done, or done this cycle
  wire  [1:0]  rq = m_req & ~m_ack & {!ram1 || snp, !snp};
  wire         g = (rq == 2'b11) ? !last : rq[1];       // the grant: in turn when both ask
  logic        do_w;              // an internal write this cycle (the word wd at addr)
  logic        do_r;              // an internal read: data to the master next cycle
  logic        r_pend;

  // interrupt sources and acknowledges this cycle (set wins)
  always_comb begin
    irq_set = 8'd0; irq_clr = 8'd0;
    if (ln_start && ln == 10'd383) irq_set[1] = 1'b1;
    if (ln_start && ln == 10'd0)   irq_set = irq_set | 8'h0D;
    if (midi_cnt == 16'd0 && midi_ctrl[5]) irq_set[6] = 1'b1;               // the transmitter is free
    if (do_w && be[3]) begin
      if (tgt_q == X_TGREG && addr[7:2] == 6'h04) irq_clr = wd[31:24];         // F1180010
      if (tgt_q == X_SYS && addr[5:2] == 4'h6)    irq_clr = wd[31:24];         // F0100018 (Supermodel)
      if (tgt_q == X_MIDI && (!addr[2] || !wd[29])) irq_clr[6] = 1'b1;         // a data byte, or RTS off
    end
    if (do_w && be[3] && tgt_q == X_MIDI && !addr[2]) irq_set[6] = 1'b0;       // the byte just written is in flight
  end

  // writes to internal targets, performed in the cycle do_w is high
  always_ff @(posedge clk) begin
    tg_reg_we <= 1'b0; tg_pal_we <= 1'b0; jt_wr <= 1'b0; io_sel <= 1'b0;
    if (rst) begin
      irq_state <= '0; irq_enable <= '0; crom_bank <= 8'hFF; pci_addr <= '0; midi_ctrl <= '0;
      midi_cnt <= '0;
      n_submit <= '0; n_cull_lo <= '0; n_cull_hi <= '0; n_poly <= '0; n_tex <= '0; n_vrom <= '0;
      n_unmapped <= '0; last_unmapped <= '0; r3d_cfg0 <= '0; r3d_cfg1 <= '0; r3d_cfg2 <= '0;
      c106_w <= '0;
    end else begin
      irq_state <= (irq_state & ~irq_clr) | irq_set;
      if (midi_cnt != 16'd0) midi_cnt <= midi_cnt - 16'd1;
      if (jt_cfg_we) begin r3d_cfg0 <= jt_cfg0; r3d_cfg1 <= jt_cfg1; r3d_cfg2 <= jt_cfg2; end

      // the Real3D's window writes, counted as they go to r3d_cmd
      if (h_go && h_we) begin
        case (h_addr[31:24])
          8'h88: n_submit  <= n_submit + 24'd1;
          8'h8C: n_cull_lo <= n_cull_lo + 24'd1;
          8'h8E: n_cull_hi <= n_cull_hi + 24'd1;
          8'h90: n_vrom    <= n_vrom + 24'd1;
          8'h94: n_tex     <= n_tex + 24'd1;
          8'h98: n_poly    <= n_poly + 24'd1;
          default: ;
        endcase
      end
      if (do_w) begin
        case (tgt_q)
          X_R3D: begin
            case (addr[31:24])
              default: begin
                if (addr[23:4] == 20'd0) begin                                       // 9C000000-0C
                  if (addr[3:2] == 2'd0) r3d_cfg0 <= wd_le[18:0];
                  if (addr[3:2] == 2'd1) r3d_cfg1 <= wd_le[18:0];
                  if (addr[3:2] == 2'd2) r3d_cfg2 <= wd_le[18:0];
                end
              end
            endcase
          end
          X_IO: io_sel <= 1'b1;
          X_MIDI: begin
            if (addr[2]) begin
              if (be[3]) midi_ctrl <= wd[31:24];
            end else if (be[3]) midi_cnt <= 16'(MIDI_GAP);
          end
          X_SYS: begin
            case (addr[5:2])
              4'h2: if (be[3]) crom_bank <= wd[31:24];
              4'h3: if (be[3]) begin jt_wr <= 1'b1; jt_wdata <= wd[31:24]; end
              4'h5: if (be[3]) irq_enable <= wd[31:24];
              default: ;
            endcase
          end
          X_PCIA: if (be == 4'hF) pci_addr <= bswap(wd);
          X_PCID: if (s2 && pci_dev == 5'd0) c106_w[pci_reg] <= 1'b1;
          X_PAL:  if (be[3] && be[2]) begin tg_pal_we <= 1'b1; tg_pal_addr <= addr[16:2]; tg_pal_wdata_q <= {wd[23:16], wd[31:24]}; end
          X_TGREG: begin
            tg_reg_we <= 1'b1; tg_reg_addr <= addr[7:2]; tg_reg_wdata_q <= wd_le;
          end
          X_NONE: begin n_unmapped <= n_unmapped + 16'd1; last_unmapped <= addr; end
          default: ;
        endcase
      end
      if (do_r && tgt_q == X_NONE) begin n_unmapped <= n_unmapped + 16'd1; last_unmapped <= addr; end
      if (do_r && tgt_q == X_IO) io_sel <= 1'b1;
    end
  end
  always_ff @(posedge clk) if (do_w && tgt_q == X_TGREG) tg_shadow[addr[7:2]] <= bswap(wd);
  // the MPC106's registers, each byte lane as written (the first write of a register keeps
  // its reset value's other lanes, as Supermodel's byte array does)
  always_ff @(posedge clk) if (STEP2 && !C106_M10K && do_w && tgt_q == X_PCID && s2 && pci_dev == 5'd0) begin
    c106[pci_reg] <= {be[3] ? wd[31:24] : cq_q[31:24], be[2] ? wd[23:16] : cq_q[23:16],
                      be[1] ? wd[15:8] : cq_q[15:8], be[0] ? wd[7:0] : cq_q[7:0]};
  end
  // C106_M10K: the write port. A configuration write merges its lanes with cq_q (the register's word, read a clock
  // or more after pci_reg settled, as in the MLAB form); from each reset's start c_sw writes the 64 reset values,
  // one a clock. The read never meets a write in a clock whose word is used (accesses are B_IDLE and B_DEC apart,
  // and the sweep holds them), checked below
  logic [6:0] c_sw;                 // the sweep's register; 64: done (registers power up 0, so it runs at power-up)
  logic       rst_q;
  wire        c_swp = STEP2 && C106_M10K && !c_sw[6];
  wire        c_we  = c_swp || (STEP2 && C106_M10K && do_w && tgt_q == X_PCID && s2 && pci_dev == 5'd0);
  wire [5:0]  c_wa  = c_swp ? c_sw[5:0] : pci_reg;
  wire [31:0] c_wd  = c_swp ? def106(c_sw[5:0]) :
                      {be[3] ? wd[31:24] : cq_q[31:24], be[2] ? wd[23:16] : cq_q[23:16],
                       be[1] ? wd[15:8] : cq_q[15:8], be[0] ? wd[7:0] : cq_q[7:0]};
  always_ff @(posedge clk) if (c_we) c106m[c_wa] <= c_wd;
  always_ff @(posedge clk) begin
    rst_q <= rst;
    if (rst && !rst_q) c_sw <= '0;
    else if (c_swp) c_sw <= c_sw + 7'd1;
  end
`ifdef VERILATOR
  always_ff @(posedge clk) if (!rst && (do_w || do_r) && (tgt_q != tgt || ((tgt_q == X_PCID) && !C106_M10K && cq_q != cq)))
    $fatal(1, "m3_bus: the target or the MPC106 register registered");
  // C106_M10K: the MLAB form kept as a shadow (written flags, the file, the lane merge), and the M10K's word against it
  // wherever one is used
  logic [31:0] sh106 [0:63];
  logic [63:0] sh106_w;
  logic [31:0] sh106_q;
  always_ff @(posedge clk) begin
    sh106_q <= sh106_w[pci_reg] ? sh106[pci_reg] : c106_def;
    if (rst) sh106_w <= '0;
    else if (STEP2 && do_w && tgt_q == X_PCID && s2 && pci_dev == 5'd0) begin
      sh106_w[pci_reg] <= 1'b1;
      sh106[pci_reg] <= {be[3] ? wd[31:24] : sh106_q[31:24], be[2] ? wd[23:16] : sh106_q[23:16],
                         be[1] ? wd[15:8] : sh106_q[15:8], be[0] ? wd[7:0] : sh106_q[7:0]};
    end
    if (!rst && C106_M10K && STEP2 && (do_w || do_r) && tgt_q == X_PCID && cq_q != sh106_q)
      $fatal(1, "m3_bus: the MPC106's M10K word against the old file");
    if (!rst && C106_M10K && STEP2 && c_swp && bs == B_DEC)
      $fatal(1, "m3_bus: an access held by the reset values' sweep (a reset shorter than 64 clocks)");
  end
`endif
  // the I/O chip takes the access the cycle after (its read side effect included)
  always_ff @(posedge clk) begin
    io_we    <= do_w;
    io_off   <= addr[5:2];
    io_wbyte <= wd[31:24];
  end

  // ---- the snoop ----
  // Not started while master 1 owns the bus: its request may still be up
  // with the access already granted.
  always_ff @(posedge clk) begin
    if (rst) begin
      sn_req <= 1'b0; snp_ok <= 1'b0;
    end else begin
      if (sn_req && sn_ack) begin
        sn_req <= 1'b0; snp_ok <= 1'b1;
      end else if (!sn_req && !snp_ok && m_req[1] && !m_ack[1] && ram1 && !((bs != B_IDLE) && who)) begin
        sn_req <= 1'b1; sn_addr <= m_addr[1][31:5]; sn_inv <= m_cmd[1][0];   // LINE_WR, WR
      end
      if (bs == B_IDLE && rq != 2'b00 && g) snp_ok <= 1'b0;
    end
  end

  // ---- R3P (R112): the Real3D host port's writes, posted ----
  // On the board the 53C810's DMA to the Real3D is a PCI-to-PCI transfer: it does not hold the 603e's bus
  // while the Real3D takes each word. Here every host-port write went word by word with the bus held
  // (B_R3D waits for h_rdy), so the CPU's misses waited behind the Real3D. With R3P a write to the host port
  // (either master's, a line's beats or a single word) goes into an eight-word buffer, a word a beat, and
  // the bus is free once the last is in; the buffer gives the words to r3d_cmd in order as h_rdy allows.
  // A read of the host port, and a write to 9C (the Real3D's configuration), wait for the buffer to empty,
  // so every access to the Real3D sees the writes before it in bus order. R114: so does every access to a
  // target other than the SDRAM (B_DEC), as the MPC105 orders them behind its posted PCI writes.
  (* ramstyle = "MLAB, no_rw_check" *) logic [59:0] r3q [8];   // {addr[31:24], addr[21:2], data}
  logic [3:0]  r3_wp, r3_rp;
  wire         r3_full  = (r3_wp[2:0] == r3_rp[2:0]) && (r3_wp[3] != r3_rp[3]);
  wire         r3_empty = (r3_wp == r3_rp);
  wire [59:0]  r3_head  = r3q[r3_rp[2:0]];
  wire         r3_go    = R3P && !r3_empty && h_rdy;            // the buffer's head to r3d_cmd
  logic        r3_push;
  logic [59:0] r3_pd;
  always_ff @(posedge clk) if (R3P && r3_push) r3q[r3_wp[2:0]] <= r3_pd;
  always_ff @(posedge clk)
    if (rst || !R3P) begin r3_wp <= '0; r3_rp <= '0; end
    else begin
      if (r3_push) r3_wp <= r3_wp + 4'd1;
      if (r3_go)   r3_rp <= r3_rp + 4'd1;
    end

  // ---- TGT (R152): the tile generator's bus timing ----
  // Supermodel 0d1dd3d (gm_matthew's measurements on Scud Race Plus), with its units: one VRAM clock (1/32 us)
  // is TG_V sixteenths of a clock.
  // - an access to tile RAM or the palette (F1000000-F111FFFF) keeps the chip busy 5 VRAM clocks (2.5 pixels):
  //   the next one is not taken before that;
  // - in the first 32 tile pairs (32 VRAM clocks each, 512 pixels) of a shown line the chip fetches tiles, and
  //   a tile RAM access is taken only at the start of the last 4 VRAM clocks of a pair, and only if it is
  //   there by then (one arriving later waits for the next pair), and keeps the chip for those 4; the palette
  //   is not fetched from;
  // - an access lands 12.6 VRAM clocks after it is taken, and a read over the bus (anything but work RAM and
  //   F1000000-F11FFFFF) waits until every access taken has landed; so does a read of tile RAM or the palette
  //   (Supermodel starts that read as a write and returns it when it has landed; here it starts then).
  // tg_free and tg_done are the busy and landing times less the current clock's, in sixteenths, plus 16
  // (0: long past), so an access that waited starts at the busy time's fraction, as Supermodel's (rounding
  // each to whole clocks drifted 4.5% slow over back-to-back writes). Unsigned, no signed casts.
  // Without it the CPU wrote tile RAM and the palette several times faster than the board: Scud Race's loader
  // filled its 128-entry palette queue (drained in the vertical blank) past its count's wrap and the race's
  // palette upload was lost (the HUD drawn with the selection screens' colours). The CPU's accesses only.
  localparam int TG_V     = (TG_MHZ + 1) / 2;            // a VRAM clock in sixteenths of a clock (Supermodel's m_busClock)
  localparam int TG_PAIR  = 2 * TG_V;                    // a tile pair, in clocks (32 VRAM clocks)
  localparam int TG_SLOT  = (28 * TG_V) / 16;            // the clock the PowerPC's slot starts in, from the pair's
  localparam int TG_SFRAC = (28 * TG_V) % 16;            // and the slot's start in it
  localparam int TG_ISSUE = 5 * TG_V;                    // busy after an access
  localparam int TG_SBUSY = 4 * TG_V;                    // busy after an access in a slot
  localparam int TG_LAND  = (63 * TG_V + 2) / 5;         // landed
  logic [7:0]  tg_pp;             // clocks into the tile pair (the line's start is clock 0)
  logic [5:0]  tg_pn;             // tile pairs into the line, held at 32
  logic        tg_act;            // a shown line
  logic [9:0]  tg_free;           // busy time - now + 16, sixteenths; 0 when long past
  logic [10:0] tg_done;           // landing time - now + 16
  logic        tg_w;              // the access in B_DEC waited last clock
  wire         tg_fetch = ln_start ? (ln < 10'd384) : (tg_act && !tg_pn[5]);   // the line starts in this clock
  wire         tg_slot  = !ln_start && (tg_pp == 8'(TG_SLOT));
  wire         tg_tile  = (tgt == X_SDR) && dswap;                                   // F1000000-F10FFFFF
  wire         tg_ram   = tg_tile || (tgt == X_PAL);                                 // and F1100000-F111FFFF
  wire         tg_rd    = !cur_w && (addr[31:23] != 9'd0) && !(a_hi == 8'hF1 && addr[23:21] == 3'd0);
  wire         tg_inslot = tg_tile && tg_fetch;
  wire         tg_hold  = TGT && !who && (tg_ram ? ((tg_inslot ? !(tg_slot && tg_free <= 10'(16 + TG_SFRAC)) : (tg_free > 10'd16)) ||
                                                    (!cur_w && tg_done > 11'd16))
                                                 : (tg_rd && tg_done > 11'd16));
  // where the access starts, + 16: the slot's start, the busy time if it waited for it, or this clock
  wire [10:0]  tg_s    = tg_inslot ? 11'(16 + TG_SFRAC) : (tg_w && tg_free != 10'd0) ? {1'b0, tg_free} : 11'd16;
  wire [10:0]  tg_land = tg_s + 11'(TG_LAND);
  always_ff @(posedge clk) begin
    if (rst || !TGT) begin
      tg_pp <= '0; tg_pn <= 6'd32; tg_act <= 1'b0; tg_free <= '0; tg_done <= '0; tg_w <= 1'b0;
    end else begin
      if (ln_start) begin
        tg_pp <= 8'd1; tg_pn <= '0; tg_act <= (ln < 10'd384);
      end else if (tg_pp == 8'(TG_PAIR - 1)) begin
        tg_pp <= '0; if (!tg_pn[5]) tg_pn <= tg_pn + 6'd1;
      end else tg_pp <= tg_pp + 8'd1;
      tg_w <= (bs == B_DEC) && tg_hold;
      if (bs == B_DEC && !who && tg_ram && !tg_hold) begin
        tg_free <= 10'(tg_s + 11'(tg_inslot ? TG_SBUSY - 16 : TG_ISSUE - 16));
        tg_done <= tg_land - 11'd16;     // starts only move on, so this landing is the latest (Supermodel's max)
      end else begin
        tg_free <= (tg_free > 10'd16) ? tg_free - 10'd16 : 10'd0;
        tg_done <= (tg_done > 11'd16) ? tg_done - 11'd16 : 11'd0;
      end
    end
  end

  // ---- the state machine ----
  always_ff @(posedge clk) begin
    m_ack <= '0; m_wnext <= '0; m_rvalid <= '0; r3_push <= 1'b0;
    s_req <= s_req && !s_ack;
    sc_req <= sc_req && !sc_ack;
    sec_req <= sec_req && !sec_ack;
    cf_req <= cf_req && !cf_ack;
    do_w <= 1'b0; do_r <= 1'b0;
    tr_v <= 1'b0;
    if (rst) begin
      bs <= B_IDLE; last <= 1'b1; s_req <= 1'b0; sc_req <= 1'b0; cf_req <= 1'b0; r_pend <= 1'b0; sec_req <= 1'b0;
    end else begin
      case (bs)
        B_IDLE: begin
          // the beat a master shows, a clock longer: m3_sdr_port's LW1 takes a cast-out's last beat
          // the clock after the bus has seen its last s_wnext (study R107); B_DEC sets s_wdata for
          // the next transaction after this
          s_wdata <= m_wdata[who];
          // FAST: the CPU's SDRAM fields loaded every idle clock (nothing reads them while s_req is low), so
          // only s_req and the state wait on the grant and the decode below
          if (FAST) begin
            s_cmd <= m_cmd[0]; s_sa <= SDR_RAM + {4'd0, m_addr[0][22:0]}; s_swap <= 1'b0; s_zero <= 1'b0; s_be <= m_be[0];
          end
          if (rq != 2'b00) begin
          who <= g; last <= g;
          cmd <= m_cmd[g]; addr <= m_addr[g]; be <= m_be[g];
          if (!WD_DEC) wd <= m_wdata[g];
          crit <= (m_cmd[g] == MC_LINE_RD) ? m_addr[g][4:2] : 3'd0;
          beat <= 3'd0;
          bs <= B_DEC;
          // FAST (R112): a read by the CPU of work RAM needs none of B_DEC's decode (its SDRAM address is
          // SDR_RAM plus the low 23 bits, word form, never zero), so it goes to the port now. Reads only:
          // s_wdata keeps following the master this clock for LW1 (above), and B_SDR takes it on
          if (FAST && !g && m_addr[0][31:23] == 9'd0 && !m_cmd[0][0]) begin
            s_req <= 1'b1;
            bs <= B_SDR;
          end
          end
        end

        B_DEC: begin
          if (WD_DEC) wd <= m_wdata[who];   // the owner's word, held until its m_ack
          if (tg_hold || c_swp) begin
            // TGT: the tile generator is busy, fetching, or has accesses still to land; C106_M10K: the reset values
            // are still being written (the 64 clocks from a reset's start; the sweep's register alone, not the target
            // decode, so the hold adds no level to B_DEC's choice: br2_s3 had the decode in it, addr -0.66 ns)
          end else if (tgt == X_SDR) begin
            s_req <= 1'b1; s_cmd <= cmd; s_sa <= dsa; s_swap <= dswap; s_zero <= dzero; s_be <= be;
            s_wdata <= WD_DEC ? m_wdata[who] : wd;
            bs <= B_SDR;
          end else if (R3P && !(tgt == X_R3DH && cur_w) && (!r3_empty || r3_push)) begin
            // R3P (R114): on the board every processor access to PCI leaves the MPC105 behind the writes it
            // has posted (MPC105 manual 8.1.2.2: two 16-byte processor-to-PCI write buffers, one PCI
            // interface), so any access here but a host-port write waits for the buffer to empty, as the
            // bus waited for each host-port write before R3P; not acknowledged until then
          end else begin
            m_ack[who] <= 1'b1;       // internal targets: taken now, beats follow
            addr <= is_line ? {addr[31:5], crit, 2'b00} : addr;
            if (tgt == X_SCSI) bs <= B_SCSI;
            else if (tgt == X_SEC) bs <= B_SEC;
            else if (tgt == X_PCID && pci_dev == 5'd14) bs <= B_CFG;
            else bs <= B_BEAT;
          end
        end

        // SDRAM: the port's handshake is the master's
        B_SDR: begin
          if (s_ack) begin
            m_ack[who] <= 1'b1;
            if (cmd == MC_WR) bs <= B_IDLE;
          end
          if (s_wnext) begin
            m_wnext[who] <= 1'b1;
            if (beat == 3'd7) bs <= B_IDLE;
            beat <= beat + 3'd1;
          end
          if (s_rvalid) begin
            m_rvalid[who] <= 1'b1; m_rdata <= s_rdata;
            if (cmd == MC_RD || beat == 3'd7) bs <= B_IDLE;
            beat <= beat + 3'd1;
          end
          s_wdata <= m_wdata[who];    // the master shows the next beat
          // the trace: tile RAM and backup RAM (not work RAM, not CROM reads)
          if ((s_rvalid || (s_ack && cmd == MC_WR) || s_wnext) && addr[31:23] != 9'd0 && addr[31:24] != 8'hFF) begin
            tr_v <= 1'b1; tr_master <= who; tr_we <= cur_w;
            tr_addr <= is_line ? {addr[31:5], (crit + beat), 2'b00} : addr;
            tr_be <= be; tr_data <= s_rvalid ? s_rdata : s_wnext ? m_wdata[who] : wd;
          end
        end

        // internal targets: one beat, or eight for a line
        B_BEAT: begin
          if (R3P && tgt == X_R3DH && cur_w) begin
            // R3P: the beat into the buffer (a line write's beat taken from the master as below), then on;
            // r3_push is registered, so the buffer has its word an edge later: B_R3P waits that edge out
            if (!r3_full && !(r3_push && 3'(r3_wp[2:0] + 3'd1) == r3_rp[2:0])) begin
              if (cmd == MC_LINE_WR) m_wnext[who] <= 1'b1;
              r3_push <= 1'b1;
              r3_pd   <= {addr[31:24], addr[21:2], (cmd == MC_LINE_WR) ? m_wdata[who] : wd};
              tr_v <= 1'b1; tr_master <= who; tr_we <= 1'b1; tr_addr <= addr; tr_be <= (cmd == MC_LINE_WR) ? 4'hF : be;
              tr_data <= (cmd == MC_LINE_WR) ? m_wdata[who] : wd;
              bs <= B_R3P;
            end
          end else if (R3P && (tgt == X_R3DH || (tgt == X_R3D && cur_w)) && (!r3_empty || r3_push)) begin
            // R3P: a read of the host port, or a 9C write, after every posted write
          end else if (tgt == X_R3DH) begin
            // the Real3D: a line write's beat is taken first, then each word goes
            if (cur_w && cmd == MC_LINE_WR) begin m_wnext[who] <= 1'b1; wd <= m_wdata[who]; be <= 4'hF; end
            bs <= B_R3D;
          end else if (cur_w) begin
            if (cmd == MC_LINE_WR) begin
              // take the beat the master shows (m_wdata), then write it
              m_wnext[who] <= 1'b1;
              wd <= m_wdata[who]; be <= 4'hF;
              bs <= B_INTW;
            end else begin
              if (tgt == X_SYS && addr[5:2] == 4'h3 && jt_busy) ;   // the JTAG chain is still applying the last edge
              else begin do_w <= 1'b1; bs <= B_INTW; end
            end
          end else begin
            do_r <= 1'b1;
            bs <= B_INT;
          end
        end
        B_INTW: begin
          if (cmd == MC_LINE_WR && !do_w) begin
            do_w <= 1'b1;            // the beat taken last cycle
          end else begin
            tr_v <= 1'b1; tr_master <= who; tr_we <= 1'b1; tr_addr <= addr; tr_be <= be; tr_data <= wd;
            if (cmd == MC_LINE_WR && beat != 3'd7) begin
              beat <= beat + 3'd1; addr <= {addr[31:5], addr[4:2] + 3'd1, 2'b00}; bs <= B_BEAT;
            end else bs <= B_IDLE;
          end
        end
        B_INT: begin
          // rd_int is from registers here (tg_sh_q read a cycle after addr settled)
          m_rvalid[who] <= 1'b1; m_rdata <= rd_int;
          tr_v <= 1'b1; tr_master <= who; tr_we <= 1'b0; tr_addr <= addr; tr_be <= be; tr_data <= rd_int;
          if (cmd == MC_LINE_RD && beat != 3'd7) begin
            beat <= beat + 3'd1; addr <= {addr[31:5], addr[4:2] + 3'd1, 2'b00}; bs <= B_BEAT;
          end else bs <= B_IDLE;
        end

        // R3P: the beat is in the buffer; the next beat, or the end
        B_R3P: begin
          if (cmd == MC_LINE_WR && beat != 3'd7) begin
            beat <= beat + 3'd1; addr <= {addr[31:5], addr[4:2] + 3'd1, 2'b00}; bs <= B_BEAT;
          end else bs <= B_IDLE;
        end

        // the Real3D's host port: the strobe goes in a cycle with h_rdy (h_go)
        B_R3D: if (h_rdy && !(R3P && r3_go)) begin
          if (cur_w) begin
            tr_v <= 1'b1; tr_master <= who; tr_we <= 1'b1; tr_addr <= addr; tr_be <= be; tr_data <= wd;
            if (cmd == MC_LINE_WR && beat != 3'd7) begin
              beat <= beat + 3'd1; addr <= {addr[31:5], addr[4:2] + 3'd1, 2'b00}; bs <= B_BEAT;
            end else bs <= B_IDLE;
          end else bs <= B_R3DR;
        end
        B_R3DR: if (h_rv) begin
          m_rvalid[who] <= 1'b1; m_rdata <= h_rdata;
          tr_v <= 1'b1; tr_master <= who; tr_we <= 1'b0; tr_addr <= addr; tr_be <= be; tr_data <= h_rdata;
          if (cmd == MC_LINE_RD && beat != 3'd7) begin
            beat <= beat + 3'd1; addr <= {addr[31:5], addr[4:2] + 3'd1, 2'b00}; bs <= B_BEAT;
          end else bs <= B_IDLE;
        end

        B_SCSI: begin
          if (!sc_req && !r_pend) begin
            sc_req <= 1'b1; sc_we_q <= cur_w; sc_addr_q <= addr[6:2]; sc_be_q <= be; sc_wdata_q <= wd; r_pend <= 1'b1;
          end
          if (sc_ack) begin
            r_pend <= 1'b0;
            tr_v <= 1'b1; tr_master <= who; tr_we <= cur_w; tr_addr <= addr; tr_be <= be; tr_data <= cur_w ? wd : sc_rdata;
            if (!cur_w) begin m_rvalid[who] <= 1'b1; m_rdata <= sc_rdata; end
            bs <= B_IDLE;
          end
        end

        // the security board: one access (a line's beats are not split; the game uses words)
        B_SEC: begin
          if (!sec_req && !r_pend) begin
            sec_req <= 1'b1; sec_we_q <= cur_w; sec_addr_q <= addr[17:2]; sec_be_q <= be; sec_wdata_q <= wd; r_pend <= 1'b1;
          end
          if (sec_ack) begin
            r_pend <= 1'b0;
            tr_v <= 1'b1; tr_master <= who; tr_we <= cur_w; tr_addr <= addr; tr_be <= be; tr_data <= cur_w ? wd : sec_rdata;
            if (!cur_w) begin m_rvalid[who] <= 1'b1; m_rdata <= sec_rdata; end
            bs <= B_IDLE;
          end
        end

        B_CFG: begin
          if (!cf_req && !r_pend) begin
            cf_req <= 1'b1; cf_we_q <= cur_w; cf_idx_q <= pci_reg;
            cf_be_q <= {be[0], be[1], be[2], be[3]}; cf_wdata_q <= bswap(wd); r_pend <= 1'b1;
          end
          if (cf_ack) begin
            r_pend <= 1'b0;
            tr_v <= 1'b1; tr_master <= who; tr_we <= cur_w; tr_addr <= addr; tr_be <= be; tr_data <= cur_w ? wd : bswap(cf_rdata);
            if (!cur_w) begin m_rvalid[who] <= 1'b1; m_rdata <= bswap(cf_rdata); end
            bs <= B_IDLE;
          end
        end

        default: bs <= B_IDLE;
      endcase
    end
  end

  assign ext_int      = |(irq_state & irq_enable) || scsi_irq;
  assign irq_state_o  = irq_state;
  assign irq_enable_o = irq_enable;
  assign crom_bank_o  = crom_bank;
  assign pingpong_o   = pingpong_i;
  // (with R3P the bus's own strobe is a read, the buffer empty; the buffer's head goes first otherwise)
  assign h_go    = r3_go || ((bs == B_R3D) && h_rdy);
  assign h_we    = r3_go ? 1'b1 : cur_w;
  assign h_addr  = r3_go ? {r3_head[59:52], 2'b00, r3_head[51:32], 2'b00} : addr;
  assign h_wdata = r3_go ? bswap(r3_head[31:0]) : wd_le;

endmodule
