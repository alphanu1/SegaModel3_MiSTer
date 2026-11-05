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
// ROM loader: the MRA's downloads (hps_io WIDE: 16-bit words, the address
// rising by 2) into the memories, by ioctl index (study R100):
//
//   0  SDRAM, byte a of the stream at SDRAM byte a (the ROM image; it comes
//      last of the ROMs, and its end is the game's reset release). With VROM_X
//      the image's VROM range (VROM_AT, 32 MiB: a Step 1.x set's VROM) goes to
//      the extra memory instead, at the same offset from the region's start
//      (study R100: the VROM placement parameter), so the same MRA serves both.
//      Not for a set that sends its VROM on index 1 (configuration flags bit 0):
//      its index 0 is banked CROM there
//   1  the extra memory (XMEM: DDR3 now, the second SDRAM later): byte a at
//      byte a of the region, one 16-bit word a write (x_* port)
//   3  SDRAM from FIX8: a Step 2 set's 8 MiB fixed CROM
//   2  the NVRAM file (<nvram>): bytes 0-0x1FFFF the backup RAM in SDRAM at
//      BACKUP, words from 0x20000 the 93C46 (64 words, little-endian)
//   4  the game configuration (Model 2's game-ID index): byte 0 the step
//      (BCD: 0x10, 0x21), byte 1 flags, bytes 2-17 the CROM bank map (bank b:
//      bit 7 empty, bits 3:0 its 8 MiB slot in SDRAM), bytes 18-19 the idle
//      inputs on 315-5649 ports C and D (default FF FF; Daytona 2's 4-speed
//      shifter rests in a gear, so its port C reads AF, as in Supermodel)
//
// Main sends each <rom index> as its own download from address 0, in file
// order (hps_io's address is 27 bits, so 168 MiB cannot be one stream). Every
// download holds the game in reset (busy); rom_loaded rises once a download of
// index 0 has ended and its FIFO has drained, and it means only that (one
// signal, one meaning: mem_ready is the SDRAM's).
//
// The configuration: an index-0 download that no index-4 download came before
// (since the last index-0 one) sets the defaults, VF3's: step 1.0, no flags,
// banks 0-7 in slots 0-7, 8-15 empty. So an MRA without index 4 runs as it
// always did.
//
// The shape is m3v_loader's (and Model 2's m2_rom_loader): one FIFO in front of
// every destination, ioctl_wait raised a margin before it fills and only while
// ioctl_download is high (the HPS stalls on ioctl_wait), nothing written until
// the destination is ready (the SDRAM's mem_ready; the extra memory's x_ok, the
// DDR3 gate of R71).

module m3_loader #(
  parameter int          AW     = 26,
  parameter logic [26:0] BACKUP = 27'h620_0000,
  parameter logic [26:0] FIX8   = 27'h730_0000,
  parameter bit          VROM_X  = 1'b0,           // index 0's VROM range to the extra memory
  parameter logic [26:0] VROM_AT = 27'h400_0000    // where the MRA's index-0 image holds VROM (32 MiB)
) (
  input  logic        clk,
  input  logic        rst,          // memory reset (PLL lock), not the game's
  input  logic        mem_ready,
  input  logic        x_ok,         // the extra memory may be written

  input  logic        ioctl_download,
  input  logic [15:0] ioctl_index,
  input  logic        ioctl_wr,
  input  logic [26:0] ioctl_addr,
  input  logic [15:0] ioctl_dout,
  output logic        ioctl_wait,

  output logic        sdr_wr_req,
  output logic [AW:1] sdr_wr_addr,
  output logic [15:0] sdr_wr_din,
  output logic [1:0]  sdr_wr_be,
  input  logic        sdr_wr_ack,

  // the extra memory: one 16-bit word a request, held to x_ack (one cycle)
  output logic        x_req,
  output logic [26:1] x_addr,       // 16-bit word in the region
  output logic [15:0] x_din,
  input  logic        x_ack,

  // the 93C46's load port (m3_93c46 LOAD)
  output logic        ee_we,
  output logic [5:0]  ee_a,
  output logic [15:0] ee_d,

  // the game configuration
  output logic [7:0]  cfg_step,
  output logic [7:0]  cfg_flags,
  output logic [15:0][4:0] cfg_bank,  // {empty, slot[3:0]} by bank
  output logic [7:0]  cfg_in2, cfg_in3, // idle inputs, ports C and D

  output logic        rom_loaded,
  output logic        busy,         // a download in progress or draining
  output logic        overflow,
  output logic [26:0] bytes         // the last index-0 download's length
);

  wire [5:0] ix   = ioctl_index[5:0];
  wire dl_any     = ioctl_download && (ix == 6'd0 || ix == 6'd1 || ix == 6'd2 || ix == 6'd3 || ix == 6'd4);
  wire dl0        = ioctl_download && ix == 6'd0;
  wire dl_cfg     = ioctl_download && ix == 6'd4;
  wire to_fifo    = dl_any && !dl_cfg;

  // destination classes in the FIFO
  localparam logic [1:0] K_SDR = 2'd0, K_X = 2'd1, K_EE = 2'd2;

  localparam int FD = 512, FA = 9;
  (* ramstyle = "M10K" *) logic [AW+1:0] f_adr [FD];    // {class, address}
  (* ramstyle = "M10K" *) logic [15:0]   f_dat [FD];
  logic [FA:0] wp, rp;
  wire  [FA:0] lvl = wp - rp;
  wire         f_empty = (wp == rp);
  assign ioctl_wait = ioctl_download && (!mem_ready || lvl >= (FA+1)'(FD - 256));

  // the entry for this word: its class and address
  logic [1:0]  n_k;
  logic [AW-1:0] n_a;
  always_comb begin
    n_k = K_SDR; n_a = ioctl_addr[AW:1];
    case (ix)
      // only for a set whose VROM comes in index 0: a Step 2 set (flags bit 0, VROM on index 1) has banked
      // CROM there (CROM2 and CROM3, slots 8-11), which must stay in SDRAM (R100: the merged builds sent it
      // over the VROM in DDR3)
      6'd0: if (VROM_X && !cfg_flags[0] && ioctl_addr[26:25] == VROM_AT[26:25]) begin
              n_k = K_X; n_a = AW'(ioctl_addr[24:1]);                // VROM_AT is 32 MiB aligned
            end
      6'd1: begin n_k = K_X; n_a = ioctl_addr[AW:1]; end
      6'd3: n_a = AW'(FIX8[26:1]) + ioctl_addr[AW:1];
      6'd2: if (ioctl_addr[26:17] != 10'd0) begin n_k = K_EE; n_a = AW'(ioctl_addr[6:1]); end
            else n_a = AW'(BACKUP[26:1]) + AW'(ioctl_addr[16:1]);
      default: ;
    endcase
  end

  logic        rd_armed, wbusy, ack_d, dl0_d, done, cfg_seen, any_d;
  logic [1:0]  h_k;
  logic [AW-1:0] h_adr;
  logic [15:0]   h_dat;

  always_ff @(posedge clk) begin
    ee_we <= 1'b0;
    if (rst) begin
      wp <= '0; rp <= '0; rd_armed <= 1'b0; wbusy <= 1'b0; ack_d <= 1'b0; dl0_d <= 1'b0; done <= 1'b0;
      sdr_wr_req <= 1'b0; x_req <= 1'b0; overflow <= 1'b0; rom_loaded <= 1'b0; bytes <= '0;
      cfg_seen <= 1'b0; any_d <= 1'b0;
      cfg_step <= 8'h10; cfg_flags <= 8'h00; cfg_in2 <= 8'hFF; cfg_in3 <= 8'hFF;
      for (int b = 0; b < 16; b++) cfg_bank[b] <= (b < 8) ? {1'b0, 4'(b)} : 5'h10;
    end else begin
      ack_d <= sdr_wr_ack;
      dl0_d <= dl0;
      any_d <= dl_any;
      if (to_fifo && ioctl_wr) begin
        if (lvl == (FA+1)'(FD)) overflow <= 1'b1;
        else begin
          f_adr[wp[FA-1:0]] <= {n_k, n_a}; f_dat[wp[FA-1:0]] <= ioctl_dout;
          wp <= wp + 1'b1;
        end
      end
      if (dl0 && ioctl_wr) bytes <= ioctl_addr + 27'd2;
      // the game configuration
      if (dl_cfg && ioctl_wr) begin
        cfg_seen <= 1'b1;
        if (ioctl_addr[26:1] == 26'd9) begin cfg_in2 <= ioctl_dout[7:0]; cfg_in3 <= ioctl_dout[15:8]; end
        if (ioctl_addr[26:5] == 22'd0) begin
          if (ioctl_addr[4:1] == 4'd0) begin cfg_step <= ioctl_dout[7:0]; cfg_flags <= ioctl_dout[15:8]; end
          else if (ioctl_addr[4:1] <= 4'd8) begin
            cfg_bank[{ioctl_addr[3:1] - 3'd1, 1'b0}] <= {ioctl_dout[7], ioctl_dout[3:0]};
            cfg_bank[{ioctl_addr[3:1] - 3'd1, 1'b1}] <= {ioctl_dout[15], ioctl_dout[11:8]};
          end
        end
      end
      if (dl0 && !dl0_d) begin
        cfg_seen <= 1'b0;
        if (!cfg_seen) begin
          cfg_step <= 8'h10; cfg_flags <= 8'h00; cfg_in2 <= 8'hFF; cfg_in3 <= 8'hFF;
          for (int b = 0; b < 16; b++) cfg_bank[b] <= (b < 8) ? {1'b0, 4'(b)} : 5'h10;
        end
      end
      if (dl0) rom_loaded <= 1'b0;
      if (dl0_d && !dl0) done <= 1'b1;
      // drain: one write per entry, to its class's destination
      if (wbusy) begin
        sdr_wr_req <= 1'b0;
        if (h_k == K_SDR && sdr_wr_ack && !ack_d) begin wbusy <= 1'b0; rp <= rp + 1'b1; end
        if (h_k == K_X && x_ack) begin x_req <= 1'b0; wbusy <= 1'b0; rp <= rp + 1'b1; end
      end else if (!f_empty && mem_ready && !rd_armed) begin
        {h_k, h_adr} <= f_adr[rp[FA-1:0]]; h_dat <= f_dat[rp[FA-1:0]];
        rd_armed <= 1'b1;
      end else if (rd_armed) begin
        if (h_k == K_EE) begin
          rd_armed <= 1'b0;
          ee_we <= 1'b1; ee_a <= h_adr[5:0]; ee_d <= h_dat;
          rp <= rp + 1'b1;
        end else if (h_k == K_X) begin
          if (x_ok) begin
            rd_armed <= 1'b0;
            x_req <= 1'b1; x_addr <= 26'(h_adr); x_din <= h_dat;
            wbusy <= 1'b1;
          end
        end else begin
          rd_armed <= 1'b0;
          sdr_wr_req <= 1'b1; sdr_wr_addr <= h_adr; sdr_wr_din <= h_dat;
          wbusy <= 1'b1;
        end
      end
      if (done && f_empty && !wbusy && !rd_armed) begin rom_loaded <= 1'b1; done <= 1'b0; end
    end
  end
  assign sdr_wr_be = 2'b11;
  assign busy = dl_any || done || !f_empty || wbusy || rd_armed || any_d;

endmodule
