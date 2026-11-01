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
// ROM loader: the MRA's stream (hps_io WIDE: 16-bit words, the address rising
// by 2, ioctl_index 0) straight into SDRAM, byte a of the stream at SDRAM
// byte a. The MRA lays the stream out as the SDRAM wants it (study 7.3,
// releases/mra and sim/system/notes.md), so the loader does no arithmetic.
//
// The shape is m3v_loader's (and Model 2's m2_rom_loader): a FIFO in front of
// the SDRAM's download port, ioctl_wait raised a margin before it fills and
// only while ioctl_download is high (the HPS stalls on ioctl_wait), nothing
// written until the SDRAM is ready. rom_loaded rises once a download of index
// 0 has ended and its FIFO has drained; it is the game's reset release, and
// nothing else (one signal, one meaning: mem_ready is the SDRAM's).

module m3_loader #(
  parameter int AW = 26
) (
  input  logic        clk,
  input  logic        rst,          // memory reset (PLL lock), not the game's
  input  logic        mem_ready,

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

  output logic        rom_loaded,
  output logic        busy,         // a download in progress or draining
  output logic        overflow,
  output logic [26:0] bytes         // the last download's length
);

  wire dl = ioctl_download && ioctl_index[5:0] == 6'd0;

  localparam int FD = 512, FA = 9;
  (* ramstyle = "M10K" *) logic [AW-1:0] f_adr [FD];
  (* ramstyle = "M10K" *) logic [15:0]   f_dat [FD];
  logic [FA:0] wp, rp;
  wire  [FA:0] lvl = wp - rp;
  wire         f_empty = (wp == rp);
  assign ioctl_wait = ioctl_download && (!mem_ready || lvl >= (FA+1)'(FD - 256));

  logic        rd_armed, wbusy, ack_d, dl_d, done;
  logic [AW-1:0] h_adr;
  logic [15:0]   h_dat;

  always_ff @(posedge clk) begin
    if (rst) begin
      wp <= '0; rp <= '0; rd_armed <= 1'b0; wbusy <= 1'b0; ack_d <= 1'b0; dl_d <= 1'b0; done <= 1'b0;
      sdr_wr_req <= 1'b0; overflow <= 1'b0; rom_loaded <= 1'b0; bytes <= '0;
    end else begin
      ack_d <= sdr_wr_ack;
      dl_d  <= dl;
      if (dl && ioctl_wr) begin
        if (lvl == (FA+1)'(FD)) overflow <= 1'b1;
        else begin
          f_adr[wp[FA-1:0]] <= ioctl_addr[AW:1]; f_dat[wp[FA-1:0]] <= ioctl_dout;
          wp <= wp + 1'b1;
        end
        bytes <= ioctl_addr + 27'd2;
      end
      if (dl) rom_loaded <= 1'b0;
      if (dl_d && !dl) done <= 1'b1;
      // drain: one SDRAM write per entry
      if (wbusy) begin
        sdr_wr_req <= 1'b0;
        if (sdr_wr_ack && !ack_d) begin wbusy <= 1'b0; rp <= rp + 1'b1; end
      end else if (!f_empty && mem_ready && !rd_armed) begin
        h_adr <= f_adr[rp[FA-1:0]]; h_dat <= f_dat[rp[FA-1:0]];
        rd_armed <= 1'b1;
      end else if (rd_armed) begin
        rd_armed <= 1'b0;
        sdr_wr_req <= 1'b1; sdr_wr_addr <= h_adr; sdr_wr_din <= h_dat;
        wbusy <= 1'b1;
      end
      if (done && f_empty && !wbusy && !rd_armed) begin rom_loaded <= 1'b1; done <= 1'b0; end
    end
  end
  assign sdr_wr_be = 2'b11;
  assign busy = dl || done || !f_empty;

endmodule
