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
// M2.5 viewer loader: the framework's ioctl stream (hps_io WIDE, 16-bit
// words, the byte address rising by 2) into SDRAM and DDR3, as study D-4 and
// D-6 place the Real3D memories. The files are tools/viewer/m3v_pack.py's.
//
//   index 1, VROM (32 MiB)  byte a -> SDRAM byte a
//   index 2, frames         17 MiB a frame, frame k at file byte 0x1100000 k:
//            0x000000  culling RAM 8C, 4 MiB  -> SDRAM SLOT(k)
//            0x400000  8E, 1 MiB               -> SDRAM SLOT(k) + 0x400000
//            0x500000  polygon RAM, 4 MiB      -> DDR3 POLY(k)
//            0x900000  texture RAM, 8 MiB      -> DDR3 TEX(k), tiled (study D-33)
//   SLOT(k) = 0x2000000 + 0x1000000 k (bytes), POLY(k) = poly_base + 0x80000 k,
//   TEX(k) = tex_base + 0x100000 k (64-bit words)
//
// Texture RAM is 2048 x 2048 16-bit texels, texel A = y 2048 + x in the file.
// In DDR3 it is held as r3d_rast_tcache's lines: the 4 x 4 texels of line
// {y / 4, x / 4} in four beats, row y % 4 a beat, texel x % 4 a lane, word
// TEX(k) + {y[10:2], x[10:2], y[1:0]}. Four texels in a row of the file are
// one beat's four lanes, so the gathering is polygon RAM's.
//
// The frame and the offset in it are counted as the stream goes (the file is
// sent in order from byte 0), so no divide by 17 MiB.
//
// Model 2's m2_rom_loader shape: a FIFO in front of the memories, ioctl_wait
// raised a margin before it fills, gated on ioctl_download (the HPS stalls on
// ioctl_wait), and nothing written until the memory is ready. DDR3 words are
// gathered four to a 64-bit beat and written one beat a request.

module m3v_loader #(
  parameter int AW = 26
) (
  input  logic        clk,
  input  logic        rst,
  input  logic        mem_ready,

  input  logic        ioctl_download,
  input  logic [15:0] ioctl_index,
  input  logic        ioctl_wr,
  input  logic [26:0] ioctl_addr,
  input  logic [15:0] ioctl_dout,
  output logic        ioctl_wait,

  input  logic [24:0] poly_base,
  input  logic [24:0] tex_base,

  // SDRAM download port (m2_sdram wr_*)
  output logic        sdr_wr_req,
  output logic [AW:1] sdr_wr_addr,
  output logic [15:0] sdr_wr_din,
  output logic [1:0]  sdr_wr_be,
  input  logic        sdr_wr_ack,

  // DDR3 (m2_ddr3 contract, one-beat writes)
  output logic        m_req,
  output logic [24:0] m_addr,
  output logic [63:0] m_din,
  input  logic        m_ack,

  // (the bench sets these three directly after loading the rest of the files
  // into its memory models)
  output logic        vrom_loaded   /* verilator public_flat_rw */,  // a VROM file has been written, all of it
  output logic        frames_loaded /* verilator public_flat_rw */,  // a frame file has been written, all of it
  output logic [2:0]  n_frames      /* verilator public_flat_rw */,  // whole frames in it
  output logic        overflow
);

  localparam logic [26:0] FRAME = 27'h110_0000;

  // ---- the stream: which memory, where ----------------------------------------
  logic [26:0] fbase;                // the current frame's first byte
  logic [2:0]  fk;
  wire  [26:0] fo = ioctl_addr - fbase;
  wire         is_vrom = ioctl_index[5:0] == 6'd1;
  wire         is_frm  = ioctl_index[5:0] == 6'd2;
  wire         dl = ioctl_download && (is_vrom || is_frm);

  // a FIFO entry: {ddr, address (SDRAM word, or DDR3 word and lane), data}
  localparam int FD = 512, FA = 9;
  (* ramstyle = "M10K" *) logic [26:0] f_adr [FD];
  (* ramstyle = "M10K" *) logic [15:0] f_dat [FD];
  (* ramstyle = "M10K" *) logic        f_ddr [FD];
  logic [FA:0] wp, rp;
  wire  [FA:0] lvl = wp - rp;
  wire         f_empty = (wp == rp);
  assign ioctl_wait = ioctl_download && (!mem_ready || lvl >= (FA+1)'(FD - 256));

  // SDRAM byte address -> word; DDR3: {64-bit word, lane}
  logic        e_ddr;
  logic [26:0] e_adr;
  always_comb begin
    logic [26:0] sb;
    logic [24:0] dw;
    logic [21:0] ta;                   // a texel of texture RAM
    e_ddr = 1'b0; e_adr = '0; sb = '0; dw = '0; ta = '0;
    if (is_vrom) sb = ioctl_addr;
    else if (fo < 27'h50_0000) sb = 27'h200_0000 + {fk, 24'd0} + fo;
    else if (fo < 27'h90_0000) begin
      e_ddr = 1'b1;
      dw = poly_base + {fk, 19'd0} + 25'((fo - 27'h50_0000) >> 3);
      e_adr = {dw, (fo[2:1])};
    end else begin
      ta = 22'((fo - 27'h90_0000) >> 1);
      e_ddr = 1'b1;
      dw = tex_base + {2'd0, fk, 20'd0} + {5'd0, ta[21:13], ta[10:2], ta[12:11]};
      e_adr = {dw, ta[1:0]};
    end
    if (!e_ddr) e_adr = {1'b0, sb[26:1]};
  end

  // ---- drain ----------------------------------------------------------------------
  logic        rd_armed, busy, ack_d;
  logic [26:0] h_adr;
  logic [15:0] h_dat;
  logic        h_ddr;
  logic [47:0] beat;                  // lanes 0-2 of the DDR3 beat being gathered
  logic        sok_d, done_v, done_f;

  always_ff @(posedge clk) begin
    if (rst) begin
      wp <= '0; rp <= '0; rd_armed <= 1'b0; busy <= 1'b0; ack_d <= 1'b0;
      sdr_wr_req <= 1'b0; m_req <= 1'b0; overflow <= 1'b0;
      vrom_loaded <= 1'b0; frames_loaded <= 1'b0; n_frames <= 3'd0;
      fbase <= '0; fk <= '0; sok_d <= 1'b0; done_v <= 1'b0; done_f <= 1'b0;
    end else begin
      ack_d <= sdr_wr_ack;
      sok_d <= dl;
      // ---- in ----
      if (dl && ioctl_wr) begin
        if (lvl == (FA+1)'(FD)) overflow <= 1'b1;
        else begin
          f_adr[wp[FA-1:0]] <= e_adr; f_dat[wp[FA-1:0]] <= ioctl_dout; f_ddr[wp[FA-1:0]] <= e_ddr;
          wp <= wp + 1'b1;
        end
        if (is_frm && fo == FRAME - 27'd2) begin fbase <= fbase + FRAME; fk <= fk + 3'd1; end
      end
      if (!dl) begin fbase <= '0; fk <= 3'd0; end
      // ---- out ----
      if (busy) begin
        sdr_wr_req <= 1'b0;
        if (h_ddr) begin
          if (m_ack) begin m_req <= 1'b0; busy <= 1'b0; rp <= rp + 1'b1; end
        end else if (sdr_wr_ack && !ack_d) begin
          busy <= 1'b0; rp <= rp + 1'b1;
        end
      end else if (!f_empty && mem_ready && !rd_armed) begin
        h_adr <= f_adr[rp[FA-1:0]]; h_dat <= f_dat[rp[FA-1:0]]; h_ddr <= f_ddr[rp[FA-1:0]];
        rd_armed <= 1'b1;
      end else if (rd_armed) begin
        rd_armed <= 1'b0;
        if (h_ddr) begin
          // lanes 0-2 are gathered, lane 3 completes the beat
          case (h_adr[1:0])
            2'd0: beat[15:0]  <= h_dat;
            2'd1: beat[31:16] <= h_dat;
            2'd2: beat[47:32] <= h_dat;
            default: ;
          endcase
          if (h_adr[1:0] == 2'd3) begin
            m_req <= 1'b1; m_addr <= h_adr[26:2]; m_din <= {h_dat, beat};
            busy <= 1'b1;
          end else rp <= rp + 1'b1;
        end else begin
          sdr_wr_req <= 1'b1; sdr_wr_addr <= h_adr[AW-1:0]; sdr_wr_din <= h_dat;
          busy <= 1'b1;
        end
      end
      // ---- the end of a file: loaded once the FIFO is empty ----
      if (dl) begin
        if (is_vrom) vrom_loaded <= 1'b0;
        if (is_frm) frames_loaded <= 1'b0;
      end else if (sok_d) begin
        // the download has just ended
        done_v <= is_vrom; done_f <= is_frm;
        if (is_frm) n_frames <= fk;
      end
      if ((done_v || done_f) && f_empty && !busy && !rd_armed) begin
        if (done_v) vrom_loaded <= 1'b1;
        if (done_f) frames_loaded <= 1'b1;
        done_v <= 1'b0; done_f <= 1'b0;
      end
    end
  end
  assign sdr_wr_be = 2'b11;

endmodule
