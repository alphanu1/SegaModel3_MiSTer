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
// The Step 2.x security board (study R100): the 315-5881 and its RAM, as
// Daytona 2 uses them. Daytona 2 writes about 46 words a frame into the RAM
// (F0180000, at its FE mirror), resets the stream (1A0010 / 1A0014), writes a
// subkey (1A0018) and reads the decrypted words back (1A001C), polling the
// status (1A0000) between them; without the right words it never reaches its
// attract mode (R100). Supermodel's Model3.cpp (ReadSecurity, WriteSecurity)
// and CCrypto, which is MAME's 315-5881 code:
//   - the stream starts at RAM word 0; the first data read after a reset
//     gives FFFF (MAME 0), and then the stream: a header of two decrypted
//     words, {flags, lines - 1, bytes a line - 1}, and words until the bytes
//     taken equal lines x bytes a line, when the next header is read;
//   - a decrypted word is block_decrypt(game key, subkey, address, RAM word)
//     with its top 14 bits from the word decrypted before it (dec_hist),
//     which a header resets; the word read is it byte-swapped, in the top
//     half;
//   - the subkey is the written word's top half byte-swapped; the status
//     reads 0.
// The RAM keeps the top half of each word (the cipher reads only that) for
// the first 512 words (Daytona 2 uses 270); the rest is not stored. Compressed
// streams (header bit 17; Model 2's, the NAOMI's) are not built: Daytona 2's
// 128,114 reads in 3,000 frames use none. A compressed header sets cmp_seen.

module m3_sec (
  input  logic        clk,
  input  logic        rst,
  input  logic [29:0] game_key,     // the set's key (MAME ROM_PARAMETER ":315_5881:key"), from the MRA

  input  logic        req,          // one access, held to ack (m3_bus sec_*)
  input  logic        we,
  input  logic [17:2] addr,
  input  logic [3:0]  be,
  input  logic [31:0] wdata,        // CPU byte order
  output logic        ack,
  output logic [31:0] rdata,
  output logic        cmp_seen
);

  (* ramstyle = "M10K" *) logic [15:0] ram [512];
  logic [8:0]  ra;
  logic [15:0] rq;
  always_ff @(posedge clk) begin
    if (req && !ack && we && !addr[17] && addr[16:11] == 6'd0 && be[3] && be[2]) ram[addr[10:2]] <= wdata[31:16];
    rq <= ram[ra];
  end

  logic [15:0] sub;
  logic        k_ld, go, done;
  logic [15:0] dec;
  m3_5881 u_dec (.clk, .k_ld, .game_key, .seq_key(sub), .go, .counter({7'd0, ra}), .data(rq), .done, .dec);

  // the stream
  typedef enum logic [1:0] { T_IDLE, T_RD, T_GO, T_WAIT } tst_t;
  typedef enum logic [1:0] { P_H1, P_H2, P_DATA } ph_t;
  tst_t        ts;
  ph_t         ph;
  logic        first, ready;
  logic        then_data;           // the header was asked for by a data read: decrypt the word after it
  logic [15:0] hist, hd_hi;
  logic [8:0]  nl, nb;              // lines, bytes a line
  logic [17:0] pos, size;
  wire         busy = (ts != T_IDLE);
  wire  [15:0] res = {hist[15:2], dec[1:0]};

  wire acc    = req && !ack && !busy;
  wire reg_wr = acc && we && addr[17];
  wire dat_rd = acc && !we && addr[17] && addr[5:2] == 4'h7;

  always_ff @(posedge clk) begin
    ack <= 1'b0; k_ld <= 1'b0; go <= 1'b0;
    size <= nl * nb;
    if (rst) begin
      first <= 1'b1; ready <= 1'b0; ra <= 9'd0; sub <= 16'd0; hist <= 16'd0; ts <= T_IDLE; ph <= P_DATA;
      then_data <= 1'b0; pos <= '0; nl <= 9'd1; nb <= 9'd1; cmp_seen <= 1'b0; k_ld <= 1'b1;
    end else begin
      if (acc && !dat_rd) begin
        // RAM writes, register writes, the status: one cycle
        ack <= 1'b1;
        rdata <= (!we && addr[17] && addr[5:2] == 4'h0) ? 32'd0 : 32'hFFFF_FFFF;   // (the RAM is not read back)
        if (reg_wr) begin
          case (addr[5:2])
            4'h4, 4'h5: begin ra <= 9'd0; ready <= 1'b0; first <= 1'b1; end
            4'h6: if (be[3] || be[2]) begin sub <= {wdata[23:16], wdata[31:24]}; ready <= 1'b0; k_ld <= 1'b1; end
            default: ;
          endcase
        end
      end else if (dat_rd) begin
        if (first) begin first <= 1'b0; ack <= 1'b1; rdata <= 32'hFFFF_0000; end
        else if (!ready) begin ph <= P_H1; then_data <= 1'b1; hist <= 16'd0; ts <= T_RD; end
        else begin ph <= P_DATA; ts <= T_RD; end
      end

      // one decrypted word: the RAM read (ra), the cipher, the result
      case (ts)
        T_RD:   ts <= T_GO;                                 // rq is RAM[ra] in T_GO
        T_GO:   begin go <= 1'b1; ts <= T_WAIT; end
        T_WAIT: if (done) begin
          hist <= dec; ra <= ra + 9'd1;
          case (ph)
            P_H1: begin hd_hi <= res; ph <= P_H2; ts <= T_RD; end
            P_H2: begin
              // the header {hd_hi, res}: lines res[7:0] + 1, bytes a line {hd_hi[0], res[15:8]} + 1
              nl <= {1'b0, res[7:0]} + 9'd1; nb <= {hd_hi[0], res[15:8]} + 9'd1;
              if (hd_hi[1]) cmp_seen <= 1'b1;
              pos <= '0; ready <= 1'b1;
              if (then_data) begin ph <= P_DATA; ts <= T_RD; then_data <= 1'b0; end
              else ts <= T_IDLE;
            end
            default: begin
              // the data word; a block complete reads the next header at once (busy meanwhile)
              ack <= 1'b1; rdata <= {res[7:0], res[15:8], 16'd0};
              pos <= pos + 18'd2;
              if (pos + 18'd2 == size) begin ph <= P_H1; hist <= 16'd0; ts <= T_RD; end
              else ts <= T_IDLE;
            end
          endcase
        end
        default: ;
      endcase
    end
  end

endmodule
