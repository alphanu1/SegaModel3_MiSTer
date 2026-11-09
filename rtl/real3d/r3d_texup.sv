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
// Real3D texture upload (D-37 stage B; Supermodel's FlushTextures,
// UploadTexture and StoreTexture). The texture FIFO's words (94) and the
// blocks the Step 1.0 VROM port (90) copies from VROM are kept in a ring in
// DDR3 as they come; at go, everything flushed (up to the last command-port
// write, t_flush) is decoded into the two 2048 x 1024 texture sheets in DDR3,
// in r3d_rast_tcache's layout (R74): texel (x, y), y the sheet row with page 1
// from row 1024, is lane x[1:0] of 64-bit word TEX + {y[10:2], x[10:2], y[1:0]}.
//
// Why a ring and not a decode on arrival: the RTL draws frame N - 1 while the
// FIFO fills for frame N, and Supermodel uploads at the command port, after
// frame N - 1 is drawn (sim/system/notes.md, Stage B).
//
// The FIFO is a list of textures {size word s, header, texels}: the next
// starts (2 + s / 2) / 4 words on; a size of 0 ends the list, and so does
// fewer than three words left. Header: x / 32 [5:0], y / 32 [11:7], width
// 32 << [16:14], height 32 << [19:17], page [20], write the low byte [21],
// the high byte [22], 16-bit [23], type [31:24]: 0 the texture and its mip
// levels, 1 the texture alone, 2 the mip levels alone, anything else skipped
// (VF3's first upload is 0x80). Mip level i (while the level before was at
// least 4 x 4) sits at x 2048 - 2048 / 2^i + x / 2^i, y 1024 - 1024 / 2^i +
// y / 2^i. Texels come in tiles of min(8, w) x min(8, h), rows of tiles,
// level after level. Supermodel's decode tables reduce to (tile-relative):
//   16-bit, tile width tw: texel (xx, yy) is word tw (yy >> 1) + yy[0] +
//          2 (xx >> 1) of the tile, its high half for even xx
//   8-bit:  word (tw / 2) (yy >> 1) + (xx >> 1), its high half for even yy,
//          the half's high byte for even xx; written to the byte(s) the
//          header selects, both bytes the same value
// (tb_r3d_texup checks these against Supermodel's tables.) Where Supermodel
// runs past a row (x + width over 2048) it spills into the next row; here x
// wraps in the row. VF3 never does it.
//
// Uploads always end at the mark: r3d_cmd marks at a frame's flip, and before
// an upload between frames. A mark during an upload moves its end on, which is
// right, as the frame then wants everything flushed before its flip.
//
// A texel every three cycles (the ring's buffer is an M10K); a beat is
// written when its block or the tile row ends, with the lanes' byte enables.
// The ring is read ahead of the decode in bursts into a 256-beat buffer, by
// beat mod 256. The decode only moves forward, so what the buffer holds is a
// count, have: words fetched from the decode's word dp on. A tile waits for
// its words, a burst goes while have <= 480 (it cannot overwrite a word still
// wanted), and a jump to the next texture is a move inside the buffer when it
// lands within have, else a fresh fetch (restart).
//
// Sizes: (2 + s / 2) / 4 is (s + 4) / 8, exactly, for any s; it is formed on
// s's low 22 bits, and a size past the ring (s[31:22] set) ends the list after
// that texture (Supermodel reads past its FIFO then, and its loop ends).
// Ports: SDRAM m2_sdram's p_ contract (VROM reads); DDR3 m2_ddr3's contract
// with byte enables.
//
// STEP2_PORT (study R100; run time: port3w): the three-word texture port of
// Step 1.5 and later (study D-46: Supermodel's singleWordTexturePort is Step
// 1.0 only; Scud Race's attract never writes the port, so on Step 1.5 it is
// benched here, not on a game). There
// the VROM port takes three words, a VROM word address, a header and a third
// word that starts the upload (Daytona 2 writes all three by DMA, 90000000-08;
// Supermodel keeps the first two and uploads on the third, discarding it; MAME
// also takes three), and the texture is uploaded from VROM with that header
// (Supermodel's WriteTexturePort, MAME's real3d_vrom_texture_dma; no size word
// in VROM). It
// goes into the ring as a FIFO texture would: a size word made for it, the
// header, and its texel words copied from VROM. The texel words are counted
// from the header as the decode below consumes them: a level is
// 2^(lw + lh - 1) words at 16 bits a texel and 2^(lw + lh - 2) at 8, the
// levels those of its type (0: all, from level 0 while a level's size is above
// 1 x 1; 1: level 0; 2: from level 1; others none, and the decode skips it).
// The size word is 8 (n + 2) - 4, so the next texture starts n + 2 words on.
// A texture of 2^19 words or more is cut there (the ring is 2^18). VROM word
// addresses are 24 bits with vrom64 (64 MiB, four 16 MiB blocks: Daytona 2), 23
// otherwise (32 MiB, four 8 MiB blocks: VF3, Scud Race), on both ports alike
// (the block size follows the set's VROM, not the port). Supermodel's polygon-RAM
// source (addresses below 0x100000, Daytona 2 Power Edition only) is not built.

`timescale 1ns/1ps

module r3d_texup #(
  parameter int          AW       = 26,
  parameter logic [26:0] SDR_VROM = 27'h400_0000,  // the MRA's four 8 MiB blocks
  parameter logic [24:0] DDR_STG  = 25'h20_0000,   // the ring, 2^(RW-1) beats
  parameter logic [24:0] DDR_TEX  = 25'hC0_0000,
  parameter int          RW       = 18,            // log2 ring words (2^18: the FIFO's 1 MiB)
  parameter int          FB       = 4,             // log2 beats a ring read burst
  parameter bit          STEP2_PORT = 1'b0         // the three-word texture port (port3w)
) (
  input  logic          clk,
  input  logic          rst,
  input  logic          port3w,          // run time: the VROM port takes three words (STEP2_PORT; Step 1.5 and later)
  input  logic          vrom64,          // run time: VROM is 64 MiB in 16 MiB blocks (else 32 MiB in 8 MiB blocks)

  // words in: t_vrom 0 the FIFO (94), 1 the VROM port (90)
  input  logic          t_valid,
  output logic          t_ready,         // also: nothing waits to go into the ring
  input  logic          t_vrom,
  input  logic [31:0]   t_data,
  input  logic          t_flush,         // the command port (only with t_ready): upload what has come
  output logic          t_pend,          // flushed words not yet uploaded
  input  logic          mark,            // a frame's draw: its textures are those flushed now
  output logic          t_mpend,         // words flushed by the last mark not yet uploaded
  input  logic          go,              // upload to the mark (a mark may come during it: it extends)
  output logic          busy,            // uploading

  // SDRAM: VROM reads
  output logic          s_req,
  output logic [AW:1]   s_addr,
  input  logic          s_ack,
  input  logic [63:0]   s_dout,

  // DDR3
  output logic          d_req,
  output logic          d_we,
  output logic [24:0]   d_addr,
  output logic [7:0]    d_blen,
  output logic [63:0]   d_din,
  output logic [7:0]    d_be,
  input  logic          d_wacc,
  input  logic          d_rvalid,
  input  logic          d_ack,
  input  logic [63:0]   d_dout,

  output logic [15:0]   n_tex,           // textures uploaded, all time
  output logic [15:0]   n_skip           // of those, skipped (type not 0-2)
);

  localparam int PW = RW + 1;            // ring word pointers, one bit over
  localparam int BW = RW;                // ring beat pointers, one bit over

  // ===========================================================================
  // the ring: append (FIFO words, VROM words), flush and mark
  // ===========================================================================
  logic [PW-1:0] wp, fp, fp_m, rp;       // words written, flushed, marked, uploaded
  assign t_pend  = (fp != rp);
  assign t_mpend = (fp_m != rp);

  logic          a_v;                    // a word to append
  logic [31:0]   a_d;

  // the VROM port's block: one SDRAM read a word
  logic          v_on, v_first, v_sgap;
  logic [23:0]   v_a;                    // the next VROM word
  logic [18:0]   v_n;                    // words left

  // the three-word port (STEP2_PORT): address, then header; then the size word and
  // the header into the ring, then the copy
  wire           s2p = STEP2_PORT && port3w;
  logic [1:0]    p2_w;                   // the port's words come: 0 address, 1 header, 2 the start
  logic [2:0]    p2_st;                  // 0 idle, 1 counting, 2 the size word, 3 the header, 4 the copy
  logic [23:0]   p2_addr;
  logic [31:0]   p2_hdr;
  logic [19:0]   p2_n;                   // texel words
  logic [3:0]    p2_lv, p2_last;
  logic [4:0]    p2_m;                   // log2 words of level 0
  wire           p2_busy = s2p && (p2_st != 3'd0);

  // ===========================================================================
  // the decode's state
  // ===========================================================================
  typedef enum logic [3:0] {
    U_IDLE, U_H0S, U_H0, U_H0W, U_H0D, U_H1W, U_H1D, U_JMPS, U_JMP, U_LV, U_TILE, U_TX, U_TW, U_WR, U_NEXT
  } ust_t;
  ust_t          us;
  logic [PW-1:0] dp, tnext;              // the decode's word (the texture's start, then its data); the next texture
  logic          sz0, far;               // the size word: 0 words (the list ends); past the ring (ends after)
  logic signed [10:0] have;              // words held from dp on
  logic [28:0]   hdr;                    // the header, the bits used: {type, 16-bit, MSB, LSB, page, h, w, y / 32, x / 32}
  logic [3:0]    lv;                     // mip level
  logic [3:0]    lw, lh;                 // the level's log2 width, height
  logic [1:0]    ltx, lty;               // log2 tile width, height
  logic [5:0]    tw;                     // words a tile
  logic [10:0]   x0, y0;                 // the level's origin (y0 with the page)
  logic [8:0]    tcx, tcy;               // tile column, row
  logic [2:0]    xx, yy;
  logic [63:0]   bacc;
  logic [7:0]    bbe;
  logic          wr_v;                   // the beat waits to be written

  // The ring's fill (R100, hand-off 5). Daytona 2's boot sends about 0.5 M words
  // of textures, one 256 x 256 (43,692 words) and a flush a frame, while r3d_cmd
  // holds every upload back (frames in flight). With no limit the appends ran
  // past rp and wrote over textures not yet uploaded, and the decode then lost
  // them: ten textures, the skid marks and car shadows on page 1 among them,
  // never reached the sheets. So: no word goes in while the ring is full of words
  // not yet decoded (room; words before dp are consumed, the decode only moves
  // forward), and from half full an upload of everything flushed starts without
  // waiting for go (ring_hi). In steady running the ring is never half full.
  wire  [PW-1:0] occ     = wp - (busy ? dp : rp);
  // room and ring_hi as registers, a cycle late (timing: dp through the subtract and the compare into the append's
  // and the VROM block's enables). Late is safe: room admits at most one word past the limit, which keeps 8 spare;
  // ring_hi only starts an upload a cycle later.
  logic          room, ring_hi;
  always_ff @(posedge clk) begin
    room    <= occ < PW'((1 << RW) - 8);
    ring_hi <= occ >= PW'(1 << (RW - 1));
  end
  assign t_ready = !a_v && !v_on && !p2_busy && room;

  wire  [7:0]  htype = hdr[28:21];
  wire         b16   = hdr[20];
  wire         wmsb  = hdr[19];
  wire         wlsb  = hdr[18];
  wire         hpage = hdr[17];
  wire  [3:0]  lw0   = 4'd5 + {1'b0, hdr[13:11]};
  wire  [3:0]  lh0   = 4'd5 + {1'b0, hdr[16:14]};
  wire  [10:0] hx    = {hdr[5:0], 5'd0};
  wire  [9:0]  hy    = {hdr[10:6], 5'd0};
  wire  [2:0]  xlast = (3'd1 << ltx) - 3'd1;
  wire  [2:0]  ylast = (3'd1 << lty) - 3'd1;

  // this texel's word in the tile, and its place
  logic [5:0]  toff;
  always_comb begin
    if (b16) toff = ({4'd0, yy[2:1]} << ltx) + {5'd0, yy[0]} + {3'd0, xx[2:1], 1'b0};
    else     toff = ({4'd0, yy[2:1]} << (ltx - 2'd1)) + {4'd0, xx[2:1]};
  end
  wire  [8:0]    tj   = dp[8:0] + 9'(toff);
  wire  [10:0]   tx_x = x0 + (11'(tcx) << ltx) + {8'd0, xx};
  wire  [10:0]   tx_y = y0 + (11'(tcy) << lty) + {8'd0, yy};

  // ===========================================================================
  // the ring's read buffer: beat k of the ring at k mod 256. Fetched beats
  // (have: how many from dp on).
  // ===========================================================================
  (* ramstyle = "M10K, no_rw_check" *) logic [63:0] rbuf [256];
  logic [63:0]   rb_q;
  logic [7:0]    rb_ra;
  logic          rsel, rsel_q;           // the word's half, with rb_ra and with rb_q
  logic [BW-1:0] fbp;                    // the next beat to fetch
  logic          f_on;                   // a burst in flight
  logic [7:0]    f_i;
  logic [1:0]    own;                    // the DDR3 port's owner: 0 append, 1 fetch, 2 texels
  logic          d_busy;
  wire           f_sel = d_busy && own == 2'd1;
  always_ff @(posedge clk) begin
    if (f_sel && d_rvalid) rbuf[f_i] <= d_dout;
    rb_q <= rbuf[rb_ra];
    rsel_q <= rsel;
  end
  wire  [31:0]   rword = rsel_q ? rb_q[63:32] : rb_q[31:0];
  wire           f_room = have <= 11'sd480;
  wire  [PW-1:0] jd     = tnext - dp;    // a jump to the next texture: inside the buffer if 0 <= jd <= have
  wire           j_in   = !jd[PW-1] && ($signed({1'b0, jd}) <= $signed({{(PW-10){have[10]}}, have}));
  // The end test and the jump test are registered (timing): U_H0S and U_JMPS
  // wait the cycle. A burst landing meanwhile only makes the jump test
  // conservative (a fresh fetch); a mark meanwhile, the end test (the upload
  // ends at the mark it tested, and the rest goes in the next one).
  logic          h0_end_r, j_in_r;
  logic [PW-1:0] h0_fpm_r;
  logic [10:0]   jd_r;
  always_ff @(posedge clk) begin
    h0_end_r <= $signed(fp_m - dp) <= $signed(PW'(2));
    h0_fpm_r <= fp_m;
    j_in_r   <= j_in;
    jd_r     <= jd[10:0];
  end

  // the texel and its byte enables
  logic [15:0]   tex_v;
  logic [1:0]    tex_be;
  always_comb begin
    logic [15:0] h;
    logic [7:0]  b;
    h = (b16 ? xx[0] : yy[0]) ? rword[15:0] : rword[31:16];
    b = xx[0] ? h[7:0] : h[15:8];
    tex_v  = b16 ? h : {b, b};
    tex_be = b16 ? 2'b11 : {wmsb, wlsb};
  end
  wire [7:0] be_new = bbe | (tx_x[1:0] == 2'd0 ? {6'd0, tex_be} : tx_x[1:0] == 2'd1 ? {4'd0, tex_be, 2'd0} :
                             tx_x[1:0] == 2'd2 ? {2'd0, tex_be, 4'd0} : {tex_be, 6'd0});

  // the write data: the owner's (its state holds still until the write is done)
  assign d_din = (own == 2'd0) ? {a_d, a_d} : bacc;
  assign d_be  = (own == 2'd0) ? (wp[0] ? 8'hF0 : 8'h0F) : bbe;

  always_ff @(posedge clk) begin
    if (rst) begin
      wp <= '0; fp <= '0; fp_m <= '0; rp <= '0; a_v <= 1'b0; v_on <= 1'b0; s_req <= 1'b0; v_sgap <= 1'b0;
      us <= U_IDLE; busy <= 1'b0; d_req <= 1'b0; d_busy <= 1'b0; f_on <= 1'b0; fbp <= '0; have <= '0;
      wr_v <= 1'b0; n_tex <= '0; n_skip <= '0; own <= 2'd0;
      p2_w <= 2'd0; p2_st <= 3'd0;
    end else begin
      logic        f_done, rs;            // a burst has landed; a restart
      logic [PW-1:0] rsa;                 // ... at this word
      logic [10:0] adv;                   // words the decode moves on
      f_done = 1'b0; rs = 1'b0; rsa = '0; adv = '0;
      v_sgap <= 1'b0;

      // ---- in: a FIFO word, or a VROM port write; the flush; the mark
      if (t_valid && t_ready) begin
        if (t_vrom && s2p) begin
          if (p2_w == 2'd0) begin p2_w <= 2'd1; p2_addr <= t_data[23:0]; end
          else if (p2_w == 2'd1) begin p2_w <= 2'd2; p2_hdr <= t_data; end
          else begin
            // the third word: the upload (its value is not used)
            logic [3:0] lw0_, lh0_, mn;
            p2_w <= 2'd0;
            lw0_ = 4'd5 + {1'b0, p2_hdr[16:14]}; lh0_ = 4'd5 + {1'b0, p2_hdr[19:17]};
            mn = (lw0_ < lh0_) ? lw0_ : lh0_;
            p2_m  <= 5'({1'b0, lw0_} + {1'b0, lh0_}) - 5'd1 - {4'd0, !p2_hdr[23]};
            p2_n  <= '0;
            p2_lv   <= (p2_hdr[31:24] == 8'h02) ? 4'd1 : 4'd0;
            p2_last <= (p2_hdr[31:24] == 8'h01) ? 4'd0 : mn - 4'd1;
            p2_st <= (p2_hdr[31:24] <= 8'h02) ? 3'd1 : 3'd2;
          end
        end else if (t_vrom) begin v_on <= 1'b1; v_first <= 1'b1; v_a <= {1'b0, t_data[22:0]}; end
        else begin a_v <= 1'b1; a_d <= t_data; end
      end
      // the Step 2.x port's texture: count its words, then the two ring words, then the copy
      if (s2p) begin
        case (p2_st)
          3'd1: begin
            p2_n <= p2_n + (20'd1 << (p2_m - {p2_lv, 1'b0}));
            if (p2_lv >= p2_last) p2_st <= 3'd2;
            p2_lv <= p2_lv + 4'd1;
          end
          3'd2: if (!a_v && room) begin
            logic [18:0] n;
            n = (p2_n >= 20'h7FFFD) ? 19'h7FFFD : p2_n[18:0];
            p2_n <= {1'b0, n};
            a_v <= 1'b1; a_d <= {10'd0, 22'(({3'd0, n} + 22'd2) << 3) - 22'd4};
            p2_st <= 3'd3;
          end
          3'd3: if (!a_v && room) begin a_v <= 1'b1; a_d <= p2_hdr; p2_st <= 3'd4; end
          3'd4: if (!a_v) begin
            if (p2_n != 20'd0) begin v_on <= 1'b1; v_first <= 1'b0; v_a <= p2_addr; v_n <= p2_n[18:0]; end
            p2_st <= 3'd0;
          end
          default: ;
        endcase
      end
      if (t_flush) fp <= wp;
      if (mark) fp_m <= t_flush ? wp : fp;
      else if (us == U_IDLE && !f_on && ring_hi && t_pend && !(go && t_mpend)) fp_m <= fp;   // the ring is half full: mark

      // ---- the VROM block: its size word, then its words into the ring.
      // VROM word i = {g, b, h} is at SDR_VROM + {b, g, h, 00}
      if (v_on && !a_v) begin
        if (!s_req && !v_sgap && !s_ack && room) begin
          s_req <= 1'b1;
          // (64 MiB VROM: 24-bit word addresses, 16 MiB blocks; 32 MiB: 23 bits, 8 MiB blocks)
          s_addr <= AW'(SDR_VROM[26:1]) + (vrom64 ? AW'({v_a[2:1], v_a[23:3], 2'b00}) : AW'({v_a[2:1], v_a[22:3], 2'b00}));
        end
        if (s_ack) begin
          logic [31:0] w;
          logic [31:0] n;
          s_req <= 1'b0; v_sgap <= 1'b1;
          w = v_a[0] ? s_dout[63:32] : s_dout[31:0];
          n = {10'd0, 22'(({1'b0, w[21:0]} + 23'd4) >> 3)};
          if (v_first) begin
            v_first <= 1'b0;
            v_n <= (w[31:22] != 10'd0) ? 19'h7FFFF : n[18:0];
            if (w[31:2] == 30'd0) v_on <= 1'b0;
          end else begin
            a_v <= 1'b1; a_d <= w;
            v_a <= v_a + 24'd1;
            v_n <= v_n - 19'd1;
            if (v_n == 19'd1) v_on <= 1'b0;
          end
        end
      end

      // ---- the DDR3 port: append, then texels, then the ring's fetch
      if (!d_busy) begin
        if (a_v) begin
          own <= 2'd0; d_busy <= 1'b1; d_req <= 1'b1; d_we <= 1'b1; d_blen <= 8'd1;
          d_addr <= DDR_STG + 25'(wp[RW-1:1]);
        end else if (wr_v) begin
          own <= 2'd2; d_busy <= 1'b1; d_req <= 1'b1; d_we <= 1'b1; d_blen <= 8'd1;
          d_addr <= DDR_TEX + {5'd0, tx_y[10:2], tx_x[10:2], tx_y[1:0]};
        end else if (busy && !f_on && f_room && us != U_IDLE) begin
          own <= 2'd1; d_busy <= 1'b1; d_req <= 1'b1; d_we <= 1'b0; d_blen <= 8'(1 << FB);
          d_addr <= DDR_STG + 25'(fbp[RW-2:0]);
          f_on <= 1'b1; f_i <= fbp[7:0];
        end
      end else begin
        if (f_sel && d_rvalid) f_i <= f_i + 8'd1;
        if (d_ack) begin
          d_req <= 1'b0; d_busy <= 1'b0;
          case (own)
            2'd0:    begin a_v <= 1'b0; wp <= wp + 1'b1; end
            2'd1:    begin f_on <= 1'b0; f_done = 1'b1; end
            default: wr_v <= 1'b0;
          endcase
        end
      end

      // ---- the decode
      case (us)
        // every upload fetches afresh: beats past the last flush point may
        // have been read before they were written
        U_IDLE: if (!f_on && ((go && t_mpend) || (ring_hi && t_pend && !mark))) begin
          busy <= 1'b1; dp <= rp;
          rs = 1'b1; rsa = rp;
          us <= U_H0S;
        end
        U_H0S: us <= U_H0;
        // a texture: three words at least before the mark, or the list ends
        U_H0: begin
          if (h0_end_r) begin
            if (!f_on) begin rp <= h0_fpm_r; busy <= 1'b0; us <= U_IDLE; end
          end else if (have >= 11'sd2) begin
            rb_ra <= dp[8:1]; rsel <= dp[0]; us <= U_H0W;
          end
        end
        U_H0W: begin rb_ra <= (dp[8:1] + {7'd0, dp[0]}); rsel <= !dp[0]; us <= U_H0D; end
        U_H0D: begin
          tnext <= dp + PW'(({1'b0, rword[21:0]} + 23'd4) >> 3);
          sz0 <= (rword[31:2] == 30'd0);
          far <= (rword[31:22] != 10'd0);
          us <= U_H1W;
        end
        U_H1W: us <= U_H1D;
        U_H1D: begin
          hdr <= {rword[31:20], rword[19:14], rword[11:7], rword[5:0]};
          if (sz0) begin rp <= fp_m; busy <= !f_on; us <= f_on ? U_JMPS : U_IDLE; dp <= fp_m; end   // ends the list
          else begin
            dp <= dp + PW'(2); adv = 11'd2;
            n_tex <= n_tex + 16'd1;
            case (rword[31:24])
              8'h00, 8'h01: begin lv <= 4'd0; us <= U_LV; end
              8'h02:        begin lv <= 4'd1; us <= U_LV; end
              default:      begin n_skip <= n_skip + 16'd1; us <= U_JMPS; end
            endcase
          end
        end
        // the next texture: inside the buffer, or fetch from it
        U_JMPS: us <= U_JMP;
        U_JMP: begin
          if (sz0 || far) begin
            if (!f_on) begin rp <= fp_m; busy <= 1'b0; us <= U_IDLE; end
          end else if (j_in_r) begin
            dp <= tnext; adv = jd_r; us <= U_H0S;
          end else if (!f_on) begin
            dp <= tnext; rs = 1'b1; rsa = tnext; us <= U_H0S;
          end
        end
        // a level: its size and origin, or the texture is done
        U_LV: begin
          logic [3:0] w, h;
          logic [1:0] tx2, ty2;
          logic [2:0] ls;
          w = lw0 - lv; h = lh0 - lv;
          if (lv != 4'd0 && (lv >= lw0 || lv >= lh0)) us <= U_JMPS;
          else begin
            tx2 = (w >= 4'd3) ? 2'd3 : w[1:0];
            ty2 = (h >= 4'd3) ? 2'd3 : h[1:0];
            ls = {1'b0, tx2} + {1'b0, ty2};
            lw <= w; lh <= h; ltx <= tx2; lty <= ty2;
            // words a tile holds: 16-bit 2^(ltx + lty - 1), 8-bit 2^(ltx + lty - 2)
            tw <= b16 ? (6'd1 << (ls - 3'd1)) : (6'd1 << (ls - 3'd2));
            // 2048 - 2048 / 2^i and 1024 - 1024 / 2^i, as 11 and 10 bits
            x0 <= (lv == 4'd0) ? hx : (11'd0 - (11'd1 << (4'd11 - lv))) + (hx >> lv);
            y0 <= {hpage, (lv == 4'd0) ? hy : (lv >= 4'd10) ? 10'd1023 : (10'd0 - (10'd1 << (4'd10 - lv))) + (hy >> lv)};
            tcx <= '0; tcy <= '0;
            us <= U_TILE;
          end
        end
        // a tile: wait for its words
        U_TILE: if (have >= $signed({5'd0, tw})) begin
          xx <= '0; yy <= '0; bbe <= '0;
          us <= U_TX;
        end
        U_TX: begin rb_ra <= tj[8:1]; rsel <= tj[0]; us <= U_TW; end
        U_TW: us <= U_WR;
        // the texel into its lane; the beat goes when its block or the row ends
        U_WR: begin
          case (tx_x[1:0])
            2'd0: bacc[15:0]  <= tex_v;
            2'd1: bacc[31:16] <= tex_v;
            2'd2: bacc[47:32] <= tex_v;
            default: bacc[63:48] <= tex_v;
          endcase
          bbe <= be_new;
          if (tx_x[1:0] == 2'd3 || xx == xlast) begin
            wr_v <= (be_new != 8'd0);
            us <= U_NEXT;
          end else begin
            xx <= xx + 3'd1;
            us <= U_TX;
          end
        end
        // the beat is written from tx_x, tx_y: they hold until it is done
        U_NEXT: if (!wr_v) begin
          bbe <= '0;
          if (xx != xlast) begin xx <= xx + 3'd1; us <= U_TX; end
          else if (yy != ylast) begin xx <= '0; yy <= yy + 3'd1; us <= U_TX; end
          else begin
            // the tile is done
            dp <= dp + PW'(tw); adv = {5'd0, tw};
            if ({1'b0, tcx} + 10'd1 != (10'd1 << (lw - {2'd0, ltx}))) begin tcx <= tcx + 9'd1; us <= U_TILE; end
            else if ({1'b0, tcy} + 10'd1 != (10'd1 << (lh - {2'd0, lty}))) begin tcx <= '0; tcy <= tcy + 9'd1; us <= U_TILE; end
            else if (htype == 8'h01) us <= U_JMPS;
            else begin lv <= lv + 4'd1; us <= U_LV; end
          end
        end
        default: us <= U_IDLE;
      endcase

      // ---- what the buffer holds from dp on
      if (rs) begin
        fbp  <= {rsa[PW-1:FB+1], {FB{1'b0}}};
        have <= -$signed({6'd0, rsa[FB:0]});
      end else begin
        if (f_done) fbp <= fbp + BW'(1 << FB);
        have <= have + (f_done ? 11'sd32 : 11'sd0) - $signed(adv);
      end
    end
  end

endmodule
