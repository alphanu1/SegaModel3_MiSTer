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
// Tile generator line fetch (M0-E). One engine, time-shared across the four
// layers: for the line being built it reads the stencil word, then walks each
// enabled layer in draw order, and hands the emitter "chunks" -- runs of up to
// eight screen pixels that come from one row of one tile.
//
// A chunk ends at a map tile boundary, at a 32-pixel screen boundary (where the
// stencil can change), and at the end of the drawn range. Within a chunk the
// stencil verdict, the stale-pair choice and the source tile are constant, so
// one name word and one pattern row serve it. A chunk the stencil hides is
// skipped without a fetch: with the pairs exclusive, a line costs about two
// layers of fetches however many are enabled.
//
// Memory: the tile generator's 1 MiB (pattern RAM, line scroll at F6000,
// stencil at F7000, name tables at F8000) is a byte image in SDRAM, 16-bit words
// little-endian (word w = bytes 2w, 2w+1), at word address VBASE. Every read is
// one four-word burst (8 bytes, aligned) on one m2_sdram-contract port: request
// held to acknowledge, one request at a time, data taken only in the acknowledge
// cycle. A one-entry buffer keeps the last pattern burst, so a tile split at a
// stencil boundary is fetched once.
//
// Name words (study R103): a layer's map row is 64 names, and eight lines share
// one row. The names are kept in a row buffer (M10K, 4 layers x 64 names),
// filled eight names at a time by one eight-word read (p_long: words 0-3 with
// p_lo, 4-7 with p_ack), with a row tag per layer and a valid bit per group of
// eight. A line whose row is the one held reads no names at all. The buffer is
// emptied at the start of each frame (go for line 0) and on `inv` (a write to
// the name tables), so it never holds a name the CPU has replaced. Before, each
// group of four names was a request of its own, on every line (about 40% of the
// tile generator's requests).
//
// The switches are compile-time. Their defaults follow Supermodel (D-19); with
// MASK_SCREEN = SCROLL_SM = RS_SWAP = STALE = 0 and CODE_WRAP = 1 the engine is
// MAME's draw_layer. sim/tilegen/tg_ref.h has the same switches in its param
// model, and sim/tilegen/notes.md says what each one is.

module m3_tilegen_fetch #(
  parameter bit MASK_SCREEN = 1'b1,  // stencil in screen space, bit 15 leftmost (SM) / map space, bit 0 (MAME)
  parameter bit SCROLL_SM   = 1'b1,  // line scroll replaces X when register bit 15 is set (SM) / added x2 (MAME)
  parameter bit RS_SWAP     = 1'b1,  // line y's scroll halfword at 2 * (y ^ 1) (SM) / 2 * y (MAME)
  parameter bit STALE       = 1'b1,  // SM: a tile pair straddling a stencil change reuses the pair 16 px left
  parameter bit CODE_WRAP   = 1'b0,  // MAME: codes past the pattern area wrap (% 31744 / % 15872)
  parameter int AW          = 25,
  parameter logic [AW:1] VBASE = '0
) (
  input  logic              clk,
  input  logic              rst_n,

  input  logic              go,          // build line y (restarts a line still in progress)
  input  logic [8:0]        y,
  output logic              busy,
  output logic              overrun,     // pulse: go arrived while busy

  input  logic [7:0]        pri,         // F1180020 bits 15:8: 4-bit [7:4], above-3D [3:0]
  input  logic [3:0][31:0]  lreg,        // F1180060-6C, little-endian values

  output logic              p_req,
  output logic [AW:1]       p_addr,
  output logic              p_long,      // an eight-word read (the name row)
  input  logic [63:0]       p_dout,
  input  logic              p_lo,        // the first four words of an eight-word read
  input  logic              p_ack,
  input  logic              inv,         // a name table was written: empty the name row buffer

  // chunk queue to the emitter
  output logic              q_valid,
  input  logic              q_pop,
  output logic [8:0]        q_x,
  output logic [3:0]        q_n,
  output logic [2:0]        q_col,
  output logic [63:0]       q_data,
  output logic [10:0]       q_palhi,
  output logic              q_is4,
  output logic              q_above,
  output logic              q_pair,
  output logic              flush,       // pulse with an abandoned line: the emitter drops what it holds
  output logic [15:0]       n_req        // requests this line
);

  typedef enum logic [3:0] {
    S_IDLE, S_MREQ, S_SLOT, S_RSREQ, S_SETUP, S_CHUNK, S_NREQ, S_NLK, S_PATA, S_PREQ, S_PUSH, S_DRAIN
  } st_t;
  st_t st;

  logic [8:0]  ly;              // the line
  logic [31:0] mask32;
  logic [2:0]  slot;            // 0-3 below (layers 3,2,1,0), 4-7 above (3,2,0,1)
  logic [1:0]  ll;              // the pass's layer
  logic [15:0] rs16;
  logic        go_pend;
  logic [8:0]  go_y;

  // pass set-up (registered in S_SETUP)
  logic [8:0]  off;
  logic [9:0]  xhi;
  logic [8:0]  my;
  logic [15:0] pmask;
  logic        odd, is4, above, pair;

  // walk
  logic [9:0]  x;

  // chunk being fetched
  logic [8:0]  cx;
  logic [3:0]  cn;
  logic [8:0]  csrc;

  // the name row buffer: entry {layer, srcx[8:5]} holds four names (one 4-word burst's image)
  (* ramstyle = "M10K" *) logic [63:0] nram [64];
  logic [5:0]  nra, nra_q;      // read address, registered in the RAM
  logic [63:0] nbuf;            // the entry read
  logic        nwe;
  logic [5:0]  nwa;
  logic [3:0][5:0] rtag;        // a layer's map row held
  logic [3:0][7:0] gv;          // its groups of eight names held
  logic        nf_ok;           // no name table write since this fill was requested
  always_ff @(posedge clk) begin
    if (nwe) nram[nwa] <= p_dout;
    nra_q <= nra;
  end
  assign nbuf = nram[nra_q];
  logic [63:0] pbuf;
  logic [16:0] ptag;
  logic        pval;
  logic [15:0] tw;              // the chunk's name word
  logic [16:0] pa;              // its pattern burst (byte address [19:3])
  logic        prow0;           // 4-bit: which half of the burst

  // A VRAM word address (byte address [19:1]) widened to the port.
  function automatic logic [AW:1] wa(input logic [18:0] w);
    wa = {{(AW - 19){1'b0}}, w};
  endfunction

  // ------------------------------------------------------------- slot layers
  function automatic logic [1:0] slot_layer(input logic [2:0] s);
    case (s)
      3'd0: slot_layer = 2'd3; 3'd1: slot_layer = 2'd2; 3'd2: slot_layer = 2'd1; 3'd3: slot_layer = 2'd0;
      3'd4: slot_layer = 2'd3; 3'd5: slot_layer = 2'd2; 3'd6: slot_layer = 2'd0; default: slot_layer = 2'd1;
    endcase
  endfunction

  wire [1:0]  sl     = slot_layer(slot);
  wire [31:0] sreg   = lreg[sl];
  wire        s_take = sreg[31] && (pri[sl] == slot[2]);
  wire        s_rs   = SCROLL_SM ? sreg[15] : 1'b1;
`ifdef TG_FAULT_RSSWAP
  wire [8:0]  yy     = ly;
`else
  wire [8:0]  yy     = RS_SWAP ? {ly[8:1], ~ly[0]} : ly;
`endif

  // ------------------------------------------------------- set-up arithmetic
  wire [31:0] rreg   = lreg[ll];
  logic [8:0] su_off;
  logic [9:0] su_xlo, su_xhi;
  logic       su_empty;
  wire [10:0] xlo_neg = 11'd1024 - {1'b0, rs16[8:0], 1'b0};
  always_comb begin
    su_xlo = 10'd0; su_xhi = 10'd495; su_empty = 1'b0;
    if (SCROLL_SM) begin
      su_off = rreg[15] ? rs16[8:0] : rreg[8:0];
    end else begin
      // MAME: rs = rs16 & 0x7fff, sign-extended from bit 8 when it is set; map x = x + sx + 2 rs, drawn
      // where 0 <= x + 2 rs <= 495.
      su_off = rreg[8:0] + {rs16[7:0], 1'b0};
      if (rs16[8]) begin
        su_xlo   = xlo_neg[9:0];
        su_empty = (su_xlo > 10'd495);
      end else begin
        su_empty = ({rs16[14:0], 1'b0} > 16'd495);
        su_xhi   = 10'd495 - {rs16[8:0], 1'b0};
      end
    end
  end

  // --------------------------------------------------------- the chunk rule
  wire [8:0] mapx    = x[8:0] + off;
  wire [3:0] n_tile  = 4'd8 - {1'b0, mapx[2:0]};
  wire [5:0] n_span  = 6'd32 - {1'b0, x[4:0]};
  wire [9:0] n_end   = xhi - x + 10'd1;
  logic [3:0] n_ch;
  always_comb begin
    n_ch = n_tile;
    if ({2'b00, n_ch} > n_span) n_ch = n_span[3:0];
    if ({6'd0, n_ch} > n_end)   n_ch = n_end[3:0];
  end
  wire [3:0] xs      = ~x[8:5];                  // 15 - span: the screen-space stencil bit
`ifdef TG_FAULT_STENCIL
  wire       m_scr   = pmask[x[8:5]];            // fault: spans counted from the wrong end
`else
  wire       m_scr   = pmask[xs];
`endif
`ifdef TG_FAULT_STENCIL
  wire       m_map   = pmask[~mapx[8:5]];
`else
  wire       m_map   = pmask[mapx[8:5]];
`endif
`ifdef TG_FAULT_PAIR
  wire       ch_en   = (MASK_SCREEN ? m_scr : m_map);          // fault: the odd layer not inverted
`else
  wire       ch_en   = (MASK_SCREEN ? m_scr : m_map) ^ odd;
`endif
  wire       m_next  = pmask[xs - 4'd1];         // the span after this one (only read when x[8:5] != 15)
  wire [5:0] s_sum   = {1'b0, x[4:0]} + {2'b00, off[3:0]};
`ifdef TG_FAULT_STALE
  wire       stale   = 1'b0;
`else
  wire       stale   = STALE && (x[8:5] != 4'd15) && (m_scr != m_next) && (off[3:0] != 4'd0) && s_sum[5];
`endif
  wire [8:0] srcx    = stale ? (mapx - 9'd16) : mapx;

  // ------------------------------------------------------------- the pattern
  wire [15:0] t_sel  = nbuf[{csrc[4], ~csrc[3], 4'd0} +: 16];
  wire [2:0]  row    = my[2:0];
  logic [14:0] code4;
  logic [13:0] code8;
  always_comb begin
    code4 = {t_sel[13:0], t_sel[15]};
    code8 = t_sel[13:0];
    if (CODE_WRAP && (code4[14:10] == 5'h1F)) code4[14:10] = 5'd0;
    if (CODE_WRAP && (code8[13:9] == 5'h1F))  code8[13:9]  = 5'd0;
  end
  wire [16:0] pa_n   = is4 ? {code4, row[2:1]} : {code8, row};

  // the row buffer's ports: read at the chunk being looked up (S_CHUNK) or the one just filled (S_NLK, from csrc)
  assign nra = (st == S_CHUNK) ? {ll, srcx[8:5]} : {ll, csrc[8:5]};
  assign nwe = (st == S_NREQ) && (p_lo || p_ack);
  assign nwa = {ll, csrc[8:6], p_ack};

  // -------------------------------------------------------------- the queue
  localparam int QW = 9 + 4 + 3 + 64 + 11 + 4;
  // Two entries in registers: left to itself Quartus 17 put this 2 x 94 array in 3 M10K (first fit).
  (* ramstyle = "logic" *) logic [QW-1:0] qe [2];
  logic [1:0]    qv;
  logic          qh;            // head index
  wire           q_full = qv[0] && qv[1];
  wire [QW-1:0]  qhead  = qe[qh];
  assign q_valid = qv[qh];
  assign {q_x, q_n, q_col, q_data, q_palhi, q_is4, q_above, q_pair} = qhead[QW-1:1];
  // Queue a chunk: the pattern burst (4-bit: the one row it needs, in the low half) and the name word's
  // palette bits. Three places push: S_PATA on a pattern already held, S_PREQ in the acknowledge cycle,
  // S_PUSH when the queue was full.
  logic          pu_en;
  logic [63:0]   pu_d;
  logic          pu_r0;
  logic [15:0]   pu_t;
`ifdef TG_FAULT_ACK
  wire           p_take = 1'b1;     // fault: data taken the clock after the request, acknowledged or not
`else
  wire           p_take = p_ack;
`endif
  wire           p_hit  = pval && (ptag == pa_n);
  always_comb begin
    pu_en = 1'b0; pu_d = pbuf; pu_r0 = prow0; pu_t = tw;
    case (st)
      S_PATA: begin pu_en = p_hit && !q_full; pu_r0 = row[0]; pu_t = t_sel; end
      S_PREQ: begin pu_en = p_take && !q_full; pu_d = p_dout; end
      S_PUSH: pu_en = !q_full;
      default: ;
    endcase
  end
  wire [QW-1:0] pu_e = {cx, cn, csrc[2:0], (is4 ? {32'd0, (pu_r0 ? pu_d[63:32] : pu_d[31:0])} : pu_d),
                        pu_t[14:4], is4, above, pair, 1'b0};

  wire restart = (go || go_pend) && (!p_req || p_ack);

  assign busy = (st != S_IDLE) || go_pend;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      st <= S_IDLE; p_req <= 1'b0; p_long <= 1'b0; p_addr <= '0; go_pend <= 1'b0; go_y <= '0; ly <= '0;
      gv <= '0; rtag <= '0; nf_ok <= 1'b0;
      qv <= 2'b00; qh <= 1'b0; pval <= 1'b0; flush <= 1'b0; overrun <= 1'b0; n_req <= '0;
      slot <= '0; ll <= '0; x <= '0; cx <= '0; cn <= '0; csrc <= '0;
    end else begin
      flush   <= 1'b0;
      overrun <= 1'b0;
      // the name row buffer: emptied each frame and on a name table write; a fill's group marked held
      if (inv) nf_ok <= 1'b0;
      if (inv || (go && y == 9'd0)) gv <= '0;
      else if (st == S_NREQ && p_ack && !restart && nf_ok) begin
        if (rtag[ll] != my[8:3]) gv[ll] <= 8'd1 << csrc[8:6];
        else gv[ll][csrc[8:6]] <= 1'b1;
        rtag[ll] <= my[8:3];
      end
      if (go) begin
        go_y <= y;
        if (busy) begin
          overrun <= 1'b1;
          flush   <= 1'b1;
          qv      <= 2'b00;
        end
      end
      // the queue: pop at the head, push into the free entry (never the head while it is full)
      if (q_pop && qv[qh]) begin
        qv[qh] <= 1'b0;
        qh     <= ~qh;
      end
      if (pu_en && !restart) begin
        if (!qv[qh]) begin qe[qh] <= pu_e; qv[qh] <= 1'b1; end
        else begin qe[~qh] <= pu_e; qv[~qh] <= 1'b1; end
      end

      if (restart) begin
        // A new line. A request still waiting is finished first (restart waits for its acknowledge), so no
        // late acknowledge is ever taken as the new line's data (R73).
        go_pend <= 1'b0;
        ly      <= go ? y : go_y;
        p_req   <= 1'b0;
        p_long  <= 1'b0;
        pval    <= 1'b0;
        n_req   <= '0;
        st      <= S_MREQ;
      end else begin
        if (go) go_pend <= 1'b1;
        case (st)
          S_IDLE: ;

          S_MREQ: begin
            if (!p_req) begin
              p_req  <= 1'b1;
              p_addr <= VBASE + wa({8'hF7, 1'b0, ly[8:1], 2'b00});
              n_req  <= n_req + 16'd1;
            end else if (p_ack) begin
              p_req  <= 1'b0;
              mask32 <= ly[0] ? p_dout[63:32] : p_dout[31:0];
              slot   <= 3'd0;
              st     <= S_SLOT;
            end
          end

          S_SLOT: begin
            ll <= sl;
            if (s_take) st <= s_rs ? S_RSREQ : S_SETUP;
            else if (slot == 3'd7) st <= S_DRAIN;
            else slot <= slot + 3'd1;
          end

          S_RSREQ: begin
            if (!p_req) begin
              p_req  <= 1'b1;
              p_addr <= VBASE + wa({8'hF6, ll, yy[8:2], 2'b00});
              n_req  <= n_req + 16'd1;
            end else if (p_ack) begin
              p_req <= 1'b0;
              rs16  <= p_dout[{yy[1:0], 4'd0} +: 16];
              st    <= S_SETUP;
            end
          end

          S_SETUP: begin
`ifdef TG_FAULT_SCROLL
            off   <= su_off + 9'd1;
`elsif TG_FAULT_ROWSCROLL
            off   <= SCROLL_SM ? rreg[8:0] : rreg[8:0];
`else
            off   <= su_off;
`endif
            xhi   <= su_xhi;
            x     <= su_xlo;
            my    <= ly + rreg[24:16];
            pmask <= ll[1] ? mask32[15:0] : mask32[31:16];
            odd   <= ll[0];
            is4   <= pri[4 + ll];
`ifdef TG_FAULT_PRIO
            above <= pri[ll ^ 2'd1];
`else
            above <= pri[ll];
`endif
            pair  <= ll[1];
            if (su_empty) begin
              if (slot == 3'd7) st <= S_DRAIN;
              else begin slot <= slot + 3'd1; st <= S_SLOT; end
            end else st <= S_CHUNK;
          end

          // A chunk is evaluated, its name word looked up and, on a miss, requested in one clock; its pattern
          // in the next; a fetched pattern goes into the queue in its acknowledge cycle. So a tile costs
          // three clocks plus the memory's latency, and a tile already held costs two.
          S_CHUNK: begin
            if (x > xhi) begin
              if (slot == 3'd7) st <= S_DRAIN;
              else begin slot <= slot + 3'd1; st <= S_SLOT; end
            end else begin
              cx   <= x[8:0];
              cn   <= n_ch;
              csrc <= srcx;
              x    <= x + {6'd0, n_ch};
              if (ch_en) begin
                // the row buffer is read at {ll, srcx[8:5]} this clock (nra), so a held name is in nbuf in S_PATA
                if ((rtag[ll] == my[8:3]) && gv[ll][srcx[8:6]] && !inv) st <= S_PATA;
                else begin
                  p_req  <= 1'b1;
                  p_long <= 1'b1;
                  nf_ok  <= !inv;
                  p_addr <= VBASE + wa({5'h1F, ll, my[8:3], srcx[8:6], 3'b000});
                  n_req  <= n_req + 16'd1;
                  st     <= S_NREQ;
                end
              end
            end
          end

          // an eight-word name read: words 0-3 (p_lo) and 4-7 (p_ack) go into the row buffer (nwe)
          S_NREQ: begin
            if (p_ack) begin
              p_req  <= 1'b0;
              p_long <= 1'b0;
              st     <= S_NLK;
            end
          end

          // the chunk's entry is read from the row buffer (nra), for S_PATA
          S_NLK: st <= S_PATA;

          S_PATA: begin
            tw    <= t_sel;
            pa    <= pa_n;
            prow0 <= row[0];
            if (p_hit) begin
              st <= q_full ? S_PUSH : S_CHUNK;
            end else begin
              p_req  <= 1'b1;
              p_addr <= VBASE + wa({pa_n, 2'b00});
              n_req  <= n_req + 16'd1;
              st     <= S_PREQ;
            end
          end

          S_PREQ: begin
            if (p_take) begin
              p_req <= 1'b0;
              pbuf  <= p_dout;
              ptag  <= pa;
              pval  <= 1'b1;
              st    <= q_full ? S_PUSH : S_CHUNK;
            end
          end

          S_PUSH: begin
            if (!q_full) st <= S_CHUNK;
          end

          S_DRAIN: begin
            if (qv == 2'b00) st <= S_IDLE;
          end

          default: st <= S_IDLE;
        endcase
      end
    end
  end

endmodule
