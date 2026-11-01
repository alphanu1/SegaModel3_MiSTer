# Tile generator (M0-E) — notes

My working notes for the standalone tile generator: what each emulator does, where
they disagree, what the RTL is built to, and every wrong turn. Study sections: §5.4
(the block), §7.4 (M10K), §8.4 (area techniques), D-6 (the 3D layer composites with
the tilemap), D-19 / D-25 (Supermodel over MAME, PC-only additions out), D-27 (the
"everything else" budget, 3,500 ALM, of which the tile generator is 1,100–2,000).

## Files

| File | What |
|---|---|
| `rtl/video/m3_tilegen.sv` | Top: registers, palette (M10K), emitter, two line buffers, scanout and colour |
| `rtl/video/m3_tilegen_fetch.sv` | One fetch engine for all four layers, on one `m2_sdram`-contract port |
| `rtl/real3d/r3d_ram.sv` | Reused for every memory (palette, line buffers) |
| `sim/tilegen/tg_ref.h` | Reference renderers: MAME literal, Supermodel literal, and the switchable `param` model the RTL is built to |
| `sim/tilegen/tg_tool.cpp` | `check` (references against each emulator's own output), `xdiff` (MAME against Supermodel, per switch), `info`, `ppm` |
| `sim/tilegen/tb_tilegen.cpp` | Verilator bench: hostile memory model, lockstep / real-time / recovery modes, synthetic variants |
| `sim/tilegen/faults.py` | Planted faults |
| `quartus/tilegen/` | Standalone fit (wrapper, sdc, `fit.sh`, `batch.sh`, `summary.sh`, `cf_top.sv` constant-fold check) |
| `tools/mame-census/mame0289-tilegen-dump.patch`, `supermodel-tilegen-dump.patch` | The dump instrumentation (README, "Tile generator dumps") |

Make targets: `test_tilegen_ref`, `test_tilegen`, `faults_tilegen`, `lint_tilegen`.

## Dumps (`sm3-builds/tilegen/`, git-ignored)

Each dump is `fNNNNNN.vram` (F1000000–F111FFFF, 0x120000 bytes, bus byte order) and
`fNNNNNN.regs` (F1180000–FF, bus order), plus the emulator's own picture of it.

| Folder | Source | Frames | Notes |
|---|---|---|---|
| `mame_vf3` | MAME 0.289 + dump patch, VF3, every 60 frames 600–9,600 | 151 | Attract (one coin gives "1/2", so no fight). `.tile` = MAME's own `draw_layer` output |
| `mame_vf3b` | as above, two coins, every 90 frames 5,700–15,000 | 104 | Fights: HUD, timer, scrolling stage backdrop |
| `sm_vf3` | Supermodel 036af1c + dump patch, every 150 frames 300–8,550 | 56 | `.surf` = Supermodel's own draw surfaces |
| `sm_bassdx` | Supermodel, Bass Fishing Deluxe attract, every 150 frames 300–11,850 | 78 | MAME cannot run Bass Fishing (I/O board error) |

What the games use, over all 389 dumps:
- VF3: all four layers enabled in every frame, 8 bits a pixel, layers 0 and 2 above the
  3D (`pri` 05 in 194 frames). Layer 3 scrolls slowly (X 0–6, Y 0–7).
- Bass Fishing: 4-bit layers (layers 0, 1 and later 2), large scrolls on layer 2
  (X 0x157–0x1F5, Y up to 0x1F8), and **colour offsets** (F1180040/44 = 0x7C7C7C, 0x707070,
  0x545454, 0x1C1C1C: the fade to white on the "Winners don't use drugs" screen).
- Line scroll: never. Every table is 0x8000 on every line and no register has bit 15 set.
- Stencil: one word for the whole frame in 256 dumps, two different words in 29 (VF3's
  character-portrait wipes, e.g. `FFFF07FF` / `FFFFFFE0`, and Bass Fishing's `0000FFFF`).

Features the games leave idle (line scroll, odd stencils, codes past the pattern area,
4-bit on every layer, random priorities) are covered by synthetic variants
(`tg::mutate`): the game's patterns, names and palette with random registers, line-scroll
tables (constant, triangle, random 16-bit, small signed with random bit 15), stencils
(constant, random per line, banded) and, in 30% of them, one layer's name table random.

## The references agree with the emulators exactly

`tg_tool check`, all dumps:
- MAME literal (`mame_literal`, statement by statement on MAME's own host-memory layout,
  the `gfx_element` bit decode and `pal5bit` pens) against MAME's `draw_layer` output:
  **0 differing pixels in 255 frames** (151 attract, 104 fights), both passes.
- Supermodel literal (`sm_literal`, `DrawLine` and `GetColour32` as written) against
  Supermodel's draw surfaces: **0 differing pixels in 134 frames**, both surfaces. In 8
  of them VRAM changed during the active display and in 71 a register did (the IRQ
  acknowledge); none changed a pixel.
- `param` with MAME's switches against the MAME literal, and with Supermodel's against
  the Supermodel literal: 0 differing pixels in every frame.

## MAME against Supermodel

Seven points of difference, each a switch in `tg::Cfg` and a parameter of the RTL. D-19
(extended to this block) puts Supermodel first except for PC-only
additions, so the RTL defaults to Supermodel and MAME's form stays a compile-time choice.

| # | Point | MAME 0.289 (`draw_layer`) | Supermodel 036af1c (`DrawLine`) | Pixels on the real dumps |
|---|---|---|---|---|
| 1 | Stencil coordinate and bit order | map space (after scroll), bit 0 = leftmost 32 pixels | screen space (fixed), bit 15 = leftmost | 1,792,811 px in 29 frames |
| 2 | Pair exclusivity | each layer masked on its own scrolled x: both layers of a pair can draw one pixel | one layer of the pair per screen pixel | (part of 1) |
| 3 | Line scroll | always applied: `rs & 0x7fff`, sign from bit 8 only, `x - 2*rs`, a clipped band left blank | only when the layer register's bit 15 is set; the table value replaces X scroll | 0 (unused) |
| 4 | Line-scroll word for line y | halfword at 2y | halfword at 2(y ^ 1) | 0 (unused) |
| 5 | Tile pair straddling a stencil change | normal | the pixels before the change take the pair 16 pixels to the left (`maskSwitch`) | 1,692–3,445 px |
| 6 | Pattern codes past 0xF8000 | wrap (`code % 31744` 4-bit, `% 15872` 8-bit: `gfx_element` count) | read the name-table area directly | 0 |
| 7 | 5 to 8 bits | `pal5bit`: (x << 3) \| (x >> 2) | x * 255 / 31; differ by 1 at 16 of 32 values | 39.7 M px (all frames) |
| 8 | Colour offset F1180040 (A/A'), F1180044 (B/B') | stored, unused (the "flash effect" TODO) | signed 8-bit per channel, x2, added after expansion, clamped | 760,256 px (Bass Fishing fades) |
| — | Draw order within a pair | above: 1 over 0; below: 0 over 1 | irrelevant (pairs exclusive) | 0 |
| — | Register bits | X 9 bits | X 10 bits (wraps at 512 anyway) | 0 |

The totals over 285 dumps: 40.4 M pixels differ in colour, 1.79 M by palette entry
(drawn / class / entry). By entry the stencil is the whole of it.

Which is the board, as far as I can tell without footage:
- **Stencil (1, 2).** Supermodel's own notes say the tile RAM (Samsung KM4132G271A, 32 MHz)
  is busy for 28 of every 32 clocks per 16 pixels, fetching "the next tile pair of layers
  A and B". That is one layer of each pair per 16 pixels: hardware that fetches like that
  cannot draw both layers of a pair at one pixel, which MAME's model needs whenever the
  two layers' scrolls differ. That favours Supermodel's screen-space exclusive stencil.
  The bit order (bit 15 leftmost) I cannot confirm: on VF3's character-portrait wipes
  (`mame_vf3/f002040`, `f009360`) the two emulators show mirror-image splits and neither
  looks obviously right in a still. **Needs cabinet footage of VF3's attract portraits.**
  *Since settled for the bit order by cabinet footage: bit 15 is leftmost, as Supermodel
  has it. Coordinate and pair form are still open. See "The stencil against cabinet
  footage" below.*
- **Line scroll (3, 4).** Nothing in VF3 or Bass Fishing uses it, so neither form is
  tested by a game. Supermodel documents the register's bit 15 as "line scroll enable";
  MAME's x2 has no comment. Supermodel's form is the default; Scud Race and Lost World
  would decide it (study §15 should list them as the games to dump).
- **Stale pair (5).** Supermodel models the fetch above (tile pairs at a fixed 16-pixel
  cadence); its comment says it reproduces the board. 1,692–3,445 pixels on VF3's wipes.
- **Colour offset (8).** A hardware effect (Bass Fishing's fades, Lost World's lightning
  per Supermodel), not a PC addition: built (`OFFSET = 1`), about 40 ALM. MAME's
  "flash effect" TODO is this register pair.
- **5 to 8 bits (7).** Both are emulator choices; the board's DAC is not documented.
  Supermodel's is the default to match its images; `pal5bit` is a switch.

## The stencil against cabinet footage (VF3 attract, SEGA logo to Great Wall)

**Result:** the board puts stencil bit 15 on the leftmost 32-pixel span, as Supermodel does,
and not bit 0 as MAME does. The RTL default (`MASK_SCREEN = 1`) already does this, so the RTL is
unchanged. This wipe cannot tell screen space from map space, or an exclusive pair from a
per-layer test.

**Footage.** YouTube kiv9COZRsys, a VF3 cabinet in attract mode, captured at 1280 x 720,
60 fps (`sm3-builds/footage/vf3_wipe/`, frames `w001..w096`). The 496 x 384 picture occupies
columns 140–1150. From the clean frames I fitted the geometry:
- **Horizontal:** column = 140.375 + 2.0365 x.
- **Vertical:** row = −1.13 + 1.883 n, where n is the line and row is the continuous
  coordinate (pixel r covers [r, r+1)). The scale comes from the period of the wipe's line
  stripes (3.766 rows for two lines, the same in 8 frames). The offset comes from 62
  horizontal edges of the SEGA logo against the tile render of `mame_wipe/f001454` (rms
  0.19 rows). At 1.883 rows a line, the capture shows lines 0 to about 381.

**What the game does.** I dumped every frame across the transition: MAME frames 1300–1560
(`sm3-builds/tilegen/mame_wipe`) and Supermodel frames 1150–1400 (`sm_wipe`). Both run the
same 16-step wipe, MAME at frames 1455–1470 and Supermodel at 1275–1290:
- **Pair A:** the stencil word stays FFFF.
- **Pair B:** it alternates by line. In step k (1–16), even lines have `FFFF >> k` and odd
  lines `FFFF << k`. In step 5 the words are `FFFF07FF` / `FFFFFFE0`.
- **Layers:** layer 2 (the white logo screen, above the 3D, X scroll 0) shows where the bit
  is 1. Layer 3 (below the 3D, which covers it) shows where the bit is 0.

So each line clears one more span a frame from one side: even lines from one edge, odd lines
from the other. The footage's "ghost" is not a translucent layer: it is every other line
cleared.

The character-portrait frames that showed the mirror-image split are the same wipe:
- `mame_vf3/f002040` is step 5 and `f009360` is step 8, with the line parities swapped
  (line 0 has `FFFFFFE0`);
- every two-word VF3 stencil in the dumps is this wipe.

The literals match their emulators with 0 differing pixels on all 512 new dumps (`tg_tool
check`). The RTL matches `tg::param` with 0 differing pixels on the 38 wipe frames in both
builds, and behind the 1–400-clock hostile memory.

**What the footage shows,** matched by wipe step. The cabinet runs one span a game frame.
The 60 fps capture repeats one frame (w063 = w064), so step k is w063 for k = 1 and w(063 + k)
for k ≥ 2:

| | Cabinet | MAME | Supermodel |
|---|---|---|---|
| Span grid | 32 px from screen x = 0: the first left band is 32 px wide, the first right band 16 px (span 15 is cut at 496) | same | same |
| Where the white closes (k = 8) | x = 256, not the screen centre 248 | 256 | 256 |
| Order | both edges together, one span per frame per line parity; clear from the centre outward from k = 9; all clear at k = 16 | same | same |
| Which lines clear from the left | **even lines** | odd lines | **even lines** |

The last row decides the bit order. I put the white-line pattern of each hypothesis on the
registered line grid and correlated it with the detrended row profile of the ghost columns
(span 0 and span 15). Then I averaged the left and right sides (`scripts/verdict.py`):

| Rows (lines) | k = 2–3 (w065–66) | k = 5–15 (w068–78) |
|---|---|---|
| 4–110 (0–58), sky | Supermodel +0.41 to +0.48 / MAME the negative | Supermodel +0.72 to +0.76 |
| 110–250 | +0.27 to +0.32 | +0.50 to +0.52 |
| 250–400 | +0.30 to +0.31 | +0.44 to +0.49 |
| 400–560 (little contrast) | +0.06 to +0.07 | +0.08 to +0.09 |

Every band and every frame has the same sign. The Supermodel score stays positive while the
line grid's offset lies anywhere from −1.95 to 0.0 rows; the registration puts it at −1.13 ±
about 0.03. A free fit of the stripe phase alone gives −1.00 (left side) and +0.88 (right
side, which is −1.13 + one line). So the two sides agree with each other and with the logo
edges.

**Which forms are distinguishable here** (`sm3-builds/tilegen/bin/tg_wipe`, coverage of
above-3D tile pixels, 38 frames):
- **b15, screen space** (Supermodel's form): 0 pixels from the Supermodel literal.
- **b15, map space, exclusive or per layer:** also 0 from the Supermodel literal.
- **b0, screen space; b0, map space, exclusive or per layer:** 0 from the MAME literal.
- **The two literals:** up to 185,154 pixels apart (k = 8).

Layer 2 has X scroll 0, and layer 3 (scroll 0–7) is under the 3D. So coordinate and pair form
cannot show on this wipe.

**One thing no form reproduces.** On the lines that clear from the right (odd lines), the
white run ends early:
- **Odd lines:** the white run ends 2.4–3.1 px before the span boundary in phase 1 (k = 4–7)
  and 4.9–6.3 px before it in phase 2 (k = 9–14).
- **Even lines:** the change from clear to white sits on the boundary (−1.6 to +1.7 px).
- **At k = 8:** this leaves a fully clear column about x = 250–257 in w071. It is visible
  top to bottom in the frame, and neither emulator draws it.
- **Not the capture:** vertical logo edges measured on even lines and on odd lines separately
  sit at −0.25 and −0.19 px.
- **Not layer 3's scroll:** the shift does not follow it; it is −5 px at k = 10 and 11, where
  both emulators have layer 3 at X 0. The cabinet's layer-3 jitter may differ from the
  emulators', though.

Unexplained and not modelled. It is about 5 px x 192 lines a frame for the 16 frames of a
wipe.

**What remains open, and what footage would settle it:**
- **One assumption is left:** stencil word y applies to line y, the same y as the tile
  content. Both emulators assume it. If the board read the word one line off, this
  footage would mean bit 0 instead. Bass Fishing's line-level stencils would settle it with
  line-resolution footage of those screens:
  - pair A `0000FFFF` on lines 0–247 (f004200) or 128–319 (f007800);
  - single-line interleaves on lines 8–16 and 63–71 (f011700).
- **Map against screen space, and exclusive against per-layer:** these need a stencil
  boundary across a visible layer with a non-zero X scroll. Nothing dumped does that
  (§15 R77: Scud Race or Lost World).

**Tools:**
- **`sim/tilegen/tg_wipe.cpp`:** the stencil split into bit order, coordinate and pair form,
  self-checked against `param()`. Built by hand:
  `g++ -O2 -std=c++17 -o sm3-builds/tilegen/bin/tg_wipe sim/tilegen/tg_wipe.cpp`.
- **Footage scripts** in `sm3-builds/footage/vf3_wipe/scripts/`: `register.py` / `edges.py`
  (geometry), `stripes.py` / `verdict.py` (parity), `seams.py` (per-parity transitions),
  `sidebyside.py` (comparison sheets in `sm3-builds/tilegen/wipe_out/cmp/`).

**Wrong turns:**
1. **I read the ghost as a translucent tile layer.** It is the line interleave of the
   stencil; the footage's horizontal stripes are the lines.
2. **I took the footage's active area as the picture.** I first scaled 384 lines into the
   718 lit rows (1.870 rows a line). The stripe period says 1.883, so the capture crops the
   last two or three lines.
3. **A sine fit of the stripe phase first put the white lines a quarter period from both
   hypotheses.** I had the sign of the phase wrong (a peak at y0 gives angle −2πy0/P, not
   +). With the sign fixed, and with the sample-centre half row, it lands on the odd lines
   (Supermodel).
4. **Whole-band sine fits mixed in rows with no contrast.** The wall and ground regions are
   about as bright as the white, and the video codec removes their stripes. The band-by-band
   correlation shows where the signal is: the sky, rows 4–110.
5. **A first logo registration fitted scale and offset together** on edges from lines
   130–260 only. That gave 1.878 rows a line and, extrapolated to the sky, an offset about
   half a row off. With the scale fixed from the stripes, only the offset is fitted.
6. **The early frames (w063–w066) are soft vertically.** There the even-line and odd-line
   profiles both show both transitions, so I measured the per-parity seams from w067 on.

## Reuse from Model 2, and where Model 3 differs

I read Model 2's tilemap first (`m2_tile_fetch`, `m2_tile_decode`, `m2_tile_mixer`,
`m2_palette`, `m2_char_cache`, `m2_scan15`, R72, R269, R313, R823, R906, R922 and their
commits) and copied none of it: Model 2's tile chip is System 24's, and Model 3's is a
different design. What carried over is the structure and two lessons.

| | Model 2 (System 24 tilemap) | Model 3 | Consequence |
|---|---|---|---|
| Priority | per tile: a category bit in the tile word; MAME draws each map twice and the 3D sits between the passes (R922 fixes the pair order) | per layer: bit l of F1180020 | no per-pixel category; `m2_tile_mixer` does not apply |
| Bits a pixel | 4 | 4 or 8 per layer (F1180020 bits 15:12) | the emitter decodes both |
| Memory | 64 KB tile RAM on chip, 512 KB character RAM in SDRAM | one 1 MiB VRAM: patterns, line scroll, stencil and name tables | name tables come from SDRAM too, through the same port |
| Masking | row mask in 8-pixel columns plus window modes (column split, vertical split) | 32-pixel stencil spans, the two layers of a pair exclusive | Model 2's R906 skip of masked columns becomes "skip hidden chunks", always on |
| Palette | colour-translation RAM and a gamma table (`m2_palette`) | direct 1-5-5-5 with a suppress bit, plus a per-pair colour offset | `m2_palette` does not apply; 32 K entries instead of 4 K |
| Clocks | char fetch on `clk_mem`, a 16-entry glyph cache to hide the crossing (R269, R313) | one 100 MHz clock with the SDRAM (D-28) | a one-entry name buffer and one-entry pattern buffer instead of the cache |
| Line banks | four as a ring (R823: overruns are miss bursts) | two | measured margin instead (below); a ring is the fallback if the full core shows overruns |
| Engines | one `m2_tile_fetch` per layer | one for all four | §8.4 "built once and shared" |

`m2_scan15` is the 15 kHz output path, not part of the tile generator; it belongs with
the video output when the full core is built.

## The RTL

One clock, 100 MHz. The pixel clock is 16 MHz (32 MHz / 2, `set_raw(32 MHz / 2, 656, 0,
496, 424, 0, 384)`), so a line is 656 x 6.25 = 4,100 clocks. The SDRAM port and the Real3D
are on the same 100 MHz (D-28), so there is no crossing.

- **Line buffers, not per-pixel fetch.** A line is built one line ahead into one of two
  512 x 18 line buffers ({drawn, above, pair, colour 15}); the scanout reads one and clears
  it behind itself while the next line is built into the other. (Model 2: `m2_tile_fetch`
  + four line banks, R823.)
- **One fetch engine for four layers** (§8.4 "built once and shared"). It reads the
  stencil word, then each enabled layer in draw order (below: 3, 2, 1, 0; above: 3, 2, 0,
  1), and splits each layer's line into chunks of at most 8 pixels that come from one row
  of one tile: a chunk ends at a map-tile boundary, a 32-pixel screen boundary and the end
  of the drawn range. Chunks the stencil hides are skipped without a fetch (Model 2's
  R906 `MASK_SKIP`, here always on): with pairs exclusive, a line costs about two layers
  of fetches however many are enabled. Model 2 had one `m2_tile_fetch` per layer.
- **Later writes win.** Because passes run in draw order, the emitter writes every opaque
  pixel and never reads the line buffer back. One colour and one above/below bit per pixel
  is all the compositor needs (D-6): an above pixel covers everything, a below pixel shows
  only where the 3D does not.
- **Palette suppress** is tested when the pixel is written, so a suppressed pixel of a
  later layer leaves the earlier one showing, as in both emulators.
- **Memory.** The 1 MiB is a byte image in SDRAM (16-bit words little-endian, word w =
  bytes 2w, 2w+1), so the CPU bridge stores the PowerPC's words byte-reversed, as
  Supermodel does. Every read is one 4-word burst; a one-entry name buffer (4 tiles) and a
  one-entry pattern buffer catch a tile split at a stencil boundary and the four name
  words of a burst. Request held to acknowledge, data only in the acknowledge cycle, a new
  line waits for an outstanding acknowledge before it starts (R73).
- **Palette** 32 K x 16 in M10K as two memories (15 colour bits, 1 suppress bit).

### Throughput

At 2-clock latency the slowest line of all dumps and synthetic variants takes 1,163
clocks (mean 663–1,006), at most 178 requests. Real time (a line every 4,100 clocks, go
and swap on time, pixels at 16/100 of the clock), Supermodel build, all 389 real dumps,
149,376 lines:

| Acknowledge latency | Slowest line | Mean | Overruns |
|---|---|---|---|
| 14 clocks | 2,620 | 1,607 | 0 |
| 20–24 | 3,913 | 2,055 | 0 |
| 24–28 | 4,098 (cap) | 2,336 | 1,630 (1.1%) |
| 28–32 | 4,098 (cap) | 2,609 | 4,694 (3.1%) |

The first FSM (wrong turn 4) took 3,196 clocks for the slowest line at 14 on the first
134 dumps, against 2,194 now. Model 2's measured burst on `m2_sdram` was 14.2 clocks, so at
that latency a line has about 1,480 clocks (36%) left for other clients' bursts; the block
holds up to a mean request latency of about 22 clocks. An overrun abandons the rest of that
line only (recovery runs above). If the full core shows overruns, the levers in order of
cost: eight-word name bursts (`p_long`, about 16% fewer requests), then Model 2's ring of
line banks (R823) to absorb bursts across lines.

## Bench results

All against `tg::param` under the build's switches; every pixel compared (drawn, above,
24-bit colour), 496 x 384 a frame.

Final RTL (after the palette and queue changes of wrong turns 7 and 8):

| Run | Build | Frames | Differing pixels | Protocol errors |
|---|---|---|---|---|
| Real dumps + 2 synthetic variants each, 2-clock memory: `sm_vf3`, `sm_bassdx`, `mame_vf3b` | Supermodel switches | 714 | 0 | 0 |
| same: `mame_vf3`, `mame_vf3b` | MAME switches | 765 | 0 | 0 |
| Hostile: acknowledge 1–400 clocks, 30% of requests a further 50–400, 25 dumps a folder + 1 variant | Supermodel | 150 | 0 | 0 |
| same | MAME | 100 | 0 | 0 |
| Recovery: real time behind the 1–400 memory (all 11,520 lines overrun and are abandoned), then the same frame line by line | Supermodel | 30 | 0 | 0 |

Before those changes (same logic): hostile 1–400 + 1 variant on all real dumps, 268 frames
(Supermodel) and 302 (MAME), 0 pixels; 20–200 clocks with 10% further, 268 frames, 0;
recovery in the MAME build, 15 frames, 0.

The memory model asserts the `m2_sdram` contract: a request is a rising edge, held to its
acknowledge with a steady address, one at a time; data outside the acknowledge cycle is
random. Hostile runs build each line to completion (lockstep); the real-time runs are the
throughput test.

Planted faults (`sim/tilegen/faults.py`), each a `define in the RTL; first the first 12
real dumps per folder, then 2 synthetic variants of each if those miss:

| Fault | Supermodel build | MAME build |
|---|---|---|
| SCROLL: X offset + 1 | real dumps | real dumps |
| ROWSCROLL: line-scroll table ignored | synthetic (no game uses it) | synthetic |
| RSSWAP: line y's word at 2y, not 2(y ^ 1) | synthetic | — |
| STENCIL: span index from the wrong end | synthetic (the first 12 dumps have one stencil word each) | synthetic |
| PAIR: odd layer not inverted | real dumps | synthetic |
| STALE: no stale-pair rule | synthetic | — |
| BPP: 4-bit nibble order reversed | real dumps (Bass Fishing) | synthetic (VF3 is 8-bit) |
| SUPPRESS: palette bit 15 ignored | real dumps | real dumps |
| PRIO: above bit from the pair's other layer | real dumps | real dumps |
| OFFSET: colour offset not doubled | real dumps (Bass Fishing fades) | — |
| EXPAND: `pal5bit` in place of x * 255 / 31 | real dumps | — |
| ACK: pattern data taken the clock after the request | real dumps (protocol errors and pixels) | real dumps |

**20 of 20 caught.**

## Wrong turns

1. **A Lua dump in stock MAME.** `model3_vid_reg_r` returns nothing for 0x60–0x6C, so the
   scroll registers cannot be read; a Lua write tap on the PowerPC's 64-bit space aborts
   MAME on the first write with bit 63 set ("integer value will be misrepresented in
   lua"); a debugger watchpoint feeding the console log works but runs VF3 at about 0.8
   frames a second (300 frames in 6 minutes). Replaced by a 67-line patch in a copy of the
   census tree, dumping from inside `screen_update_model3`; only `model3_v.cpp` rebuilt.
2. **Editing Supermodel's CRLF sources with Python's text mode** turned three files into
   7,000-line diffs. Rewritten with CRLF kept.
3. **`busy` fell when the fetch engine stopped, not when the last pixel was written**, and
   the emitter took its line-buffer bank from a live register. In lockstep the first line
   of a frame was swapped and the next line started while the last chunk was still being
   written, so its last few pixels went into the other bank: 93 pixels in 34 of 200
   synthetic frames, all on line 0 near the right edge (x 437–494). Fixed: the bank is
   latched with each chunk, and `busy` covers the emitter pipeline.
4. **The first fetch FSM spent 7 clocks per tile besides the memory's latency** (chunk,
   name check, name select, pattern address, pattern compare, request, push): 3,196
   clocks for the slowest line at 14-clock latency and 125 overruns at 24–28. Folding the
   name check into the chunk clock, the pattern compare into the address clock and the
   push into the acknowledge clock brings it to 3 clocks per fetched tile and 2 per tile
   already held: 2,194 at 14 and no overruns at 24–28.
5. **A `task` that wrote the queue** from inside the `always_ff` drew Verilator's
   MULTIDRIVEN warning on `qv`; the push is now a combinational enable and one write in
   the clocked block.
6. **The stencil planted fault was invisible in MAME's configuration** because it only
   touched the screen-space bit; it now flips the span index in both.
7. **The palette took 60 M10K, not 48.** Left to itself Quartus 17 built 32 K x 15 from
   8 K x 1 slices (8,192 bits a block); only the x5 / x10 / x20 / x40 shapes use all
   10,240. The `max_depth = 2048` attribute on the one array was ignored (the map report
   shows `MAXIMUM_DEPTH 0`), so `m3_tilegen_pal` builds it as sixteen 2 K x 15 memories
   and a 16:1 read mux.
8. **The two-entry chunk queue went into 3 M10K** (2 x 94 bits, inferred as RAM). Now
   `ramstyle = "logic"`.
9. **My constant-fold expectation for the chunk length was wrong, not Quartus**: I wrote
   32 - 29 for x = 489 where x mod 32 is 9. Quartus's 5 is right; every other value
   matched (`quartus/tilegen/constfold.sh`: 1024 - 2 x 0x1A5 = 182, 495 - 2 x 0xB5 = 133,
   the colour clamp at 0 / 255 / 92 / 125, `pal5bit(16)` = 132, 5 - 16 mod 512 = 501).

## Area

Standalone fit, Quartus 17.0, 5CSEBA6U23I7, 10 ns (100 MHz), every port but `clk` a
virtual pin, the block one level down in `tg_fit_top` so the entity table gives it without
pins (`quartus/tilegen/batch.sh`, `summary.sh`). Supermodel switches (the default build).

| Seed | ALM, block without pins | ALM, whole fit | Registers | M10K | DSP | Fmax slow 100C | Fmax slow −40C |
|---|---|---|---|---|---|---|---|
| 1 | 553.0 | 677 | 940 | 54 | 0 | 132.86 MHz | 133.64 MHz |
| 2 | 560.0 | 684 | 939 | 54 | 0 | 135.41 MHz | 135.50 MHz |
| 3 | 557.5 | 682 | 942 | 54 | 0 | 130.84 MHz | 124.32 MHz |

Against D-27's 1,100–2,000 ALM estimate for the tile generator: 553–560 ALM, half the
low end, and 24–35 MHz of margin over 100 MHz at both slow corners. M10K: 54, of which the palette is 52 (48 colour, 4 suppress) and the line buffers 2,
against §7.4's 32 budgeted for the palette. **§7.4's 32 is wrong:** 32 K x 16 bits is
524,288 bits and an M10K holds at most 10,240, so 52 is the floor (at 32 M10K the palette
would have to drop to 20 K entries). The worst path at seed 1 is the name-word select into
the queue entry (`csrc` to `qe`, 2.47 ns slack), then the emitter's palette address.

First fit, before wrong turns 7 and 8 (seed 1): 495.5 ALM, 944 registers, **69 M10K**
(palette colour 60, suppress 4, line buffers 2, queue 3), 127.5 / 128.1 MHz.

### Model 2's area savings, applied (§8.4)

| Technique | Here | Measured / estimated |
|---|---|---|
| Built once and shared | One fetch engine walks all four layers in turn (Model 2: one `m2_tile_fetch` per layer); one emitter, one palette read port, one colour expander at the scanout | the engine is 299 ALM; four would be about 1,200, so about 900 ALM saved `[EST, not built]` |
| Storage out of flops | Palette (524,288 bits) and both line buffers (2 x 9,216) in M10K; per-layer state is not kept: one set of pass registers (offset, range, map row, stencil half) is loaded per pass instead of four | the pass registers are about 50 bits, x 4 avoided `[EST ~100 registers]` |
| MLAB / M10K trades | Palette rebuilt as sixteen 2 K x 15 memories so it packs as 2 K x 5: 60 → 48 M10K for a 16:1 read mux. The two-entry queue moved out of 3 M10K into registers | −15 M10K for +57.5 ALM (495.5 → 553.0, seed 1); the palette mux is 79 ALM against 11 |
| Narrower words | The line buffer holds the 15-bit palette colour, not 24-bit RGB: 18 bits fit one 512 x 20 M10K, 26 would need two. The colour expansion and offset run once, at the scanout. The queue carries one 64-bit burst per chunk, not per pixel | −2 M10K; one expander instead of one per write path |
| Skip what is hidden | Chunks the stencil hides are not fetched (Model 2's R906 `MASK_SKIP`, here unconditional) | about half the requests on VF3, where all four layers are on and each pair shows one |
| Reuse | `r3d_ram` for every memory | — |

What did not work: the `max_depth` attribute (ignored by Quartus 17, so the banks are
explicit); leaving the queue to inference (3 M10K for 188 bits). Not tried: the queue in
MLAB (5 MLABs for 94 bits x 2, likely no better than the 188 registers, which share ALMs
with logic, R952).
