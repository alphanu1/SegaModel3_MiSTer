# M3 working notes: the Real3D rasteriser

Scratch notes for M3: the golden images, the rasteriser model, and the architecture I
intend to build. They are not the study. Anything here that changes the study goes in
as an R-entry; section 7 lists the candidates.

Everything below is measured on the 400 VF3 and 400 Bass Fishing Deluxe attract dumps
that M2 uses (`sm3-builds/census/dumps_*`, every 30th frame from 1800 to 13770).

## 1. Golden data

### What was made

| Output | Where | How |
|---|---|---|
| MAME's 3D layer, per dump | `sm3-builds/rast/mame_{vf3,bassdx}/<dump>.3d` | `mame0289-real3d-frames.patch`, `M3FRAMES` |
| MAME's composite (tile layers + 3D) | `.../<dump>.full` | the same, `M3TILES`: Supermodel's tile RAM and registers for that frame, loaded for one `screen_update_model3` |
| MAME's triangle lists, same run | `sm3-builds/rast/tris_{vf3,bassdx}/` | `M3TRIS` in the same replay |
| Supermodel finished frames | `sm3-builds/rast/sm_*_full/grab/` | `supermodel-real3d-frames.patch`, `M3SHOT3D_LIST` |
| Supermodel 3D alone | `sm3-builds/rast/sm_*_3d/grab/` | the same with `M3NO2D` |
| Supermodel tile state | `sm3-builds/rast/sm_*_full/tiles/` | `M3TILE_DIR` |

Supermodel is deterministic from an empty `NVRAM/`: the fresh runs reproduced all 800
census dumps byte for byte, so the grabs, the tile state and the dumps are the same
frames. The run without tile layers (`M3NO2D`) saved tile state identical to the full
run's on all 800 frames, so its 3D-only grabs are the same frames too. All of it is
ROM-derived and stays in `sm3-builds/`.

The triangle lists had to come from the same MAME run as the images. MAME's undefined
vertex slot (`m2-notes.md` section 2) holds whatever the stack held: frame f009060 has
a fan whose two reused vertices carried u = 0.0046 and 0.0037 in the M2 golden run and
exactly 0 in the image run. Lists and images from one run agree.

### MAME against Supermodel

Per-pixel RGB over all 400 frames of each set:

| | VF3 | Bass Fishing |
|---|---|---|
| 3D coverage: MAME draws / Supermodel draws (not black) | 85.0% / 83.9% | 52.2% / 54.1% |
| 3D coverage disagrees | 4.47% of pixels | 4.47% |
| 3D colour where both draw: different / >8 / >32 | 99.9% / 87.6% / 64.2% | 100.0% / 97.0% / 89.4% |
| 3D layer PSNR, per frame p1 / p50 / p99 | 0.9 / 13.5 / 17.0 dB | 6.9 / 10.7 / 99 dB (99: both empty) |
| Finished frame PSNR, per frame p1 / p50 / p99 | 6.9 / 12.9 / 17.1 dB | 7.1 / 10.1 / 18.8 dB |
| Tile layers alone (no 3D in either), Supermodel moved 2 px right and down: different / >8 / >32 | 75.6% / 7.5% / 7.4% | 70.4% / 4.5% / 4.4% |

What the difference is, by eye on the PNGs (`sm3-builds/rast/png_*`):
- **The same frames, the same poses.** Supermodel's grab N and MAME's render of dump N
  show the same scene. `m2-notes.md` section 3 says Supermodel's frame did not show
  MAME's poses; on these grabs it does.
- **Lighting and colour.** Supermodel's VF3 floor is near white where MAME's is grey.
  Rows 18-22 of the `m2-notes.md` difference list (light vector, clamp, ambient /255,
  fixed shading) are the likely cause.
- **Fog.** Supermodel draws Bass Fishing's underwater scenes through a fog colour; MAME
  has no fog.
- **Translucency.** Bass Fishing's water surface is translucent with reflections in
  Supermodel and an opaque cyan sheet in MAME (row 17, transparency width).
- **Coplanar layers.** Where two water surfaces share a plane, MAME z-fights in stripes;
  Supermodel draws them cleanly (it implements the polygon header's layered bit).
- **Tile layers sit 2 px right and 2 px down in MAME** relative to Supermodel. With
  that offset applied, 93-96% of tile pixels are within 8 levels; the small differences
  are 5-bit palette expansion (MAME `x << 3 | x >> 2`, Supermodel `x * 255 / 31`).

**What "within tolerance of Supermodel" can mean.** Not a PSNR on the 3D: the two
references are 8-13 dB apart, so any tolerance that admits MAME admits almost anything.
Coverage is the part they agree on (95.5% of pixels). I propose M3's exit as three
separate checks: the rasteriser matches its own model bit for bit (RTL bench); the
model matches MAME's 3D layer within the arithmetic budget of section 2; and VF3's
coverage against Supermodel disagrees on no more than MAME's own 4.5%. Lighting, fog and
translucency are M2 decisions (which reference to follow per row of the difference
list), not rasteriser ones.

## 2. The rasteriser model

`sim/real3d/r3d_rast.h`, `r3d_rast.cpp` (MAME form, shared integer stages, lockstep),
`r3d_rast_rtl.cpp` (RTL form), driver `r3d_rast_ref.cpp`. `make test_real3d_rast
R3D_GAME=vf3|bassdx RAST_ARITH=mame|rtl`.

Inputs: the 'T' records (screen x, y; w; 256u/w, 256v/w; shade; texture header bits)
and texture RAM from the dump. Output: the 496 x 384 3D layer with MAME's alpha flag.

### Shared by both forms (integer, exact by construction)

- **Draw order.** Four priority passes. Z cleared to 1e10 at the start of each pass
  that has triangles. Opaque triangles in buffer order, then alpha triangles in reverse
  buffer order.
- **Draw kinds:**
  - solid;
  - solid translucent (blend factor `(t << 3) | (t >> 2)`; t = 32, the opaque value,
    gives 264 and so a factor of 8);
  - textured;
  - textured with colour modulation;
  - contour (alpha test `>= 0xf8`, then a blend by that alpha, Z written);
  - textured alpha (blend by texel alpha, no Z write; the polygon's own transparency is
    not used, as in MAME).
- **The eight texel formats** of `get_texture`:
  - 0 ARGB1555 (alpha is the inverted top bit);
  - 1 and 2 A4L4 in the low byte (MAME decodes them identically);
  - 3 A4L4 in the high byte;
  - 4 A4L4 in the high byte, nibbles the other way;
  - 5 L8 low; 6 L8 high;
  - 7 ARGB4444 stored RGBA.
- **Mirroring and sheets.** Both mirrors are built into the addressing: a coordinate in
  the second half of an axis reads the first half reflected. The masks are
  `(64 or 32) << size - 1`. The sheet address is linear,
  `(ty * 32 + v) * 2048 + tx * 32 + u`, in a 2048 x 1024 page, so a texture that runs
  past x = 2047 continues on the next row.
- **The filter.** MAME's SSE2 bilinear filter: the u lerp at full width, each row
  halved, then the v lerp and `>> 15`. The generic C path rounds differently; the
  x86-64 build runs the SSE2 one.
- **Colour.**
  - `scale_and_clamp` by the polygon colour, then `scale_imm_and_clamp` by the shade;
    alpha is carried through and forced to 0xff on write;
  - `(int)` of the shade is a 32-bit truncation (out of range gives INT_MIN);
  - `(uint32_t)` of u and v is a 64-bit truncation, low half kept.

### MAME form: bit-exact

| | VF3 | Bass Fishing |
|---|---|---|
| Frames bit-exact | 400 / 400 | 400 / 400 |
| Pixels drawn by MAME | 64.76 M | 39.74 M |
| Pixels different | 0 | 0 |
| Excluded | 347,124 pixels in 39 frames | none |

All eight formats are exercised in VF3 (format 4: 6,482 pixels; format 0: 58.2 M), and
every one is exact.

The exclusion is MAME reading past a texture sheet. 1.59 M texel reads in VF3 address
rows 1024 and up of a page: a 512 x 512 texture at row 640, for one. MAME reads past
`m_texture_ram[page]` into whatever the heap holds; the model reads the next page, as
Supermodel's 2048 x 2048 array would. A pixel is excluded when its colour, or the Z it
was tested against, could depend on such a read. The model tracks this with a taint
bit per pixel that follows Z writes and blends.

### RTL form: what I will build

| Stage | Arithmetic |
|---|---|
| Set-up | FP32 in MAME's order (poly.h), once per triangle per band: 15 coefficients for a textured triangle |
| Edges | vertices rounded to 1/65536 px; slope `(dx << 20) / dy`; x at each row centre exact (a DDA gives the same bits); poly.h's rounding |
| Interpolants | each FP32 coefficient rounded to a 40-bit mantissa at one exponent per parameter per triangle; a pixel's value is `2c + dx(2x+1) + dy(2y+1)`, exact |
| Z | w, linear in screen space as MAME interpolates it, as a 24-bit unsigned float (8-bit exponent, 16-bit mantissa, truncated), compared as an integer |
| Perspective | u = (256u/w) * recip(1/w): a 24-bit reciprocal (round to nearest), then truncate |
| Shade | integer part of its plane |
| Triangles with a non-finite x, y or w | dropped (30 in VF3; MAME writes no pixel from them) |

Against MAME's 3D layer:

| | VF3 | Bass Fishing |
|---|---|---|
| Frames bit-exact | 0 / 400 | 90 / 400 |
| Pixels different | 22.8% | 35.9% |
| PSNR, all frames | 47.1 dB | 37.1 dB |
| PSNR per frame, min / p1 / p50 | 34.1 / 39.0 / 49.7 dB | 17.2 / 29.5 / 49.1 dB |
| Coverage differences in the final image | 578 pixels in 400 frames | 1,709 |

Per feature, each triangle run in lockstep against MAME's own frame state, so a
difference in one triangle does not cascade:

| Feature | VF3 | Bass Fishing |
|---|---|---|
| Span extents (coverage) differ | 0.0069% of 29.9 M spans | 0.0038% of 10.2 M |
| Z test outcome differs | 0.0033% of 296 M | 0.148% of 129 M |
| Texel address (u1, v1) differs | 4.35% of 163 M fetches | 8.58% of 61 M |
| Integer shade differs | 3.09% | 2.21% |
| Written colour differs | 22.7% | 37.7% |

Where the differences come from:
- **MAME's per-pixel float accumulation**, not the RTL. MAME adds d/dx to each
  parameter once per pixel in FP32, from a span start that is itself rounded. Measured
  against MAME's own set-up planes, evaluated exactly, MAME's u and v are:

  | MAME's u, v off its own plane by | VF3 | Bass Fishing |
  |---|---|---|
  | more than 4/256 texel | 35.8% of fetches | 54.4% |
  | more than 16/256 texel | 13.3% | 25.3% |

  The RTL form is within 1/256 texel of the exact plane on 99.8% (VF3) and 99.7%
  (Bass). Its integer shade equals the exact plane's always; MAME's misses on 3.1% and
  2.8%. No exact datapath can follow that noise; a pipelined one cannot reproduce a
  serial float add per pixel at two pixels a clock.
- **Z-fighting.** Bass Fishing's worst frame (f003090, 17 dB) is an underwater view
  through two coplanar water layers. Both forms z-fight on them, in different patterns:
  MAME in stripes, the RTL in dots. Supermodel draws them cleanly.
- **Coverage.** The remainder is 1/65536 px vertex rounding against MAME's float edges.

Precision sweeps, VF3 frames 1800-4770, against MAME:

| Change | Spans different | Z outcome different | Image PSNR |
|---|---|---|---|
| vertices 1/256 px | 0.42% | | 45.3 dB |
| vertices 1/4096 px | 0.029% | | 46.7 dB |
| vertices 1/65536 px (chosen) | 0.0035% | | 46.8 dB |
| Z 20 bits | | 0.036% | 41.7 dB |
| Z 24 bits (chosen) | | 0.0048% | 45.3 dB |
| Z 32 bits | | 0.0069% | 45.6 dB |
| reciprocal 18 bits | | | 40.8 dB; u, v within 1/256 on 38% |
| reciprocal 24 bits (chosen) | | | 45.3 dB; within 1/256 on 99.8% |

32-bit Z is worse than 24 against MAME, because MAME's own Z is noisy.

**Exact fixed-point set-up rejected.** Planes computed exactly from the fixed-point
vertices, instead of MAME's FP32 set-up, are cheaper in area (about 22 DSP products per
triangle), but much further from MAME:

| | FP32 set-up (chosen) | Exact set-up |
|---|---|---|
| Integer shade differs | 3.1% | 27.2% |
| Texel address differs | 4.4% | 10.3% |
| VF3 PSNR | 47.1 dB | 38.7 dB |

MAME's FP32 set-up turns a constant shade of 255 into 254.99..., and `(int)` makes it
254. The exact planes give 255, which is plainly what the hardware intends, but it is a
departure from the oracle on every flat-shaded polygon. I keep MAME's set-up and record
the choice; it is one switch (`--fixed-setup`) if Supermodel or the board says
otherwise.

## 3. Load, and cycles at 50 MHz

Per frame, from the model. A frame at 57.52 Hz is 869,265 cycles of 50 MHz; the shares
below are of that.

| | VF3 p50 / p99 / max | Bass p50 / p99 / max |
|---|---|---|
| Triangles | 9,859 / 15,502 / 16,004 | 2,120 / 16,589 / 16,589 |
| Polygons (fans) / their vertices, p99 | 9,807 / 35,062 | 8,970 / 34,529 |
| Pixels Z-tested | 681 K / 2.18 M / 2.27 M | 317 K / 837 K / 1.28 M |
| Pixels passing Z | 390 K / 1.07 M / 1.25 M | 177 K / 504 K / 706 K |
| Opaque winners per pass (what a Z pre-pass shades) | 121 K / 239 K / 261 K | 83 K / 190 K / 190 K |

VF3's attract p99 is 11.4x overdraw, as R18's Supermodel replay found, not the 8.27x of
R13's MAME play run. Cycles, each span costing at least one:

| Scheme | VF3 p99 / max | Bass p99 / max |
|---|---|---|
| One pass, 2 px/clk | 1.10 M (127%) / 134% | 429 K (49%) / 75% |
| One pass, 4 px/clk | 565 K (65%) / 70% | 230 K (26%) / 40% |
| **Z walk 4 px/clk, shading 2 px/clk, FIFO between** | **577 K (66%) / 72%** | **275 K (32%) / 41%** |
| Z pre-pass: opaque walked twice at 4 px/clk, shading 1 px/clk | 831 K (96%) / 111% | 409 K (47%) / 77% |
| Z pre-pass, shading 2 px/clk | 831 K (96%) / 111% | 409 K (47%) / 77% |

**Two pixels a clock (§8.3 risk 3) does not cover VF3's attract p99 at 50 MHz.** Four Z
tests a clock with two shaded pixels a clock does, at 66%. The Z walk is cheap per lane
(a 40-bit add, a normaliser, a 24-bit compare); shading is the expensive lane, and it
only sees pixels that passed Z (1.07 M at p99, against 2.18 M tested).

The pre-pass is walk-bound, because the heavy frames are heavy in translucent layers:
VF3's p99 frame shades 742 K pixels with the pre-pass, against 1.07 M without.

## 4. Architecture

```
 geometry --polygons--> [bin by band] --DDR3: polygon records + band bins-->
     per band: [fetch bin] -> [FP32 set-up, MAME order] -> [edge walk, 1/65536 px]
        -> [Z walk, 4 px/clk, 4 Z banks] --passing pixels--> [shading, 2 px/clk]
           texel cache 64 KiB, 4 banks by (u&1, v&1), true dual port: 8 texels/clk
        -> colour band (2 banks by x&1, double-buffered) --finished band--> DDR3, 16 bpp
```

### Bands, and why the polygons go to DDR3

Z and colour for a whole frame do not fit on chip (190 K pixels at 48 bits is 9 Mbit,
against 5.6 Mbit of M10K), and Z in DDR3 is 1.09 GB/s (open question 1). So Z and
colour live on chip for one band at a time. A band cannot be finished until every
polygon of the frame has been seen, since the walk emits them in display-list order. So
the frame's polygons have to be stored, and VF3's p99 frame is about 815 KB of them
(9,807 headers of 8 B, 35,062 vertices of 21 B: x, y, w, 256u/w, 256v/w in FP32, an
8-bit shade). Model 2 keeps its frame on chip (151 M10K for 2,048 quads a bank); Model
3's is ten times that. It goes to DDR3, written once per frame and read once per band
it touches, through per-band index lists ("bins"), as Model 1 keeps its quad payload in
DDR3 (Model 1 `9c05b79`).

DDR3 traffic, VF3 p99 frame (the sum of each item's p99, so an upper bound):

| Band height | (polygon, band) pairs | Store: write + read + bins | 3D layer, 16 bpp, write + scan-out | Polygon RAM (R43) | Total |
|---|---|---|---|---|---|
| 8 rows | 17,890 | 141 MB/s | 43.8 | 31.7 | 216 MB/s |
| 16 rows | 12,814 | 114 MB/s | 43.8 | 31.7 | 190 MB/s |
| 32 rows | 10,861 | 104 MB/s | 43.8 | 31.7 | 179 MB/s |

Bass Fishing at 16 rows is 105 MB/s for the store.

**This is not in the study's DDR3 budget.** §7.3 and D-6 carry the 3D layer and
polygon RAM (76 MB/s) against "about 100 MB/s usable". With the polygon store it is
180-216 MB/s. Either:
- the port sustains about 200 MB/s with bursts (the framework's scaler moves more than
  that through the same DDR3), and M0-C must measure it with burst traffic, not Model
  2's access pattern; or
- the store moves to the second SDRAM in the dual-SDRAM build. There it sits beside
  the texture sheets, which take 146-163 MB/s at p99 (R17), so it fits only at the
  module's 150 MHz.

The bins keep the order MAME draws in: one list per (band, pass, opaque or alpha).
Opaque lists are read forwards; alpha lists backwards, which is MAME's reverse buffer
order. A polygon is set up again in each band it touches (14,595 / 20,340 triangle-band
pairs a frame at p50 / p99 with 16 rows, against 9,859 / 15,502 triangles).

### Priority passes and translucency in a band

A band runs passes 0-3 in order. Each (band, pass) starts with an empty Z buffer, by a
generation tag rather than a clear:
- a valid map of 3-bit-stamped 16-pixel words in MLAB;
- a stale word reads as empty;
- a sweep re-stamps every word once in 8 generations.

Colour persists across passes within a band and is cleared per band the same way. MAME
does exactly this per pass over the whole screen (`clear_zb` per pass with triangles),
so banding changes nothing.

Translucent and contour pixels blend with the colour already in the band. The shading
lane keeps pixels in order and forwards a just-written colour to a following pixel at
the same address. A contour pixel's Z write depends on its texel, so the Z walk holds
the next pixel at the same address until that contour pixel has shaded: a small
scoreboard on the 4-pixel group address. Contour is 2.2% of VF3's Z-tested pixels.

### Z pre-pass (R17)

Per band and pass:
- **Pass A:** the opaque lists walk Z only at 4 px/clk. Contour and translucent
  triangles shade as they draw.
- **Pass B:** the opaque lists walk again and shade only where Z equals the stored Z.
  Ties resolve as MAME's `<=` does: the later of two equal-Z triangles shades last.

It cuts texel fetches by 42-44% (R17), but costs 30 points of a frame at VF3's p99
(96% against 66%). It is not exact either: a contour pixel blends with the colour under
it at the time it draws, which pass A has not shaded yet. That is 8/256 of the colour
at most, on 2.2% of pixels.

I plan it as a per-frame mode: on in the single-SDRAM build, where texture fills bind
(R16), off in the dual-SDRAM build, where cycles bind. Both modes share the datapath;
the pre-pass adds the second walk of each opaque list and the equality test.

### Texture cache

64 KiB as R16/R17 measured, holding raw 16-bit texels (decoded after the read, so one
cache serves all eight formats):
- **Data.** Four banks by (u&1, v&1), so one bilinear quad reads one texel from each.
  Each bank is 8 K texels in 512 x 20 M10K: 16 per bank, 64 in all.
- **Two lanes.** True dual port, which M10K offers at x20, gives each shading lane its
  own port: 8 texels a clock. Fills take a port cycle.
- **Tags.** 4-way, lines of 4 x 4 texels (32 B, as the census modelled), 2,048 lines.
  Banked by line parity, so a quad that straddles lines looks up one tag per bank:
  8-12 M10K.

The cache is 72-76 M10K against Model 2's 37 for 32 KiB.

### M10K

| Item | 8-row bands | 16-row bands | 32-row bands |
|---|---|---|---|
| Z, 4 banks, 24 bits (2K x 5 mode, 25 bits wide) | 10 | 20 | 40 |
| Colour, 2 banks, 24 bits + drawn bit, double-buffered | 20 | 40 | 80 |
| Texture cache, data + tags | 74 | 74 | 74 |
| Polygon fetch, bin and span FIFOs | 12 | 12 | 12 |
| Reciprocal seed tables (2 lanes) | 2 | 2 | 2 |
| **Total** | **118** | **148** | **208** |

Model 2's renderer holds 236 (texture 37, quad store 151, band buffers 48). I plan
**16-row bands, about 150 M10K**: 24 bands a frame, against Model 1's and Model 2's 48 of
8 rows. 8-row bands save 30 M10K and cost 27 MB/s of DDR3; that is the trade if M10K
binds harder than DDR3.

### Area estimate (ALM / DSP)

From Model 2's measured blocks: m2_raster3d is 8,021 ALM, of which the quad store is
1,689.

| Block | ALM | DSP | Basis |
|---|---|---|---|
| Polygon fetch, bins, band scheduler | 600-900 | 0 | Model 1 m1_ddram_payload; replaces the 1,689 ALM quad store |
| FP32 set-up: 4 fp_mul, 2 fp_add as a dot-product unit, sequencer | 1,300-1,700 | 4 | 144 ALM fp_mul (Model 1, flush-to-zero), ~300 fp_add |
| Slope divider and edge walker | 2,500 measured; ~1,800 with a shared normaliser | 0 | `r3d_rast_front`, fitted (section 5) |
| Span planes (5 x 40-bit) | 300-500 | 2 | |
| Z walk, 4 lanes (adder, normaliser, compare) | 500-700 | 0 | |
| Shading, 2 lanes: plane adders, reciprocal, u v products, address, 8 decodes, filter, colour, blend | 2,000-2,600 | 20-28 | m2_span_tex / m2_texel_bl; filter and colour products in DSP |
| Texture cache control, tags, SDRAM fills | 1,000-1,500 | 0 | Model 2 m2_texel_bl |
| Band buffers, valid maps, DDR3 flush | 500-800 | 0 | Model 2 m2_fb_wcomb |
| **Total** | **8,000-11,200** | **26-34** | §8.1 row: 7,500-10,000 |

The DSP figure is the new pressure: Model 2 sat at 101 of 112. The filter can go to
logic (12 products of 9 x 9 bits a lane, about 500 ALM a lane) if DSP binds; the
colour products cannot reasonably.

Cycles at p99, 50 MHz, 16-row bands (all in parallel through FIFOs):

| Stage | VF3 | Bass Fishing |
|---|---|---|
| Set-up, 15 coefficients a cycle each + overhead, ~17 cycles a triangle-band pair | 20,340 pairs: 346 K (40%) | 18,808 pairs: 320 K (37%) |
| Z walk / shading | 577 K (66%) | 275 K (32%) |

Not modelled: load imbalance between bands, DDR3 latency on bin and polygon fetch (both
prefetchable a band ahead), and texture-cache miss stalls, which R16/R17 size as
bandwidth.

## 5. RTL: the front end (slope divider and edge walker)

`rtl/real3d/r3d_rast_div.sv`, `r3d_rast_edge.sv`, `r3d_rast_front.sv` (the two
chained); benches `sim/real3d/tb_r3d_rast_div.cpp` and `tb_r3d_rast_edge.cpp`
(`-DFRONT` for the chain); `make test_real3d_rast_edge test_real3d_rast_front`.

**The divider.** Three restoring divisions side by side, `(|dx| << 20) / dy` with
the sign applied after (truncation toward zero, as the model's `__int128` division),
two quotient bits a cycle. Each starts at its own leading quotient bit: the dividend
is normalised, and its top `msb(dy)` bits come down at once, since they cannot make a
quotient bit. A triangle takes as long as its longest edge: 15.4 cycles on average in
VF3, 15.9 in Bass Fishing, with 20% gaps and stalls. No DSP.

| Divider check | Result |
|---|---|
| Every finite triangle, VF3 / Bass | 3,614,699 / 1,831,136, 0 wrong |
| Random vertex sets over the whole 32-bit range, all magnitudes | 2,000,000, 0 wrong |

**The edge walker.**
- One triangle in: y-sorted 16.16 vertices and the three slopes `(dx << 20) / dy`.
- Its spans out, one row a clock.
- Each edge's first value is one product of the sub-pixel row offset and the slope,
  two bits a cycle. The offset is under one pixel, because poly.h's rounding puts the
  first row centre in [y, y + 1), so the product is 16 bits and takes 8 cycles.
- Every later row adds `s << 16`, in 64-bit modular arithmetic. That is exact for
  every row that is sent.
- The multiply of the next triangle overlaps the walk of the current one.
- No DSP.

**Checks** (the walker fed the model's slopes; the front fed vertices only gives the same
results, through its own divider):

| Check | Result |
|---|---|
| Every triangle of both games against `rtl_tri::span`, 20% input gaps, 20% output stalls | VF3 400 / 400 frames, 3,614,699 triangles, 29,924,115 spans, 0 wrong; Bass 400 / 400, 1,831,136 triangles, 10,239,120 spans, 0 wrong |
| Also at 0 / 0% and 50% gaps / 70% stalls | 0 wrong |
| 200,000 synthetic triangles on a quarter-pixel grid, with flat and very long edges, for exact half-pixel ties in rows and spans | 38,519,880 spans, 0 wrong |

Planted faults, each in a copy (`sm3-builds/rast/faults/`):

| Fault | 20 VF3 frames | Synthetic |
|---|---|---|
| span rounding takes ties up | not caught | caught |
| clamp at 495 | caught | caught |
| row rounding takes ties up | caught | caught |
| product one step short | caught | caught |
| upper / lower edge switch one row late | caught | caught |
| wrong edge's step | caught | caught |
| no swap of reversed spans | caught | caught |
| rows above the screen sent | not caught | caught |

The game data never puts a vertex or a span end exactly on a half pixel; the synthetic
set exists for that.

**Cycles, whole screen, no bands, one triangle at a time, no stalls:**

| | VF3 p99 | Bass p99 |
|---|---|---|
| Walker alone | 197 K (23% of a 50 MHz frame) | 202 K |
| Divider + walker | 254 K (29%) | 278 K (32%) |

With 16-row bands the count becomes triangle-band pairs (20,340 at VF3's p99, against
15,502 triangles), so about 35%. The divider is the bound on small triangles; a
second divider would halve it if bands need it.

**Fit** (Quartus 17.0, 5CSEBA6U23I7, standalone `quartus/r3d_rast_front`, 50 MHz
constraint, 225 virtual pins):

| | Seed 1 | Seed 2 | Seed 3 |
|---|---|---|---|
| Fmax, slow 100C | 64.8 MHz | 60.7 MHz | 61.7 MHz |
| ALM | 2,515 | 2,512 | 2,512 |
| Registers / M10K / DSP | 1,895 / 0 / 0 | the same | the same |

Before the fit, every file passed `quartus_map --analyze_file`. Two constant-fold
modules put the signed idioms through Quartus and read back what it computed (the
signed rounding, sign-extended offsets, `<<<` on negatives, the size casts, the
leading-one search, the normalising shifts and the restoring step). 27 of 27 values
match Verilator and Python (`sm3-builds/rast/cf_edge`, `cf_div`).

By entity (synthesis):

| Entity | ALUTs | Registers |
|---|---|---|
| Divider | 2,510 | 798 |
| Walker | 1,227 | 1,097 |

The worst path is the walker's row: `w_y` -> upper/lower select -> round -> compare
-> swap -> clamp -> `s_x0`, 15.1 ns.

The divider is two thirds of the block, and most of that is its three normalisers
(two 53-bit barrel shifts each, plus the leading-one search), used once a triangle.
§8.4's time-sharing applies directly: one normaliser for the three lanes, at two cycles
a triangle, should take it to about 1,500 ALUTs. Not done yet.

**Not in it yet:**
- the y sort on FP32;
- the float-to-16.16 conversion (round to nearest, saturating; no game vertex is
  beyond 32,768 px).

## 6. Not done yet

- The rest of the RTL: FP32 set-up, span planes, Z walk, shading, the texture cache
  and the band buffers.
- Supermodel's layered-polygon rule, fog, and the transparency and lighting rows; they
  are the main visible differences from Supermodel. Each is a decision about which
  reference to follow, and belongs in M2's row list.
- Per-band load imbalance and the bin traffic as DDR3 bursts.

## 7. For the study

Candidates for R-entries:

1. **§8.3 risk 3 and R13.** Two pixels a clock at 50 MHz covers VF3 play (R13, 8.27x)
   but not VF3 attract (11.4x): 127% of a frame at p99. Four Z tests a clock with two
   shaded pixels a clock: 66%. Measured with the model's span counts over 400 frames.
2. **Open question 1, D-6, §7.3.** Z on chip per band needs the frame's polygons stored
   off chip: 104-141 MB/s at VF3 p99 by band height. With the 3D layer and polygon RAM,
   DDR3 carries 180-216 MB/s, against the 100 MB/s the study assumes.
3. **§9 M3 exit.** MAME and Supermodel differ by 8-13 dB on the 3D layer. Coverage
   agrees on 95.5% of pixels. "Within tolerance of Supermodel" has to be a coverage
   test plus a bit-exact model test, not an image PSNR.
4. **Bit-exactness against MAME stops at the set-up.** Coverage and Z decisions match
   on 99.99%. Pixel colour cannot, because MAME's per-pixel FP32 accumulation is
   off its own planes by more than 4/256 texel on 36-54% of fetches. The MAME form of
   the model is bit-exact on 800 of 800 frames, so the reading of MAME is complete.
5. **MAME reads past a texture sheet** (1.59 M texel reads, 39 VF3 frames). What the
   hardware does with a texture that runs past row 1023 is unknown. MAME's result is
   undefined; Supermodel reads page 1.
6. **Tile layers are 2 px apart between MAME and Supermodel** in both x and y.
7. **M10K.** The rasteriser at 16-row bands is about 150 M10K, against Model 2's 236.
   This is the saving §8.1 predicted from dropping the sorted quad store.
8. **§8.1 rasteriser row.** Estimated at 8,000-11,200 ALM and 26-34 DSP, against
   7,500-10,000 and no DSP line. The first fitted block (divider and walker, 2,512 ALM,
   50 MHz closed at 60.7-64.8 MHz) is above its share until its normalisers are shared.

## 8. Supermodel form (D-19)

`sim/real3d/r3d_rast_sm.cpp`: a behavioural model of Supermodel's New3D renderer
(036af1c), in floating point as the GPU computes it, fed by our triangle lists. It also
needs each polygon's header words, viewport and model matrix, which the triangle reader
now carries. Each behaviour is a switch, and off means MAME's behaviour for that
aspect. `r3d_rast_ref --sm-ablation` renders the MAME form, all switches, none, all but
one, and each alone, and measures them against Supermodel's 3D-only grabs.

### What Supermodel does differently, by reading New3D

| Switch | Supermodel | MAME |
|---|---|---|
| COLOUR | texture x polygon colour always | only with header[3] bit 7 (Supermodel: U smooth/wrap bit) |
| DECODE | L8 white and contour A4L4 luminance 15 transparent; formats 1-4 lose alpha under contour; format 7 no alpha test; header[6] bits 24-30 = 2 inverts | alpha from the nibble, L8 opaque |
| SAMPLE | bilinear about texel centres (half-texel offset); repeat / clamp / mirror from header[3] bits 7, 6 and header[2] bits 1, 0; sheet coordinate wraps (x & 2047, y & 1023) | no offset, doubled mirror image, linear address past the sheet |
| ALPHA | contour: discard alpha < 32/255, colour dilated from opaque neighbours; texture alpha: opaque part in the opaque pass; discard alpha < 1/32 | contour >= 248/255 then blend; alpha poly blend by texel alpha |
| TRANS | two translucency layers (header[6] bit 17), nearest fragment of each kept (depth written), composited after the scene; both layers present = opaque mix | every translucent polygon blended back to front |
| DEPTH | 1/w (perspective-correct), larger nearer, GEQUAL opaque, GREATER translucent | w interpolated linearly in screen space |
| LAYER | layered polygons (header[6] bit 3) after the rest of their viewport, depth x (1 + 1/512), first one per pixel only (stencil) | in draw order, z-fights |
| CULL | back faces dropped: dot(view vertex, view face normal) > 0 unless double-sided | computed and forced off |
| FOG | per pixel: fogIntensity (header[6] bits 11-15 / 16) x clamp(start + depth x density), colour x fog ambient; scroll fog layer behind the scene | none |
| MIP | LOD = -log2((1/depth)^2 x dot(v, n) x -cota x NP), trilinear; mip levels at Supermodel's sheet bases (x 1024, 1536, ...; y 512, 768, ...) | level 0 |
| PRIO | high-priority polygons (header[6] bit 4) in a second sub-pass with Z cleared | none |

Not modelled: lighting (Supermodel lights per pixel from interpolated normals with its
own light vector, normal matrix, ambient /255 and clamp; that is the geometry's vertex
output, m2-notes rows 18-22), micro-textures, spotlights, specular, quad interpolation
(Supermodel draws quads as quads; QuadRendering is on in the configuration the grabs
used), and the block-culling white flash.

### Against Supermodel's 3D-only grabs, 400 + 400 frames

| | VF3: PSNR all / p50 frame / coverage disagree | Bass: PSNR all / p50 frame / coverage disagree |
|---|---|---|
| MAME form | 8.58 / 13.50 dB / 4.47% | 10.91 / 10.67 dB / 4.47% |
| Supermodel form, every switch | 8.72 / 14.16 dB / 5.13% | 17.29 / 21.82 dB / 3.13% |

What each switch is worth (all switches on, then that one off; p50 frame PSNR):

| Switch off | VF3 p50 | Bass p50 | Bass PSNR all |
|---|---|---|---|
| none (all on) | 14.16 | 21.82 | 17.29 |
| FOG | 14.16 | 12.48 | 11.54 |
| TRANS | 13.68 | 21.85 | 16.26 |
| CULL | 14.07 | 21.82 | 16.10 |
| SAMPLE | 14.06 | 21.43 | 17.24 |
| MIP | 14.08 | 21.47 | 17.25 |
| DEPTH | 14.13 | 21.82 | 17.17 |
| COLOUR, DECODE, ALPHA, LAYER, PRIO | within 0.2 dB | within 0.4 dB | within 0.02 dB |

- **Fog is Bass Fishing's gap**: 10.7 to 21.3 dB p50 on its own.
- **VF3's gap is lighting.** Forcing the shade to full brightness (an experiment, not a
  behaviour) moves VF3's "differ by more than 8" from 65% to 56% of pixels on 80 frames:
  Supermodel's sand floor is lit to white where MAME's light vector leaves it at ambient.
  That is the geometry's row 22, not the rasteriser.
- Translucency, culling, sampling and mipmaps are each worth 0.1-1.2 dB.
- The coverage disagreement rises slightly with TRANS in VF3 (translucent shadows now
  drawn); the 3D layer's offset against Supermodel is within a pixel (no shift scores
  better than another by more than 0.1 dB).

### Area effect of each behaviour (estimates for the RTL form)

| Behaviour | Cost | Note |
|---|---|---|
| FOG | per shading lane: one 24 x 16 product (depth x density; depth = 1/ooz from the perspective reciprocal already there), add, clamp, a 5-bit scale, and the rgb lerp (3 products): about 2-3 DSP and 80 ALM a lane, 5 DSP and 160 ALM for two; fog words per viewport in MLAB; scroll fog is a fill colour at band clear (~30 ALM) | the largest image gain for the least hardware |
| TRANS | two more colour+alpha layers and their Z in the band: per pixel 49 bits become about 161. At 16-row bands that is roughly 60 -> 200 M10K; at 8 rows about 100 | binds M10K; see below |
| CULL | one dot product per triangle (view vertex, view face normal): the geometry has both; one flag bit in the triangle record | ~0 in the rasteriser |
| SAMPLE | half-texel offset (a subtract) and clamp/mirror per axis: ~60 ALM a lane | |
| MIP | LOD per pixel: log2 of a product (leading-one + small table), per-triangle dot/cota/NP from the geometry; trilinear doubles texel reads (8 a pixel) and the cache bandwidth | the expensive one in bandwidth; nearest-level (no blend) halves it |
| DEPTH on 1/w | removes the w plane: the Z walk uses the 1/w plane the shading needs anyway | saves ~100 ALM and one plane |
| LAYER | one header bit, a bias multiply on Z (shift-add, 1 + 1/512), a stencil bit per pixel (MLAB with the valid map) | small |
| COLOUR, DECODE, ALPHA, PRIO | table and mux changes | small |

**Translucency, a hardware question.** Header[6] bit 17 is "translucency pattern select" in
Supermodel's own naming, and Supermodel's composite rule (both layers present = opaque)
is exactly what two complementary screen-door stipple patterns would give. If the
Pro-1000 dithers rather than blends, the hardware form is: a translucent pixel writes
opaque colour and depth on its pattern's pixels, and no extra layer buffers at all.
That would cost nothing in M10K but would not match Supermodel pixel for pixel, only
on average. A board photo of a translucent surface (VF3's shadows) would settle it.

## 9. Sharing the FP32 set-up with the geometry engine

The set-up is 25 dot operations a textured triangle (16 untextured):
- 6 differences;
- 3 cross terms;
- the determinant;
- 15 coefficients.

A dot operation is `m * ((a1 b1 + a2 b2) + a3 b3)`, rounded at each step: 74 FP32
operations, plus one reciprocal (1/w comes from the geometry, which already computes it
for projection, bit for bit the same value). Flush-to-zero changes no pixel on 800
frames (6 subnormal results in 546 M set-up operations; the RTL-form images are
identical), so the units can be the flush-to-zero ones.

| | Own unit | On `r3d_vpu` (4 lanes, one triangle a lane) |
|---|---|---|
| Hardware | 4 fp_mul + 2 fp_add as a dot unit, a 5-triangle software-pipelined scheduler, 6 small operand banks (MLAB): about 1,300-1,600 ALM, 4 DSP | an arbitration port and a set-up kernel in the micro-code: about 150 ALM, 0 DSP |
| Issue cycles a triangle | 25 (one dot op a cycle) | 74 MAC ops / 4 lanes = 18.5, latency hidden by 15 independent chains |
| Cycles a frame, VF3 p99, once per triangle (15,502) | 388 K (45%) on its own unit | 287 K added to the VPU (33%) |
| The same per triangle-band pair (16-row bands, 20,340) | 509 K (59%) | 376 K added to the VPU (43%) |

The VPU is the geometry's only FP pool and today runs at 177% of a 50 MHz frame at VF3
p99 (R43, latency-bound, being restructured). The set-up can share it only if the
restructured geometry comes in at about 40-55% of a frame. Then sharing saves
1,300-1,600 ALM and 4 DSP. The micro-code is the geometry's (`gen_ucode.py`), so this is
a request to that agent: a SIMD set-up kernel and one more client on the reciprocal.

If it shares, the set-up should run once per triangle at the geometry's output, with
the 15 coefficients stored in the polygon record (+60 B a triangle in DDR3, about +55
MB/s at p99). Otherwise the per-band re-set-up costs the VPU 43%.

The slope divider and 1/det do not need the VPU. 1/det is a 24-bit mantissa division
with a remainder, the same operation as the slopes, so it goes on the slope divider's
lanes with one normaliser shared by all four jobs. The perspective reciprocal is per
pixel (two a clock, pipelined) and cannot share a sequential divider; the per-triangle
jobs (4 a triangle-band pair, 81 K a frame at p99) could take its idle slots only if
the slopes were redefined to 24-bit precision, which changes coverage.

## 10. Status of the next blocks

Done this pass: the set-up arithmetic restated as dot operations with flush-to-zero in
the model (`rtl_tri` keeps the FP32 coefficients for the bench); the Supermodel form
and its measurement; the sharing analysis above.

Not started:
- the set-up RTL;
- span planes;
- the Z walk;
- shading;
- the texture path and cache;
- the band buffers;
- the polygon-store memory client with its DDR3 adapter and SDRAM2 stub (D-20).

The RTL form of the Supermodel behaviours also has to be defined before the shading RTL:
fog, the half-texel sampling and wrap modes, 1/w depth, culling flag, mip level, and
the translucency decision (layers or stipple).

## 11. The 100 MHz half-width rasteriser (D-27, D-28)

D-28 moves the rasteriser to 100 MHz every cycle at half R46's width: two Z tests
and one shaded pixel a clock. D-27 gives it 8,000 ALM. This section is the RTL-form
model at that scheme with the D-25 rows, then the blocks built so far.

### The RTL form with the D-25 rows (`render_d25`, `r3d_rast_ref --d25`)

`r3d_rast_rtl.cpp` keeps render(rtl)'s arithmetic and adds each D-25 row in the
integer form the RTL builds (the comment above `render_d25` is the definition):

- **Fog.** Per viewport the fog colour times the fog ambient, the start in 1/1024 and
  K = density / kz in FP32; per pixel P = floor(w K 1024) with w the perspective
  reciprocal of the 1/w plane (the same r the texture path computes), a 5-bit
  intensity and an 11-bit lerp after the shade and before any blend. Scroll fog fills
  its viewport with the fog colour, not drawn: a blend reads the colour itself and a
  pixel nothing draws shows it at the scroll-fog alpha.
- **Culling** from a per-triangle flag (`--cull 2`): the geometry's facing verdict.
- **Translucency** three ways: 0 MAME's blends, 1 a 4 x 4 ordered-dither screen door
  (pattern inverted by header[6] bit 17, density transparency / 32), 2 Supermodel's two
  layers with their own nearest fragment, composited at the end.
- **Depth on 1/w** (`--oow`): plane 0 is 1/w for every triangle, larger nearer.

**Fog in integers equals the float form.** Against `render_sm(SM_FOG)` it is 31.75 dB
over 400 Bass Fishing frames; against Supermodel's grabs both give 19.12-19.13 dB at
p50 (no fog: 10.68). R47's "fog alone 10.7 to 21.3 dB" is not what this harness
measures for fog alone. Two bugs on the way: untextured triangles have no 1/w plane
(plane 1 is the shade), so their fog read the shade; and the scroll fog first went in
premultiplied, which darkened every translucent pixel blended over it.

**The rasteriser cannot cull from the screen winding.** The set-up's determinant sign
(with the y sort's parity) against Supermodel's view-space verdict, dot(view vertex,
view face normal), single-sided triangles of all 800 frames: they agree on 69%. A
direct cross product on MAME's vertex order, 40 frames each, agrees on 63% (VF3) and
62% (Bass Fishing), and on 58% for triangles over 100 px. No sign convention fixes
that; the models' winding does not follow their normals. Culling by the winding made
VF3 worse (p50 13.50 to 12.19 dB, coverage disagreeing 4.46% to 8.82%). Culling by the
normal (`--cull 2`, the verdict as a record flag) is image-neutral in this form and is
worth its cycles: VF3 p99 66.9% to 53.6% of a frame. So the facing test needs the
normal, which the geometry has and the rasteriser does not. This also bears on the
geometry's "facing test beside the pool" (m2-notes section 14), whose hardware form is
the 2D cross product.

Against Supermodel's 3D-only grabs (400 frames each; PSNR all / p50 frame / coverage
disagreeing; depth on 1/w, the cull flag, fog):

| Translucency | VF3 | Bass Fishing |
|---|---|---|
| no D-25 rows (render(rtl) at 100 MHz) | 8.60 / 13.50 / 4.46% | 10.74 / 10.68 / 4.72% |
| 0, MAME's blends | 8.72 / 13.63 / 4.53% | 15.67 / 19.05 / 2.22% |
| 1, stipple | 8.65 / 13.45 / 5.93% | 15.59 / 19.05 / 3.32% |
| 2, two layers | 8.85 / 14.12 / 4.91% | 16.64 / 19.13 / 2.22% |

Depth on 1/w against w: VF3 13.63 against 13.61 dB, Bass Fishing 19.05 against 19.06:
no measurable change. It removes the w plane (12 of 66 set-up products, 16 B a stored
triangle) and gives fog one path through the reciprocal for every triangle, so I take
it. It needs the geometry to send 1/w for untextured triangles too (its section 14
record sends w and shade).

### Cycles at 100 MHz, half width (1,738,530 cycles a frame)

Per band, the slowest of three stages (walker: rows, or 13 cycles for the start
product; Z walk: max(1, ceil(len / 2)) a span; shading: one a Z-passing pixel), summed
over bands; "serial" sums the slowest stage per (triangle, band) pair instead, the
bound with no FIFO between them. p99 over 400 frames, share of a frame:

| Build | VF3 FIFO / serial | Bass Fishing FIFO / serial |
|---|---|---|
| no culling, no fog, MAME blend, w depth (D-28's case) | 66.9% / 90.3% | 32.5% / 44.0% |
| default: 1/w, cull flag, fog, translucency 0, 16-row bands | 57.1% / 60.0% | 26.3% / 33.7% |
| the same, 8-row bands | 57.1% / 60.3% | 26.3% / 34.1% |
| translucency 1 (stipple), 8 rows | 38.5% / 48.1% | 26.2% / 34.0% |
| translucency 2 (two layers), 8 rows | 45.8% / 55.6% | 26.3% / 34.1% |

D-28's 66% is reproduced for its own case. With culling the frame is shading-bound
and the Z walk sits at 33% (VF3), so 2 Z tests a clock is ample. 8-row bands cost
nothing at the frame level, which makes them the M10K-cheap choice. The FIFO between
the Z walk and shading is worth 3-10 points.

Phase 1 (the set-up records into the store) runs beside the geometry, once a
triangle: about 1.06 MB a frame at VF3 p99 with 1/w depth (144 B a textured triangle
raw, 128 with 1/w).

### The set-up: option (a) by default

The geometry's pool can carry the set-up (R60: 83.6% / 79.6% at p99 with a third
register read and the facing test beside the pool), so the rasteriser takes the
coefficients from the geometry's records (m2-notes section 14) and its own set-up
unit (option b) stays a parameter only: not built beyond the record format, which a
local unit would produce unchanged. Daytona 2 is not measured yet.

`r3d_rast_intake.sv` turns a record into the walker's and the planes' fixed point on
one shared FP32-to-fixed converter (study 8.4, converters time-shared): six vertices
(round to nearest, ties away, saturated to int32) and three coefficients a parameter at
a shared exponent e = max(E(ps), E(pdx) + 9, E(pdy) + 9) - 124, which is make_plane's
frexp of max(|ps|, 512 |pdx|, 512 |pdy|) plus 2. In 4.9 M triangles no coefficient is
non-finite or subnormal, e runs from -17 to 39, and the left shift never exceeds 14.

### The front at 100 MHz

**Divider.** The 50 MHz divider normalised each of its three dividends and divisors
(two 53-bit barrel shifts and two leading-one searches a lane) to start at the first
quotient bit. That was two thirds of the front. Now each lane (`r3d_div_lane.sv`) is a
plain radix-4 restoring divider with no shifter: the dividend {0, |dx|, 20 zeros} is 54
bits, and when dy >= 2^16 (an edge a pixel tall) its top 16 bits go straight into the
remainder, a 2:1 select, so a lane runs 19 cycles, else 27. A triangle takes 27.7
(VF3) / 29.7 (Bass Fishing) cycles with 20% gaps and stalls, against 15.4 before. That
is still well inside phase 1's budget, where the divider runs once a triangle.

**Walker.** It now takes a (triangle, band) pair: rows max(r1, b0) .. min(r3, b1) - 1.
The band's first row can be far below y1, so the start product takes a 32-bit offset,
two bits a cycle, and stops when what is left of both offsets is zero (8 cycles for a
triangle's own first row, about 13 for a band start). The accumulators are 56 bits, not
64: every sent row's true value is under 2^53. The row is two pipeline stages (the
rounds; then order, clamp and test), which takes the 15.1 ns path of the 50 MHz build
apart.

| Check (20% gaps, 20% stalls) | Result |
|---|---|
| Divider, every finite triangle, VF3 / Bass | 3,614,699 / 1,831,136, 0 wrong |
| Divider, random vertices over the 32-bit range | 2,000,000, 0 wrong |
| Walker, 16-row bands, VF3 / Bass | 5,358,786 / 2,185,133 pairs, 29,924,115 / 10,239,120 spans, 0 wrong |
| Walker, one band (whole screen) | the same spans, 0 wrong |
| Walker, 7-row bands, 200,000 synthetic quarter-pixel triangles | 6,491,759 pairs, 38,519,880 spans, 0 wrong |
| Divider + walker, 16-row bands, both games and the synthetic set | 0 wrong |

**Intake checks:** every triangle of both games as a section 14 record (3,405,478 and
1,469,469, 136 M fields) and 2 M random records over the converter's whole range
(saturation, infinities, NaN, exact ties, zeros), 0 wrong.

### The depth plane along a span (`r3d_rast_zspan.sv`)

Two pixels a clock of v2 = 2c + dx (2x + 1) + dy (2y + 1), exact in 42 bits, and their
Z words. Per span B takes two products on one 40 x 10 multiplier (so a span costs two
cycles at least; the Z walk is not the bound, above), then each lane adds 4 dx a clock.
One leading-one search and shift a lane makes the Z word.

Checks: every span of both games with 1/w depth (424 M pixels) and of 100 VF3 frames
with w depth, and 3 M random spans at make_plane's bounds, zero planes included, 0
wrong.

### Planted faults (`python3 sim/real3d/faults_rast.py`)

| Fault | Block | 20 VF3 frames | Synthetic / random |
|---|---|---|---|
| D1 sign from bit 31 | divider | not caught | caught (random range) |
| D2 skip takes the wrong 16 bits | divider | caught | caught |
| D3 exact quotient bit lost | lane | not caught | caught |
| E1 start product two steps short | walker | caught | caught |
| E2 band start off by one | walker | caught | caught |
| E3 edge switch a row late | walker | caught | caught |
| E4 clamp at 495 | walker | caught | caught |
| I1 ties round down | intake | caught | caught |
| I2 gradient exponent +8 | intake | caught | caught |
| I3 saturation one exponent late | intake | not caught | caught |
| I4 all-zero exponent 1 | intake | caught | caught |
| I5 NaN to INT_MAX | intake | not caught | caught |
| Z1 mantissa window one bit low | zspan | caught | caught |
| Z2 exponent bias + 1 | zspan | caught | caught |
| Z3 lane 1 starts at + dx | zspan | caught | caught |
| Z4 no zero test | zspan | not caught | caught once the random set had zero planes |

16 of 16 caught. Z4 was missed until I added zero planes to the random set.

### Fits (Quartus 17.0, 5CSEBA6U23I7, standalone, 10 ns, every port a virtual pin)

`quartus/r3d_rast100/fit.sh`, three seeds a batch; figures include the logic the
virtual pins pull in.

| Block | ALM s1 / s2 / s3 | Registers | M10K | DSP | Fmax slow 100C s1 / s2 / s3 |
|---|---|---|---|---|---|
| Front at 50 MHz (section 5, for reference) | 2,515 / 2,512 / 2,512 | 1,895 | 0 | 0 | 64.8 / 60.7 / 61.7 MHz |
| Front: divider + walker | 1,600 / 1,595 / 1,596 | 2,389 | 0 | 0 | 107.3 / 105.2 / 114.3 MHz |
| Intake, first build (one-cycle converter) | 328 / 326 / 329 | 182 | 0 | 0 | 67.7 / 72.0 / 66.2 MHz: fails |
| Z span, first build (one-cycle Z word) | 665 / 668 / 667 | 644 | 0 | 2 | 97.2 / 99.0 / 98.9 MHz: fails |
| Intake, four-stage converter | 313 / 311 / 310 | 435 | 0 | 0 | 156.2 / 146.8 / 150.7 MHz |
| Z span, two-stage Z word | 676 / 676 / 676 | 771 | 0 | 2 | 127.8 / 140.6 / 141.3 MHz |
| Front with both savings (below) | 1,450 / 1,456 / 1,448 | 1,669 | 0 | 0 | 109.1 / 107.2 / 107.8 MHz |
| **Running total, seed 1** | **2,439 of 8,000** | | **0** | **2** | |

By entity (synthesis, front): divider 900 ALUTs (each lane 112, the rest its input
differences, the load select, the output negations and the vertex pass-through),
walker 1,057 ALUTs and 1,415 registers. The divider was 2,510 ALUTs at 50 MHz.

The two first builds that failed had one long cycle each: the intake's select, two
subtracts, 53-bit shift, 39-bit round, 40-bit negate and saturate; the Z span's 41-bit
leading-one search into a 42-bit shift. The converter is now four stages that move
together with the output, and the Z word two. Both benches pass unchanged, all nine of
their planted faults are caught again, and the pipelined intake came out smaller than
the one-cycle build (the fitter no longer duplicated the long path).

### Where the 8,000 ALM go: built, and estimated for the rest

Estimates are mine, from the fitted blocks above and Model 2's measured ones; the
DSP and M10K columns assume 8-row bands, one shading lane and translucency 0 or 1
(two layers add about 36 M10K: 3,968 pixels x 88 bits of working Z and two RGBA
layers).

| Block | ALM | M10K | DSP | Basis |
|---|---|---|---|---|
| Front (fitted, with both savings) | about 1,450 | 0 | 0 | above |
| Intake, option (a) (fitted) | about 310 | 0 | 0 | above |
| Z span, 2 a clock (fitted) | about 676 | 0 | 2 | above |
| Z band memory, generation map, forwarding, contour scoreboard | 350 | 10 | 0 | two banks by x parity, 24 bits |
| Shading planes (u/w, v/w, shade) | 450 | 0 | 2-4 | Z span's form, one lane |
| Perspective reciprocal, 24-bit round to nearest | 300 | 0 | 4-6 | r3d_recip's seed and Newton form |
| u, v products and shifts, integer shade | 330 | 0 | 4 | |
| Fog | 160 | 0 | 5 | R47 |
| Texture address, mirrors, 8 decodes x 4 texels | 310 | 0 | 0 | |
| Bilinear, MAME's SSE2 form | 150 | 0 | 5 | 12 products |
| Colour and blends | 150 | 0 | 6 | |
| Texture cache, 32 KiB, 4 banks by (u & 1, v & 1) | 750 | 30-36 | 0 | Model 2 m2_texel_bl |
| Colour band, double-buffered, DDR3 write | 450 | 20 | 0 | Model 2 m2_fb_wcomb |
| Polygon store client, bins, band scheduler (D-20) | 750 | 8-12 | 0 | Model 1 m1_ddram_payload |
| Shading control, Z-to-shading FIFO, blend forwarding | 300 | 0 | 0 | |
| Draw kinds, header fields | 200 | 0 | 0 | |
| **Total** | **about 7,000-7,150** | **about 70-80** | **about 28-32** | against 8,000 |

So the budget looks reachable with 600-800 ALM to spare, if the estimates hold; R46's
estimates at full width ran 8,000-11,200, so the margin is not safe. Two savings are
measured or close to it and not taken yet:
- the divider's vertex pass-through and output registers (about 150 ALM): in the
  design the store takes the vertices from the intake;
- the walker's start products in their own registers so the next pair overlaps the
  walk (about 200 ALM of the walker's 1,415 registers): without the overlap the walker
  rises from about 10% to about 20% of a frame, still below shading.

### The two front savings, taken

- **Walker:** the start products accumulate in the edge registers themselves (x1, x2
  loaded, two offset bits a cycle, then the walk), so the separate 3 x 56-bit product
  accumulators, the step copies and the vertex copies are gone. The product no longer
  overlaps the previous pair's walk; stage 0 still holds the next pair. Walker alone,
  VF3 p99 with 16-row bands and 20% gaps and stalls: 243 K to 359 K cycles (14% to 21%
  of a frame), still below shading.
- **Divider:** the lanes load straight from the input differences on the accepting
  edge (no registered load stage of 3 x 123 bits), each lane keeps its own divisor,
  and the slopes are formed from the quotients the lanes hold (no 3 x 54-bit output
  registers).

Measured: 1,596-1,600 to 1,448-1,456 ALM (-148), registers 2,389 to 1,669 (-720).
I had estimated about -350: registers alone buy about half what they look like, as
Model 2's R952 found. Both benches and all seven front faults pass unchanged. A first try had the lane read
its divisor live from the input every cycle: 3.57 M of 3.61 M slopes wrong, caught by
the divider bench at once.

### Next: the Z band block (designed, not built)

- **Memory:** two banks by x parity, one pixel each, 248 x 8 rows = 1,984 words of 24
  bits: 2K x 5 M10K mode, 5 a bank, 10 in all. One read and one write port a bank a
  clock, which is the two Z tests a clock.
- **Clear per (band, pass):** a generation map, a 3-bit stamp per 16-pixel word in MLAB
  (248 words); a stale word reads as the clear value (0 with 1/w depth); a sweep
  re-stamps once in 8 generations.
- **Read-modify-write hazard:** read in cycle t, compare in t + 2, write in t + 2; a
  pixel of the next span (same row, overlapping x, another triangle) can read in t + 1
  or t + 2 before the write lands. Forward from the two in-flight writes by address
  compare (2 lanes x 2 stages).
- **Contour:** its Z write depends on the texel's alpha test, so the Z walk holds a
  pixel whose address matches an unresolved contour pixel until shading returns the
  verdict (a scoreboard of the in-flight contour addresses, sized by the Z-to-shading
  FIFO's depth).
- **Bench:** the model's per-pass Z buffer and draw order (render_d25's loop), the RTL
  fed by r3d_rast_zspan's output and the per-triangle Z-write kind; compare every Z
  outcome and every written Z, with the contour verdict taken from the model.
