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
a request to the geometry work: a SIMD set-up kernel and one more client on the reciprocal.

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

## 12. The shading lane, the Z band and the texture cache (D-28, D-29)

Seven more blocks, each benched bit for bit against `render_d25` (depth on 1/w, the
cull flag, fog, translucency 0), each with planted faults, each fitted standalone on
three seeds at 10 ns. D-29 (plain blend, mode 0) is the build: the Z band's screen
door is a parameter that folds away at 0, and mode 2's two-layer memory is not built.

### The model's pixel stream (`d25_hook`)

`render_d25` takes an observer (`d25_opts::hook`): each triangle after its set-up,
and each pixel of each span in the model's order with every intermediate the RTL
produces: Z and the Z it was tested against, the stipple, the pass, the texel, u, v,
the shade, fog's P and record (K, S, fog colour, intensity), the colour before and
after fog, the colour under it and the colour written. Every bench below compares
against that stream, rebucketed band by band where the RTL's order differs. Three
public wrappers give the random benches the model's own arithmetic: `d25_fog_p`,
`d25_colour` (frag then fog) and `fetch_texel`. The images are unchanged (the hook
is passive).

### The intake on the R65 record

The geometry's record moved (R65, m2-notes section 15): a 4-word header (word 3 the
fog intensity, layer and facing bits), and SETUP4's planes 1/w, u/w, v/w, shade
textured (23 words) or 1/w, shade (17), with no w plane. `r3d_rast_intake` takes it
by default (`SETUP4 = 1`); `SETUP4 = 0` keeps R62's form. Its fields keep their tags,
but the plane numbers follow the new order (plane 0 is now 1/w, 3 the shade).

The header beats now carry tag 31, not 15: 15 was also plane 1's dy (8 + 4 + 3), so a
consumer had to count header beats (the M2.5 viewer found it). No parameter tag
reaches 31, so every tag now names one field. That is an interface change for anything
that matched tag 15.

Checks: every triangle of both games as a SETUP4 record (3,405,478 and 1,469,469;
125 M fields) and 2 M random records, and the R62 form on 40 VF3 frames and 500,000
random records, 0 wrong; I1-I5 caught again.

### Fits (Quartus 17.0, 5CSEBA6U23I7, standalone, 10 ns)

The fitter reports the ALMs the virtual pins take ("due to virtual I/Os"); inside the
rasteriser those are wires, so the ledger is now kept both ways. The first three
blocks had 122, 43 and 158 of them.

| Block | ALM s1 / s2 / s3 (pins' ALMs) | ALM without them | M10K | DSP | Fmax slow 100C s1 / s2 / s3 |
|---|---|---|---|---|---|
| Z band, first build | 878 / 877 / 877 | | 12 | 0 | 79.6 / 78.2 / 75.4 MHz: fails |
| Z band | 788 / 785 / 778 (146) | 642 / 639 / 632 | 12 | 0 | 119.3 / 114.9 / 114.1 MHz |
| Interpolants, plane products one 31 x 11 each | 1,584 / 1,582 / 1,582 | | 0 | 21 | 119.7 / 114.1 / 115.5 MHz |
| Interpolants | 1,510 / 1,513 / 1,511 (318) | 1,192 / 1,195 / 1,193 | 0 | 15 | 113.6 / 117.5 / 122.0 MHz |
| Fog P | 183 / 183 / 180 (44) | 139 | 0 | 1 | 132.1 / 139.4 / 145.0 MHz |
| Colour, first build (four stages) | 257 / 257 / 259 | | 0 | 13 | 95.9 / 93.6 / 96.3 MHz: fails |
| Colour | 237 / 237 / 235 (107) | 130 | 0 | 13 | 182.4 / 172.0 / 175.8 MHz |
| Texel, first build (v lerp's product and add in one stage) | 667 / 666 / 667 | | 0 | 12 | 92.3 / 93.5 / 92.1 MHz: fails |
| Texel | 671 / 672 / 671 (144) | 527 / 528 / 527 | 0 | 12 | 112.0 / 113.1 / 115.3 MHz |
| Intake on SETUP4 | 312 / 312 / 312 (43) | 269 | 0 | 0 | 141.7 / 154.3 / 147.6 MHz |
| Texture cache, 32 KiB 4-way | 1,124 / 1,125 / 1,126 (152) | 972 / 973 / 974 | 43 | 1 | 50.1 / 47.9 / 47.1 MHz: fails |

The cache's path is the L1 cycle: the tag RAM's word, the hit, the victim and the
row update into the tag write, 17.5 ns. It needs L1 split into a compare stage and an
allocate stage, with an allocation forwarded into the rows of the pixel waiting
behind it; not done yet. The colour block's 13 DSP is one product a block: the three
8 x 8 colour products, the three 8 x 5 wrap terms and the 5 x 11 fog term could go to
logic (about 150 ALM, my estimate) for 6 blocks.

### The Z band (`r3d_rast_zband.sv`)

Two banks by x parity, word {stamp, Z} at {row, x[8:1]}, 2K x 30 a bank (six 2K x 5
M10K). Lane 0 is pixel x, lane 1 x + 1, bank 0 holds the even one at x[8:1] + x[0].
A (read issued) -> B (M10K out, forwarding matches and the stamp test) -> C (select,
compare, write) -> O. C forwards from the two writes it cannot see: the one on the
previous advance and the one on the edge it was read on.

Done differently from section 11's design, and why:
- **Stamps in the Z word, not an MLAB map.** A 6-bit stamp a pixel costs 2 M10K (12
  against 10); the MLAB map (248 words of 16 valid bits and a stamp a bank) would be
  about 16 LABs plus its own read-modify-write and forwarding, roughly 200 ALM.
- **The sweep is 32 words of each bank a generation, and it writes the clear value.**
  A swept word then reads as the clear value whether or not its stamp comes round, so
  only words a pixel wrote must be swept within 64 generations: 2048 / 32 = 64 is
  exactly enough. I had first reasoned that 33 were needed; that held only while the
  sweep wrote zero, which is also wrong for w depth (its clear is all ones).
- **The contour scoreboard is a count and an x range per band row,** not a CAM of
  addresses: a pair holds while its row's range covers it, or while a contour pair of
  another triangle is in B, C or the push register. Pending contour pairs wait in a
  32-deep MLAB FIFO with their Z; a verdict that says write takes its bank's port the
  next cycle and stalls the pipe one cycle if C holds a pair.

Generations: 23,958 over 400 VF3 frames (60 a frame) and 16,990 for Bass Fishing,
each about 35 cycles (drain, 32-word sweep), so about 2,100 cycles a frame (0.12%).

Cycles with no gaps or stalls and a 20-cycle verdict delay: VF3 p50 / p99 / max
275,413 / 583,638 / 699,438 (33.6% of a frame at p99), Bass Fishing 121,502 /
351,327 / 516,319 (20.2%); 1.033 and 1.014 cycles a pair, the excess being the
contour holds and the generation steps.

Checks: every Z-tested pixel of all 800 frames, 208.4 M (VF3) and 95.6 M (Bass
Fishing), with RAM contents random at power-up, 20% gaps, 20% stalls, verdicts after
up to 40 cycles; eleven synthetic runs of 20,000 generations (micro-triangles
stacked on the same pixels, sparse phases on one row for longer than the stamp's
period, verdict delays 0 and 200); the screen-door build on 20,000 synthetic
generations a seed and 40 VF3 frames. 0 wrong.

Wrong turns:
- A verdict from the previous generation landed just after the sweep had passed its
  word, so its stamp came round live 64 generations later (seed 1 of the synthetic
  set; the frames never showed it). Verdicts from an older generation now write
  nothing.
- The same race inside the sweep: a current-generation verdict landing behind the
  sweep pointer. Once the sweep starts no pixel of the old generation is left to read
  Z, so verdict writes are dropped while it runs, and the generation steps only on an
  edge with no verdict arriving.
- Timing: the first fit made 75-80 MHz; the compare in C fed the scoreboard's range
  in the same cycle. The push now goes through a register (covered by the hold), and
  B does the address matches and the stamp test, so C is a select and a compare:
  114-119 MHz.

### The shading lane's interpolants (`r3d_rast_interp.sv`, `r3d_rast_recip.sv`)

One pixel a clock: shade, u/w and v/w planes, the 1/w value from the Z walk, the
perspective reciprocal, u and v, and r, n out for fog. Each plane is two DSP blocks,
each summing two 18 x 18 products: the gradients split at bit 18, lo = dxl (2x+1) +
dyl (2y+1), hi likewise, v2 = 2c + (hi << 18) + lo. Written as one 31 x 11 product
each, Quartus used two blocks a product: 21 DSP against 15 now, and 74 ALM more.

The reciprocal, round(2^47 / m) for a 24-bit m, is r3d_recip's seed and two Newton
steps cut to the reciprocal, with a clock enable (the geometry's runs free; the
shading pipe stalls on texture misses). Exhaustive: all 8,388,608 mantissas with 25%
stalls, 0 wrong. The values wait for it in a 16-deep MLAB delay line, not 11 stages
of about 190 registers.

Checks: every Z-passing pixel, 149.1 M (VF3) and 61.6 M (Bass Fishing), and 3 M
random planes at make_plane's bounds (any exponent, 1/w of either sign or zero)
through rtl_tri::eval; 0 wrong.

### Fog (`r3d_rast_fogp.sv`, and the lerp in `r3d_rast_colour.sv`)

P = min(floor(r km 2^(ke - eo - n - 123)), 4096), 4096 when 1/w <= 0 and 0 when K
is not positive: r km lies in [2^46, 2^49), so only shifts of 35-49 need a shifter.
One DSP. It runs beside the texel fetch and its 13 bits go with the pixel. Checks:
146.7 M and 60.2 M pixels and 3 M random, 0 wrong.

### Colour (`r3d_rast_colour.sv`)

MAME's six draw kinds, colour modulation, the shade as clamp(int32(c s) >> 8) with
the 32-bit wrap (c s[26:0] in one 27 x 27 and the low 5 bits of c s[31:27] in logic),
then fog's lerp. The blend itself belongs to the band stage (the colour under the
pixel). Checks: 32.5 M and 20.2 M pixels (100 frames each) and 3 M random (every
kind, the shade over the whole int32 range), 0 wrong.

Quartus 17 put one product in each DSP block even in "two independent 18 x 18"
mode: the first build used 13 blocks for 13 products, so packing has to be written
in (a sum of two, or one wider product).

### The texel (`r3d_rast_texel.sv`)

MAME's addressing (mirrors, the linear sheet address past x = 2047), the eight
formats and the SSE2 filter as three products a channel (256 a + (b - a) fu, and the
same in v: the same integers). Banks by (A[11], A[0]): the two u neighbours differ by
an odd amount or not at all, the two v neighbours by an odd multiple of 2048 or not
at all, carries included, so the four corners land in four banks or two corners
share one address. Checks: 136.1 M and 46.9 M texels and 3 M random (any size,
mirror and format over random texture RAM), 0 wrong.

### The texture cache (`r3d_rast_tcache.sv`) and what it shows

Non-blocking and in order, Model 2's m2_texel_bl in shape. Lines of 4 x 4 texels,
tiled in the texture RAM (a line one burst of four 64-bit beats). Four data banks
(a bank's 2 x 2 texels of a line in one word, two 32-bit lanes), four tag copies.
Lookup runs up to 64 pixels ahead of the data read and keeps 8 fills in flight; a
pixel reads when the fills allocated by its lookup are written, which is exact for
an in-order reader since fills complete in order; a fill writes only after every
earlier pixel has read (they may still want the victim). Default 256 sets x 4 ways,
32 KiB: 32 M10K of data and 8 of tags.

Checks: every textured Z-passing pixel's four words, 136.1 M (VF3, 400 frames) and
46.9 M (Bass Fishing) with latency 20-40, and 100 frames each with latency 20-60,
gaps and stalls; 5 M random quads, scattered, clustered and with corners in one set
under other tags, with a texture upload (invalidate) between halves. 0 wrong.

**Texture fills, not logic, bind the frame.** Measured in the RTL's order
(`make measure_real3d_tcache`, `r3d_tcache_sim`), level-0 sampling as the model
does:

| Cache | VF3 fills p50 / p99 | MB/s at p99 | Bass fills p50 / p99 |
|---|---|---|---|
| 16 KiB 4-way | 75,452 / 406,757 | 749 | 11,527 / 113,466 |
| 32 KiB 4-way | 67,187 / 297,099 | 547 | 7,875 / 104,993 |
| 64 KiB 4-way | 55,443 / 231,272 | 426 | 5,955 / 94,663 |

Textured pixels' cycles at p99, lookup 64 ahead, 8 fills in flight, latency 24
(share of a 100 MHz frame):

| Cache | VF3, a line every 4 cycles (64-bit port) | VF3, every 16 (16-bit SDRAM) | Bass, 4 | Bass, 16 |
|---|---|---|---|---|
| 32 KiB 4-way | 89.0% | 274.8% | 30.9% | 99.6% |
| 64 KiB 4-way | 67.1% | 214.3% | 30.3% | 90.4% |

The RTL with the bench's memory (acks refused 30% of cycles, beats missing 15%,
latency 20-40) takes VF3 p99 to 132.7% and Bass Fishing to 46.6%. The p50 matches
R16/R18 (64 KiB: 96 MB/s here, 94-99 there); the p99 tail is R18's 331 MB/s problem
and worse in these frames. A 16-bit SDRAM cannot carry VF3's p99 at any size I can
afford; a 64-bit DDR3-class port with 64 KiB just does. The levers are study
decisions: a mip level (D-25 set LOD aside for its image effect, but its bandwidth
effect is the large one here), the Z pre-pass (R17: -42% fetches), 64 KiB (+32
M10K), and where the texture sheets live.

Planted faults, this section:

| Fault | Block | 20 frames | Synthetic / random |
|---|---|---|---|
| B1 no forward from the older write | zband | caught | caught |
| B2 forward priority swapped | zband | not caught | caught |
| B3 stamp compared on 5 bits | zband | caught | caught |
| B4 sweep 31 words | zband | not caught | caught |
| B5 no scoreboard | zband | caught | caught |
| B6 no hold for contour in B/C | zband | caught | caught |
| B7 strict compare | zband | caught | caught |
| B8 verdict lane order | zband | caught | caught |
| B9 range high end | zband | caught | caught |
| B10 old-generation verdict writes | zband | not caught | caught |
| B11 verdict writes in the sweep | zband | not caught | caught |
| N1 no round-up | recip | caught | caught |
| N2 delay line 10 | interp | caught | caught |
| N3 shade floors negatives | interp | not caught | caught |
| N4 u shift one short | interp | caught | caught |
| N5 saturation at 32 | interp | not caught | caught |
| N6 2x for 2x + 1 | interp | caught | caught |
| F1 shift bias | fogp | caught | caught |
| F2 saturation a step early | fogp | not caught | caught |
| F3 negative K | fogp | not caught | caught |
| C1 no 32-bit wrap | colour | not caught | caught |
| C2 contour test 0xf7 | colour | not caught | caught |
| C3 trans >> 2 wrong | colour | not caught | caught |
| C4 fog shift toward zero | colour | caught | caught |
| C5 fog g bits | colour | caught | caught |
| T1 mirror seam | texel | caught | caught |
| T2 format 4 nibbles | texel | not caught | caught |
| T3 bank pick | texel | caught | caught |
| T4 v lerp rounding | texel | caught | caught |
| T5 u2 not wrapped | texel | caught | caught |
| K1 beat counters a bit short | tcache | caught | not caught |
| K2 no fill gate | tcache | caught | caught |
| K3 victim may be this pixel's way | tcache | not caught | not caught |
| K4 read a fill early | tcache | caught | caught |
| K5 lanes swapped | tcache | caught | caught |
| K6 no dedup of a shared line | tcache | not caught | not caught |
| K7 texel select swapped | tcache | caught | caught |
| K8 tag valid ignored | tcache | not caught | caught |

36 of 38 caught. K3 and K6 are near-equivalent: round-robin victims give one pixel's
allocations distinct ways, so losing K3's protection needs five allocations in one
set within one pixel; without K6's dedup two banks on one line fill two slots with
the same line, which costs a fill and no wrong texel. Both stay in.

### Where the 8,000 ALM go now

Seed 1; "logic" is without the virtual pins' ALMs, which become wires inside the
rasteriser.

| Block | ALM with pins | logic | M10K | DSP | 10 ns |
|---|---|---|---|---|---|
| Front | 1,450 | 1,328 | 0 | 0 | yes |
| Intake (SETUP4) | 312 | 269 | 0 | 0 | yes |
| Z span | 676 | 518 | 0 | 2 | yes |
| Z band | 788 | 642 | 12 | 0 | yes |
| Interpolants and reciprocal | 1,510 | 1,192 | 0 | 15 | yes |
| Fog P | 183 | 139 | 0 | 1 | yes |
| Colour | 237 | 130 | 0 | 13 | yes |
| Texel | 671 | 527 | 0 | 12 | yes |
| Texture cache, 32 KiB | 1,124 | 972 | 43 | 1 | no (47-50 MHz) |
| **Built** | **6,951** | **5,717** | **55** | **44** | |
| Band buffers, blend, DDR3 band write (estimate) | | 450-600 | 20 | 2 | |
| Polygon-store client, bins, scheduler (estimate) | | 750 | 8-12 | 0 | |
| Z-to-shading FIFO, shading control, triangle records by tag (estimate) | | 300-500 | 4-8 | 0 | |

Against my section 11 estimates the built shading blocks came out larger: the
interpolants 1,192 logic ALM against 1,080 (planes, reciprocal, u v), the Z band 642
against 350 (its 84-bit payload rides four stages), the cache 972 against 750. By
the logic count the whole rasteriser lands at about 7,300-7,700 of 8,000; by the
pins-included count used for the first three blocks it would not fit. The DSP count
is the larger miss: about 44 built against 28-32 planned, most of it products
Quartus will not pack two to a block.

### Not done

- The band buffers (colour, the blend, 8-row bands double-buffered) and the DDR3
  band write.
- The D-20 polygon-store client, its SDRAM2 stub and a DDR3-like memory model.
- The top level, the whole-rasteriser bench on 400 + 400 frames, and its fit.
- The Z-to-shading FIFO and the shading control (the triangle record by tag).

## 12, continued. Mip levels against texture bandwidth, and the cache at 100 MHz

### Mip levels in `render_d25` (`d25_opts::mip`, `--mip`, `--lod`)

The MIP row of section 8 lived only in `render_sm` (the float Supermodel form);
`render_d25` had no switch. It has one now, off by default:
- `--mip 0` level 0, as before (images unchanged, checked: 13.63 / 19.05 dB p50);
- `--mip 1` the nearest level of Supermodel's LOD (rounded), no blend;
- `--mip 2` trilinear: the two levels around the LOD, blended with an 8-bit weight
  after each level's bilinear filter.

Level L of a page sits at Supermodel's sheet bases (x 0, 1024, 1536, ...; y 0, 512,
768, ...), the texture at (base + tex_x 32 >> L, base + tex_y 32 >> L), sized
(32 << w) >> L, read with MAME's addressing and filter on u >> L, v >> L. The highest
level is Supermodel's, floor(log2(min side)) - 1. `texel_addr4` gives the four
addresses for any level and is shared with the fill tool.

LOD = -log2((kz / w)^2 dot(v, n) (-cota) NP), as `render_sm` has it. Two forms:
- `--lod 0`: Supermodel's, per pixel in double, the view position rebuilt from the
  pixel and its 1/w.
- `--lod 1`, the hardware candidate. On a flat triangle dot(v, n) is constant, so
  LOD = -log2 B - 2 log2(1/w), with B = kz^2 dot(v0, n) (-cota) NP one FP32 word a
  triangle, and log2(1/w) read off the pixel's Z word (it is already float-like:
  exponent, then a 64-entry table on six mantissa bits). B <= 0 gives level 0.

The two forms are not the same where the header's normal is not the face's normal.
On 32 VF3 frames, 70.9% of textured pixels get the same LOD, 27.7% differ by under
1/16 level, and 0.89% differ by two levels or more. Those come from triangles where
dot(v, n) changes by about 10 a pixel and changes sign across the face. On one of
them B at vertex 0 is negative, so the hardware form gives level 0 where
Supermodel's goes up to level 8. Supermodel interpolates the dot per vertex, so
matching it needs a fifth plane (below).

### The image cost (400 + 400 frames, depth on 1/w, the cull flag, fog, translucency 0)

Against Supermodel's 3D-only grabs, and against level 0 of the same form:

| Form | VF3 PSNR all / p50 / differ >8 | Bass PSNR all / p50 / differ >8 | VF3 pixels changed (>8) vs level 0 | Bass pixels changed (>8) vs level 0 |
|---|---|---|---|---|
| Level 0 | 8.72 / 13.63 dB / 75.76% | 15.67 / 19.05 dB / 37.32% | | |
| Nearest level | 8.73 / 13.72 dB / 75.47% | 15.73 / 19.17 dB / 37.11% | 21.65% (11.35%), 29.29 dB | 10.76% (3.98%), 35.29 dB |
| Trilinear | 8.73 / 13.72 dB / 75.43% | 15.74 / 19.16 dB / 37.08% | 26.40% (11.32%), 29.50 dB | 12.67% (3.82%), 35.97 dB |

The coverage disagreement does not move (4.51-4.53%, 2.22%). The per-triangle LOD
against the per-pixel one: nearest, 1.14% of VF3 pixels change (0.70% by more than
8) and 0.12% of Bass Fishing's; trilinear, 4.52% (0.64%) and 1.52% (0.00%). Against
the grabs the two LOD forms are equal to 0.01 dB.

So mip levels move the image towards Supermodel by 0.09 dB (VF3) and 0.12 dB (Bass
Fishing) at p50. That is the size section 8's switch table gave (0.08 and 0.35 dB
in the float form). Trilinear is worth nothing over the nearest level.

### Fills, and the frame on three memories (`r3d_tcache_sim`)

The tool now takes `--mip` and `--lod`. It replays the cache's own victim rule (a
global round-robin way, never one the pixel uses; `TC_REPL=1` for true LRU) and
runs each cache shape against these memories:
- **ideal64**: R66's figure, a line every 4 cycles and latency 24.
- **sdram**: one 16-bit SDRAM behind Model 2's `m2_sdram`. One transfer at a time;
  3 cycles of arbitration and dispatch, then a CAS a word (16 a line). Rows stay
  open; tRP 2, tRCD 2, tRAS 5, tRC 7, CL 2 plus 4 capture cycles; refresh every 700
  cycles. So a line costs 19 cycles on a row hit, 21 on a closed bank and 24 on a
  conflict. Map 0 is Model 2's {bank, row, column}, with the 8 MiB sheet in one bank.
  Map 1 puts 8 x 4 lines in a row and spreads neighbouring blocks over the four
  banks. Texture traffic only, nothing else on the chip.
- **ddr3**: the 64-bit f2sdram port at 100 MHz. Reads are pipelined and data comes
  back in order at a beat a cycle, after a fixed latency of 30 or 60 cycles. The
  port is shared with R64's measured clients at their burst sizes: layer writes
  43.8 MB/s in 248-beat bursts, scan-out 43.8 MB/s in 256-beat bursts, polygon RAM
  28.4 MB/s in 32-beat bursts. "D-20" scales these to D-20's 180 MB/s.

Lookup is 64 quads ahead with 8 fills in flight, as the RTL. A trilinear pixel is two
quads (two lookups, so two cycles at four texels a clock) when it is minified. The
cycles count textured pixels alone, as R66's table did. Under LRU the tool gives
R66's figures exactly (VF3 297,099 fills p99 and 89.0%; Bass 104,993 and 30.9%). The
RTL's round-robin victim costs 1.8% (VF3) and 7.2% (Bass) more fills at p99.

**VF3**, 400 frames; p99 share of a 100 MHz frame (1,738,530 cycles):

| Form | Cache | Fills p50 / p99 | MB/s p99 | ideal64 | SDRAM map 0 | SDRAM map 1 | DDR3 L30 | DDR3 L60 | DDR3 L60, 16 fills |
|---|---|---|---|---|---|---|---|---|---|
| Level 0 | 32 KiB | 69,429 / 302,324 | 556.5 | 90.7% | 406.2% | 390.8% | 117.0% | 183.1% | 110.6% |
| Level 0 | 64 KiB | 57,249 / 222,723 | 410.0 | 67.3% | 298.2% | 287.2% | 81.9% | 128.9% | 80.9% |
| Nearest | 32 KiB | 34,011 / 192,391 | 354.1 | 64.7% | 259.5% | 250.5% | 75.9% | 110.2% | 75.3% |
| Nearest | 64 KiB | 30,261 / 166,758 | 306.9 | 57.2% | 228.1% | 220.6% | 64.9% | 97.3% | 65.2% |
| Trilinear | 32 KiB | 47,293 / 204,853 | 377.1 | 86.8% | 276.6% | 266.2% | 99.7% | 128.0% | 99.3% |
| Trilinear | 64 KiB | 40,879 / 180,026 | 331.4 | 79.6% | 244.2% | 235.5% | 88.6% | 106.3% | 89.5% |

**Bass Fishing**, 400 frames:

| Form | Cache | Fills p50 / p99 | MB/s p99 | ideal64 | SDRAM map 0 | SDRAM map 1 | DDR3 L30 | DDR3 L60 | DDR3 L60, 16 fills |
|---|---|---|---|---|---|---|---|---|---|
| Level 0 | 32 KiB | 8,785 / 112,590 | 207.2 | 31.3% | 141.3% | 132.7% | 41.1% | 66.4% | 38.2% |
| Level 0 | 64 KiB | 7,034 / 96,393 | 177.4 | 30.3% | 121.9% | 114.7% | 37.0% | 58.3% | 34.7% |
| Nearest | 32 KiB | 7,619 / 58,200 | 107.1 | 29.8% | 82.8% | 76.7% | 34.3% | 46.2% | 34.2% |
| Nearest | 64 KiB | 5,823 / 49,545 | 91.2 | 28.5% | 72.7% | 67.5% | 32.5% | 42.4% | 32.8% |
| Trilinear | 32 KiB | 8,529 / 76,032 | 139.9 | 37.3% | 108.0% | 100.0% | 44.3% | 59.7% | 44.1% |
| Trilinear | 64 KiB | 6,428 / 64,467 | 118.7 | 35.7% | 94.1% | 87.3% | 41.7% | 54.5% | 42.1% |

The DDR3 port at D-20's 180 MB/s and latency 30: VF3 nearest 80.3% (32 KiB) and 67.9%
(64 KiB); Bass Fishing 36.6% and 34.4%.

The per-triangle LOD (`--lod 1`), nearest level, VF3: 224,472 fills p99 at 32 KiB
(+16.7%) and 200,620 at 64 KiB (+20.3%). The p99 cycles are 55.5% and 54.5% ideal,
75.0% and 67.9% on DDR3 at L30, and 127.6% and 115.0% at L60. The p99 frames are not
the same frames: the extra fills come from triangles that the per-triangle form
puts at level 0. Bass Fishing is within 0.3% of the per-pixel form.

What the tables say:
- **The nearest level is the largest single lever.** VF3 p99 fills fall 36% at 32 KiB
  and p50 fills 51%; Bass Fishing's p99 fills fall 48%. It also moves the image
  slightly towards Supermodel. Minified pixels are 31.9% of VF3's textured pixels
  and 35.0% of Bass Fishing's; levels 1-5 take 13.9 / 9.8 / 4.1 / 2.4 / 1.4% (VF3).
- **Trilinear costs more cycles than its fills save.** 36-41% of textured pixels need
  a second quad: VF3 p99 is 1,262,495 quads, 72.6% of a frame before any miss. On
  the ideal port it is 86.8% against the nearest level's 64.7%.
- **No 16-bit SDRAM carries VF3.** The best case is 220.6% (nearest level, 64 KiB,
  map 1). R66's 274.8% assumed 16 cycles a line; Model 2's controller takes 19-24, so
  level 0 at 32 KiB is 390.8%. Bass Fishing fits only with mip levels (67.5-76.7%).
  The 2-D map is worth 3-8%.
- **DDR3 carries it if its latency is near 30 cycles.** Nearest level: 75.9% at 32 KiB
  and 64.9% at 64 KiB. At 60 cycles the 8 fills in flight bind (110.2% / 97.3%), and
  16 fills bring it back to 75.3% / 65.2%. Level 0 at 32 KiB fails on every DDR3 case
  except 16 fills (110.6%).
- **The DDR3 latency is an assumption, not a measurement.** Nothing on the board has
  timed an f2sdram read in this core. The viewer could count cycles from request to
  first beat. That number decides between 8 and 16 fills in flight (16 is 4 KiB more
  beat buffer, MLAB) and how much 64 KiB buys.
- **The only board figure** is `m2_ddr3`'s header (Model 2 R351): 13 cycles typical
  and 50 worst, a whole burst, from Model 2's self-test. MiSTer's own guidance is
  about 200 ns typical. At 13 cycles, pipelined: VF3 70.0% (nearest, 32 KiB), 60.9%
  (64 KiB), and level 0 102.2% / 74.7%. Bass Fishing is 31.7% / 31.0%.
- **`m2_ddr3` as built takes one request at a time.** Run that way at 13 cycles it is
  no better than the SDRAM: VF3 229.7% (nearest, 32 KiB), 201.4% (64 KiB), 363.5%
  (level 0, 32 KiB); Bass Fishing 77.4%. A texture port on DDR3 needs a pipelined
  master: several reads issued before the first returns, data in order. I have not
  checked that MiSTer's f2sdram bridge accepts more than one pending read.

### What the LOD needs from the geometry (not built)

The per-triangle form needs one word a polygon (a polygon's triangles share it):
- **B = d x NP x (-cota) x kz^2, FP32.**
  - d is the facing dot `r3d_face` already forms, d = (V0 N0 + V1 N1) + V2 N2. Today
    only its sign leaves the unit.
  - NP is header[5] bits 8-31 / 4096.
  - cota is viewport word 3.
  - kz is the viewport record's w scale.
- The record becomes 5 + 6n words. At about 7,300 VF3 polygons a frame that is about
  1.7 MB/s more into the store.
- **Geometry cost:**
  - d as a 32-bit output of `r3d_face`;
  - two FP32 products a polygon (d x NP, then x the viewport's constant). These fit
    the face unit's free round-3 slots: 7 of every 28 are idle, so no throughput is
    lost;
  - NP (u12.12) through the existing word converter;
  - the viewport constant -cota kz^2, two products once per viewport.
- **Not checked:** whether `r3d_face`'s d has the same scale as the model's
  dot(v, n). R65 checked only its sign. A factor of 2 is a whole level, so this has
  to be benched before anything is built on it.

Supermodel's exact form needs more, for no measured image gain (0.00 dB p50):
- a per-vertex d_i (three more products a vertex);
- a fifth plane d/w in the record (4 + 7n words), so SETUP4 gains three coefficient
  dots and the intake one field;
- the interpolants gain a plane (2 DSP, about 100 ALM);
- a log2 of that plane per pixel.

My estimate for the rasteriser side of the nearest level is 160-230 ALM, 0 DSP and
0 M10K. The parts are the log2 table (64 x 8), the subtract, clamp and round, and in
the texel path the level shift of u and v, the level bases and the size masks.
Trilinear adds the second quad's cycles above, a lerp between levels (4 products)
and about 250 ALM more.

### The texture cache split into compare and allocate (`r3d_rast_tcache.sv`)

R66's L1 did the whole lookup in one cycle: the tag word out of the M10K, the hit,
the first missing bank, the victim and the row with the new tag into the tag write
(17.5 ns, 47-50 MHz). It is now L0 (the read), C (compare) and A (allocate), with
W writing the tag a cycle after A decides:
- **C** compares the rows and registers the hits and ways. It does not keep rows: it
  reads the tag RAM again every cycle the pixel waits, and forwards the two writes
  a read cannot see. One is Wq, the write on the read's own edge (the M10K's
  read-during-write is don't-care). The other is W, the write in flight.
- **A** takes registered hits and does the allocation: the first missing bank, the
  victim, the fill queue and the row for W. The pairwise same-line and same-set
  relations of a pixel's four banks are computed once in L0 and registered. A
  carries the four rows only to build W's word, and updates them with its own
  allocations, as R66's L1 did.
- **Nothing is forwarded into A.** A pixel leaves A only on a cycle with no
  allocation (it leaves when it has no miss left), so the hits it brings from C
  already hold every allocation before it. I had planned to forward A's allocation
  into the pixel arriving behind it, with a hit fix-up, an install fix-up and two
  4 x 4 compare matrices. Working through the hand-over cycle showed that case
  cannot happen.
- **A pixel still takes k + 1 cycles in A for k distinct missing lines,** as in R66.
  The extra stage adds a cycle of latency only: the bench's frames run 1.989 against
  1.988 cycles a pixel, and the random set 12.638 in both.
- **A sweep (`inval`) holds C and A for the sweep and one cycle after,** so no row
  read under the sweep is used. It clears A's rows, so a row read before the upload
  is never written back. As before, `inval` is meant for an empty pipeline (between
  frames); a pixel in A across it keeps the hits it had.
- **The parameters stay:** SB 8 (32 KiB) or 9 (64 KiB at four ways), WAYS, DB and FB.
  The cache takes bank word addresses, so a mip level's address (base plus
  u, v >> L) needs no change here. Only the texel block's address path changes.

**The bench now checks the fills.** A C++ model of the allocation rule gives every
fill address in order: a pixel's distinct missing lines, lowest bank first; the
victim a global round-robin way, never one the pixel uses. Every fill the RTL
issues is compared with it, so a duplicate fill or a different victim is caught
even when every texel is right. R66's RTL and the split one give the same sequence
on every run (976,676 fills on the 300,000-quad random set, 196,592 on 4 VF3 frames).

Checks, 0 wrong (texels and the fill sequence):

| Set | Pixels | Fills |
|---|---|---|
| VF3, 400 frames, latency 20-40, gaps and stalls | 136,144,792 | 30,476,655 |
| Bass Fishing, 400 frames, the same | 46,929,718 | 11,596,985 |
| VF3 and Bass Fishing, 100 frames each, latency 20-60 | 28,591,281 + 20,139,953 | |
| Random, 2 M quads twice with an upload between | 4,000,000 | 6,507,796 |
| Random, latency 2-4, no gaps or stalls | 1,000,000 | 1,630,358 |
| 64 KiB (SB 9): both games 100 frames, random 2 M and 500 K | as above | |

Planted faults, 13 of 13 caught (10 VF3 frames / 200,000 random quads):

| Fault | 10 frames | Random |
|---|---|---|
| K1 beat counters a bit short | caught | - |
| K2 no fill gate | caught | caught |
| K3 victim may be this pixel's way | - | caught |
| K4 read a fill early | caught | caught |
| K5 lanes swapped | caught | caught |
| K6 no dedup of a shared line | caught | caught |
| K7 texel select swapped | caught | caught |
| K8 tag valid ignored | - | caught |
| K9 no forward of the write on the read's edge (Wq) | caught | caught |
| K10 no forward of the write in flight (W) | caught | caught |
| K12 A's rows not updated by its own allocation | - | caught |
| K13 no re-read while C waits | caught | caught |
| K14 round-robin pointer not advanced | caught | caught |

K3 and K6 were R66's two misses ("near-equivalent"); the fill check catches both.

### Block 7: the colour bands, the blend and the layer write (`r3d_rast_band.sv`)

Shaded pixels arrive one a clock in band order (the passes in turn, draw order),
each with its colour after fog and, when it blends, MAME's factor f. An end-of-band
mark follows each band. Translucency is D-29's plain blend (mode 0).
- **Two band buffers,** each 8 rows x 512 of {drawn, RGB888} in two banks by x
  parity: 4 x (2048 x 25) = 20 M10K. One is drawn while the other is written out to
  the layer and cleared for the band two ahead.
- **P0** reads the word, P1 has the colour under it, P2 the product, P3 the result,
  W the write.
- **The blend** is d + ((c - d) f >> 8). That equals render_d25's
  (c f + d (256 - f)) >> 8 exactly, because 256 d is a multiple of 256, so it is one
  product a channel, not two. It is done as d + (u f >> 8) - f with u = c - d + 256,
  which is unsigned, so red and green share one 27 x 8 product
  ({u_r, 9'd0, u_g} x f; u f < 2^17, so the halves cannot meet) and blue takes one
  of its own. That is 2 DSP blocks for 3 channels.
- **No forwarding.** A blending pixel waits in P0 while P1, P2, P3 or W holds a write
  to its word. A plain write never waits, because writes stay in order.
- **Scroll fog, render_d25's rule.** A band is cleared to the fog colour inside the
  rectangle and to 0 outside, which is what a blend reads. A pixel still undrawn at
  the flush shows the fog colour at the scroll-fog alpha. That colour is a constant
  of the frame (`sf_out`, worked out once by the control), so the flush has no
  products: six 8 x 8 products a beat would have been needed otherwise.
  `d25_scroll_fog` now holds render_d25's rule, and render_d25 calls it; its images
  are unchanged (13.63 / 19.05 dB p50, identical figures).
- **The flush is the M2.5 viewer's** (`m3v_raster`, bit-exact on the board): a row
  is one 248-beat burst, two pixels a beat in `m2_fb_read`'s form, from an 8-entry
  queue read ahead of the port. The queue is now MLAB, not 400 flops: it is read
  asynchronously and never written over an entry still to go.
- **The port** is `m2_ddr3`'s request contract (request held to its ack, a beat taken
  per `wacc`). `m2_sdram`'s ports share that contract, which is D-20's switch:
  moving the layer to the second SDRAM is that controller's port, not a change here.
  The 16-bit layer of D-6 is not built: four pixels a beat need four banks a buffer
  (12 M10K a buffer against 10), or a two-cycle beat that a write burst cannot wait
  for.
- **An end-of-band mark** waits for the pixels in flight (3-4 cycles) and for the last
  flush, then starts this band's flush and switches buffers.

Checks against render_d25, every pixel of the layer (190,464 a frame) after the
last flush, memory BUSY 30% and input gaps 10% unless stated:

| Set | Pixels written (blended) | Wrong |
|---|---|---|
| VF3, 400 frames | 147,340,341 (36,087,187) | 0 |
| Bass Fishing, 400 frames | 60,210,676 (10,889,020) | 0 |
| VF3, 100 frames, BUSY 60%, no gaps | 31,737,817 (12,421,617) | 0 |
| Random, 2,000 frames: pixels piled on a few words, half blending, random fog rectangles | 14,423,925 (7,210,583) | 0 |
| Random, 300 frames, gaps 50%, BUSY 80% | 2,156,170 (1,077,710) | 0 |

The cost of waiting instead of forwarding: VF3 34,357 cycles over 400 frames (86 a
frame) and Bass Fishing 11,418, against 1,738,530 cycles a frame. The end-of-band
waits in the bench (VF3 4,355 a frame, Bass Fishing 40,298) are an upper bound: the
bench feeds a pixel a clock with nothing upstream, so an empty band leaves only the
flush (about 2,000-2,900 cycles a band at 30% BUSY). In the rasteriser a band takes
about 36,000 cycles on average, so only near-empty bands would wait.

The pixel counts are render_d25's shaded counts exactly. The bench also runs its own
C++ reading of the rules against render_d25 (0 wrong), so the random set's model is
render_d25's.

Wrong turn: the first build gated the shading writes by "not the flush's buffer"
instead of "the buffer drawn". After reset neither buffer belongs to the flush, so
band 0 went into both buffers and band 1 showed band 0's pixels. Only the first
frame after reset shows it, and the VF3 and Bass Fishing frames draw nothing in rows
0-7, so 3 frames passed. The random set caught it (79 pixels in band 1). It is
planted as G8, which only the random set catches.

Planted faults, 11 of 11 caught (10 VF3 frames / 200 random frames):

| Fault | 10 frames | Random |
|---|---|---|
| G1 no wait for the write in W | caught | caught |
| G2 no wait for the write in P3 | caught | caught |
| G3 red without the - f | caught | caught |
| G4 green's half shifted a bit | caught | caught |
| G5 undrawn fog shows the raw colour | - | caught |
| G6 cleared for the next band, not two ahead | - | caught |
| G7 fog rectangle's right edge inclusive | - | caught |
| G8 shading writes both buffers | - | caught |
| G9 end of band without the drain | caught | caught |
| G10 lanes swapped | caught | caught |
| G11 banks swapped | caught | caught |

G5-G7 need a scroll fog, which the first 10 VF3 frames do not have.

Fit (Quartus 17.0, standalone, 10 ns, three seeds):

| Seed | ALM (pins' ALMs) | ALM without pins | Registers | M10K | DSP | MLAB bits | Fmax slow 100C |
|---|---|---|---|---|---|---|---|
| 1 | 582 (134) | 448 | 490 | 20 | 2 | 400 | 123.6 MHz |
| 2 | 579 (134) | 445 | 490 | 20 | 2 | 400 | 126.1 MHz |
| 3 | 578 (134) | 444 | 490 | 20 | 2 | 400 | 125.6 MHz |

The worst path is a band RAM's read into the flush queue (1.9-2.1 ns slack).
Section 11's estimate for this block was 450-600 ALM, 20 M10K and 2 DSP.

Area, section 8.4 technique by technique:

| Technique | Here | Effect |
|---|---|---|
| Storage out of flops | the band buffers in M10K; the flush queue (8 x 50) in MLAB, not flops | queue: 400 flops to 400 MLAB bits (registers 490) |
| MLAB / M10K trade | none; a single buffer with a row interlock would save 10 M10K (not built) | 20 M10K |
| Built once and shared | one flush engine for the frame clear, the band write and the clear for the band two ahead; each bank's one read port serves shading or the flush by role | no second engine |
| Sums in the DSP | red and green in one 27 x 8 product, {u_r, 9'd0, u_g} x f | 2 DSP for 3 channels; the plain formula would take 6 products |
| Narrower / exact rewrite | d + ((c - d) f >> 8) for (c f + d (256 - f)) >> 8; u = c - d + 256 so it packs unsigned | one product a channel, exact |
| Constants of the frame | the scroll fog's undrawn colour given as sf_out | no products at the flush (6 a beat otherwise) |
| No forwarding | a blend waits for a write in flight to its word | 4 12-bit compares; 86 cycles a VF3 frame |
| Parameters | FB_STRIDE | |
| Did not work | gating shading writes by "not the flush's buffer" | band 0 drawn into both buffers after reset (G8) |


**The cache's first split fit did not close** (`tcache2`, three seeds):
- 73.6 / 77.4 / 80.0 MHz;
- 1,965-1,989 ALM (152 of them the pins'), so 1,813-1,837 without pins against
  R66's 972;
- 864 registers, 43 M10K, and 3 DSP against R66's 1.

The worst path ran from `a_hit` through the first missing bank, the victim and a
variable part-select (`row[vic * EB +: EB]`) into A's rows and W's row word, 11.9-13.0
ns. Quartus builds `vic * EB` as a multiplier, which is where the DSPs came from.
These indexed writes were also the area: four rows of 44-bit demultiplexers, twice
over in C's forwarding.

The second form (`tcache3`):
- every row update is a loop over constant ways (`put`);
- A's rows take W's entry one cycle later, from registers;
- W builds the tag word itself from the set's row and the way, so the victim reaches
  only 2-bit registers;
- the victim is worked out for each of the four candidate banks in parallel with the
  choice of the first missing bank.

It is bit-exact on the quick sets: 300,000 random quads (976,676 fills) and 4 VF3
frames (196,592 fills), the same sequences. Its full benches pass: VF3 and Bass
Fishing 400 frames, 100 frames each at 64 KiB, and the random sets at both sizes, 0
wrong. Planted faults: 13 of 13 caught.

Fit, three seeds at 10 ns:

| Seed | ALM (pins' ALMs) | ALM without pins | Registers | M10K | DSP | Fmax slow 100C | Worst path |
|---|---|---|---|---|---|---|---|
| 1 | 933 (152) | 781 | 866 | 43 | 0 | 106.9 MHz | a_hway -> a_way |
| 2 | 942 (152) | 790 | 866 | 43 | 0 | 108.3 MHz | a_hit -> a_way |
| 3 | 942 (152) | 790 | 866 | 43 | 0 | 106.0 MHz | a_ok -> a_way |

Against R66's single cycle (972 ALM without pins, 1 DSP, 47-50 MHz): 182-191 ALM and
1 DSP fewer, and it closes.

Area, section 8.4 technique by technique:

| Technique | Here | Effect |
|---|---|---|
| Storage out of flops | C keeps no rows: the tag RAM is read again every cycle the pixel waits | no 176-flop row copy in C |
| Built once and shared | the pixel's same-line and same-set relations computed once in L0 and registered | no 18-bit compares in A's path |
| Narrower / exact rewrite | row updates as loops over constant ways (`put`), not `row[vic * EB +: EB]` | the first form's 1,813 to 781-790 ALM, 3 DSP to 0 |
| Narrower / exact rewrite | W builds the tag word from the set's row and the way, so the victim reaches only 2-bit registers | |
| In parallel, not in series | the victim for each of the four candidate banks, beside the choice of the first missing bank | the path is now about 9.2-9.4 ns |
| Parameters | SB (8 = 32 KiB, 9 = 64 KiB), WAYS, DB, FB | 64 KiB benched, not fitted |
| Did not work | variable part-selects (Quartus builds the index as a multiplier) | 73.6-80.0 MHz, +840 ALM |
| Did not need | forwarding into A (fix-ups for the pixel behind) | none built |

## 12, continued. Nearest-level sampling built (study D-33)

The nearest mip level of the hardware LOD form (R67's `lod = 1`) is now RTL, behind
`MIP` (default 0) in three blocks, with the geometry's LOD word behind `LOD_WORD`
(m2-notes section 15). The arithmetic is render_d25's `mip = 1, lod = 1` exactly.

### The LOD per pixel, in the interpolants (`r3d_rast_interp.sv`)

The pixel's log2(1/w) is read off its Z word: exponent then a 64-entry table on six
mantissa bits. The interpolants already have both for the reciprocal: the leading one
n of |o2| and the normalised mantissa m2. So the Z word's exponent is n + eo + 86 (with
the Z word's bounds: 0 for 1/w <= 0 or an exponent <= 0, all ones from 255) and its six
bits are m2[22:17]; nothing is recomputed. With L = {exponent, T[m6]}:

  lv = clamp((K - 2 L) >> 8, 0, lvmax), K = lb8 + 65152 (0 when B gives level 0)

which folds render_d25's 2 x 127 x 256 and its +128 rounding into K. One 18-bit
subtract and a compare against lvmax, formed off the S2 registers into the delay line,
which grows by four bits; `o_lv` comes out with the pixel.

### The triangle's term, in the intake (`r3d_rast_intake.sv`)

With `MIP` the header has a fifth word, B, and its tag-31 beat carries the LOD terms in
its place: [16:0] K = 97664 - {E(B), T[B's top six bits]} (0 when B is not positive and
finite), [20:17] lvmax = 4 + min(width code, height code). The table is a second copy
(eight ALM-sized LUTs).

### The level's texel, in the texel block (`r3d_rast_texel.sv`)

Level L of a page sits at Supermodel's sheet bases x 2048 - (2048 >> L), y 1024 - (1024
>> L). Both bases come out of one shift each: x0 = ((2^L - 1) 2^11 + 32 tx) >> L and y0 =
page 1024 + (((2^L - 1) 2^10 + 32 ty) >> L). The side is 2^max(0, 5 + w - L), so the
mirror masks are a decoder, and u, v and their fractions are read off u >> L and v >> L.
All of it sits in stage 1 beside the masks; stage 2 takes x0 and y0 where it took
32 tx and page 1024 + 32 ty, so `MIP = 0` is the form before.

### Checks (render_d25, `mip = 1, lod = 1`; `make test_real3d_rast100_mip`)

| Block | Frames | Cases | Wrong |
|---|---|---|---|
| Texel, each pixel at its level | 400 VF3 + 400 Bass Fishing | 136,169,683 + 46,930,728 texels | 0 |
| Texel, random levels up to each texture's highest | | 3,000,000 | 0 |
| Interpolants, the level of every textured pixel (and u, v, shade, r, n as before) | 400 + 400 | 149,078,975 + 61,645,760 pixels | 0 |
| Interpolants, random B (none, negative, zero, infinite, NaN, any positive) and lvmax 4-15 | | 3,000,000 | 0 |
| Intake, the fifth beat (random B, random size codes) | 40 VF3 frames | 338,783 triangles | 0 |
| Intake, random records | | 500,000 | 0 |
| Texture cache, each pixel's quad at its level, fill sequence address by address | 400 + 400 | 136,169,683 + 46,930,728 pixels, 13,966,763 + 5,363,845 fills | 0 |

With `MIP = 0` every block's own bench passes unchanged on 400 + 400 frames and its
random sets (texel 136.1 M + 46.9 M texels, interpolants 149.1 M + 61.6 M pixels, intake
on R65's and R62's records).

Planted faults (`faults_rast.py` M1-M12): 12 of 12 caught: the LOD one level up, K's
rounding half a level off, the x and y sheet bases, no upper clamp, lvmax one high, no
lower clamp, the level-0 fraction, the level-0 mask, the Z exponent one off, B's sign
not checked, the table read one bit low.

### Fills with the hardware LOD, and with the geometry's B (`r3d_tcache_sim --mip 1 --lod 1 [--geo-lod VROM]`)

The RTL's level is render_d25's, so R67's per-triangle figures stand; the tool gives them
again to the fill (VF3 224,472 p99 at 32 KiB, 75.0% of a frame on DDR3 at latency 30).
But render_d25 forms B from MAME's list, and the list is not what the geometry sends.
`--geo-lod` runs the geometry model on each dump (MAME's frustum, every node walked, so
the polygon numbers match) and gives each polygon the B of the record the geometry
sends.

p99 over 400 frames, 32 KiB / 64 KiB:

| Form | VF3 fills | VF3 DDR3 L30 | VF3 SDRAM map 1 | Bass fills | Bass DDR3 L30 | Bass SDRAM map 1 |
|---|---|---|---|---|---|---|
| R67 per pixel (Supermodel's LOD) | 192,391 / 166,758 | 75.9% / 64.9% | 250.5% / 220.6% | 58,200 / 49,545 | 34.3% / 32.5% | 76.7% / 67.5% |
| B from the list (render_d25 `lod = 1`, the RTL's level) | 224,472 / 200,620 | 75.0% / 67.9% | 292.5% / 263.7% | 58,344 / 49,648 | 34.3% / 32.5% | 76.8% / 67.6% |
| B from the geometry, the polygon's own record (32 KiB) | 220,917 | 75.0% | 292.4% | 58,365 | 34.3% | 76.9% |
| **B from the geometry, the record sent** | **53,790 / 47,015** | **56.5% / 55.7%** | **77.5% / 71.1%** | **41,569 / 33,831** | **31.4% / 29.7%** | **58.3% / 50.7%** |

(Textured cycles only, as R66 and R67 count them; the SDRAM carries texture traffic and
nothing else.)

The difference is the double-sided polygon facing away. MAME's list draws it once with
its own normal, so B < 0 and render_d25 (both LOD forms, and R67's figures) puts it at
level 0. The geometry, like Supermodel, sends its inverted copy instead, whose B is -B > 0,
so it gets its level. With the polygon's own B the geometry's figure is the list's
again (220,917 against 224,472; Bass Fishing 58,365 against 58,344), so that one rule is
the whole change. Minified pixels go from 32.8% to 34.2% (VF3) and 35.1% to 42.1% (Bass
Fishing), but the p99 frames are the ones where large double-sided surfaces face away,
and those lose three quarters of their fills.

This bears on D-33: R67's DDR3 figures for the nearest level overstate VF3's p99 by 19
points at 32 KiB (75.9% against 56.5% at latency 30), and the 16-bit SDRAM that R67 ruled out (220.6% at best) is 71-79% for
VF3 and 51-58% for Bass Fishing on this form, texture traffic alone.

Against Supermodel's grabs the image does not move (`r3d_rast_ref --d25 --mip 1 --lod 1
--geo-lod`): VF3 13.72 dB p50 frame, Bass Fishing 19.16 dB, against 13.72 and 19.17 with
the list's B. Coverage disagreement 4.55% against 4.50% (VF3; alpha-tested texels at the
new levels) and 2.22% unchanged.

### Area (Quartus 17.0, 10 ns, standalone, ALM without the virtual pins)

| Block | Seed | ALM (pins' ALMs) | ALM without pins | Against `MIP = 0` (R66) | Registers | M10K | DSP | Fmax slow 100C |
|---|---|---|---|---|---|---|---|---|
| Texel, `MIP = 1` | 1 / 2 / 3 | 865 / 863 / 862 (146) | 719 / 717 / 716 | +189 to +192 (527-528) | 618 | 0 | 12 | 114.5 / 113.2 / 115.0 MHz |
| Texel, first MIP form (21-bit window, all size codes) | 1 / 2 / 3 | 897 / 902 / 896 (146) | 751 / 756 / 750 | +223 to +229 | 624 | 0 | 12 | 114.4 / 113.4 / 114.7 MHz |
| Interpolants, `MIP = 1` | 1 / 2 / 3 | 1,571 / 1,574 / 1,568 (330) | 1,241 / 1,244 / 1,238 | +46 to +49 (1,192-1,195) | 1,551 | 0 | 15 | 113.3 / 121.8 / 123.7 MHz |
| Intake, `MIP = 1` | 1 / 2 / 3 | 340 / 344 / 339 (43) | 297 / 301 / 296 | +27 to +32 (269) | 439 | 0 | 0 | 146.2 / 151.9 / 150.2 MHz |

Nearest-level sampling costs the rasteriser **+263 to +273 ALM, 0 M10K, 0 DSP**, against my
estimate of 160-230: about 40 over its top. The texel's shifts are most of it. Every
seed closes 10 ns; the texel's worst path is still the bilinear lerp (1.26 ns of slack),
not the level logic. Rasteriser built so far: about 6,240 ALM without pins, 75 M10K and 45
DSP.

The first MIP texel kept u >> L 21 bits wide and the bases full width, exact for size codes
6 and 7 too. The geometry never sends those (it ends a model at a code of 6 or more, as
MAME does), so the form now is exact for codes up to 5 and levels up to 9: u >> L 19 bits
wide, the bases from 20 and 19 bits. That saved 34 ALM in the fit (40 at map).

Area, section 8.4 technique by technique:

| Technique | Here | Effect |
|---|---|---|
| Built once and shared (pathway) | the LOD's log2(1/w) from the reciprocal's own leading one and normalised mantissa (the Z word's exponent n + eo + 86 and its six bits), not from a second normaliser of the Z word | about 30-40 ALM not built (my estimate: a 41-bit leading-one and shift) |
| Built once and shared | the +128 rounding and 2 x 127 x 256 folded into the triangle's K; one 18-bit subtract a pixel, then a compare with lvmax | no separate round or offset adders |
| Built once and shared | each sheet base and the texture's place in one shift: ((2^L - 1) 2^11 \| 32 tx) >> L, an OR for an add (the two fields cannot overlap) | no 11-bit adders, no MIPX / MIPY tables |
| Narrower / exact rewrite | u >> L 19 bits wide and the bases from 20 and 19 bits: exact for every record the geometry sends | -34 ALM (fit) |
| Storage out of flops | the level rides the interpolants' MLAB delay line (four bits more), not a register chain | about 40 flops not built |
| Per-triangle work out of the pixel path | K and lvmax formed once a triangle in the intake's raw-field path (one table, one subtract), sent as the fifth header beat | no per-pixel table for B |
| Parameters | `MIP` (texel, interpolants, intake) and `LOD_WORD` (geometry), all default 0 | default builds unchanged |
| Did not work / not built | shifting u in the interpolants' quotient shifter (it is already a barrel shifter): a negative u's logical shift needs a sticky of the bits shifted out, which costs a wider shifter than it saves | estimated, not built |

### Wrong turns (mip)

- **The geometry model was not thread-safe.** Its matrix stack (`g_stack`) was a file-scope
  global, and the fill and image tools run it in 12 threads. My first `--geo-lod` runs gave
  55,566 p99 fills (VF3, 32 KiB) and changed between runs (57,333). The stack is now
  `thread_local`; one thread and twelve now give the same figures to the fill on 40
  frames. No single-threaded result (the geometry benches, `r3d_ref`) was affected.
- R67's fill figures for the per-triangle LOD came from MAME's list, which has no
  inverted copies, so a facing-away double-sided polygon stayed at level 0. It is the
  largest error in R67's table, larger than the per-triangle against per-pixel
  difference it reported.
- I typed the log2 table by hand first; about a third of the entries were wrong. It is now
  generated from 256 log2(1 + (i + 1/2) / 64).
