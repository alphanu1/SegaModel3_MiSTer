# M2 working notes: Real3D geometry

These are scratch notes for M2: the golden data, the reference model, the
MAME/Supermodel differences and the architecture I intend to build. They are not
the study. Anything here that changes the study goes into it as an R-entry.

## 1. Golden data

MAME 0.289 with `tools/mame-census/mame0289-real3d-tris.patch` replays
Supermodel's Real3D dumps (one per frame, every 30th emulated frame of attract)
and writes each triangle exactly as `draw_opaque_triangles` / `draw_alpha_triangles`
receive it. The patch also writes the viewport, model and per-polygon (pre-clip)
records, so a difference can be traced to its stage. The format is in
`r3d_trilist.h`.

| Set | Frames | Polygons / frame (mean, p99) | Triangles / frame (mean, p99) | Size |
|---|---|---|---|---|
| vf3, attract, `dumps_vf3` f001800-f013770 | 400 | 13,161 / 16,612 | 9,037 / 15,494 | 1.5 GB |
| bassdx, attract, `dumps_bassdx` | 400 | 7,828 / 23,916 | 4,578 / 16,589 | 0.8 GB |

Files: `sm3-builds/r3d/tris_vf3/*.tri`, `sm3-builds/r3d/tris_bassdx/*.tri`, and the
VROM images MAME used, `sm3-builds/r3d/vrom_{vf3,bassdx}.bin` (ROM-derived, never in
the repository).

## 2. Reference model and match rates

`r3d_geo.cpp` walks a dump the way MAME does and runs every FP32 operation in
MAME's order. `make test_real3d R3D_GAME=vf3 R3D_ARITH=rtl` builds it and diffs
it against the golden lists.

Arithmetic forms, and what they give over all 400 + 400 frames:

| Form | VF3 | Bass Fishing |
|---|---|---|
| host: MAME's expressions, libm for the frustum | every field bit-exact (384/400 frames; the other 16 differ only through MAME's undefined polygons) | every field bit-exact, 400/400 frames |
| rtl, `--setup cr`: affine matrices with constant products skipped, bit-level Newton reciprocal, exponent halver, correctly rounded atan2/tan setup | every field bit-exact | every field bit-exact |
| rtl, `--setup cr --ftz`: as above with subnormals flushed | every field bit-exact | every field bit-exact |
| rtl, algebraic setup: tan(atan2(y,x)/2) as y / (sqrt(x^2+y^2) + x) | 0 frames bit-exact; screen x, y within 1/16 px on 100% (max 0.053 px) | 174/400 frames bit-exact; x, y max 0.0025 px |

"Every field" excludes 308 VF3 polygons (0.006%) that MAME computes from
uninitialised memory (below); they move the triangle count in 16 of 400 VF3 frames.
Field-level rates for the algebraic setup, VF3 (exact % / within 1 ulp %):

| Field | exact | 1 ulp | within 1/16 unit | max |diff| |
|---|---|---|---|---|
| projection matrix | 89.34 | 94.67 | | 9.5e-7 |
| view-projection matrix | 65.50 | 82.59 | | 0.375 |
| screen x | 74.65 | 90.10 | 100.000 px | 0.017 px |
| screen y | 63.64 | 81.84 | 100.000 px | 0.053 px |
| z | 99.65 | 99.999 | | 1.16 |
| w | 98.61 | 99.58 | | 2.19 |
| u/w, v/w (unit 1/256 texel) | 98.6 | 99.3 | 99.9998 | 10.5 (values near 2^24) |
| shade | 99.67 | 99.77 | | 0.025 |
| everything else | 100 | | | |

What the arithmetic results establish:

- **The only stage that is not bit-exact by construction is the per-viewport
  frustum set-up.** MAME computes `tanf(atan2(y, x) * 0.5f)`, with atan2 in double
  rounded to float. glibc's `tanf` is correctly rounded on all 1,070,141,403 floats
  in [0, pi/2) (checked against `tanl` rounded once), and `(float)atan2(double)`
  matched the once-rounded `atan2l` on 50 M random pairs. A correctly rounded
  atan2/tan unit therefore reproduces MAME exactly. The algebraic form is 1-2 ulp
  off in the projection matrix and moves screen x/y by at most 0.053 px.
- **The correctly rounded reciprocal** (64-entry seed, two Newton steps, one
  remainder test) equals `1.0f / x` for every mantissa at 16 exponent/sign
  combinations and on subnormal inputs (`make test_real3d_fp`). The Newton estimate
  is never more than one unit out, so one remainder test in each direction
  suffices.
- **Skipping constant products** (zeros and ones of the affine matrices, the
  coordinate matrix and the projection) is exact, provided the RTL keeps the IEEE
  sign of zero: x + (+0) is +0 when x = -0. The model computes those terms without
  counting them, so the RTL must reproduce that sign rule.
- **No subnormal reaches the geometry** in 800 frames: FTZ units give identical
  output, so the cheaper SUBNORM=0 fp_mul/fp_add serve.
- **Pre-clip clip-space vertices, matrices and viewport fields** are 100% exact in
  every form except the algebraic setup's projection.

### MAME reads an uninitialised vertex slot (VF3)

`draw_model` zeroes `prev_vertex[]` per model, but `vertex[4]` is a stack array
that is never initialised. When a polygon reuses slot 3 before any quad in that
model has written it, MAME reads stale stack data. The stale data reaches
`prev_vertex[]` and later polygons through the copy. VF3 does this on 308 polygons
in 400 frames: header word 0 = 0x...C9 (reuse slots 0 and 3) right after a triangle.
In MAME the result depends on whatever the previous call left on the stack; I saw
both a plausible stale vertex and 3e36. Supermodel keeps `m_prev[]` as a member
that persists across models (and frames). The model follows Supermodel; the diff
tracks slot definedness exactly as MAME copies slots and excludes those polygons.
**The M2 exit test cannot be "frame for frame" on these polygons.**

## 3. MAME and Supermodel: where traversal and decode disagree

"Followed" is what the reference model and the M2 RTL do. M2's exit test is
MAME's list, so MAME is followed throughout, except where MAME is undefined.
"Per frame" is the mean over the 400 dumps (VF3 / Bass Fishing).

| # | Item | MAME 0.289 | Supermodel 036af1c | Per frame | Followed |
|---|---|---|---|---|---|
| 1 | Viewport chain order | recurses to the end of `node[1]` first, so the last viewport draws first; one walk per priority pass | chain order, once; priority sorted at draw | 8.2 / 5.2 viewports | MAME |
| 2 | Viewport disable `node[0] & 0x20` | ignored | viewport skipped | 0 / 0.03 | MAME |
| 3 | Viewport with subnormal base matrix | drawn | skipped (VF3 boot work-around) | not seen | MAME |
| 4 | Frustum | `tan(atan2(plane)/2)` from words 12-19, near 1, far 100000 | words 8-11 as tangents, reversed-Z, no far plane | every vertex | MAME |
| 5 | Node culling | none: every node and model is walked | node radius against 4 planes; nodes outside culled | ~50% of polygons are clipped away whole | MAME (M2); see §5 |
| 6 | LOD list `node[0] & 8` | always entry 0 | entry chosen by distance and blend radius, blended with alpha | 6.4 / 106 nodes | MAME in the RTL; Supermodel in the model (§11) |
| 7 | Sibling link | after the child, skipped when `node[0] & 7 == 6` | **before** the child; same rule, plus point refs | every node | MAME |
| 8 | Node discard `0x300` | ignored | node skipped | 0 / 0 | MAME |
| 9 | Reset matrix `node[0] & 0x80` | ignored | rotation reset to the viewport's | 0 / 0 | MAME |
| 10 | Link type decode | `link >> 24`: 0 node, 1 and 3 model, 4 list, others ignored | `(link >> 24) & 5`: 0 node, 1 model, 4 list, so 2 and 8 walk as nodes | 3.6 / 0 links of other types | MAME |
| 11 | Pointer lists | scan to the end, walk **backwards**; stop at an entry with any upper-byte bit; nesting deeper than 3 dropped | forward; entries with `0x01000000` skipped, not ending the list; no nesting limit | 8.2 / 5.2 lists, 0 null entries | MAME |
| 12 | Colour table (indexed colour) | `polygon_ram[0x400 + (h4 >> 8 & 0x7ff)]`, fixed base | base from node words 3, 7, 8 (`node[0] & 4`), 12-bit index | 68 / 0 polygons | Supermodel (§11) |
| 13 | First polygon reuses a vertex | model ends | decoded | 9.7 / 0 | MAME |
| 14 | Both discard bits on a polygon | model ends | that polygon skipped, model continues | 1.5 / 0 | MAME |
| 15 | One discard bit (`0x200` only) | drawn | transparency halved | 2.7 / 0 | MAME |
| 16 | Texture size field >= 6 | model ends (after the polygon is transformed) | size taken as 32 | 1.0 / 2.6 | MAME |
| 17 | Transparency | 5 bits, `h6 >> 18 & 0x1f`, 32 = opaque | 6 bits, `* 255 / 32`; transparency 0 polygons dropped | 0.5 / 3.0 with transparency 0 | Supermodel (§11) |
| 18 | Vertex normals | always the vertex's s.7 normal / 127 | only with smooth shading (`h1 & 8`), else the face normal; (2b + 1) / 255 | 5 / 102 flat polygons | Supermodel (§11) |
| 19 | Normal transform | by the transpose of the model matrix | normal matrix in the shader | lit polygons, 83% | Supermodel (§11) |
| 20 | Fixed shading `h1 & 0x20` | ignored | vertex byte as the shade | 1.3 / 0.5 | Supermodel (§11) |
| 21 | Double-sided `h1 & 0x10` | drawn once, no culling (back-face test computed, then forced off) | drawn twice with flipped normals | 1,323 / 2,460 | Supermodel (§11) |
| 22 | Light vector | `(n5, n6, n4)`, intensity unclamped, ambient / 256 | `(n5, -n6, -n4)`, intensity clamped to 1, ambient / 255 | every lit vertex | Supermodel, per vertex (§11) |
| 23 | Reused-vertex storage | `vertex[]` uninitialised per model (UB) | persists across models | 308 VF3 polygons in 400 frames | **Supermodel** |

Rows 5, 6, 12, 17 and 22 change what is drawn, not just the order. If M3 makes
Supermodel the image oracle, they are the first candidates to switch.

**Wrong turn, recorded.** From the viewport words I first read row 4 as MAME
drawing VF3 2.08x magnified. MAME's VF3 left-plane normal is (0.3827, 0.9239),
which is 22.5 deg. MAME halves it again to 11.25 deg (P00 = 4.82), while Supermodel's
words 8-11 give tan = 0.414, which is 22.5 deg. I checked by eye:
`supermodel-frame-grab.patch` grabs Supermodel's finished frames 7800, 7860 and
7920, and the dumps it writes there are byte-identical to the census set. MAME
replaying those dumps with words 8-11 as the frustum came out about 2x *smaller*
than Supermodel's own frames, which match MAME's native scale. So MAME's half-angle
gives the right image scale. The factor turned up while building culling: MAME
puts the frustum's -1 at P[3][2] and -2fn/(f-n) at P[2][3], transposed for its
row-vector convention, so clip w comes out as about 2 x depth (P23 = -2.00004)
instead of depth. Halving the half-angle tangent doubles P00 and P11, which
cancels it. MAME's image scale is right, by two compensating departures from a
textbook frustum. The grabs: `sm3-builds/r3d/compare_sm_over_mame.png` and
`compare_sm_over_mame_smproj.png`. Supermodel's frame at a given number also does
not show the same poses as MAME's render of that frame's dump, which fits
Supermodel drawing from its ping-pong copy, captured earlier than the dump.

## 4. Measured load (MAME traversal, 400 frames each)

| Per frame | VF3 p50 / p99 / max | Bass p50 / p99 / max |
|---|---|---|
| Viewports (mean) | 8.2 | 5.2 |
| Nodes | 1,255 / 1,831 / 1,919 | 170 / 2,812 / 3,416 |
| Models | 743 / 1,096 / 1,165 | 99 / 1,353 / 1,662 |
| Polygons | 13,691 / 16,612 / 17,082 | 8,995 / 23,916 / 25,572 |
| New vertices | 35,940 / 46,278 / 46,856 | 17,522 / 46,404 / 49,603 |
| Output vertices (after clip) | 22,709 / 34,930 / 35,868 | 4,570 / 34,529 / 38,718 |
| Triangles | 9,859 / 15,494 / 16,004 | 2,120 / 16,589 / 16,589 |
| Matrix stack depth (max over all frames) | 8 | 11 |
| Pointer-list nesting (max) | 1 | 1 |
| Polygons entering the clipper (mean) | 7,784 (59%) | 5,302 (68%) |
| Clip divides | 1,698 / 3,252 / 3,736 | 524 / 1,706 / 1,922 |

Most polygons that enter the clipper are rejected whole: VF3's 7,784 a frame
produce 1,796 divides. An outcode test with trivial reject keeps them off the
datapath.

FP operations per frame, with shared vertices kept (transform and light once per
new vertex, projection once per vertex while the polygon is unclipped):

| | VF3 p50 / p99 / max | Bass p50 / p99 / max |
|---|---|---|
| Multiplies | 1,076,229 / 1,225,308 / 1,262,858 | 481,417 / 1,292,039 / 1,348,272 |
| Adds | 888,077 / 1,007,616 / 1,058,424 | 394,633 / 1,054,253 / 1,096,344 |
| Reciprocals | 16,594 / 25,608 / 25,830 | 2,769 / 20,670 / 22,259 |

Per new vertex: 12 multiplies for position, 9 for the normal, 5 for light (14 of
them skipped on unlit polygons). Per output vertex: 1 reciprocal, 8 multiplies
(5 by 1/w, 3 viewport scales), 3 halvings. Per matrix node: 36 multiplies, 27 adds
(translation nodes 9 and 9). Per model: 36 multiplies (top x coordinate, x
projection), 9 adds. Without the vertex cache, as MAME does it, VF3 p99 is 1.73 M
multiplies.

Memory words read per frame (MAME's walk reads every model; culling would cut
this):

| Region (placement) | VF3 p50 / p99 | Bass p50 / p99 | MB/s at p99, 57.52 Hz |
|---|---|---|---|
| Culling RAM 8C (SDRAM) | 6,510 / 9,219 | 1,001 / 19,901 | 2.1 / 4.6 |
| Culling RAM 8E (SDRAM) | 4,780 / 8,369 | 676 / 2,679 | 1.9 / 0.6 |
| Polygon RAM (DDR3, D-6) | 97,379 / 125,404 | 0 / 6,821 | 28.9 / 1.6 |
| VROM (SDRAM) | 154,307 / 188,057 | 126,232 / 347,214 | 43.3 / **79.9** |

A polygon is 7 header words plus 4 per new vertex: 18.0 words (72 B) on average
in VF3, 15.1 words (60 B) in Bass Fishing. Bass Fishing's VROM model fetch at p99,
80 MB/s, is twice the 40 MB/s the study carries for VF3. That is a single-SDRAM
budget item (study open question 4).

## 5. Architecture (M2 RTL), Real3D clock 50 MHz (D-3)

```
 culling RAM (SDRAM)          polygon RAM (DDR3) / VROM (SDRAM)
        |                                   |
   [walker] --events--> [matrix unit] --model start--> [model fetch + prefetch FIFO]
   viewport chain,      stack 32 x 12 words              header + vertex words
   nodes, lists,        (2 M10K), node x stack                 |
   link decode          on the FP pool                    [decoder] prev-vertex slots,
        |                     | per model: top x coord x proj    s17.7 -> FP32, s.7/127 table
   [viewport setup] ----------+---------> vp matrix, light --> [vertex T&L] --+
   tangents, projection,                                       12+9+5 mul      |
   light, viewport                                                             v
                                                       [outcode / trivial reject]
                                                                |      \
                                                       [clipper, 7 planes]  (unclipped)
                                                                |      /
                                                       [1/w (Newton, DSP)] -> [x 1/w, viewport]
                                                                |
                                                       triangle FIFO -> rasteriser (M3)
         FP pool: N fp_mul + N fp_add (round to nearest, SUBNORM=0), 1 fp_div (clip t, setup)
```

- **Walker.** Takes the place of MAME's recursion with an explicit return stack
  (MLAB): block frames hold the pending sibling link, list frames the list base and
  index. The viewport chain is collected into a 64-entry buffer and walked in reverse,
  once per pass. List entries are re-read backwards instead of buffered (347 VF3
  entries a frame, so the second read is negligible). It emits PUSH, TRANSLATE,
  MATRIX, MODEL, POP and VIEWPORT events through a FIFO, so it runs ahead of the
  arithmetic and its dependent reads overlap it.
- **Matrix unit.** A 32-entry stack (measured maximum depth 11) of 12-word affine
  matrices. The constant column is not stored. Node multiplies run on the shared
  pool: 36 multiplies, or 9 for a translation.
- **Model fetch.** A prefetch FIFO per model, as Model 1's `m1_geo_walk` does for
  ROM records. Polygon RAM comes from DDR3 and VROM from SDRAM, both prefetchable
  because the model address is known when the walker emits MODEL.
- **Vertex T&L.** A fixed schedule, as `m2_geo_xform` does: MAME's left-to-right
  sums, products issued across vertices. Four shared vertex slots hold the
  transformed and lit results.
- **Clip.** An outcode test first. Only crossing polygons go through MAME's
  Sutherland-Hodgman order: W, -X, +X, -Y, +Y, -Z, +Z, with a 16-entry vertex buffer
  in MLAB and the divide on fp_div. MAME's buffer is 10 entries; none of the 800
  frames exceeded it.
- **Projection.** The correctly rounded Newton reciprocal, pipelined, in DSPs
  (Model 1's `m1_geo_recip`, plus the remainder test), then 5 + 3 multiplies.
- **Set-up.** Per viewport, a handful a frame. The algebraic form needs a square
  root and divides on the slow units. A CORDIC atan2/tan with about 50-bit
  internals would make it bit-exact.

### Cycles per frame at 50 MHz (869,265 cycles at 57.52 Hz)

Model: the busiest unit's issue count divided by its unit count, or the model
fetch at one word a cycle, whichever is larger. Plus 16 cycles per node and model
for dependent reads, and 30 per divide. This is an issue-count bound: Model 2's
pool runs at about 65% occupancy, so divide the shares by ~0.7 for a schedule
estimate.

| Pool | VF3 p50 / p99 / max | Bass p50 / p99 / max |
|---|---|---|
| 2 fp_mul + 2 fp_add | 72.1% / 81.3% / 83.9% | 29.9% / 87.6% / 90.3% |
| 3 + 3 | 51.5% / 58.7% / 59.7% | 20.5% / 62.7% / 64.7% |
| 4 + 4 | 41.2% / 47.2% / 47.8% | 17.2% / 53.7% (fetch-bound) / 57.0% |

Two multipliers do not hold once the 0.7 occupancy factor is applied. Three hold
VF3 and Bass Fishing at about 84-90% of a frame. Four leave the margin that
Daytona 2 needs: R20 has 67,664 new vertices at p99, which scales to about 1.8 M
multiplies, 52% of a frame at 4 + 4 before occupancy. **I plan 4 + 4.** Supermodel's
node culling (row 5) would roughly halve all of this, and the fetch with it.

### Area (estimates, from my Model 1 and Model 2 blocks)

| Block | ALM | M10K | DSP | Basis |
|---|---|---|---|---|
| Walker, return stack, viewport chain, memory port | **744 (fitted)** | 0 | 0 | `r3d_walker`, section 6 |
| Matrix stack and sequencer | 250-400 | 2 | 0 | 32 x 12 x 32 bit |
| Model fetch FIFOs and decoder (int24 to FP32, normal table, 4 slots) | 500-700 | 2-3 | 0 | normal table 256 x 32 |
| FP pool: 4 fp_mul, 4 fp_add, 1 fp_div, grant logic | 2,600-3,000 | 0 | 4 | fp_mul 144 ALM + 1 DSP each (Model 1 M0); Model 2 pool 1,226 for 1 + 1 + 1 |
| Vertex T&L schedule | 250-350 | 0 | 0 | `m2_geo_xform` |
| Reciprocal, correctly rounded | 300-450 | 0 | 5-6 | `m1_geo_recip` 4 DSP + remainder test |
| Clipper (outcodes, 7 planes, MLAB buffer) | 1,000-1,300 | 0 | 0 | Model 2 clipper 1,078 (different algorithm) |
| Projection and viewport | 200-350 | 0 | 0 | Model 2 projector 343 |
| Viewport set-up (algebraic / CORDIC) | 250 / 800 | 0 | 0 | |
| Triangle FIFO to the rasteriser (word stream) | 100 | 2-4 | 0 | |
| **Total** | **6,200-7,650** | **6-9** | **9-10** | study 8.1 row: 6,000-8,000 |

Against Model 2's measured `m2_geometry` (4,609 ALM with a 1+1+1 pool), the
increase is the three extra multiply/add pairs (about 1,800 ALM), the walker and
the matrix stack. With 3 + 3 the total drops by about 600 ALM.

## 6. Walker RTL (first block)

`rtl/real3d/r3d_walker.sv` follows the walker above. Its events (VP, PUSH,
TRANS, MAT, MODEL, POP, END) are compared one by one against the reference model's
trace, frame by frame (`make test_real3d_walker R3D_GAME=vf3|bassdx`):

| Set | Frames matching | Events | Latency / consumer stall | Walker cycles p50 / p99 / max | p99 share of a 50 MHz frame |
|---|---|---|---|---|---|
| VF3 | 400 / 400 | 1,778,540 | 8 / 20% | 42,097 / 61,966 / 64,757 | 7.1% |
| VF3 | 400 / 400 | 1,778,540 | 20 / 50% | 68,605 / 101,261 / 105,861 | 11.6% |
| Bass | 400 / 400 | 777,530 | 8 / 20% | 5,366 / 88,055 / 111,358 | 10.1% |
| Bass | 400 / 400 | 777,530 | 1 / 0% | 3,579 / 57,181 / 71,707 | 6.6% |

One request is in flight at a time: a node is one 7-word burst, a viewport two
single reads, a list entry one read. The first run matched 395 of 400 VF3 frames:
VF3 chains up to 28 viewports, which overflowed a 16-entry chain buffer. The
buffer is now 64 entries.

Standalone fit (Quartus 17.0, 5CSEBA6U23I7, 50 MHz constraint, three seeds,
`quartus/r3d_walker`): **741-744 ALM, 622 registers, 0 M10K, 0 DSP, 98.7-106.7 MHz**
(slow 100C). The return stack (64 x 46) and the viewport chain (64 x 34) are in
MLAB. On the way there:

| Build | ALM | Registers | M10K | Fmax | What was wrong |
|---|---|---|---|---|---|
| first | 1,993-2,008 | 3,793 | 1 | 79.5-83.4 | stack written from three branches, so not inferred as RAM; node words copied out of the read buffer |
| one write port | 3,077-3,085 | 3,740 | 1 | 70.4-76.8 | still registers: Quartus 17 rejects an unregistered read of a word written in the same cycle ("unsupported read-during-write") |
| `no_rw_check` on the stack | 1,671-1,684 | 2,786 | 0 | 98.9-99.4 | the viewport buffer, moved from M10K to MLAB, hit the same rule |
| `no_rw_check` on both | **741-744** | **622** | **0** | **98.7-106.7** | |

## 7. Geometry datapath RTL

`rtl/real3d/r3d_geo.sv` is the whole geometry: walker events in, triangles out,
exactly as the reference model (`r3d_geo.cpp`) and so as MAME. Blocks:

| Block | File | What |
|---|---|---|
| Engine | `r3d_vpu.sv` | 4 lanes, each r3d_fp_mul into r3d_fp_add: the 4 + 4 pool. One op a cycle per lane, r = (a * b) + c; a pure product adds -0, which IEEE leaves unchanged, so MAME's separate multiplies and adds come out bit for bit. 128-word register file per lane (two MLAB copies), a 4-bank broadcast file (lane j writes bank j), a constant table, and the correctly rounded reciprocal shared by the lanes |
| Micro-code | `r3d_ucode.sv`, generated by `sim/real3d/gen_ucode.py` | kernels list-scheduled against the engine's fixed latencies (no interlocks): node matrix product 44 cycles, model set-up 84, T&L 130 per 16 vertices (81-100 for 4-12), projection 71 per 16 (47-57 for 4-12), clip terms 22, lerp 22, culling 41 + 31 |
| Reciprocal | `r3d_recip.sv` | 128-entry seed, two Newton steps, remainder test; 9 stages, 5 DSP |
| Set-up | `r3d_setup_cr.sv` (SETUP_CR = 1, default) | tanf((float)atan2(y, x) * 0.5f) by 64-bit CORDIC, about 210 cycles a call, four per viewport; or the algebraic form on r3d_fsqrt + r3d_fp_div (SETUP_CR = 0) |
| Clip divide | `r3d_fp_div.sv` | MAME's t = fabs(num / wdiv), correctly rounded, 29 cycles |
| Stream | `r3d_mstream.sv` | model words from polygon RAM / VROM, 8-word bursts, four in flight, 64-word FIFO; colour-table reads between bursts |
| Decoder | in `r3d_geo.sv` | headers, shared-vertex slots, 17.7 and s.7 conversion, chunks of up to 16 vertex transforms, staged in two banks while the engine works on the previous chunk |
| Control | in `r3d_geo.sv` | viewport set-up, matrix stack (32 deep, MLAB), model set-up, T&L, assembly with per-plane outcodes, MAME's clip (planes where every vertex is inside are skipped, where every vertex is outside end the polygon: the same result), projection queue of 16 vertices, triangle fans out |
| Walker | `r3d_walker.sv` | as before, plus the culling handshake (CULL) |

### Checks

| What | Result |
|---|---|
| r3d_fp_mul, r3d_fp_add (SUBNORM = 1) against host float, edge-biased random | 78.7 M ops each, 0 wrong |
| Reciprocal against 1.0f / x: every mantissa at 7 exponents, plus random | 76.8 M, 0 wrong (subnormal in or out, outside the geometry's range, documented) |
| r3d_fsqrt against sqrtf: every mantissa at 2 exponents | 16.8 M, 0 wrong |
| CORDIC set-up RTL against its integer model / against libm | 958,556 inputs: 0 / 0 different (model alone: 8 of 19.2 M differ from libm, all at angle < 2^-20 rad) |
| Engine kernels against host float in MAME's order | 68,400 results, 0 wrong |
| **r3d_geo against the model, every triangle field**, memory latency random 12-20 cycles per request, consumer not ready 20% | **VF3 400/400 frames (3,614,728 triangles), Bass Fishing 400/400 (1,831,136)**, for SETUP_CR = 1, SETUP_CR = 0 and CULL = 1 (culled lists against the culling model) |

Planted faults, each in a copy of the RTL, run on 20 VF3 and 20 Bass Fishing frames:

| Fault | Caught |
|---|---|
| F1 matrix stack pop restores the wrong level | yes, every frame |
| F2 shared vertex taken from the wrong slot | yes |
| F3 clip planes -X / +X swapped in order | yes |
| F4 reciprocal without its rounding correction | yes (x, y, z, u/w, v/w) |
| F5 viewport x and y offsets swapped | not in these 40 frames (every viewport there is at 0, 0); yes over all 400 Bass frames (66 differ) |
| F6 priority pass label | yes |
| F7 colour-table base off by one | yes (VF3; Bass has no indexed colour) |
| F8 lit / unlit shade swapped | yes |
| F9 shade clamp at 255 removed | yes (VF3) |
| F10 texture x low bit | yes |
| F11 viewport width and height swapped | yes, every frame |
| F12 staging bank of the decoder hand-off swapped | yes, every frame |
| F13 CORDIC set-up rounding truncates | yes |

### Load, memory and cycles (bench, 400 frames, 12-20 cycle latency)

| | VF3 | VF3, CULL | Bass | Bass, CULL |
|---|---|---|---|---|
| Cycles p50 / p99 | 1.36 M / 1.54 M | 1.10 M / 1.38 M | 0.47 M / 1.76 M | 0.30 M / 1.23 M |
| p99 share of a frame at 50 / 100 MHz | 177% / 88.5% | 159% / 80% | 202% / 101% | 141% / 71% |
| Culling RAM words p99 (MB/s) / requests | 19,793 (4.6) / 4,058 | 20,123 (4.6) / 3,853 | 22,909 (5.3) / 4,614 | 15,584 (3.6) / 2,642 |
| Polygon RAM words p99 (MB/s) / requests | 137,704 (31.7) / 17,213 | 136,544 (31.4) / 17,068 | 7,248 (1.7) / 906 | 7,248 (1.7) / 906 |
| VROM words p99 (MB/s) / requests | 216,960 (49.9) / 27,120 | 129,024 (29.7) / 16,128 | 407,016 (93.6) / 50,877 | 211,624 (48.7) / 26,453 |

The stream reads up to a burst past a model's end, so these are 10-15% above the
words the model actually uses (section 4). Culling removes 0.06% (VF3) and 0.01%
(Bass) of the triangles: Supermodel's sphere test is not quite conservative there.

Where the cycles go (VF3 frames 200-202): kernels 42%, the clip 15%, assembly 12%,
staging and read-back 10%, waiting for the decoder 9%. The kernels are latency
bound (a T&L chain is 8 dependent multiply-adds of 10 cycles), and the control is
sequential around them: only the decoder overlaps. What took it from 2.19 M to
1.54 M cycles at p99 (VF3): T&L and projection batches sized to the work, the
projection queue kept across chunks, the decoder split off to run a chunk ahead,
the outcode test per plane, the decoder started before the model set-up.

### Fit (Quartus 17.0, 5CSEBA6U23I7, standalone, `quartus/r3d_geo`)

50 MHz constraint, three seeds: **Fmax 73.3 / 63.6 / 70.4 MHz; 19,920 / 19,883 /
19,928 ALM; 14,952 registers; 9 M10K; 9 DSP.** The 9 M10K are Quartus turning
short delay lines into shift-register RAM; the project now switches that off.
The ALM figure includes the ALMs that hold the 1,200 virtual pins of the
standalone top. Seed 1 by unit (Fmax from the worst path ending in the unit):

| Unit | ALM | Registers | DSP | Fmax |
|---|---|---|---|---|
| Control and decoder (own) | 9,097 | 10,962 | 0 | 73.3 MHz |
| Engine total | 5,881 | 2,292 | 9 | 81.7 MHz |
| - fp_mul (each of 4) | 393 | 171 | 1 | 84.1 MHz |
| - fp_add (each of 4) | 421-435 | 171 | 0 | 83.0 MHz |
| - reciprocal | 265 | 357 | 5 | 81.7 MHz |
| - micro-code ROM (in logic) | 687 | | | |
| - register files (8 x MLAB) | 800 (memory ALMs) | | | |
| CORDIC set-up | 2,212 | 653 | 0 | 80.8 MHz |
| Walker | 628 | | 0 | 94.8 MHz |
| Stream | 163 | 166 | 0 | 101.3 MHz |
| fp_div | 130 | 132 | 0 | 98.9 MHz |
| fsqrt | 117 | 126 | 0 | 86.3 MHz |

100 MHz constraint (the section 8.3 option), three seeds, delay lines kept in
registers: **Fmax 74.4 / 80.3 / 78.8 MHz, not closed; 20,454-20,517 ALM; 1 M10K;
9 DSP.** By unit (seed 2): walker 104.9, stream 117.7, fsqrt 99.2, fp_div 95.7,
fp_add 95.0, fp_mul 88.8, CORDIC 86.0, reciprocal 82.5, control 80.3 MHz (its
worst paths run from the shared counter into the next-state logic of the big
state machine). At 100 MHz the cycle counts above fit a frame for VF3 (88.5% at
p99) and just miss for Bass Fishing (101%; 71% with CULL), so 100 MHz is the
option that makes the present design real time, at the cost of closing timing on
the FP units, the reciprocal and the control. At 50 MHz it needs half the cycles.

The first fit of this design ran at 32 MHz: the CORDIC's rounding (64-bit leading
one, shift and round in one cycle) and the assembly (two chained MLAB reads into
seven float compares) were single-cycle paths. Both are now split over cycles.

Against the section 5 estimate (6,200-7,650 ALM) this is 2.5x. The difference is
known: the control holds about 11 K registers (output triangle, staging rows,
read-back and clip vertices in flip-flops), the CORDIC is 2.2 K, fp_mul / fp_add
with subnormals are 393 / 430 ALM against 144 for my Model 1 fp_mul without them
(no subnormal reaches the geometry in 800 frames, so SUBNORM = 0 is safe on the
data), and the micro-code ROM sits in logic (687 ALM; 2 M10K instead).

## 8. Open

- Real time. At 50 MHz the p99 frame is 1.6-2.0x too long; at 100 MHz it fits VF3
  but timing does not close yet. Cycles: overlap the clip, assembly, read-back and
  projection queue with the kernels (the engine idles in them), and interleave two
  chunks' T&L so its 10-cycle chain latency is hidden. Timing: pipeline the control
  (one-hot next-state, the shared counter split), a fifth stage in fp_mul and the
  reciprocal's Newton steps.
- Area. Control registers (about 11 K) into MLAB where they are read a word at a
  time; SUBNORM = 0 for fp_mul / fp_add if the study accepts flushing (no subnormal
  in 800 frames); the CORDIC (2.2 K ALM) shares one 64-bit adder between X, Y and
  Z if the set-up may take three times longer (it runs about 30 times a frame).

- Whether the exit test should be bit-exact (it needs the correctly rounded
  set-up unit, about 550 ALM more) or exact everywhere except screen x/y within
  1/16 px with the algebraic set-up. The set-up derivation is MAME's construction,
  not documented silicon behaviour.
- MAME's undefined vertex slot: the exit test has to exclude those polygons.
- Rows 5, 6, 12, 17 and 22 are the behaviour differences that change images.
  Following MAME for M2 and Supermodel for M3 means a second diff target later.

## 9. The shared-pool datapath (second build)

Sections 7 and 8 describe the first build: bit-exact, 19.9K ALM and 1.5-1.8M
cycles at p99. This is the rewrite that brings it to its row. Every FP operation
now runs on the one pool (`r3d_vpu`: 4 multipliers, 4 adders and the divide), and
the four machines around it are coupled through memories instead of sharing one
state machine.

### Structure

| Block | What |
|---|---|
| Engine `r3d_vpu.sv` | 4 lanes of flush-to-zero `r3d_fp_mul` into `r3d_fp_add` (PIPE_A2); register files in M10K (synchronous, 256 words: the upper half is the matrix stack); micro-code ROM in M10K (454 words of 67 bits; a NOP word holds the sequencer for n cycles); side ports so T&L reads its inputs from the staging memory and writes the vertex store, and projection reads the queue and writes the output buffer, without the controller copying anything; a side channel into the broadcast file for the node matrix; write-back registered. MAC readable at +13, divide lane k at +16+k |
| Divide `r3d_recip.sv` | the correctly rounded reciprocal turned into a divide: Q0 = A * (r2 >> 24) is within [-1, 0] of the rounded quotient on 67 M pairs, and the reciprocal's remainder test picks it. 12 stages, 6 DSP. The clip's t and every set-up divide run here; `r3d_fp_div` is gone |
| Main | walker events; viewport set-up; the matrix stack as aliased slots (a pushed level without a matrix shares the slot below, so PUSH and POP need no kernel); model set-up; T&L of each chunk, up to two chunks ahead of the polygon machine |
| Decoder | as before, but vertex slots in MLAB with a bank bit per slot; the 17.7 and s.7 words go straight into the staging memory; polygon records into two rings (assembly, emission) and a u/v ring in M10K |
| Polygon machine | assembly from the vertex store (2n + 1 cycles), outcodes, MAME's clip one crossing per CLIP kernel (subtracts, |N / Z| and both lerp sets in one kernel), the queue |
| Emitter | triangle fans from the output buffer; v0 stays in the output register and the fan shifts, so one vertex is loaded per triangle |
| CORDIC `r3d_setup_cr.sv` | MAME frustum only: one shared 66-bit adder and one shifter with a registered output; 958,556 inputs, 0 different from the integer model |

### Checks

| What | Result |
|---|---|
| FP units, SUBNORM = 1 and 0, divide, reciprocal (`make test_real3d_pool`) | 95.5 M mul / add, 0 wrong; FTZ units 0 wrong in range; divide 20.7 M, 0 wrong; reciprocal 72.3 M (every mantissa at 7 exponents), 0 wrong |
| Engine kernels against host float | 58,900 results, 0 wrong |
| r3d_geo against the model, 400 VF3 + 400 Bass frames, latency 12-20, 20% stall | every triangle field on every frame, in five builds: Supermodel frustum with culling (default) and without; MAME's correctly rounded frustum with culling and without; MAME's algebraic frustum without |
| Subnormal or flushed value at any FP unit input or output (simulation monitor) | 0 in 800 frames, every build |

The model and the RTL take the frustum from a switch (`--setup sm|cr|alg`, RTL
FRUSTUM_SM / SETUP_CR) and culling from `--cull` / CULL; `make test_real3d_geo2`
builds the default.

### Supermodel's frustum (study D-19)

MAME builds its projection from tan(atan2(plane) / 2) with -1 and -2fn/(f-n)
transposed; Supermodel from viewport words 8-11: left = (0 - jo) / cv,
right = (1 - jo) / cv, bottom = -(1 - io) / cw, top = -(0 - io) / cw, then
`Mat4::FrustumRZ` (w = depth, z = near 1e-3, no far plane). That is plain FP32 on
the CPU, so the pool computes it bit for bit: two subtract and two divide kernels.
The CORDIC is not built (966 ALM in the last fit that had it).

The two are not the same image. MAME's x / w is 2.5135 X / -Z on VF3's main
viewport, Supermodel's 2.4142: MAME's frame is 4.1% wider and 2.5% taller, up to
about 10 px at the edges. Against MAME's lists only 12.8% (VF3) and 24.9% (Bass)
of screen x lie within 1/2 px. Against Supermodel's own frames 7800, 7860 and
7920 (edge correlation; the grabs show earlier poses for moving figures, so only
the stage lines up), Supermodel's frustum scores higher on all three
(0.104 / 0.069 / 0.090 against 0.087 / 0.041 / 0.067), and scaling MAME's
triangles shows a step between 0.98 and 0.96 on all three frames with local
peaks at 0.96 on two, where Supermodel's ratio is 0.9605. The metric favours
small scales, so this supports Supermodel's construction without proving it.
z is now GL window depth (z / w * 0.5 + 0.5, near = 1) and w is depth, not twice
depth; u / w and v / w scale with it.

**Wrong turn, recorded.** Section 3 row 4 and R35 concluded that MAME's image
scale is right. The grabs only ruled out the 2x error; at the 4% level MAME is
the one off.

### Cycles (bench, 400 frames, 12-20 cycle latency, 20% stall; 100 MHz frame 1,738,530 cycles)

| Build | VF3 p99 | Bass p99 | VROM p99 VF3 / Bass |
|---|---|---|---|
| first build (section 7) | 1,537,878 (88.5%) | 1,758,000 (101%) | 49.9 / 93.6 MB/s |
| MAME frustum, no culling | 1,118,538 (64.3%) | 1,474,458 (84.8%) | 49.8 / 91.7 |
| MAME frustum, culling | 1,085,174 (62.4%) | 1,056,788 (60.8%) | 29.6 / 48.4 |
| Supermodel frustum, no culling | 1,112,149 (64.0%) | 1,456,485 (83.8%) | 49.8 / 91.8 |
| **Supermodel frustum, culling (default)** | **1,054,554 (60.7%)** | **1,037,368 (59.7%)** | **29.6 / 48.4** |

Pool occupancy, default build, p99 frame: VF3 390,067 instructions issued
(37.0% of the frame's cycles, 22.4% of a 100 MHz frame), Bass 353,136 (34.0%,
20.3%); lane operations 36.8% and 33.8% of four a cycle.

Memory per frame at p99 (default): culling RAM 20,159 / 15,612 words in 3,862 /
2,649 requests; polygon RAM 136,576 / 7,248 words in 17,072 / 906; VROM 128,624 /
210,208 words in 16,078 / 26,276 (VF3 / Bass). The stream still reads up to a
burst past each model's end.

What moved the cycles, in order (Bass p99, MAME frustum, no culling, unless
stated): decoupling the four machines with the side memories, 1.76 M to 1.52 M;
the aliased stack (no PUSH / POP kernels) and the assembly overlap, to 1.41 M;
the latencies for 100 MHz (adder align register and divide split, 1.44 M; the
registered write-back, 1.48 M, 85.1%); the direct engine hand-off and the
accepted polygon's skipped cycle, 1.474 M (84.8%); then culling, 1.06 M, and
Supermodel's frustum, 1.04 M. The algebraic set-up builds to 1,107,189 (63.7%) /
1,463,054 (84.2%).

**Wrong turns, recorded.** A reuse window of one chunk (main three ahead
instead of two) was slower: 1,487,421 against 1,478,814, the re-transforms cost
more than the deeper run-ahead. Writing the node matrix into the broadcast file
through a side channel, so the engine is held only for the product, did not move
Bass's p99 at all (1,479,229): the heavy frames wait on main's own kernels and on
the polygon machine, not on that write.

### Fit (Quartus 17.0, 5CSEBA6U23I7, 10 ns, three seeds, `sm3-builds/r3d/fit2.sh`)

Default build: **Fmax 110.1 / 110.6 / 112.3 MHz; 8,796 / 8,792 / 8,815 ALM, of
which 622 hold the 1,242 virtual pins (8,170-8,193 without them); 8,712
registers; 13 M10K; 10 DSP.** Guarding MAME's set-up states in this build took
about 90 ALM off the fit before it (8,881-8,901, 106.7-114.7 MHz). Seed 1 of
that fit by unit:

| Unit | ALM | Registers | M10K | DSP |
|---|---|---|---|---|
| Engine (own glue about 966) | 3,136 | 4,890 | 12 | 10 |
| - fp_add (each of 4) | 321-327 | 293 | 0 | 0 |
| - fp_mul (each of 4) | 87-89 | 154 | 0 | 1 |
| - divide | 363 | 854 | 0 | 6 |
| - micro-code ROM | 0 | 0 | 4 | 0 |
| - register files (8 blocks) | 0 | 0 | 8 | 0 |
| - broadcast file (MLAB) | 160 | | | |
| Walker | 670 | 640 | 0 | 0 |
| Stream | 168 | 166 | 0 | 0 |
| Controller memories (MLAB) | 1,404 | | 1 (u/v ring) | |
| Controller logic | about 2,880 (3,502 with the pin ALMs) | | | |

Timing on the way there: the first fit of this structure ran at 74 MHz (CORDIC
shifter and adder in one cycle, the adder's align stage, the divide's DSP-add and
remainder stages); the align register, the divide split and the CORDIC's
registered shifter took it to 92-97; the registered write-back, the registered
vertex reference, the viewport word registered before its conversion, the
CORDIC's two-cycle rounding and a registered reuse-window flag closed it.

### Planted faults (default build; 20 VF3 + 20 Bass frames)

| Fault | Caught |
|---|---|
| F1 stack: pop restores the wrong level | yes |
| F2 shared vertex from the wrong slot | yes |
| F3 clip planes -X / +X sign | yes |
| F4 divide without its rounding correction | yes |
| F5 viewport x and y offsets swapped | yes over the 400 Bass frames (66 differ), not in the 40 |
| F6 priority pass label | yes |
| F7 colour-table base off by one | yes on VF3 frames 0-19 (3 differ); frames 190-209 have no indexed colour |
| F8 lit / unlit shade swapped | yes |
| F9 shade clamp at 255 removed | yes |
| F10 texture x low bit | yes |
| F11 viewport width and height swapped | yes |
| F12 T&L staging bank swapped | yes |
| F13 CORDIC rounding truncates (MAME frustum build) | yes |
| N1 chunk order: a chunk's end from the next queue slot | yes, every frame |
| N2 output buffer reused before it is emitted | yes with the consumer 90% not ready (6 frames); never at 20% |
| N3 projection buffer tag swapped | yes |
| N4 flush-to-zero multiplier loses the sign of a zero product | **no**: no zero product's sign reaches an output in these frames |
| N5 main three chunks ahead with the window of two (store hazard) | yes |
| N6 reuse window of four chunks (store hazard) | **no**: no reuse reaches two chunks back in these frames |
| N7 divide lanes' numerators crossed | yes (Bass) |
| N8 Supermodel frustum: top not negated | yes, every frame |
| N9 culling verdict ignores the fourth plane | yes (VF3, 16 frames) |

## 10. Open after the second build

- Area: 8,170-8,193 ALM without the pin ALMs against the 8,000 row. Candidates:
  constants into the broadcast file (the kb / kc muxes, about 100), the divide's
  tag pipeline, the walker (670), the output registers (the triangle a beat is
  672 flip-flops; a word stream would remove them and most of the pins).
- D-19 rows not yet followed: 6 (LOD by distance, blended), 12 (colour-table base
  from node words), 17 (6-bit transparency, 0 dropped) and 18-22 (lighting).
  Supermodel lights per fragment (`R3DShaderTriangles.h`): face normals for flat
  polygons (which breaks reuse of a shared vertex's lighting), fixed shading from
  the vertex byte, double-sided polygons drawn twice, the light vector in GL eye
  space, sunFactor clamped to [-1, 1] and floored at 0 only with sunClamp or
  translucency, intensity limited to 1. That touches the decoder's reuse, the T&L
  kernel and the emitter.
- N4 and N6 above need frames that exercise them, or a directed bench.

## 11. The D-19 rows (third build)

Study D-19 makes Supermodel the reference where it and MAME differ. Section 9
built rows 4 and 5 (frustum, culling). This pass adds rows 12, 17, 18, 20, 21 and
22 (with 19, which is the same code) to the model and the RTL, row 6 to the
model only, and the word-stream output. Lighting is Supermodel's formula
evaluated per vertex: Real3D's R3D-100 overview (study R51) says the raster
processor interpolates per-vertex colours (Gouraud), so per-fragment lighting is
a GL convenience of Supermodel's, measured below in the model only.

### What each row does, model and RTL

Every row is a switch in `r3d_geo.cpp` (`sm_flat`, `sm_fixed`, `sm_double`,
`sm_light`, `sm_trans`, `sm_ctab`, `sm_lod`; `rows(true)` is Supermodel) and,
except row 6, a parameter of `r3d_geo.sv` (`SM_FLAT`, `SM_FIXED`, `SM_DOUBLE`,
`SM_LIGHT`, `SM_TRANS`, `SM_CTAB`, all 1 by default). `r3d_ref` takes MAME's rows
when it diffs against MAME's lists and Supermodel's otherwise (`--rows`); the
bench's `--rows` must match the RTL build.

| Row | Supermodel (followed) | Model | RTL |
|---|---|---|---|
| 22 (+19) Lighting | sun (n5, -n6, -n4) in GL eye space; normal x mat3(modelview); sunFactor clamp(-1, 1), floored at 0 with the sun clamp or translucency; I = sf d + a with d = clamp(n7, 0, 1), a = byte x (float)(1/255); Step 1.0 limits I to 1 | per vertex, FP32 in the T&L kernel's order; shade = 255 I, not clamped below | the T&L kernel's broadcast reads of the model matrix transposed (micro-word bit `tr`), light z negated by the controller, post-ops 1 clamp(r, 0, 1), 2 min(r, 1); d and a converted by logic (`dclamp`, `k255`) |
| 18 Normals | flat polygons (h1 & 8 clear) light every vertex with the header's face normal (h1..h3 >> 8, 2^-22), and the vertices keep it in m_prev; smooth normals (2b + 1) / 255 (BYTE_TO_FLOAT), not b / 127 | as written | face normal converted as h1..h3 arrive; a normal per vertex slot (3 x 8 words MLAB) so a reused vertex keeps its producer's; a flat polygon re-lights every reused vertex |
| 20 Fixed shading | h1 & 0x20 with h1 & 8 clear: sunFactor is the vertex's x byte, signed (2b + 1) / 255, or b / 255 with the specular bit; a reused vertex keeps its producer's | as written; a vertex made by a non-fixed polygon gets its own byte (Supermodel's is whatever its stack held) | the normal is staged as zero and the shade as a fifth T&L input F: Dp = (N0 L0 + F) + N1 L1 gives F exactly; per slot a "lit as fixed" bit decides whether a stored vertex can be reused |
| 21 Double-sided | h1 & 0x10: the polygon is drawn twice, first with the normals (and face normal) inverted; the shader culls the copy facing away | the copy first, shade from -n (sf' = N2 (-L2) + (-Dp), exact), the same shade for fixed or unlit polygons; word 35 bit 0 marks the copy | the T&L kernel computes both shades of every vertex; the second goes through the store, the clip (lane 3 of the second lerp set, unused before), the queue and PROJ; the emitter sends the record twice |
| 17 Transparency | (h6 >> 18 & 0x3f) * 255 / 32 as a byte; a polygon whose byte is 0 is not drawn | dropped polygons keep their P record and vertex slots | the assembly record's skip bit |
| 12 Colour table | base from node words 3, 7, 8 when node[0] & 4, kept until the next such node; 12-bit index | as written; the base restarts at 0x400 each frame | the walker holds the base and sends it with each MODEL event; the decoder adds the 12-bit index |
| 6 LOD | LODscale = blend radius / \|centre\|, the node's entry of the table at viewport word 0x17; below the last delete size the child is not drawn; a LOD list takes the first entry in reach and fades into the next; every other node fades by the last entry; alpha multiplies down the tree | as written, FP32 (squares summed in order, sqrtf, a divide), alpha in 'T' word 36 | **not built** (below) |

Findings on the way:

- **Supermodel's sun clamp is on for both games.** It comes from the Mars ASIC's
  JTAG mode word, bit 18 clear. A Supermodel build that logs the mode words
  (`sm3-builds/r3d/sm_mw`, `run_mw.sh`) shows VF3 and Bass Fishing writing Mars = 0
  from boot. So sunFactor is floored at 0 for every polygon, and translucency
  never changes the floor in these games. The RTL builds only the clamped form;
  the model keeps the other (`--no-sun-clamp`).
- **MAME's view space is GL eye space with y negated.** MAME applies the
  coordinate matrix as diag(c01, c12, -c20); Supermodel applies its fix-up matrix
  times the whole of matrix 0. For the axis permutation every game's matrix 0
  holds, the two differ by y only. Supermodel's sun (n5, -n6, -n4) is therefore
  (n5, n6, -n4) in MAME's space: only z changes against MAME's (n5, n6, n4).
  Negating y as well, as the difference list's wording suggests, would have
  flipped it twice.
- **Row 17 is not a width change on these games.** (h6 >> 18) & 0x3f includes
  bit 23, which is the opaque flag, so a translucent polygon's 6-bit field is its
  5-bit one. What the row changes in the geometry is the drop of transparency-0
  polygons (0.34 / 3.0 a frame); the * 255 / 32 scale is the rasteriser's.
- **Row 12 changes no base on these games** (the colour-table nodes, 288 a frame
  in VF3, all give 0x400), but the 12-bit index does change VF3's indexed colours:
  PSNR over all pixels 9.55 dB with MAME's 11 bits, 9.71 with 12.
- Definitions I had to choose: a vertex reused from a flat polygon keeps the face
  normal (180 times in 400 VF3 frames, 0 in Bass Fishing); a fixed polygon reusing
  a vertex made by a non-fixed one takes that vertex's own byte (124 in VF3,
  Supermodel reads stale data there); the colour base restarts each frame
  (Supermodel keeps it across frames, which a replayed dump cannot know).

### Lighting against Supermodel's 3D-only grabs

`r3d_light_cmp` runs each dump through the model and the rasteriser's
Supermodel form (`render_sm`, every switch) in-process and compares with
Supermodel's 3D-only grabs; all variants use Supermodel's frustum and culling.
"fragment" cuts each lit triangle into pieces of at most 4 px a side and gives
their corners the unclamped sunFactor interpolated perspective-correct, so the
clamps act per piece: Supermodel's per-fragment shader within a 4 px
tolerance. For double-sided polygons both copies are culled by facing, as
Supermodel's shader culls them. 400 frames each:

| Variant | VF3: PSNR all / p50 frame / differ > 8 / coverage | Bass Fishing |
|---|---|---|
| MAME's lighting (rows 18-22 off) | 9.31 / 17.64 dB / 45.7% / 3.898% | 18.29 / 22.11 dB / 24.0% / 3.022% |
| **Supermodel's formula per vertex (built)** | **9.71 / 19.62 dB / 28.4% / 3.889%** | **18.89 / 23.32 dB / 19.9% / 3.022%** |
| Supermodel's formula per fragment | 9.70 / 19.63 dB / 26.3% / 3.867% | 18.90 / 23.33 dB / 19.4% / 3.022% |

Each row back to MAME's, everything else per vertex:

| Row off | VF3 | Bass Fishing |
|---|---|---|
| 18 normals | 9.71 / 19.62 / 28.40% / 3.889% | 18.89 / 23.32 / 19.91% / 3.022% |
| 20 fixed shading | 9.71 / 19.62 / 28.39% / 3.889% | 18.89 / 23.32 / 19.91% / 3.022% |
| 21 double-sided | 9.71 / 19.40 / 29.00% / 3.898% | 18.86 / 23.17 / 20.21% / 3.022% |
| 22 lighting | 9.30 / 17.75 / 45.38% / 3.889% | 18.27 / 22.11 / 23.83% / 3.022% |
| 17 transparency | 9.71 / 19.62 / 28.39% / 3.889% | 18.89 / 23.32 / 19.91% / 3.022% |
| 12 colour table | 9.55 / 19.62 / 28.38% / 3.889% | 18.89 / 23.32 / 19.91% / 3.022% |
| + row 6 (LOD), model only | 9.71 / 19.63 / 28.28% / 3.888% | 18.89 / 23.32 / 19.88% / 3.022% |

- **Row 22 is nearly all of it**: +1.9 dB at the p50 frame on VF3 and +1.2 dB on
  Bass Fishing, and VF3's pixels more than 8 out drop from 45% to 28%. That is
  R47's white sand floor.
- **Per vertex and per fragment are the same image to within 0.01 dB.** Per
  fragment has 2.1 points (VF3) and 0.5 points (Bass) fewer pixels more than 8
  out: where a triangle's sunFactor crosses 0, the clamp per pixel and the clamp
  per vertex differ. Gouraud costs nothing measurable against Supermodel.
- Double-sided copies are worth 0.2 dB at p50 and the copies' culling a little
  coverage; rows 17, 18 and 20 move under 0.01 points on these frames.
- Row 6 moves 0.1 points of VF3's pixels. Its translucency had to go in as the
  header's 5-bit transparency, since the rasteriser model takes no model alpha.

### The output: polygon records as a word stream (for the rasteriser)

The triangle a beat (672 vertex flip-flops and about 1,100 output wires) is gone.
`r3d_geo` now sends polygon records, a 32-bit word a beat:

| Port | |
|---|---|
| `out_valid`, `out_ready` | a word moves when both are high |
| `out_data[31:0]` | the word |
| `out_last` | the record's last word |

| Word | Bits |
|---|---|
| 0 | [1:0] priority pass, [2] alpha buffer (MAME: h6 bit 0 or translucent), [3] inverted copy of a double-sided polygon, [4] texture on, [5] texture page, [8:6] texture width code, [11:9] height code, [14:12] texel format, [20:15] MAME's TRI_PARAM bits, [26:21] transparency (32 = opaque), [31:27] vertex count n (3..16 after the clip) |
| 1 | [19:0] polygon number in the frame (the 'P' record index), [25:20] texture x / 32, [30:26] texture y / 32 |
| 2 | colour: h4 >> 8 or the colour-table word, R in [23:16] |
| 3 + 7k .. 9 + 7k | vertex k: screen x, screen y, z (GL window depth, larger nearer), w (depth), 256 u / w, 256 v / w, shade (FP32, 255 x intensity; may be negative, never above 255) |

(Replaced in section 15: a fourth header word, six words a vertex with 1/w for z
and w, a viewport record, and the facing test in the geometry.)

The polygon is the fan (v0, v[i-1], v[i]), i = 2..n-1, in MAME's order. A
double-sided polygon comes as two records, the inverted copy first; the
rasteriser culls whichever faces away (the copy with the face normal negated).
The buffer index MAME's lists carry is not sent: it is the running count of
triangles per alpha buffer, which the bench recounts. A record is 3 + 7n words:
31 for a quad, 24 for a triangle. Row 6, if built, adds the model alpha as a
fourth header word.

Kept minimal for the rasteriser: one shade per vertex and no normals (lighting
is per vertex, R51). Two words could still go once the rasteriser's form is
fixed: z is only needed for the bit-exact comparison with MAME's lists if the
rasteriser takes depth on 1/w (m3-notes section 8, DEPTH), and the rasteriser
wants 1/w, which the projection kernel computes (its OOW register) but does not
send. Sending 1/w in place of w and dropping z gives 6 words a vertex.

Area: the vertex registers and the emitter's fan shifting went with it, and the
standalone fit's virtual pins fell from 1,242 to about 110.

### Row 6 (LOD): the model, and why the RTL waits

Per frame, 400 frames each (`r3d_ref --lod`):

| | VF3 | Bass Fishing |
|---|---|---|
| Nodes with a LOD table (every node in both games) | 1,063 | 366 |
| Children culled by distance (below the last delete size) | 0.56 | 1.11 |
| LOD lists: entry 0 / 1 / 2 / 3 taken | 0 / 0 / 0 / 1.33 | 0.19 / 0 / 0 / 22.1 |
| Blends (a second LOD drawn, fading) | 0 | 0.17 |
| Models drawn with alpha < 1 (of them alpha 0) | 1.33 (1.33) | 0.33 (0.01) |
| Polygons p50 / p99 (without row 6) | 8,791 / 12,049 (8,821 / 12,049) | 4,469 / 10,819 (4,475 / 13,998) |

- VF3's 1.33 LOD lists a frame are one junk node at 0x00FFFF with an all-zero
  table and zero entries: MAME draws "model 0" (polygon RAM word 0) there;
  Supermodel's alpha comes out 0 and draws nothing.
- Bass Fishing's LOD lists are real: entry 0 valid down to LODscale 0.004, then
  entry 3, a lower-detail model. MAME always draws entry 0, so at p99 MAME's walk
  draws 13,998 polygons where Supermodel's draws 10,819 (-23%).
- Against Supermodel's grabs row 6 moves 0.1 points of VF3's pixels and 0.03 of
  Bass Fishing's (table above).

What the RTL needs, and its cost: LODscale is blend radius / sqrt(u0^2 + u1^2 +
u2^2) of the culling kernel's centre, so three more multiply-adds on the pool, a
square root (`r3d_fsqrt`, 117 ALM, or a pool kernel) and a divide, per node with a
table (every node: 1,063 a frame in VF3); 8 words of LOD table per node from
culling RAM; four compares and two multiply-adds for the entry and its alpha;
an alpha per level beside the matrix stack; and in the walker, LOD-list entries
walked as nodes when the link's bit 24 is clear and a second, faded entry. I
estimate 400-600 ALM and about 150 cycles a node (+9% of a 100 MHz frame on VF3,
less on Bass Fishing, where the lower-detail models take 23% of the polygons off
p99). For a 0.1-point image change on these games that does not fit in the 8,000
ALM row, so I left it in the model. The output already carries the field (word 3).

### Two latent bugs of the second build, found on the way

Both were in section 9's RTL and passed all 800 frames there by luck of timing.

- **The batch polygon list was rewritten under the emitter.** The polygon machine
  writes a batch's list `{buf, slot}` while queueing, and only checks that the
  output half is free when it projects. Two batches later it wrote the list of
  the half the emitter was still reading. The second emission of double-sided
  polygons slowed the emitter enough to show it: VF3 frame f001800 sent a
  polygon eight positions late in the polygon number only (the two polygons'
  other fields were equal). The queue now waits for that half's batch to pop
  before writing its list. It is fault D11 below.
- **A translation's y and z were read after the event was taken.** M_TR0 wrote
  the walker's `ev_b` and `ev_c` into the viewport words in the two cycles
  after accepting the TRANS event. With culling, the walker's next event (CULLQ)
  sets only `ev_a`, so the words survived. Without culling the next event is a
  MODEL, and once MODEL carried the colour base in `ev_b`, every translated
  node's y was wrong: the no-culling builds failed from their first triangle. They
  are now held in two registers idle in that state. Fault D12 (no-culling build).

Neither bench had caught them because both depend on which event or batch comes
next. N2 (the output buffer reused early) was the nearest planted fault and needs
90% output stall to show.

### Checks

| What | Result |
|---|---|
| Model, MAME's rows, against MAME's lists (`r3d_ref --golden`) | unchanged: every field bit-exact (40 + 40 frames re-run) |
| r3d_geo against the model, 400 VF3 + 400 Bass frames, latency 12-20, 20% output stall (a word at a time) | every triangle field on every frame in seven builds: Supermodel frustum with and without culling; MAME's correctly rounded frustum with and without; MAME's algebraic frustum; the default with MAME's rows; MAME throughout (`make test_real3d_geo3_all`) |
| The same with one row back to MAME's (six builds, `rowoff.sh`) | every frame |
| Subnormal or flushed value at any FP unit (monitor) | 0 in every build |
| Walker events against the model (colour base on MODEL) | 400 + 400 frames |
| Engine kernels against host float (`tb_r3d_vpu`, Supermodel's T&L form) | 66,300 results, 0 wrong |
| `k255`, `b2f`, `dclamp` as Quartus 17 builds them (gate-level netlist in iverilog) | all 256 bytes and the NaN, infinity, -0 and > 1 cases equal to host float |

### Cycles (400 frames each, 12-20 cycle latency, 20% output stall per word; 100 MHz frame 1,738,530 cycles)

| Build | VF3 p99 | Bass p99 |
|---|---|---|
| Section 9 default (R50) | 1,054,554 (60.7%) | 1,037,368 (59.7%) |
| **This build, default** | **1,093,545 (62.9%)** | **1,100,322 (63.3%)** |
| Default with MAME's rows | 1,086,125 (62.5%) | 1,064,399 (61.2%) |
| Supermodel frustum, no culling | 1,164,909 (67.0%) | 1,542,350 (88.7%) |
| MAME's frustum (cr), culling / no culling | 1,119,797 / 1,167,771 | 1,117,828 / 1,557,106 |

The rows cost 0.4 (VF3) and 2.1 (Bass) points of a frame, nearly all of it the
double-sided copies' records; the T&L kernel's 16 more cycles a chunk (the second
shade and the F input) and the stream's word-a-cycle output cost the rest of the
2.2 / 3.6 points against R50. Both games stay well under 85%; the builds without
culling do not (they never did on Bass Fishing).

### Area and timing (Quartus 17.0, 10 ns, `sm3-builds/r3d/fit4.sh`)

| Build | Seed | ALM (with the 232 pins' ALMs) | Registers | M10K | DSP | Fmax slow 100C |
|---|---|---|---|---|---|---|
| Default (every row but 6), final RTL | 1 | 8,949 | 8,236 | 12 | 10 | 106.06 MHz |
| | 2 | 8,908 | 8,216 | 12 | 10 | 109.89 MHz |
| | 3 | 8,922 | 8,218 | 12 | 10 | 105.88 MHz |
| The same before the last three changes (4-word header, the / 255 product's leading one from the sum) | 1-3 | 8,855-8,881 | 8,161-8,177 | 12 | 10 | 106.72-107.31 MHz |
| Section 9 (R50), for reference | 1-3 | 8,792-8,815 (1,242 pins) | 8,712 | 13 | 10 | 110.1-112.3 MHz |

All three seeds close 10 ns. The pins now cost about 116 ALM (R50's 622 for 1,242,
the same ratio), so the default build is about **8,790-8,830 ALM without them:
not inside the 8,000 row**, about 600 above R50's 8,170-8,193. M10K is one fewer
(the micro-code ROM at 60 bits fits three blocks). One fit between them (the
4-word header with the leading one taken from the sum) missed 10 ns by 0.075 ns
on seed 1 at that converter step, which the last change fixed.

By unit, seed 1 of the pre-final fit (the final one: engine 3,108, walker 685,
stream 183, controller 4,973):

| Unit | ALM | Registers | M10K | DSP | Fmax (worst path ending there) |
|---|---|---|---|---|---|
| Engine | 3,070 (R50 3,133) | 4,686 | 11 | 10 | 111.6 MHz |
| - fp_add (each of 4) | 314-325 | 285-287 | 0 | 0 | 112.3 MHz |
| - fp_mul (each of 4) | 87-92 | 158 | 0 | 1 | 118.6 MHz |
| - divide | 352 | 834 | 0 | 6 | 115.5 MHz |
| - micro-code ROM | 0 | 0 | 3 | 0 | |
| - broadcast file (MLAB) | 160 | | | | |
| Walker | 690 (676) | 660 | 0 | 0 | 109.0 MHz |
| Stream | 182 (168) | 166 | 0 | 0 | 120.8 MHz |
| Controller, memories and pins | 4,937 (4,819 with 622 pin ALMs) | | 1 | | 108.0 MHz |

### Each row's cost (seed 1, 10 ns; cycles at p99, 400 frames each, 100 MHz frame)

| Row | ALM with the row off (default 8,879) | Row's cost | VF3 p99 off / on | Bass p99 off / on |
|---|---|---|---|---|
| 18 normals | 8,627 | **+252** | 1,094,813 / 1,094,813 | 1,103,114 / 1,105,314 |
| 20 fixed shading | 8,865 | +14 | same | same |
| 21 double-sided | 8,889 | -10 (seed noise; the kernel's second shade stays in every build) | 1,086,030 / 1,094,813 (+0.5 points) | 1,065,964 / 1,105,314 (+2.3 points) |
| 22 lighting | 8,897 | -18 (seed noise) | same | same |
| 17 transparency | 8,871 | +8 | 1,094,011 / 1,094,813 | same |
| 12 colour table | 8,866 | +13 | same | same |
| all six off | 8,434 | +446 together (shared converters: rows 18, 20 and 22 each need the / 255 product) | 1,085,590 (62.4%) | 1,064,173 (61.2%) |

Row 18 is the only expensive one: the per-slot normal memory, the face normal's
registers and its path through the converter, for no measured image change on
these games (above). Rows 20, 21, 22, 17 and 12 cost little in the controller;
row 21's real cost is in cycles and in the T&L kernel's 16 extra cycles a
chunk, which every build carries.

Against R50 (8,170-8,193 without pins) the default build is about +570. The
six rows together are +446 (one at a time they add up to about +260: rows 18,
20 and 22 share the / 255 converter, so removing one leaves it in place). The
MAME-rows build, 8,434 with pins (about 8,320 without), is still about +140 on
R50: the second shade through the clip pool and queue, the staging converter's
second step, and the controller's paths closing 10 ns again, less what the area
work below took out.
The area work that went the other way, by section 8.4 technique:

| Technique (§8.4) | Change | Measured |
|---|---|---|
| Small storage out of flip-flops | the divide's destination tag: a 4-entry ring instead of 13 stages x 23 bits | in the engine's -63 (R50 3,133 -> 3,070 ALM, with the post-ops and the transposed read added) |
| Converters time-shared | the micro-code's constants as broadcast entries: the per-lane constant muxes and the two constant registers gone | in the same -63 |
| Converters time-shared | the face normal through the staging converter's int24 converter, idle while a header streams in | removed a 40-ALM converter and the build's worst path (-1.6 ns) |
| MLAB/M10K trade | the micro-code word 61 -> 60 bits (the end flag folded into the empty word) | 13 -> 12 M10K |
| Storage out of flip-flops | the triangle output (672 vertex flip-flops, 117 field bits) replaced by the word stream | registers 8,712 -> 8,176; virtual pins 1,242 -> 232 |
| Small storage out of flip-flops (tried, reverted) | the engine's c and control delay lines as MLAB rings | +90 MLAB ALMs for -38 logic ALMs and -500 registers: worse, reverted |

### Planted faults (`make test_real3d_faults2`; default build; frames differing)

Section 9's 22 re-run on this RTL (F7, F8 and F9 rewritten for the new code), and
14 new ones. Frames 190-209 of each game unless stated (Bass Fishing has content
in 3 of those 20).

| Fault | VF3 | Bass Fishing |
|---|---|---|
| F1-F4, F6, F10-F13, N1, N3, N5, N8 | 20 / 20 | 3-4 / 20 |
| F5 viewport x and y offsets swapped | 0 / 20 | 66 / 400 |
| F7 colour-table base off by one (frames 0-19) | 3 / 20 | 0 / 20 |
| F8 lit / unlit shade swapped; F9 intensity limit removed | 20 / 20 | 0 / 20 |
| N7 divide lanes' numerators crossed | 0 / 20 | 3 / 20 |
| N9 culling verdict without the fourth plane | 16 / 20 | 0 / 20 |
| **N2** output buffer reused before it is emitted | 0 | 0: **now masked**, the list hazard's guard (D11) holds the queue earlier |
| **N4** a zero product's sign lost; **N6** reuse window of four | 0 | 0, as in section 9 |
| D1 face normal taken for smooth polygons, not flat ones | 20 / 20 | 0 / 20 |
| D2 double-sided: the second record not sent | 20 / 20 | 3 / 20 |
| D3 sunFactor not floored at 0 | 20 / 20 | 0 / 20 |
| D4 colour base built from the wrong bits of node word 3 (frames 0-19) | 3 / 20 | 0 / 20 |
| D5 fixed shade signed / unsigned swapped (400 frames) | 33 / 400 | 1 / 400 |
| D6 the transparency-1 polygon dropped instead of 0 | 0 / 20 | 20 / 20 |
| D7 light z not negated | 20 / 20 | 0 / 20 |
| D8 normal transform not transposed (MAME's) | 20 / 20 | 0 / 20 |
| D9 a flat polygon reuses stored vertices (400 frames) | 15 / 400 | 0 / 400 |
| D10 the copy sent with the polygon's shade | 20 / 20 | 0 / 20 |
| D11 batch list written under the emitter (the bug above) | 20 / 20 | 3 / 20 |
| D12 translation y, z read after the event (no-culling build) | 20 / 20 | 3 / 20 |
| D13 the constant 0.5 loaded wrong | 20 / 20 | 3 / 20 |
| D14 divide ring retired a lane early | 20 / 20 | 3 / 20 |

33 of 36 caught.

## 12. The rasteriser's set-up on the geometry's pool, measured

m3-notes section 9 asks whether the triangle set-up (25 dot operations a textured
triangle, 16 untextured) can run on this pool and save the rasteriser its own
unit (1,300-1,600 ALM, 4 DSP). I measured it in the RTL rather than estimating.

**The kernels** (`gen_ucode.py --setup-kernels`, one triangle a lane, the
rasteriser model's order). A MAC reads two lane registers at most (port A, and
port B shared by b and c), and every term of the set-up's dot products is a
per-lane value, so a three-term dot is three products, two adds and the * idet:
six instructions, not four.

| Kernel | Instructions | Cycles (4 triangles) | ROM words |
|---|---|---|---|
| SETUP5 (textured: w, 1/w, u/w, v/w, shade) | 108 + one divide | 129 | 114 |
| SETUP2 (untextured: w, shade) | 54 + one divide | 93 | 64 |

**The load** (`SETUP_LOAD = 1`, off by default): after each projected batch the
polygon machine keeps the engine and runs one kernel per four of the batch's
triangles. The results are not used (the set-up's data path into lane registers
and out to the polygon store is not built), so this measures the contention
only, which is the question. 400 frames each, p99 cycles, share of a 100 MHz frame:

| | VF3 | Bass Fishing |
|---|---|---|
| Today (no set-up) | 1,094,813 (63.0%) | 1,105,314 (63.6%) |
| Set-up for every emitted triangle (both copies of a double-sided polygon) | 1,750,463 (100.7%) | 1,720,565 (99.0%) |
| Set-up for one copy only | 1,645,168 (94.6%) | 1,511,970 (87.0%) |

Pool issue rises to about 50% of a frame. **It does not fit under 85% as the
pool stands.** What it would take:
- the facing test in the geometry (one dot per polygon with the face normal it
  now carries), so that back faces get no set-up: Supermodel culls about half
  the single-sided triangles, which would bring the set-up's +550,000 cycles
  (VF3) down to roughly +300,000;
- a third lane-register read (a dot term in one MAC): SETUP5 to about 78
  instructions, another 25% off;
- with both, VF3 near 80% and Bass Fishing near 75% at p99, on estimates.

**Area if it were shared:** the micro-code ROM grows past 512 words (666 with
both kernels): +3 M10K at 60 bits; a port that loads each triangle's 21 inputs
into a lane (the output buffer holds vertices across lanes, so a transpose,
about 36 controller cycles per 4 triangles, overlappable) and returns 15
coefficients per textured triangle to the polygon store (+60 B a triangle, as m3-notes
section 9 says). I estimate 150-300 ALM and +3 M10K against the 1,300-1,600 ALM
and 4 DSP of its own unit. The trade is worth having only with the facing test
and the third read port, and the cycle budget then leaves Daytona 2 little room.

The rasteriser's hand-off stays one shade a vertex: lighting is Gouraud (R51),
so it receives no normals, and the record header is three words (the model
alpha word goes only if row 6 is built).

## 13. Open after the third build

- **Area: 8,790-8,830 ALM without the pins against the 8,000 row.** The rows cost
  446 together, row 18 alone 252 for no measured image change on VF3 or Bass
  Fishing; dropping row 18 from the default (`SM_FLAT = 0`) would be a D-19
  decision, so I left it on. Other candidates, not yet measured: the walker's
  node buffer (8 x 32 flip-flops, read a word at a time into MLAB), the
  controller's viewport set-up written as micro-code instead of states with wide
  `xw_data` multiplexers (§8.4 "converters time-shared"), and the ambient's own
  / 255 converter folded into the decoder's.
- Row 6 in the RTL (section 11): about 400-600 ALM; it takes 23% of Bass
  Fishing's p99 polygons away.
- Sun clamp off (Mars mode word bit 18 set): the floor then depends on each
  polygon's translucency, which the store's reuse key would have to carry. No
  Step 1.0 game measured so far sets it.
- The set-up on the pool (section 12) needs the facing test in the geometry and
  a third lane-register read before it fits a frame.
- N4 and N6 still need frames that exercise them, or a directed bench; N2 is now
  masked by D11's guard and could be removed or re-aimed at P_Q0.

## 14. The area pass (fourth build)

R54 puts the whole design 4,000-5,000 ALM over the part, and R55 left the
geometry at 8,790-8,830 ALM without its pins. D-27 now gives the block a ceiling
of 8,550 ALM without pins. This pass applies study §8.4 to every unit and takes
row 18's face normals out (D-25). Every change is bit-exact on all 800 frames in
the default build; the seven builds of `test_real3d_geo3_all` and the new
block-RAM option pass as well.

### Row 18 (D-25)

The RTL no longer builds face normals: the per-slot normal memory (3 x 8 words
of MLAB), the face normal's registers and its path through the converter are
gone. `SM_FLAT` keeps the part of row 18 that costs nothing, Supermodel's
(2b + 1) / 255 vertex normal, which shares the fixed shade's / 255 product;
MAME's b / 127 table now exists only in the MAME-rows build. The model splits
the row in two: `sm_n255` (built) and `sm_flat`, the face normals, now a model
switch that `rows()` leaves off (bench list keys `flat` and `face`).

Against Supermodel's 3D-only grabs (`sm3-builds/r3d/area/light.sh`, 400 frames
each, PSNR all / p50 frame / differ > 8 / coverage):

| Variant | VF3 | Bass Fishing |
|---|---|---|
| as built (normals (2b + 1) / 255) | 9.71 / 19.62 / 28.387% / 3.889% | 18.89 / 23.32 / 19.913% / 3.022% |
| + face normals (R55's default) | 9.71 / 19.62 / 28.387% / 3.889% | 18.89 / 23.32 / 19.913% / 3.022% |
| row 18 off (b / 127) | 9.71 / 19.62 / 28.395% / 3.889% | 18.89 / 23.32 / 19.908% / 3.022% |

The face normals change nothing to three decimals on either game.


### What changed, by study §8.4 technique

ALM before (R55, seed 1, `fits/d1_s1`) and after (this build, seed 1). Memories
are itemised by the fitter's entity table; the controller's own logic is one
entity, so its changes are measured together and split by estimate (marked ~).

**Small storage out of flip-flops, and memories one MLAB deep.** A 64-word MLAB
memory costs half again its storage in a read multiplexer: Quartus 17 builds it
from two 32-word halves (a 64 x 32 test, async or registered read, with or without
`max_depth`, is 40 memory ALMs plus 16.5 logic ALMs every time).

| Change | Before | After |
|---|---|---|
| Row 18's per-slot normal memory and face-normal path (D-25) | 60 (memory) + ~190 (logic) | 0 |
| Vertex slots by reference: eight raw-word entries, MAME's prev and the polygon's vertex as maps of entry numbers; a reuse copies a 3-bit reference, not four words | 4 memories, 80 | 1 memory, 20 |
| Clip vertex lists by entry: a vertex stays where it was written, each plane's list is a list of entry numbers, so an inside vertex costs a 5-bit write, not a 263-bit copy | in the controller | ~-80 |
| Polygon rings 64 -> 32 polygons (assembly and emission records) | 55 + 148 | 22 + 51 |
| Emission record without its two repeated bits (page, texture enable) | in the 148 | in the 51 |
| Model stream FIFO 64 -> 32 words (three 8-word bursts still in flight) | 53 | 21 |
| Walker return stack 64 -> 32 frames (the matrix stack holds 32) | 80 | 30 |
| Walker viewport chain: address and priority in one memory | 55 + 22 | 58 |

**Duplicated arithmetic built once, converters time-shared.**

| Change | After |
|---|---|
| One int16 to FP32 converter for the viewport words (five calls in M_VPW) and the assembly's u and v; the polygon machine is idle in M_VPW (M_VPD) | ~-120 (four converters) |
| One / 255 product for the decoder (normal byte, signed and unsigned fixed shade) and the ambient, which goes through it in M_VPW while the decoder is idle (M_VPD now also waits for `d_quiet`) | ~-60 |
| Outcodes: each pair fle(-a, w), fle(a, w) from one magnitude compare of a against w, 6 + 1 float compares to 3 + 1; a miter against the fle form over 200 M cases, 0 different | ~-30 |
| Culling verdict: the radius is positive and finite, so rr < t is a sign test and a magnitude compare | ~-30 |
| One adder for every culling-RAM address of the viewport and matrix states (eight base + constant adders) | ~-60 |
| Walker: one adder for the viewport's words 22 and 2 | in the walker's -52 |
| Region ends (walker reads, the controller's culling reads, the stream's bursts): the regions are 2^18, 2^20 and 2^24 words, so "offset + beat past the end" is a 4-bit sum on the last eight words, not a 21- or 25-bit add and compare | walker logic 527 -> 475, stream logic 130 -> 111 |
| Engine: the lane's result (MAC or divide) chosen once for both write ports | in the engine's -153 |

**Narrower arithmetic.** The divide's remainder test (stage 10) compared M (2Q0 +- 1)
against A 2^(s+1) in 51 bits, twice. Q0 lies within [-1, 0] of the rounded quotient,
so d = A 2^(s+1) - 2 M Q0 is below 3M < 2^26 and its low 30 bits are exact: one
30-bit difference and two 30-bit sums. Standalone fit: 424.5 -> 366.5 ALM, 923 ->
865 registers, 127.9 -> 155.0 MHz; in the design 365 -> 288-300. `test_real3d_pool`:
20.7 M divides and 72.3 M reciprocals, 0 wrong.

**The pool does the moves (one shared FP pool).** Supermodel's frustum was four
kernels with the controller moving lanes between them through the read-back port
(`x_rdata` into `xw_data`, with negations and constants). It is now one kernel, SMFR
(15 operations, 114 cycles), whose lanes read each other through the broadcast file;
the controller only loads {jo jo io io} and {cv cv cw cw}, and the constant registers
and the projection rows' constant lanes go in once a frame. That removed the
`x_rdata` sources from `xw_data` and four states: ~-150. The micro-code ROM is now
exactly 512 words (3 M10K).

**Idle path removed.** R50's broadcast side channel (the node matrix written into
the broadcast file while the engine runs) is gone; the matrix goes in through the
external port with the engine held: the side channel's write port and handshake
(~-50 in the engine) for +0.4% of a frame at p99.

**MLAB and M10K traded (parameters, off by default).** Measured on seed 1:

| Parameter | What goes to block RAM | ALM | M10K |
|---|---|---|---|
| `SIDE_M10K` | T&L staging and projection queue (16 x 32 x 32, read registered either way) | -336 (16 x 21) | +16 |
| `OUT_M10K` | projection output buffer (4 x 64 x 32, read a cycle ahead) | -228 (4 x 57) | +4 |
| `BC_M10K` | the engine's broadcast file (8 x 32 x 32) | -140 | +8 |
| all three (`fits/e2_s1`) | | 7,364 -> 6,679 (-685) | 12 -> 40 |

The best ratio is `OUT_M10K`, 57 ALM a block. Whether to take any of them is a study
decision (M10K binds as hard as ALM); all three pass the bench (`OUT_M10K` in
`allbuilds.sh`, the others are the same memories with a registered read).

### Area and timing (Quartus 17.0, 10 ns, `sm3-builds/r3d/area/fit5.sh`)

| Build | Seed | ALM (with the 232 pins' ALMs) | Registers | M10K | DSP | Fmax slow 100C |
|---|---|---|---|---|---|---|
| This build, default (`fits/e7`) | 1 | 7,277 | 8,145 | 12 | 10 | 111.54 MHz |
| | 2 | 7,293 | 8,101 | 12 | 10 | 104.14 MHz |
| | 3 | 7,276 | 8,107 | 12 | 10 | 104.69 MHz |
| R55, for reference | 1-3 | 8,908-8,949 | 8,216-8,236 | 12 | 10 | 105.9-109.9 MHz |

All three seeds close 10 ns. Without the pins (about 116 ALM, as in R55) the default
build is **about 7,160-7,177 ALM: 1,610-1,650 below R55, 1,370 below D-27's 8,550
ceiling and 820 below the 8,000 row**. Registers and M10K are unchanged within 2%.

By unit, seed 1 (R55's seed 1 in brackets):

| Unit | ALM | Registers | M10K | DSP | Fmax (worst path ending there) |
|---|---|---|---|---|---|
| Engine | 2,938 (3,108) | 4,654 | 11 | 10 | 111.5 MHz |
| - fp_add (each of 4) | 314-322 (315-327) | 284-287 | 0 | 0 | 119.4 MHz |
| - fp_mul (each of 4) | 88-92 (87-90) | 146 | 0 | 1 | 133.5 MHz |
| - divide | 294 (365) | 792 | 0 | 6 | 120.7 MHz |
| - broadcast file (MLAB) | 160 (160) | | | | |
| Walker | 564 (685) | 653 | 0 | 0 | 116.3 MHz |
| Stream | 127 (183) | 162 | 0 | 0 | 157.3 MHz |
| Controller memories | 1,079 (1,329) | 90 | 1 | 0 | |
| Controller logic, with the pins | 2,570 (3,644) | 2,586 | 0 | 0 | 113.1 MHz |

p99 cycles, 400 frames each: VF3 1,104,813 (63.5% of a 100 MHz frame, R55 62.9%),
Bass Fishing 1,108,405 (63.8%, R55 63.3%). The 0.6 points are the rings at 32
polygons (+0.3), the side channel's removal (+0.4 on VF3) and the stream FIFO
(+0.1), less a little the frustum kernel gave back.

### What the engine costs, and what it cannot give back

- **The adders.** fp_add is 314-322 ALM a lane. Flush-to-zero is already on; what
  is left of subnormal handling in it is the exponent-0 to 1 substitution, two
  8-bit 2:1 multiplexers, a few ALMs. The rest is one 27-bit align shifter, a
  28-bit leading-zero count and normalising shifter, two carry chains and the
  round: each is needed for FP32 rounded as MAME rounds it. A shared normaliser
  does not apply: every lane adds every cycle it issues (lane operations are
  36-37% of four a cycle at the same 34-37% issue occupancy, so lanes are almost
  never idle within an instruction).
- **Four lanes.** Dropping one lane would save about 880 ALM (an fp_add, an fp_mul
  and a DSP, the lane's operand, delay and write-back logic, its eight side
  memories and two broadcast banks) and 2 M10K. The kernels are written for four
  (matrix columns, x y z w, four cull planes, four tangents); with three, the
  vertex kernels issue a third more and the 4-wide ones twice, which I estimate at
  +7 to +10% of a frame at p99. That is the largest lever left and a rewrite of the
  generator and the controller's lane-indexed memories; not done in this pass.
- **The divide**: 288-300 ALM, 6 DSP, about 790 registers, of which 704 are the
  11-stage payload (sign, exponent, special, both significands). The remainder
  test was the large logic piece; the seed table (128 x 12) and the exponent path
  are the rest.
- **The micro-code ROM** is in 3 M10K, 0 ALM: 512 words of 60 bits, now full. The
  set-up kernels take it to 1,024 words, 6 M10K.

### The rasteriser's set-up on the pool, with the facing test

Section 12 left two levers: a third register read and the facing test. Both are
built behind parameters (`SETUP_LOAD`, `SETUP_3R`, `SETUP_FACE`, `SETUP_FACEK`,
`SETUP_PORT`; micro-code `gen_ucode.py --setup-kernels --3r`).

- **Port C** (`SETUP_3R`): a third register-file copy per lane; micro-word csel = 3
  takes c from it at the bc1 field's address, so the word stays 60 bits. The
  set-up kernels become 75 operations, 100 cycles (SETUP5) and 39, 91 (SETUP2),
  rounding order unchanged. Cost: +4 M10K, +7 ALM (`fits/e3_s1` against `e5_s1`).
- **The facing verdict** is Supermodel's test, dot(view vertex 0, view face
  normal) > 0, never for a double-sided polygon. The model writes it per polygon
  (`r3d_geo::backface`), the bench loads it into the RTL (simulation only). A
  double-sided polygon then gets one set-up, a back face none. VF3: 1.23 M of 3.30 M
  polygons are back faces over 400 frames (37%); Bass Fishing 0.48 M of 1.95 M (25%).
- **On the pool** (`SETUP_FACEK = 1`): a FACE kernel per projected batch, four
  polygons a lane-row, normal into view space, folded through the projection,
  dotted with vertex 0 in clip space: 64 operations, 95 cycles. It costs nearly what
  it saves, because a batch holds four to six polygons and the kernel is latency
  bound.
- **Beside the pool** (`SETUP_FACEK = 0`): the verdict costs no pool cycles. The
  hardware form is a sign test in the set-up's input port: the triangle's 2D cross
  product on the fixed-point screen coordinates the edge walker uses anyway
  (two 25 x 25 products, one subtract): about 100-150 ALM and 1-2 DSP, estimated.
- **The port's load** (`SETUP_PORT`): the inputs go into the lanes through the
  external port with the engine held, a lane word a cycle: 21 cycles before each
  SETUP5, 12 before each SETUP2. The results leave through the side port, written
  by the kernel (no engine cycles).

400 frames each, p99 cycles, share of a 100 MHz frame:

| Case | VF3 | Bass Fishing |
|---|---|---|
| No set-up (this build) | 1,104,813 (63.5%) | 1,108,405 (63.8%) |
| Set-up for every emitted triangle, two reads (section 12) | 1,750,473 (100.7%) | 1,720,617 (99.0%) |
| Port C, every emitted triangle | 1,628,806 (93.7%) | 1,569,025 (90.3%) |
| Port C, facing on the pool | 1,601,499 (92.1%) | 1,506,714 (86.7%) |
| Port C, facing on the pool, port load | 1,668,503 (96.0%) | 1,544,053 (88.8%) |
| Port C, facing beside the pool | 1,393,099 (80.1%) | 1,327,069 (76.3%) |
| **Port C, facing beside the pool, port load** | **1,453,700 (83.6%)** | **1,383,366 (79.6%)** |

**With the facing test in the input port, the set-up fits under 85% on both games**:
83.6% and 79.6%. The margin on VF3 is 1.4 points, and Daytona 2 has about 45% more
new vertices at p99 (R20), so it must be measured there before the rasteriser
drops its own unit. Cost on the geometry side: port C +4 M10K, the ROM +3 M10K,
the input port and its facing test an estimated 250-450 ALM and 1-2 DSP, against
1,300-1,600 ALM and 4 DSP for the rasteriser's own set-up unit.

### The set-up's output, for the rasteriser

(Section 15 replaces this: no w plane, 1/w for every triangle, a 4-word header,
one record per double-sided polygon from the normal test.)

With the set-up on the pool, the geometry would hand the rasteriser triangles, not
fans, as word-stream records (the record header of section 11 unchanged, its
vertex count field 3):

| Word | Content |
|---|---|
| 0-2 | the polygon record's header (section 11), n = 3; bit 3 (inverted copy) as before |
| 3, 4 / 5, 6 / 7, 8 | screen x, y of the three vertices, sorted by y as `rtl_tri::setup` sorts them (FP32; the edge walker converts them to fixed point) |
| 9 | det (FP32): its sign is the winding, zero or \|det\| < 1e-5 is a flat triangle |
| 10 + 3p .. 12 + 3p | parameter p: pdx, pdy, ps (FP32), `dot(p, a_c) * idet` in `rtl_tri::setup`'s order |

Textured triangles carry p = w, 1/w, u/w, v/w, shade (15 words, a 25-word record);
untextured p = w, shade (6 words, 16). 1/w is the projection's own reciprocal
(its OOW register), so the output buffer would carry it in place of z, which only
the bit-exact comparison with MAME's lists needs. A double-sided polygon is set up
once; its record carries both shades (shade and shade' as two parameters, +3 words)
and the rasteriser keeps the copy whose winding faces it. Back faces are not sent.

### Planted faults (`make test_real3d_faults3`; default build; frames 190-209 unless stated)

All of section 11's faults re-run on this RTL; F8, N8, N9, D5, D9 and D10 rewritten for
the new code, D1 re-aimed at the (2b + 1) / 255 conversion (the face normal it
targeted is gone), and 14 new ones for what this pass restructured.

| Fault | VF3 frames differing | Bass Fishing |
|---|---|---|
| F1_stack_pop | 20 / 20 | 3 / 20 |
| F2_vertex_reuse | 20 / 20 | 3 / 20 |
| F3_clip_order | 20 / 20 | 3 / 20 |
| F4_recip_round | 20 / 20 | 3 / 20 |
| F5_viewport_xy | 0 / 20 | 66 / 400 |
| F6_pass_label | 20 / 20 | 3 / 20 |
| F7_colour_table | 3 / 20 | 0 / 20 |
| F8_shading_lit | 20 / 20 | 0 / 20 |
| F9_intensity_limit | 20 / 20 | 0 / 20 |
| F10_tex_x | 20 / 20 | 3 / 20 |
| F11_viewport_wh | 20 / 20 | 3 / 20 |
| F12_tls_bank | 20 / 20 | 3 / 20 |
| F13_cordic_round | 20 / 20 | 3 / 20 |
| N1_chunk_order | 20 / 20 | 4 / 20 |
| N2_buffer_hazard | 0 / 20 | 0 / 20 (not caught) |
| N3_pbuf_tag | 20 / 20 | 3 / 20 |
| N4_ftz_sign | 0 / 20 | 0 / 20 (not caught) |
| N5_vs_window | 6 / 20 | 0 / 20 |
| N6_reuse_window | 0 / 20 | 0 / 20 (not caught) |
| N8_sm_frustum | 20 / 20 | 3 / 20 |
| N9_cull_verdict | 16 / 20 | 0 / 20 |
| D1_normal_conv | 20 / 20 | 0 / 20 |
| D2_double_second | 20 / 20 | 3 / 20 |
| D3_sun_clamp | 20 / 20 | 0 / 20 |
| D4_colour_base | 3 / 20 | 0 / 20 |
| D5_fixed_sign | 33 / 400 | 1 / 400 |
| D6_trans_drop | 0 / 20 | 20 / 20 |
| D7_light_z | 20 / 20 | 0 / 20 |
| D8_normal_matrix | 20 / 20 | 0 / 20 |
| D9_reuse_key | 15 / 400 | 0 / 400 |
| D10_copy_shade | 20 / 20 | 0 / 20 |
| D11_list_hazard | 20 / 20 | 3 / 20 |
| D12_trans_latch | 20 / 20 | 3 / 20 |
| D13_const_load | 20 / 20 | 3 / 20 |
| D14_div_ring | 20 / 20 | 3 / 20 |
| G1_slot_alloc | 20 / 20 | 3 / 20 |
| G2_slot_reuse | 20 / 20 | 3 / 20 |
| G3_smfr_const | 20 / 20 | 3 / 20 |
| G4_conv_ebase | 20 / 20 | 3 / 20 |
| G5_outcode_eq | 0 / 20 | 0 / 20 (not caught) |
| G6_amb_byte | 20 / 20 | 0 / 20 |
| G7_div_rem | 20 / 20 | 3 / 20 |
| G8_region_end | 0 / 20 | 0 / 20 (not caught) |
| G9_record_bits | 20 / 20 | 0 / 20 |
| G10_matrix_bank | 20 / 20 | 3 / 20 |
| G11_addr_adder | 0 / 20 | 0 / 20 (not caught) |
| G5b_outcode_sign | 6 / 20 | 0 / 20 |
| G11b_addr_adder | 20 / 20 | 3 / 20 |
| G12_result_mux | 20 / 20 | 0 / 20 |
| N7_div_lane | 0 / 20 | 3 / 20 |

44 of 50 caught.

### Wrong turns and findings

- **Quartus 17 crashes on a 2-D unpacked array** (`logic [4:0] clst [2][16]` with
  both indices variable): a segment violation in elaboration, 13 s into the map, at
  770 MB, so not memory pressure. The flat `[32]` form with a concatenated index
  elaborates. `quartus_map --analyze_file` passes the 2-D form; only
  `--analysis_and_elaboration` finds it.
- **The first fit of this build missed 10 ns by 0.87 ns**: the clip lists put a
  16-entry list read in front of the pool's MLAB read. The pool is now read at
  entry numbers held in registers (vertex ci, ci - 1, the list's last, the queue's
  vertex), updated a step ahead. Then the shared converters' select (M_VPW and cnt)
  sat in front of the converter on seed 3 (-0.08 ns); the selects are now
  registered from the next cnt.
- **R50's side channel did buy cycles here** (0.4% of a frame at p99, VF3), where
  R50 measured none on Bass Fishing. I removed it anyway: it is the engine's fourth
  write-back source on every lane.
- **The facing test on the pool does not pay** (92.1% against 93.7% without it):
  section 12's estimate assumed it costs nothing.
- **`max_depth` does not make a 64-word MLAB one block deep** on this part.
- The walker's node buffer (8 x 32 flip-flops) stays: its words are read four and
  five at a time in two states, so one MLAB read port does not serve it.
- Not caught: G5 (the outcode's equal-magnitude case: no vertex lies exactly on a
  clip plane in these frames; the miter covers it), G8 (no read crosses a region's
  end), G11 (viewport word 25 for 26: no visible difference in these frames, G11b on
  word 12 is caught); N2, N4 and N6 as before.

### Open after the fourth build

- The MLAB / M10K trades above (up to -685 ALM for +28 M10K; `OUT_M10K` alone -228
  for +4) are a study decision.
- A three-lane engine: about -880 ALM and -2 M10K for an estimated +7 to +10% of a
  frame; the largest lever left.
- The set-up on the pool fits at 83.6% / 79.6% with the facing test in its input
  port; that port is not built, and Daytona 2 has to be measured before the
  rasteriser drops its own set-up unit.
- N4, N6, G5 and G8 need directed benches; N2 stays masked by D11's guard.

## 15. The facing test beside the pool, the R62 record, Step 2.1 (fifth build)

Study R62 found that the screen winding agrees with Supermodel's view-space
back-face test on only 58-69% of single-sided triangles, and that culling by the
winding damages the image. Section 14's 83.6% / 79.6% used the model's verdict
loaded into the RTL (simulation only), while its "hardware form" was a 2D cross
product. This build makes the normal test real hardware beside the pool, adds what
the rasteriser asked for in R62, and runs Daytona 2 (Step 2.1) through the model
and the RTL.

### The facing test

Supermodel's test (`R3DShaderTriangles.h`, `CalcBackFace`) is d = dot(view vertex,
mat3(modelview) x face normal), the polygon facing away when d > 0. The model takes
vertex 0. MAME's view space is GL eye space with y negated, which leaves the dot
unchanged. I first measured in the model what precision the test needs. Disagreements
with the same test in double, single-sided polygons of 400 + 400 frames (VF3
2,930,614, Bass Fishing 1,134,655):

| Form | VF3 | Bass Fishing |
|---|---|---|
| FP32, V = ((x T0 + y T1) + z T2) + T3, N = (n0 T0 + n1 T1) + n2 T2, d = (V0 N0 + V1 N1) + V2 N2 | 0 | 0 |
| the same at a 20-bit mantissa | 0 | 0 |
| 18 / 16 / 14 / 12 bits | 13 / 18 / 49 / 128 | 2 / 3 / 11 / 44 |
| block fixed point: the matrix at one exponent, V and N exact, then 27 bits each, d exact | 0 | 0 |
| the same at 20 / 18 / 16 bits | 1 / 10 / 44 | 1 / 7 / 27 |
| model space with the eye position, matrix taken as a similarity | 31 | 0 |
| model space with the eye from the true inverse | 840 | 1 |
| Q = M M^T and r = M t in FP32 per model, then d = v0 Q n + r.n exactly | 28,715-805,427 | 7,707-204,685 |

The last row fails by cancellation: for a large model whose origin is far from the
camera, t.N and (v0 M).N are both large and d is their difference, so Q and r would
need far more than FP32. The model-space forms differ from Supermodel's where the
model matrix is not a similarity (0.54% of VF3's polygons have a deviation over 0.1;
Supermodel uses mat3(modelview), not its inverse transpose). The test is cheap only in
its inputs: vertex 0 and the face normal are integers, but the matrix is FP32 and V
has to be formed before the dot. So the model's test is now the FP32 form of the first
row, in that order (`r3d_geo.cpp`, `sm_cull`), and the hardware computes exactly that.
Over all 800 frames it equals the double test on every polygon; on Daytona 2 it
differs on 14 of 2,136,440 (20 more have d exactly 0).

`r3d_face.sv` is one FP32 multiply-add beside the pool: the pool's own `r3d_fp_mul`
and `r3d_fp_add` (flush to zero, the adder's align stage split as in the lanes), a
32-word MLAB for four polygons' inputs (vertex 0 and the face normal as raw 24-bit
words, converted to FP32 on the way to the multiplier; then V0..V2 over the
position), and a 32-word MLAB for the
model matrix of two models and N0..N2 of two polygons. A period is four rounds of
seven slots: the V and N chains of the polygon in hand in slots 0-5, the d chain of the
polygon before it in slot 6. A chain's steps are seven cycles apart, so each step takes
its predecessor's sum straight from the adder's output, two cycles later: no
accumulators. 28 cycles a polygon; its random bench (`make test_real3d_face`, three
seeds of 300,000 polygons with gaps on both ports) has no verdict wrong.

Where its inputs come from:
- **The model matrix**: main reads the TR rows out of the pool after the model's
  set-up kernel, through the external read port, a word a cycle (17 cycles a model,
  the engine idle and held). Two buffers alternate by model; main clears a buffer
  before the decoder can commit a polygon of the new model, and waits if the model
  two back still has a polygon in the test.
- **Vertex 0 and the face normal**: the decoder writes the header's words 1-3 as they
  stream in, and vertex 0's x y z either as they stream in (a new vertex 0) or from
  its slot entry's raw words during the header (a reused one; the slot memory's read
  port is idle there). No decoder cycle is added unless all four slots are full.
- **The verdict** goes into a 32-entry ring by polygon number. The polygon machine
  waits for it, and with `FACE_DROP` (the default) skips a single-sided back face
  before its assembly and clip, so back faces cost no CLIP or projection kernel.

Back faces are 42% of the single-sided polygons in every game (VF3 1,227,146 of
2,930,333; Bass Fishing 478,963 of 1,133,455; Daytona 2 908,093 of 2,136,440), and
46-49% of the triangles: VF3 1,959,033 of 4,272,181, Bass Fishing 1,328,382 of
2,718,227.

### The record (R62)

Polygon record, 4 + 6 n words (a triangle 22, a quad 28; section 11 was 3 + 7 n):

| Word | Bits |
|---|---|
| 0 | [1:0] priority pass, [2] alpha buffer, [3] inverted copy of a double-sided polygon, [4] texture on, [5] texture page, [8:6] texture width code, [11:9] height code, [14:12] texel format, [20:15] MAME's TRI_PARAM bits, [26:21] transparency (32 = opaque), [31:27] vertex count n (3..16) |
| 1 | [19:0] polygon number, [25:20] texture x / 32, [30:26] texture y / 32 |
| 2 | colour: h4 >> 8 or the colour-table word |
| 3 | [4:0] fog intensity (header[6] bits 11-15), [5] translucency layer select (header[6] bit 17), [6] faces away (the normal test) |
| 4 + 6k .. 9 + 6k | vertex k: screen x, screen y, 1/w, 256 u / w, 256 v / w, shade (FP32) |

1/w is every triangle's, textured or not: the projection kernel's own reciprocal
(its OOW register) goes to the output buffer where z was. Window z and w are no
longer computed or sent, which took the projection kernel from 60 to 48 operations
and 86 to 74 cycles (PROJ3 69 to 68, PROJ1 61 to 60). The bench checks 1/w as the
model's reciprocal of its w.

Viewport record, 6 words, sent before the viewport's polygons (vertex count 0 marks
it): {0, pass 2}; node word 0x20 (scroll fog); 0x22 (fog colour); 0x23 (fog density,
FP32); 0x25 (fog ambient [23:16], fog start [15:0]); kz (FP32, the depth scale the
rasteriser's fog divides by: 1.0 with Supermodel's frustum, 2fn / (f - n) with
MAME's). Main reads words 0x20-0x25 at the end of the viewport's set-up.

A double-sided polygon's inverted copy has -d exactly. With `FACE_DROP` the geometry
sends only the record facing the viewer (the copy when d > 0, the polygon when d < 0,
both when d is exactly 0) and no single-sided back face, so word 3 bit 6 is always 0.
`FACE_DROP = 0` sends every record (both copies, every back face) with bit 6 set on
those facing away.

The set-up's output (section 14), with these changes: the same 4-word header with
n = 3; the three vertices' screen x, y sorted by y (6 words); det (1); then pdx, pdy,
ps for p = 1/w, u/w, v/w, shade (textured, 12 words, a 23-word record) or p = 1/w,
shade (untextured, 6 words, 17). There is no w plane. A double-sided polygon is set up
once with the facing copy's shade (twice when d = 0), and back faces get no set-up
and are not sent. The textured kernel is now SETUP4: 63 operations and 97 cycles
(SETUP5 was 75 and 100), with an 18-word port load (21 before).

### Step 2.1 and Daytona 2

MAME changes two things with the step (`model3_v.cpp`): positions are 13.11 from Step
1.5, and nodes have words 1 and 2 (offset 2). The model already had both (`--step`).
The RTL has a `step15` input: the walker reads word 0, then words 3-9 into the same
buffer places, and the decoder and the facing test take 2^-11 for positions.

I made a new Daytona 2 set as `tools/mame-census/README.md` describes: Supermodel
`036af1c` with the dump patch, the link at Single in its NVRAM (identical to
`sm3-builds/daytona2.nv.bak`), every 30th frame from 1800 to 13770,
`xvfb-run -n 212` (`sm3-builds/census/dumps_daytona2_r62`). A first short run had shown
that Supermodel does not reproduce Daytona 2's dumps byte for byte (all five checked
frames differ from the 2026-10-30 set), unlike VF3 and Bass Fishing, so the old set is
kept beside it. MAME's replay of each set gives its triangle lists and the VROM image.

The model, MAME's rows and correctly rounded frustum, `--step 21`, against MAME's
lists: **every field bit-exact on all 400 frames of both sets** (4,173,093 triangles in
the new one). Step 2.1's formats need nothing more in the model.

Per frame at p99 (new set): 21,270 polygons, 66,880 new vertices, 2,582 models and 3,927
nodes, against VF3's 16,612, 46,278, 1,096 and 1,831.

The RTL with `step15`, Supermodel's frustum, culling and every row against the model
(`--step 21`): **every field on all 400 frames** (2,868,892 triangles), in the default
build and with the set-up load.

### Cycles (bench, 12-20 cycle latency, 20% output stall; 100 MHz frame 1,738,530 cycles)

p99 share of a frame, 400 frames each:

| Build | VF3 | Bass Fishing | Daytona 2 |
|---|---|---|---|
| Section 14 (R60), no set-up | 63.5% | 63.8% | not run (Step 1.0 only) |
| This build without the facing test (`SM_CULL = 0`, every record sent) | 62.5% | 62.1% | 96.9% |
| This build, `FACE_DROP = 0` (the test runs, nothing dropped) | 64.9% | 64.1% | |
| **This build, no set-up** | **61.3%** (1,066,378) | **56.9%** (989,111) | **93.9%** (1,632,908) |
| Section 14, set-up with port C, facing beside the pool (the model's verdict), port load | 83.6% | 79.6% | |
| **This build, set-up with port C, the facing test in hardware, port load** | **78.7%** (1,367,371) | **73.2%** (1,273,022) | **120.7%** (2,098,550) |

Against section 14, the shorter projection kernel and record take 1.0 (VF3) and 1.7
(Bass Fishing) points off; the facing test's own cost, its stalls and the 17-cycle
matrix copy a model, adds 2.4 and 2.0; dropping back faces before their clip and
projection takes 3.6 and 7.2 off again, a net 1.2, 5.2 and (Daytona 2) 3.0 points. The set-up
on the pool fits under 85% on VF3 and Bass Fishing with 6 and 12 points to spare, but
**Daytona 2 does not fit even without it**. Its heaviest frames hold the engine 76% of
the time: transform and light 27.6% of the frame (TL and TL3), node culling 15.7% (CULLU
53 cycles and CULLT 40 a node, latency bound), the node matrix product 9.4%, model set-up
9.4%, projection 8.0%. That is 2,582 models and 3,927 nodes a frame against VF3's 1,096
and 1,831: per-node and per-model work, which the vertex count R20 scaled by does not
see.

### Checks

| What | Result |
|---|---|
| `r3d_face` against the model's FP32 test (`make test_real3d_face`): random matrices (exponents over 64 octaves, zeros), positions and normals, gaps on both ports, its holds honoured | 900,000 polygons on three seeds, 0 verdicts wrong, 0 subnormals; the other alignment of the adder's zero select (one cycle off) gets 47% wrong |
| r3d_geo against the model, 400 VF3 + 400 Bass Fishing frames, every field of every triangle and every viewport record | every frame in ten builds: Supermodel's frustum with and without culling; MAME's correctly rounded frustum with and without; MAME's algebraic one; the default with MAME's rows; MAME throughout; `FACE_DROP = 0`; `OUT_M10K`; the set-up load |
| The same, Daytona 2 (new set, `--step 21`) | every frame, default build and with the set-up load |
| Subnormal or flushed value at the pool's units or the facing test's (monitors) | 0 in every build |
| The model's FP32 facing test against the same test in double | VF3 0 of 2,930,333, Bass Fishing 0 of 1,133,455, Daytona 2 14 of 2,136,440 |
| The model against MAME's lists, MAME's rows, `--step 21`, Daytona 2 | every field, 400 + 400 frames (two sets) |
| `quartus_map --analyze_file` (the micro-code, `r3d_face`, `r3d_walker`, `r3d_geo`), then analysis and elaboration | pass |

### Area and timing (Quartus 17.0, 5CSEBA6U23I7, standalone, 10 ns, `sm3-builds/r3d/r62/fit.sh`)

| Build | Seed | ALM (with the 117 pin ALMs) | Registers | M10K | DSP | Fmax slow 100C |
|---|---|---|---|---|---|---|
| This build, default (`fits/face3`) | 1 | 8,197 | 8,878 | 12 | 11 | 106.15 MHz |
| | 2 | 8,194 | 8,878 | 12 | 11 | 105.56 MHz |
| | 3 | 8,201 | 8,895 | 12 | 11 | 112.55 MHz |
| Section 14 (R60, `fits/e7`) | 1-3 | 7,276-7,293 | 8,101-8,145 | 12 | 10 | 104.1-111.5 MHz |

All three seeds close 10 ns. Without the pins the geometry is **8,077-8,084 ALM: about
910 above section 14, within D-27's 8,550 but about 480 above the 7,600 that the
set-up port was to fit under**, before the port itself.

By unit (seeds 1 / 2 / 3; section 14's seed 1 in brackets):

| Unit | ALM | Registers | M10K | DSP | Fmax (worst path ending there) |
|---|---|---|---|---|---|
| Facing test (`r3d_face`) | 792 / 766 / 780 | 698-708 | 0 (40 ALM of MLAB) | 1 | 111.6 / 105.6 / 114.6 MHz |
| - its adder | 332-336 | 284 | 0 | 0 | |
| - its multiplier | 92-93 | 146 | 0 | 1 | |
| Engine | 2,934 / 2,949 / 2,942 (2,938) | 4,628-4,656 | 11 | 10 | 117.1 / 113.5 / 118.1 MHz |
| Walker | 583 / 577 / 586 (564) | 656 | 0 | 0 | 124.2 / 124.1 / 127.9 MHz |
| Stream | 144 / 135 / 143 (127) | 162 | 0 | 0 | |
| Controller, its memories and the pins | 3,744 / 3,767 / 3,749 (3,647) | | 1 | 0 | 106.1 / 109.9 / 112.5 MHz |

The controller's +97 is the record (the fourth header word, the fog and layer bits in
the emission ring, the six-word vertex), the viewport record, the decoder's feed, the
verdict ring and main's matrix copy.

**The facing test is the cost: about 780 ALM and 1 DSP**, of which the FP32 adder is
a third. R60 had estimated 250-450 ALM for the input port and the facing test
together, because the test was then a 2D cross product. With the normal it is a
dot product of two vectors that are each formed from the FP32 matrix: 21 products and
17 sums a polygon, and no integer form short of 20 bits (table above) avoids an
adder's alignment and normalisation. The trades that would pay for it are those of
section 14: `OUT_M10K` (-228 ALM, +4 M10K), `BC_M10K` (-140, +8), `SIDE_M10K` (-336,
+16), all a study decision.

### Planted faults (`make test_real3d_faults4`; default build; frames 190-209 unless stated)

Section 14's 50 re-run on this RTL (D2 now in the `FACE_DROP = 0` build, D11 at 90%
output stall, F5, D5 and D9 over all 400 frames as before), and 15 new FA faults for the
facing test, the record and Step 2.1 (FA11 and FA12 add 20 Daytona 2 frames). A fault
that hangs the RTL stops at 20 M cycles a frame.

| Fault | VF3 frames differing | Bass Fishing | Daytona 2 |
|---|---|---|---|
| F1_stack_pop | 20 / 20 | 3 / 20 |  |
| F2_vertex_reuse | 20 / 20 | 3 / 20 |  |
| F3_clip_order | 20 / 20 | 3 / 20 |  |
| F4_recip_round | 20 / 20 | 3 / 20 |  |
| F5_viewport_xy | 0 / 20 | 66 / 400 |  |
| F6_pass_label | 20 / 20 | 4 / 20 |  |
| F7_colour_table | 3 / 20 | 0 / 20 |  |
| F8_shading_lit | 20 / 20 | 0 / 20 |  |
| F9_intensity_limit | 20 / 20 | 0 / 20 |  |
| F10_tex_x | 20 / 20 | 3 / 20 |  |
| F11_viewport_wh | 20 / 20 | 3 / 20 |  |
| F12_tls_bank | 20 / 20 | 3 / 20 |  |
| F13_cordic_round | 20 / 20 | 3 / 20 |  |
| N1_chunk_order | 20 / 20 | 19 / 20 |  |
| N2_buffer_hazard | 0 / 20 | 0 / 20 |  | not caught
| N3_pbuf_tag | 20 / 20 | 3 / 20 |  |
| N4_ftz_sign | 0 / 20 | 0 / 20 |  | not caught
| N5_vs_window | 3 / 20 | 0 / 20 |  |
| N6_reuse_window | 0 / 20 | 0 / 20 |  | not caught
| N8_sm_frustum | 20 / 20 | 3 / 20 |  |
| N9_cull_verdict | 16 / 20 | 0 / 20 |  |
| D1_normal_conv | 20 / 20 | 0 / 20 |  |
| D2_double_second | 20 / 20 | 3 / 20 |  |
| D3_sun_clamp | 20 / 20 | 0 / 20 |  |
| D4_colour_base | 3 / 20 | 0 / 20 |  |
| D5_fixed_sign | 29 / 400 | 0 / 400 |  |
| D6_trans_drop | 0 / 20 | 20 / 20 |  |
| D7_light_z | 20 / 20 | 0 / 20 |  |
| D8_normal_matrix | 20 / 20 | 0 / 20 |  |
| D9_reuse_key | 7 / 400 | 0 / 400 |  |
| D10_copy_shade | 20 / 20 | 0 / 20 |  |
| D11_list_hazard | 20 / 20 | 3 / 20 |  |
| D12_trans_latch | 20 / 20 | 3 / 20 |  |
| D13_const_load | 20 / 20 | 3 / 20 |  |
| D14_div_ring | 20 / 20 | 3 / 20 |  |
| G1_slot_alloc | 20 / 20 | 3 / 20 |  |
| G2_slot_reuse | 20 / 20 | 3 / 20 |  |
| G3_smfr_const | 20 / 20 | 3 / 20 |  |
| G4_conv_ebase | 20 / 20 | 3 / 20 |  |
| G5_outcode_eq | 0 / 20 | 0 / 20 |  | not caught
| G6_amb_byte | 20 / 20 | 0 / 20 |  |
| G7_div_rem | 20 / 20 | 3 / 20 |  |
| G8_region_end | 0 / 20 | 0 / 20 |  | not caught
| G9_record_bits | 20 / 20 | 0 / 20 |  |
| G10_matrix_bank | 20 / 20 | 3 / 20 |  |
| G11_addr_adder | 0 / 20 | 0 / 20 |  | not caught
| G5b_outcode_sign | 5 / 20 | 0 / 20 |  |
| G11b_addr_adder | 20 / 20 | 3 / 20 |  |
| G12_result_mux | 20 / 20 | 0 / 20 |  |
| FA1_facing_sign | 20 / 20 | 3 / 20 |  |
| FA2_normal_matrix | 20 / 20 | 0 / 20 |  |
| FA3_copy_choice | 20 / 20 | 3 / 20 |  |
| FA3b_cull_flag | 20 / 20 | 3 / 20 |  |
| FA4_fog_bits | 20 / 20 | 3 / 20 |  |
| FA5_layer_bit | 20 / 20 | 3 / 20 |  |
| FA6_vp_words | 0 / 20 | 20 / 20 |  |
| FA7_vp_kz | 20 / 20 | 20 / 20 |  |
| FA8_oow_word | 20 / 20 | 3 / 20 |  |
| FA9_matrix_wait | 20 / 20 | 0 / 20 |  |
| FA10_slot_gate | 20 / 20 | 0 / 20 |  |
| FA11_step15_scale | 0 / 20 | 0 / 20 | 19 / 20 |
| FA12_step15_node | 0 / 20 | 0 / 20 | 19 / 20 |
| FA13_back_skip | 20 / 20 | 2 / 20 |  |
| FA14_reused_v0 | 20 / 20 | 0 / 20 |  |
| N7_div_lane | 0 / 20 | 3 / 20 |  |

59 of 65 caught.

### Wrong turns and findings

- **The decoder wrote a polygon's face normal into a slot still in use.** Its writes
  stalled the stream while the facing test's four slots were full, but the write
  enable did not wait. The polygon that is first of its model waits for its matrix,
  the slots fill behind it, and the next polygon's normal overwrote its own: 112
  verdicts wrong in VF3 frame 190, every one the first polygon of a model (19% of the
  models). The write is now gated by the slot's state; FA10 plants the old form.
- **The model-space shortcuts do not hold.** Q = M M^T and r = M t per model on the
  pool, then an exact integer d, fails by cancellation (28K-806K polygons wrong on
  VF3); the eye position in model space assumes a similarity and misses 31 VF3
  polygons (840 with the true inverse). Only forming V first works.
- **Converting the inputs as they are written broke timing.** It took 60 ALM off the
  unit but put the decoder's slot read, a mux and the int24 converter in front of the
  MLAB's write: 72-75 MHz on three seeds (the fitter then gave up on timing elsewhere
  too, so the build looked 260 ALM smaller). The conversion is back on the read side;
  the split-align adder and the shorter sum delay stayed (20-45 ALM).
- **My count of the multiply-add's latency was one cycle off** for the adder's zero
  select and forwarding delay; the random bench caught the other alignment (47% of
  verdicts wrong).
- **Supermodel does not reproduce Daytona 2's dumps** byte for byte, unlike VF3 and
  Bass Fishing. Both sets are bit-exact between the model and MAME.
- **The facing test's 780 ALM is about twice what I estimated** before building it,
  and three times R60's estimate, which was for the 2D cross product. Using the pool's
  idle issue slots instead (a fifth operand source into a lane) would save the adder
  and multiplier, but Daytona 2's heaviest frames leave the engine idle only 24% of the
  time, so one lane would give the test 100 cycles a polygon. I did not build it.
- **D2 and D11 need other conditions now.** With FACE_DROP a double-sided polygon is one
  record unless d is exactly 0 (never in the default build's frames), so D2 runs the
  `FACE_DROP = 0` build; the emitter's six-word vertices and the dropped back faces
  drain the output fast enough that D11's list hazard needs a 90% output stall.
- **Daytona 2 is per-node and per-model bound** (above). R20 scaled the geometry from
  the new vertex count alone and put it at 72% of a 50 MHz frame on four multipliers.

### Open after the fifth build

- **The set-up on the pool** fits VF3 (78.7%) and Bass Fishing (73.2%) at p99 but not
  Daytona 2 (120.7%); the geometry alone is 93.9% there. Whether the rasteriser keeps
  its own set-up unit, or Daytona 2 runs at the reduced frame rate D-10 allows, is a
  study decision.
- **Area**: 8,077-8,084 ALM without pins, 910 above section 14, all but 97 of it the
  facing test. The set-up port is not built (an estimated 150-300 ALM more). The
  MLAB-to-M10K trades of section 14 would pay part of it.
- **Daytona 2's node culling** (CULLU and CULLT, 93 latency-bound cycles a node, 15.7%
  of its heaviest frames) is the largest lever found there.
- N2, N4, N6, G5, G8 and G11 still need directed benches (G11b catches the address
  adder itself).

### D-32: the output buffer and the side memories in M10K by default

Study D-32 takes two of section 14's trades: `OUT_M10K` and `SIDE_M10K` now default to 1
in `r3d_geo.sv`; `BC_M10K` stays 0. Nothing else had to change with them:
- **The output buffer** was already built to be read a cycle ahead (section 14). The
  emitter reads the first vertex word during the header, the next word on each beat
  it sends, and the same word in a stall. The R62 record's six-word vertex and its
  shade word (7 for an inverted copy) were already in that read-ahead: `OUT_M10K` was
  one of the ten builds in section 15's checks.
- **The side memories** (T&L staging and the projection queue, 16 of 32 x 32) are read
  registered either way, so their timing does not change. `sa_hi` is registered to line
  up with the data either way.
- **No interaction between the two.** The emitter reads the output buffer; the engine
  reads the side memories through its own port. They share no read path.
- **Read during write.** It differs between the forms: the registered-address MLAB gives
  the new word, the M10K the old one. Every frame is exact in both, so no caller reads a
  word in the cycle it is written, as `r3d_ram` says.

The M2.5 viewer instantiates `r3d_geo` with defaults, so it takes this at its next build.

**Checks.** Each build is the same Verilator bench with the same parameters and bench
arguments as `test_real3d_geo4_all` and `test_real3d_setup4`. The runs are split into
8 slices a game, 16 for Daytona 2 (`sm3-builds/r3d/d32/verify.sh`, `bench.sh`).

| Build | VF3 | Bass Fishing | Daytona 2 |
|---|---|---|---|
| Default | 400 / 400 | 400 / 400 | 400 / 400 |
| Set-up load (`SETUP_LOAD`, `SETUP_3R`, `SETUP_PORT`, the set-up ROM) | 400 / 400 | 400 / 400 | 400 / 400 |
| Supermodel frustum without culling; MAME's correctly rounded frustum with and without; MAME's algebraic one | 400 / 400 each | 400 / 400 each | |
| The default with MAME's rows; MAME throughout | 400 / 400 each | 400 / 400 each | |
| `FACE_DROP = 0` | 400 / 400 | 400 / 400 | |

Every field of every triangle and viewport record matches the model. There are no
subnormals at the pool's units or at the facing test's. The Makefile targets
themselves, run whole and not sliced, give the same result. These are
`test_real3d_geo4` on Daytona 2 and the first three builds of `test_real3d_geo4_all` on
VF3; I stopped the rest, which take about 6.5 minutes a run.

**Cycles.** I ran each build again with `-GOUT_M10K=0 -GSIDE_M10K=0`. In every one of the
ten, the per-frame cycles p50, p99 and maximum are identical to the new default
(Daytona 2's default at 16 slices included). p99, share of a 100 MHz frame:

| Build | VF3 | Bass Fishing | Daytona 2 |
|---|---|---|---|
| Default, no set-up (section 15) | 61.3% | 56.9% | 93.9% |
| Default, D-32 | **61.3%** (1,066,378) | **56.9%** (989,111) | **93.9%** (1,632,908) |
| Set-up on the pool (section 15) | 78.7% | 73.2% | 120.7% |
| Set-up on the pool, D-32 | **78.7%** (1,367,371) | **73.2%** (1,273,022) | **120.7%** (2,098,901 at 8 slices) |

**Planted faults** (`sim/real3d/faults.py`, 20 M cycles a frame cap, in
`sm3-builds/r3d/d32/faults`): 64 of 70 caught. The 65 faults of section 15 give exactly
section 15's table, and the six misses are the same six. No fault hung. I added five
faults on the new read timing, and all five are caught:

| Fault | What it plants | VF3 | Bass Fishing |
|---|---|---|---|
| FO1_out_noahead | output buffer read on the beat's own address, with no read-ahead: each word one late | 20 / 20 | 3 / 20 |
| FO2_out_stall | the read-ahead advances in an output stall | 20 / 20 | 3 / 20 |
| FO3_out_vertex | at a vertex's last word, the read-ahead keeps the vertex instead of taking the next | 20 / 20 | 3 / 20 |
| FS1_side_early | side-memory half select (`sa_hi`) taken from the address, a cycle ahead of the data | 20 / 20 | 3 / 20 |
| FS2_side_late | the same select registered twice, a cycle behind the data | 20 / 20 | 3 / 20 |

FO2 shows only when the output stalls: with `out_ready` always high, `out_go` is
always 1 in E_OUT. The bench stalls 20% of output words.

**Area and timing** (Quartus 17.0, 5CSEBA6U23I7, standalone, 10 ns,
`sm3-builds/r3d/r62/fit.sh`, `fits/d32_s1..3`):

| Seed | ALM (with the 117 pin ALMs) | Without pins | Registers | M10K | DSP | Fmax slow 100C |
|---|---|---|---|---|---|---|
| 1 | 7,649 | 7,532 | 8,813 | 32 | 11 | 103.14 MHz |
| 2 | 7,685 | 7,568 | 8,803 | 32 | 11 | 103.99 MHz |
| 3 | 7,677 | 7,560 | 8,784 | 32 | 11 | 106.30 MHz |
| Section 15 (`fits/face3`) | 8,194-8,201 | 8,077-8,084 | 8,878-8,895 | 12 | 11 | 105.56-112.55 MHz |

All three seeds close 10 ns. The saving is **-509 to -548 ALM (-527 on average) for
+20 M10K**, against the -564 that the two trades measured one at a time.

By unit (seeds 1 / 2 / 3; section 15's in brackets):

| Unit | ALM | M10K |
|---|---|---|
| Facing test | 789 / 787 / 799 (792 / 766 / 780) | 0 |
| Engine | 2,931 / 2,948 / 2,940 (2,934 / 2,949 / 2,942) | 11 |
| Walker | 575 / 581 / 588 (583 / 577 / 586) | 0 |
| Stream | 133 / 133 / 138 (144 / 135 / 143) | 0 |
| Controller memories | 524 / 523 / 524 (1,090 / 1,090 / 1,088) | 21 (1) |
| Controller logic, with the pins | 2,697 / 2,712 / 2,688 (2,655 / 2,678 / 2,662) | 0 |

- **The memories saved what was measured:** -566 ALM against -228 - 336 = -564. The two
  trades add, with no interaction.
- **Most of the shortfall is in the controller's logic,** about +35 ALM on average; the
  seed spread there is about 25. Registers fell by 65-111, mostly the MLABs' registered
  read addresses, which the M10K holds inside the block.
- **M10K is exactly +20** (12 -> 32). Quartus packs each pqa/pqb pair, which share an
  address, as one 64-bit memory of 2 blocks, so the count is unchanged.
- **Fmax fell 2.4-6.3 MHz.** The worst path is now the decoder's k255 conversion
  (`vq_r` -> `tc_k`), with 0.30-0.59 ns of slack; before it was `cq_cnt` -> `pst` or the
  facing test's MLAB. That path did not change, so the fitter placed it worse once the
  MLABs went to M10K columns. 100 MHz still closes with 0.3 ns to spare on the worst
  seed.

The geometry is now about 7,530-7,570 ALM without pins, close to the 7,520 aimed for and
30-70 under the 7,600 target, before the set-up port (an estimated 150-300).

### Wrong turns and findings (D-32)

- **An R65 fault group was still running** (`r62/faults_g0.log`, started 20:28) in the
  shared `sm3-builds/r3d/faults2/` when I started. Each fault copies `rtl/real3d` when
  it starts, so that log's last eight lines (D1, D5, D9, D13, G3, G7, G11, N7, from about
  21:45) ran on the D-32 RTL, not R65's. Their verdicts are the same as section 15's.
  So that two runs cannot clear each other's copies, `faults.py` takes `R3D_FAULTDIR`
  (default unchanged), and mine ran in `sm3-builds/r3d/d32/faults`.
- **R65's first verify pass** (`r62/verify.log`: sm0, cr1, cr0, alg0) differs from mine
  by a few hundred cycles at p99, e.g. sm0 on VF3 1,116,352 against 1,116,751. That is
  not D-32: the same builds with the old defaults give my numbers to the cycle. R65's
  later passes (def2, def3, su3, keep3, sm1m, cr0m) also match mine to the cycle, so the
  first pass ran on an earlier state of the RTL or the bench.
- **The sum of the one-at-a-time trades overstates the joint saving** by about 40 ALM in
  this fit. The memories gave exactly the sum; the controller logic around them did not.

### The LOD word (`LOD_WORD`, study D-33, R67)

Study R67 left the hardware LOD needing one FP32 word a polygon from the geometry,
B = d NP (-cota) kz^2, and one thing unchecked: whether `r3d_face`'s d has the scale of
the dot render_d25 uses. A factor of 2 is a whole level, so I checked that first.

**d's scale** (`sim/real3d/r3d_lod_cmp.cpp`). The geometry model, run on each dump
with MAME's correctly rounded frustum and every node walked (so its polygon numbers
are MAME's), against render_d25's dot(v0, n) from MAME's list, polygon by polygon:

| | VF3 | Bass Fishing |
|---|---|---|
| polygons matched | 2,313,039 | 1,060,013 |
| same sign / opposite / either zero | 2,294,819 / 196 / 18,024 | 1,059,986 / 18 / 9 |
| \|d / dv - 1\| p50 / p99 / p99.9 | 5.6e-8 / 3.4e-5 / 0.030 | 5.8e-8 / 8.6e-6 / 0.0015 |
| off by 1/32 level or more: vertex 0 clipped / not clipped | 2,776 / 1 | 62 / 0 |
| textured, lb8 equal (1/256 level) / within 4 / apart by half a level or more | 1,090,734 / 1,091,838 / 37 of 1,093,039 | 435,700 / 435,911 / 2 of 436,003 |

So d has the model's scale: the ratio is 1 to FP32 rounding on 99% of polygons, and no
bin sits at a factor of 2. Every polygon that is off but one has its vertex 0 clipped:
the geometry takes the polygon's own vertex 0, the list's triangles a clipped one, and
the header normal is not the face's normal, so the dot moves. The geometry's value is
the polygon's, the same for every triangle of it.

**The geometry side.** With `LOD_WORD = 1` the polygon record is 5 + 6 n words, B in
header word 4 (the viewport record stays 6 words); 0 keeps R62's 4 + 6 n.
- **NP** (header word 5 bits 8-31) goes into the facing test's fill slot as word 7, as
  the decoder streams the header; it goes through the test's one converter, unsigned
  at 2^-12. The three converters of the test (position, normal and now NP) became one
  with a muxed exponent.
- **C = -cota kz^2 once a viewport.** Main reads viewport word 3 with the viewport (a
  one-word culling read, `M_VP8`) and loads -cota into the test at `M_VPW`, after the
  last viewport's polygons have all had their verdict. With Supermodel's frustum kz = 1
  and C = -cota needs no product. With MAME's frustum kz is the constant -F_P23, so kz^2
  is a constant (0x408000A8) and the test forms C = kz^2 x (-cota) itself, idle, before
  the viewport's first polygon starts.
- **The two products go in round 3's idle slots** (the period is 28 cycles; slots 3-6 of
  round 3 were idle): slot 3 forms Q = NP x C for the polygon in hand into B memory
  {its N buffer, 3, 3}, which was free; d is kept in a register as it leaves the adder;
  slot 4 of the period after that forms B = d x Q for the polygon two back. B leaves in
  polygon order into a 32-entry ring by polygon number (one MLAB), and the emitter waits
  for it before a record. The inverted copy of a double-sided polygon carries -B.
- **The model** (`r3d_geo.cpp lod_word`) ends each 'T' record with d and B in that
  order, B = fl(d x fl(NP x C)) with flush to zero.

**Checks.**
- `make test_real3d_face_lod`: 1.8 M random polygons on three seeds with kz^2 = 1 and
  MAME's, C changed while idle (91 K times): 0 verdicts and 0 B words wrong.
- The geometry against the model with `LOD_WORD = 1`, every field and B: 400 VF3, 400 Bass
  Fishing and 400 Daytona 2 frames in the default build, and VF3 and Bass Fishing in
  MAME's correctly rounded frustum with and without culling, the algebraic frustum,
  `FACE_DROP = 0` and MAME's rows (no facing test, B = 0): every frame, no subnormal.
- With `LOD_WORD = 0` the default build is bit-exact on all 1,200 frames and its
  cycles are R68's to the cycle (VF3 1,066,378, Bass Fishing 989,111, Daytona 2
  1,632,908 at p99).

**Cycles at p99, 400 frames, 100 MHz frame:**

| Game | `LOD_WORD = 0` | `LOD_WORD = 1` | Change |
|---|---|---|---|
| VF3 | 1,066,378 (61.3%) | 1,067,801 (61.4%) | +1,423 (+0.13%) |
| Bass Fishing | 989,111 (56.9%) | 990,422 (57.0%) | +1,311 (+0.13%) |
| Daytona 2 | 1,632,908 (93.9%) | 1,632,908 (93.9%) | 0 (p50 +367) |

The cost is the record's fifth header word (one emitter beat a record), the NP word's
write into the test, and the extra period a burst of polygons ends with.

**Area and timing** (Quartus 17.0, 10 ns, `sm3-builds/r3d/r62/fit.sh`, ALM without the
117 pin ALMs):

| Build | Seed | ALM | Registers | M10K | DSP | Fmax slow 100C | Facing test (entity) |
|---|---|---|---|---|---|---|---|
| R68 (D-32 default) | 1 / 2 / 3 | 7,532 / 7,568 / 7,560 | 8,784-8,813 | 32 | 11 | 103.1 / 104.0 / 106.3 MHz | 789 / 787 / 799 |
| This default (`LOD_WORD = 0`) | 1 / 2 / 3 | 7,582 / 7,565 / 7,571 | 8,781-8,797 | 32 | 11 | 107.6 / 107.0 / 101.2 MHz | 774 / 771 / 778 |
| `LOD_WORD = 1` | 1 / 2 / 3 | 7,616 / 7,598 / 7,597 | 8,939-8,970 | 32 | 11 | 108.7 / 111.6 / 109.6 MHz | 803 / 810 / 813 |

- The LOD word costs about +31 ALM over this default (+26 to +34 seed by seed), 0 M10K and 0
  DSP: the facing test +29 to +39 (two registers, d and C, the b-side mux, two operand
  kinds and the write-back kinds), the B ring 21 (one MLAB), and the rest about none.
  Registers +160 to +175.
- The default moved 3 to 50 ALM against R68, within the seeds' spread: the facing test's
  converters shared into one gave back about 15 inside the unit (771-778 against
  787-799). The converter's operand path (A's word through the exponent mux) is now the
  test's worst path (102.8-110.9 MHz) but closes. The geometry's worst seed (101.2 MHz,
  default seed 3) is the pool's adder, which did not change.
- With `LOD_WORD = 1` the geometry is 7,597-7,616 ALM, at the 7,600 target and up to 16
  over it, before the set-up port.
- The trees are not quite the same: the `LOD_WORD = 1` fits copied the RTL at 00:42-01:28,
  the default fits at 02:45-03:37, after R70's model-stream change (`ready_start` and
  `M_END` also wait for the stream's last request), which is in the default fits only. It
  is a few gates; the +31 is within a few ALM of the LOD word alone.

**Planted faults** (`faults.py` L1-L10, the LOD build, 20 + 20 frames): 9 of 10 caught.
- Caught: the copy's sign; NP's scale (x 2); C's sign; kz^2 left out (MAME's frustum);
  NP converted as signed (19 of 20 VF3 frames: NP reaches 2048, bit 23); Q in the other
  N buffer; the zero-select alignment below; viewport word 2 for 3; header word 4 for 5.
- Not caught: the emitter not waiting for B (L8). B is always ready long before the
  polygon's record leaves (it follows the verdict by about two periods, and the polygon
  still has its clip and projection to go), so the wait never holds in these frames. It
  stays as an interlock.

### Wrong turns and findings (LOD word)

- **The adder's zero select belongs to the operation one slot earlier.** `c0_d[5]` meets
  the product of the operation issued one cycle before the one that set it, because the
  multiplier's latency is 4 and the line is 5. The R62 schedule worked because its
  `op_c0 = (r == 0)` is the same for neighbouring slots, except at the d chain's first
  step (round 0 slot 6), which took round 1's 0 and so added the idle round-3-slot-6
  product from the period before. That product is a raw normal word read as FP32 times
  B memory's {par, 3, 3}, which nothing wrote: zero in an MLAB that powers up clear. My
  first build put Q there and set `op_c0` on round 3 slot 3 itself: 9,676 verdicts of
  300,000 went wrong (V2's last step lost its addend) and 289 K of 300 K B words. The fix
  starts each chain from the slot before (round 0, round 1 slot 0 for the d chain, round
  3 slots 4 and 5 for the two products). It goes into the default build too, so the d
  chain no longer depends on that word being zero; the default is bit-exact on every
  frame with it. Planted as L7.
- **I typed the 64-entry log2 table by hand first** for the rasteriser's copy; about a
  third of the entries were off by one to six. I generated it from the formula before
  any bench ran.

### The model stream and a late acknowledge (M2.5 viewer, frame 1800)

**Symptom.** The viewer's bench (`sim/viewer/tb_m3v.cpp`) lost triangles from VF3 frame
1800 when polygon RAM reads waited at the DDR3 bridge. In my copy of the viewer, BUSY 30%
and latency 30-120 cycles:

| Reads the bridge holds | Seed 1 | Seed 2 | Seed 3 |
|---|---|---|---|
| 1 | geometry hangs (frame timeout) | geometry hangs | 7,361 of 7,621 |
| 4 | 7,621 | 7,613 | 7,621 |
| 8 | 7,621 | 7,621 | 7,621 |

The case as it was handed to me (8 reads, seed 3) was exact in my copy: the viewer had
moved on since, so the same seed no longer lands on the same cycle. The 4-read seed-2
case gives the same 7,613.

**Reproduced alone.** The viewer bench now writes the geometry's port inputs a byte a
cycle (`M3V_GTRACE`: the three ports' acknowledge and valid, `out_ready`, `start`), and
`tb_r3d_geo --replay` drives `r3d_geo` from it with the words from the files. The replay
gives the viewer's result exactly, so the loss is inside `r3d_geo`, from port timing
alone. The facing test's inputs then showed 16 committed polygons missing after commit
7,213: one model's polygons were never decoded. The record order shifted after it, so
the triangles lost are not a fixed set of 8; the count happens to come out 8 short.

**Root cause** (`r3d_mstream.sv`). A model ends with `stop` while the stream is still
prefetching. The stream counts only acknowledged bursts (`outst`); a burst it has asked
for and that is not yet acknowledged is in `pram_req` / `vrom_req`, and the request must
hold until its ack. `stop` drained the acknowledged bursts and then raised `ready_start`
with that request still waiting. When the memory took it late, after the last
acknowledged burst had drained, three things could happen:
- acknowledged in the cycle the decoder saw `ready_start`: `outst` went to 1, the start
  pulse a cycle later was ignored, and the decoder waited for words forever (the hangs);
- acknowledged in the start cycle (frame 1800): the start's `next <= addr` lost to the
  ack's `next <= next + 8`, so the new model streamed from the old model's next address;
- acknowledged after the start: the old burst's 8 words went to the head of the new
  model's FIFO and `next` stepped past the new model's first 8 words.

In the frame 1800 trace the old model's last acknowledged burst took 380 cycles to drain
and the waiting one was acknowledged at once after it, in the cycle the new model
started. The bench's memory had always acknowledged in 1-3 cycles, so the window never
opened there: the `MS_MON` counters show the decoder never waited on the stream in it.
What opens it is a late acknowledge, as a bridge whose queue is full of other masters'
reads gives; long latency alone does not (latency 150-400 with one read held: 0 bad
starts in the frames run).

**Fix.** `ready_start` also waits for `!pram_req && !vrom_req`. A late burst is then taken
while the stream is idle, and its words are dropped as a drained burst's are. One AND
term. While the stream is idle nothing else raises a request, so `ready_start` cannot fall
between the decoder's test and its start pulse.

**Bench** (`tb_r3d_geo --hostile MASK`): the masked ports get their own random stream; a
request is acknowledged `--ack LO..HI` cycles after it is raised, held a further cycle
with chance `--busy`, and with `--maxrd K` held while K reads are acknowledged and not all
back; words come in order `--lat` cycles after the ack, each held back with chance
`--gap`. `--stall` is the output stall as before. The default bench is unchanged (its
random stream is not touched). `make test_real3d_geo_timing` runs six profiles on 20
frames of each game.

**Which port.** Late acknowledges (100-400 cycles, latency 2-10) on the old RTL:
- polygon RAM alone, frame 1800: 2 starts with a request waiting, frame wrong;
- polygon RAM and VROM, frame 1800: 3, frame wrong;
- VROM alone, frames 190-193: 7, all 4 frames wrong, and the geometry hung in 4 frames
  (decoder in its header state, stream idle with 5 words: the ignored start);
- culling alone, frames 190-199: exact. The walker and the viewport reads hold their
  request until the ack and then wait for their own words; nothing is abandoned there.

**A second fault, found by the new bench after the first fix.** The first full sweep of
the late-acknowledge profile hung in 3 to 19 of each 25 VF3 frames: one frame stopped with
a single viewport record out, and every frame after it put out nothing. Each frame alone
was exact on 12 seeds, so it needed the frame before. `busy` falls in `M_END`, which waits
for the decoder, rings, emitter, walker and facing test but not for the model stream; the
stream's last prefetch of the frame (a VROM request in every case I printed) was still
waiting when `busy` fell. The bench starts each frame with empty ports, as any owner that
hands the port on at `!busy` would, so that request was never answered and the next
frame's first model waited for it for ever. The viewer keeps its ports across frames, so
it does not see this. `M_END` now also waits for `ms_ready`, so `busy` low leaves no read
open at any port, and the bench fails a frame whose `busy` falls with a request waiting or
a word still due. The default bench never ended a frame with a port open (1,200 frames,
old and new RTL), and cycles are identical on all 1,200.


**Checks after both fixes:**
- **Viewer** (my copy of `rtl/viewer` and its bench, the live `rtl/real3d`), VF3 frame
  1800, BUSY 30%, latency 30-120: exact (7,621 triangles, every set-up record, every
  layer pixel) with 1 read held, seeds 1-4; 2 reads, seed 6; 4 reads, seeds 2 and 5; 8 reads,
  seed 3. Five frames (1800, 7800, 7860, 7920, 13770) at BUSY 40%, latency 30-160, one read,
  seed 7: exact, 37,245 triangles. The geometry phase of the 4-read seed-2 case is
  5,527,325 cycles with the first fix and with both.
- **Default bench**, 400 frames each of VF3, Bass Fishing and Daytona 2: exact, and the
  cycle count of every one of the 1,200 frames identical to the old RTL.
- **Timing sweep**, every one of the 1,200 frames under each profile, 25-frame runs:

| Profile | Ports | Ack (cycles) | Latency | BUSY | Gap | Reads held | Stall | Seed | Frames exact | Longest frame |
|---|---|---|---|---|---|---|---|---|---|---|
| 1 | all | 50-400 | 2-20 | 0 | 0 | 1 | 0 | 1 | 1,200 / 1,200 | 10.4 M |
| 2 | all | 1-200 | 1-200 | 60% | 30% | 2 | 90% | 2 | 1,200 / 1,200 | 12.0 M |
| 3 | all | 1-20 | 30-200 | 60% | 50% | 1 | 50% | 3 | 1,200 / 1,200 | 6.9 M |
| 4 | polygon RAM | 1-400 | 1-120 | 30% | 0 | any | 20% | 4 | 1,200 / 1,200 | 5.5 M |
| 5 | culling, VROM | 1-300 | 1-200 | 40% | 20% | 1 | 70% | 5 | 1,200 / 1,200 | 14.4 M |
| 6 | all | 1-2 | 1-2 | 0 | 0 | any | 90% | 6 | 1,200 / 1,200 | 2.6 M |

  6,571,885 triangles a profile; no frame reached the 20 M-cycle cap, none ended with a
  port open. (Profile 4's other ports and profile 5's polygon RAM keep the default model.)

**Wrong turns:**
- My first hostile profiles (acknowledge in 1-3 cycles, or after the port's own read
  drained, latency up to 400) never opened the window: 0 bad starts in 20 frames each. I
  had modelled the bridge's limit per port, but in the viewer the bridge is full of the
  scan-out's and the store's reads, so the geometry's request waits there regardless of
  its own traffic. The acknowledge delay is the parameter that matters.
- I declared the fix done after the viewer cases passed and launched the sweep and the
  fits. The sweep's first profile then hung on the second fault, and I stopped both. A
  fix confirmed in the bench that found the bug is not confirmed in a harsher one.
- Replaying the trace on the fixed RTL cannot pass: once the RTL behaves differently the
  trace's acknowledges no longer meet requests (984 and 11,240 events off the trace). The
  replay is for reproducing; the check is the closed-loop bench.
- Four planted faults' patterns no longer matched the RTL, so `faults.py` stopped at the
  first of them (FA2) with an assertion: the LOD work had changed those lines (FA2: the
  converter select is now `A_NRM`; FA3b: the cull flag is beat `3'd3`'s; FA9 and FA11: the
  start test and the exponent select gained LOD terms). I moved the four patterns to the
  current lines; the faults are the same.

### The set-up on the pool (`SETUP_OUT`, study D-28 option (a), R60, R65)

With `SETUP_OUT = 1` the geometry sends set-up records instead of polygon records. The
format is section 15's SETUP4, which `r3d_rast_intake` takes with `SETUP4 = 1`: the
polygon header with n = 3 (and the LOD word with `LOD_WORD`), x1 y1 x2 y2 x3 y3 sorted
by y, det, then pdx, pdy, ps of 1/w, u/w, v/w and shade. That is 23 words textured and
17 untextured. The parameter defaults to 0, so the viewer's input does not change. With
it on, the geometry does on the pool what the viewer's temporary `m3v_setup` does (the
RTL form of `rtl_tri::setup`), and it drops a triangle with a non-finite x, y or 1/w as
`m3v_setup` does.

**How it is built.** It follows R65's model: one triangle a lane, four triangles a
kernel, results out through the side port. Three things differ from the model, and
each one made it cheaper:

- **No load through the external port.** R65's `SETUP_PORT` held the engine idle for
  18 cycles (12 untextured) before each kernel. Instead a set-up port gathers each
  triangle's sorted inputs into a staging memory while the engine runs other kernels.
  The kernel reads them as operand a through the side port, as T&L and PROJ read
  theirs. X and Y are also needed at ports B and C, so the kernel copies those six into
  registers. Each copy writes the register and the output memory at one address (a new
  `sdup` micro-word, wrf and wside together, which `r3d_vpu` already allowed). The
  copies are therefore also the record's x1 .. y3, and det's last add writes both places
  too.
  - SETUP4: 73 operations, 110 cycles. The model was 18 + 97 = 115.
  - SETUP2: 47 operations, 104 cycles. The model was 12 + 91 = 103.
- **The staging is the projection queue's upper half.** Each `pqa`/`pqb` pair is two
  M10K a lane holding 32 of 256 words (D-32). With `SETUP_OUT` the address gains a bit,
  so {1, half, row, comp} is the staging. It has two halves: the port fills one while
  the kernel reads the other. The port writes one memory of the pair at a time
  (`pq_wea`/`pq_web`), and only when the polygon machine cannot write the next cycle
  (`s_blk`: in P_Q1, or in P_Q0 about to enter it). No memory is added for the inputs.
- **The set-up runs beside the polygon machine, not inside it.** In R65's model the
  polygon machine ran the kernels after each PROJ and kept the engine through all of
  them. Here the port is a third owner in the engine's turn (main, polygon, set-up).
  Without `SETUP_OUT` the turn is the old two-way one, cycle for cycle.

The port takes each projected batch in the emitter's place. For each polygon record it
takes each fan triangle (v0, v[i-1], v[i]), with the inverted copy first as in E_P1:
- It reads the three y words, registers them, and does MAME's sort with three
  comparators.
- It reads the sorted vertices' x, y, 1/w, u/w, v/w and shade (shade' for an inverted
  copy unless fixed) into staging word 3 j + v, one word a cycle. That is about 25 cycles
  a textured triangle and 19 untextured, when the queue write is not blocked.
- Untextured triangles put shade where u/w goes, so one layout serves both kernels.
- A group of four, or the batch's last, runs SETUP4 if any of it is textured, else
  SETUP2.
- The emitter takes its triangles from a 16-entry queue, in order.

Set-up output memory, two halves of 32 words a lane (`SO_M10K`, M10K by default):

| Word | Content |
|---|---|
| 0-5 | x1 y1 x2 y2 x3 y3 (the copies) |
| 6 | det |
| 8 + 4 p + {0, 1, 2} | pdx, pdy, ps of parameter p |
| 8 + 4 p + 3 | parameter p of the first sorted vertex (a flat triangle's ps) |

A flat triangle (|det| < 0.00001) is sent with pdx = pdy = +0, and ps taken from word
8 + 4 p + 3. The emitter registers the flat flag on det's beat. ps is read ahead two
beats later, so no compare sits in front of the memory's address. The header and
viewport words are now one shared `e_hw`/`e_vw` for both emitter paths.

The generator writes two ROMs:
- `r3d_ucode` is unchanged, byte for byte.
- `r3d_ucode_su` holds the same kernels at the same addresses, then SETUP4 and SETUP2
  (625 of 1,024 words). `r3d_vpu`'s `SU_ROM` selects it.
- `--setup-kernels` builds R65's measurement ROM as before.

**Checks.** `make test_real3d_setup_out` and `make test_real3d_geo_timing_su`. With
`--setup-out` the bench cuts each set-up record and checks every word against
`rtl_tri::setup` on the model's triangle (depth on 1/w, flush to zero), and every
header field as before. The model's set-up is linked only with `-DGEO_SETUP_OUT`, so
the existing targets build as they did.

| | VF3 | Bass Fishing | Daytona 2 |
|---|---|---|---|
| Frames exact, every word of every record | 400 / 400 | 400 / 400 | 400 / 400 |
| Set-up records | 2,313,148 | 1,389,845 | 2,868,892 |
| of which textured / flat | 1,834,256 / 72,162 | 1,124,135 / 17,396 | 2,617,302 / 41,861 |
| Left out (x, y or 1/w not finite) | 0 | 0 | 0 |
| Subnormal or flushed values at the pool's units | 0 | 0 | 0 |

- **Hostile timing.** The six profiles of `test_real3d_geo_timing`, 20 frames of each
  game, on the `SETUP_OUT` build: 18 / 18 runs exact, no port open at the end. Two
  profiles also ran on all 1,200 frames: profile 6 (90% output stall), 1,200 exact;
  profile 1 (acknowledges 50-400 cycles late), 1,200 exact. No frame ended with a port open.
- **Default build** (`SETUP_OUT = 0`): exact on all 1,200 frames, and every frame's
  cycle count is identical to the RTL before this work. `LOD_WORD = 1`: 1,200 / 1,200.
  MAME's rows (`SM_LIGHT = 0`, the 255 clamp): 800 / 800.
- **The facing test** (`test_real3d_face`, `test_real3d_face_lod`): 900,000 and 1.8 M
  random polygons, 0 wrong.
- **Quartus.** `quartus_map --analyze_file` passes on every changed file. The default
  build elaborates without `r3d_ucode_su.sv` in its file list. Quartus 17's
  constant-folding of the new size-cast and concatenation idioms gives the expected
  values for all eight.

**Frame load, p99 of a 100 MHz frame over 400 frames:**

| Build | VF3 | Bass Fishing | Daytona 2 |
|---|---|---|---|
| Default, no set-up (R68) | 61.3% (1,066,378) | 56.9% (989,111) | 93.9% (1,632,908) |
| R65's model: set-up load, port C, port load | 78.7% (1,367,371) | 73.2% (1,273,022) | 120.7% |
| **`SETUP_OUT`, built** | **76.9%** (1,337,476) | **71.3%** (1,239,863) | **117.6%** (2,043,682) |

The built form is 1.8, 1.9 and 3.1 points under the model, and Daytona 2 still does not
fit (D-10's reduced rate, or the rasteriser keeps a set-up unit; a study decision).

**Area and timing** (Quartus 17.0, 5CSEBA6U23I7, standalone, 10 ns,
`sm3-builds/r3d/su/fit.sh`: R62's `fit.sh` plus `r3d_ucode_su` and both slow corners):

| Build | Seed | ALM without pins | Registers | M10K | DSP | Fmax 100C | Fmax −40C |
|---|---|---|---|---|---|---|---|
| `SETUP_OUT = 1` | 1 | 7,864 | 9,050 | 43 | 11 | 109.82 MHz | 104.94 MHz |
| | 2 | 7,868 | 9,060 | 43 | 11 | 111.83 MHz | 110.55 MHz |
| | 3 | 7,860 | 9,117 | 43 | 11 | 109.65 MHz | 106.61 MHz |
| Default | 1 | 7,517 | 8,875 | 32 | 11 | 112.87 MHz | 109.22 MHz |
| | 2 | 7,505 | 8,882 | 32 | 11 | 116.86 MHz | 111.17 MHz |
| | 3 | 7,526 | 8,843 | 32 | 11 | 108.04 MHz | 105.15 MHz |
| R68 (D-32) | 1-3 | 7,532-7,568 | 8,784-8,813 | 32 | 11 | 103.1-106.3 MHz | 96.5-99.7 MHz |

Every seed of both builds now meets 10 ns at both slow corners. The default build is
7,505-7,526 ALM, 7-63 below R68.

By unit, the `SETUP_OUT` build, seeds 1 / 2 / 3 (worst path ending there; MHz at 100C,
then at −40C):

| Unit | 100C | −40C |
|---|---|---|
| Controller (with the set-up port) | 109.8 / 111.8 / 109.6 | 104.9 / 110.9 / 106.6 |
| Engine | 116.8 / 116.0 / 120.9 | 113.2 / 110.5 / 116.8 |
| Divide | 116.8 / 121.4 / 123.5 | 117.0 / 118.5 / 119.6 |
| Facing test | 123.7 / 118.1 / 115.0 | 119.5 / 115.3 / 109.6 |
| Walker | 126.7 / 128.6 / 126.3 | 122.4 / 126.1 / 125.3 |
| Model stream | 129.2 / 136.6 / 132.7 | 134.1 / 138.5 / 132.0 |

The worst paths are now the decoder's vertex reuse (`h0` -> `nref`, `nref_chunk`), and
one adder's internal path in seed 2.

The set-up against the default at the same seed (seed 1): **+347.5 ALM, +175 registers,
+11 M10K, +0 DSP.** That is 50 ALM above R60's 150-300 estimate, and inside the
7,600 + 300 the geometry was given.

| §8.4 technique | Where | ALM | M10K |
|---|---|---|---|
| One shared pool | the set-up's 73 / 47 FP operations on the pool's 4 multipliers, 4 adders and divide; no FP unit added. The rasteriser's own unit was 1,300-1,600 ALM and 4 DSP (R60); `m3v_setup` is 1,780 ALM and 7 DSP in the viewer | 0 | 0 |
| MLAB and M10K traded as whichever binds | the staging in the projection queue's unused M10K depth (still two blocks a lane), not a 32 × 32 memory a lane (80 ALM in MLAB, or 4 M10K) | 0 | 0 |
| | the set-up output, 2 × 32 × 32 a lane in M10K; about 160 ALM in MLAB (`SO_M10K = 0`) | 0 | +4 |
| Small storage out of flip-flops | the triangle queue (16 × 9) in one MLAB; the polygon's three flags in the `SETUP_LOAD` flag MLAB, into the batch list's spare width (17 → 20 bits, the same MLAB) | +22 | 0 |
| Duplicated arithmetic built once | the X/Y register copies are also the record's x/y words, and det's last add writes both (`sdup`): 7 operations fewer, no copy logic | 0 | 0 |
| Converters time-shared | the inputs read straight off the side port, so no new source on the external write port's 4 × 32-bit data and no 18-cycle hold | 0 (est. 64-128 ALUT saved) | 0 |
| | one 4:1 lane select of the projection output for every gathered word; one for the output memory | in the +325 | |
| Per-game parameters | `SETUP_OUT`, `SO_M10K` | | |
| Port C and the ROM (R65) | a third register copy a lane; ROM 512 → 1,024 words | +12 | +7 |
| The port's controller | gather, sort, groups, kernel turn, emitter path, queue-write guard | +325 | 0 |

### The cold corner: three paths, none of them an M10K

All three D-32 fits miss 10 ns at slow −40C (97.2 / 96.5 / 99.7 MHz), and so does one of
the three late-acknowledge fits (lateack_s3, 97.9). R65's fits, before D-32, met −40C
at 103.4-109.5 MHz. R68 reported only the 100C corner. I refitted lateack_s3: same RTL
and seed, so the same placement, 101.15 / 97.87 MHz. This time I kept the database for
both corners. Its failing paths at −40C:

| Path | −40C slack | 100C slack | Data delay |
|---|---|---|---|
| decoder `vq_r` → `tc_k` (the byte select, the 9-bit negate of 2b + 1, k255a's 24-bit sum) | −0.218 ns | +0.408 | 9.54 ns |
| pool lane 0, adder result → post-op clamp → write-back register `wd` | −0.193 | +0.114 | 9.52 ns |
| facing test, MLAB word → `itof` (24-bit negate, leading one, shift) → `opa` | +0.060 | +0.275 | 9.54 ns |

None of them starts or ends at an M10K. They are long logic paths that lose about 3%
cold. They were placed worse once the memories moved to the M10K columns. Each fix leaves
every cycle and every result as it was:

- **k255.** 2b + 1 is odd, so −(2b + 1) is {~b, 1} in nine bits. Its magnitude byte is
  {~b[6:0], 1}: an XOR, not a subtract (checked for all 256 bytes). That alone was not
  enough. In the next fit the worst path started at the ambient byte, `vq_r[8]`, so the
  ambient byte now has its own register `amb_b`, loaded beside `amb_ld` from the same
  word. `vq_r` is the main machine's widely used register.
- **Post-op.** The clamp only ever puts +0 or the clamp constant (1.0, or 255 for MAME's
  rows) in place of the sum. The flags are decided at +11 and registered beside the raw
  result, and the constant goes in at +12 on the way into the memory. Until now that
  cycle only routed the register to the memory's input register, so the write still
  lands at +12. Only the lane registers (and, with MAME's rows, the side memories) get
  the select. The micro-code never puts a post-op result in the broadcast file:
  `gen_ucode.py` asserts it, and the bench's `$error` catches it.
- **Facing test.** The sign and the 24-bit magnitude are formed in the read cycle. The
  converter's cycle keeps only the leading-one search, the shift and the exponent. Its
  unit fell from about 789 to 735-738 ALM.

The default build's seed 1 went from R68's 103.14 / ~97 MHz to 112.87 / 109.22 MHz,
and from 7,532 to 7,517 ALM.

### Planted faults (`sim/real3d/faults.py`; the SU faults run the `SETUP_OUT` build with `--setup-out`)

Every existing fault re-run on this RTL, plus 15 SU faults for the set-up port. A fault that hangs
the RTL stops at 20 M cycles a frame. The 82 faults that were there before give the same verdicts as
before (R68 and the LOD word): the misses are R60's six and L8. SU11 runs at 90% output stall.

| Fault | VF3 frames differing | Bass Fishing | Daytona 2 |
|---|---|---|---|
| F1_stack_pop | 20 / 20 | 3 / 20 |  |
| F2_vertex_reuse | 20 / 20 | 3 / 20 |  |
| F3_clip_order | 20 / 20 | 3 / 20 |  |
| F4_recip_round | 20 / 20 | 3 / 20 |  |
| F5_viewport_xy | 0 / 20 | 66 / 400 |  |
| F6_pass_label | 20 / 20 | 4 / 20 |  |
| F7_colour_table | 3 / 20 | 0 / 20 |  |
| F8_shading_lit | 20 / 20 | 0 / 20 |  |
| F9_intensity_limit | 20 / 20 | 0 / 20 |  |
| F10_tex_x | 20 / 20 | 3 / 20 |  |
| F11_viewport_wh | 20 / 20 | 3 / 20 |  |
| F12_tls_bank | 20 / 20 | 3 / 20 |  |
| F13_cordic_round | 20 / 20 | 3 / 20 |  |
| N1_chunk_order | 20 / 20 | 19 / 20 |  |
| N2_buffer_hazard | 0 / 20 | 0 / 20 |  | not caught
| N3_pbuf_tag | 20 / 20 | 3 / 20 |  |
| N4_ftz_sign | 0 / 20 | 0 / 20 |  | not caught
| N5_vs_window | 3 / 20 | 0 / 20 |  |
| N6_reuse_window | 0 / 20 | 0 / 20 |  | not caught
| N8_sm_frustum | 20 / 20 | 3 / 20 |  |
| N9_cull_verdict | 16 / 20 | 0 / 20 |  |
| D1_normal_conv | 20 / 20 | 0 / 20 |  |
| D2_double_second | 20 / 20 | 3 / 20 |  |
| D3_sun_clamp | 20 / 20 | 0 / 20 |  |
| D4_colour_base | 3 / 20 | 0 / 20 |  |
| D5_fixed_sign | 29 / 400 | 0 / 400 |  |
| D6_trans_drop | 0 / 20 | 20 / 20 |  |
| D7_light_z | 20 / 20 | 0 / 20 |  |
| D8_normal_matrix | 20 / 20 | 0 / 20 |  |
| D9_reuse_key | 7 / 400 | 0 / 400 |  |
| D10_copy_shade | 20 / 20 | 0 / 20 |  |
| D11_list_hazard | 20 / 20 | 3 / 20 |  |
| D12_trans_latch | 20 / 20 | 3 / 20 |  |
| D13_const_load | 20 / 20 | 3 / 20 |  |
| D14_div_ring | 20 / 20 | 3 / 20 |  |
| G1_slot_alloc | 20 / 20 | 3 / 20 |  |
| G2_slot_reuse | 20 / 20 | 3 / 20 |  |
| G3_smfr_const | 20 / 20 | 3 / 20 |  |
| G4_conv_ebase | 20 / 20 | 3 / 20 |  |
| G5_outcode_eq | 0 / 20 | 0 / 20 |  | not caught
| G6_amb_byte | 20 / 20 | 0 / 20 |  |
| G7_div_rem | 20 / 20 | 3 / 20 |  |
| G8_region_end | 0 / 20 | 0 / 20 |  | not caught
| G9_record_bits | 20 / 20 | 0 / 20 |  |
| G10_matrix_bank | 20 / 20 | 3 / 20 |  |
| G11_addr_adder | 0 / 20 | 0 / 20 |  | not caught
| G5b_outcode_sign | 5 / 20 | 0 / 20 |  |
| G11b_addr_adder | 20 / 20 | 3 / 20 |  |
| G12_result_mux | 20 / 20 | 0 / 20 |  |
| FA1_facing_sign | 20 / 20 | 3 / 20 |  |
| FA2_normal_matrix | 20 / 20 | 0 / 20 |  |
| FA3_copy_choice | 20 / 20 | 3 / 20 |  |
| FA3b_cull_flag | 20 / 20 | 3 / 20 |  |
| FA4_fog_bits | 20 / 20 | 3 / 20 |  |
| FA5_layer_bit | 20 / 20 | 3 / 20 |  |
| FA6_vp_words | 0 / 20 | 20 / 20 |  |
| FA7_vp_kz | 20 / 20 | 20 / 20 |  |
| FA8_oow_word | 20 / 20 | 3 / 20 |  |
| FA9_matrix_wait | 20 / 20 | 0 / 20 |  |
| FA10_slot_gate | 20 / 20 | 0 / 20 |  |
| FA11_step15_scale | 0 / 20 | 0 / 20 | 19 / 20 |
| FA12_step15_node | 0 / 20 | 0 / 20 | 19 / 20 |
| FA13_back_skip | 20 / 20 | 2 / 20 |  |
| FA14_reused_v0 | 20 / 20 | 0 / 20 |  |
| N7_div_lane | 0 / 20 | 3 / 20 |  |
| FO1_out_noahead | 20 / 20 | 3 / 20 |  |
| FO2_out_stall | 20 / 20 | 3 / 20 |  |
| FO3_out_vertex | 20 / 20 | 3 / 20 |  |
| FS1_side_early | 20 / 20 | 3 / 20 |  |
| L1_copy_sign | 20 / 20 | 2 / 20 |  |
| L2_np_scale | 20 / 20 | 3 / 20 |  |
| L3_c_sign | 20 / 20 | 3 / 20 |  |
| L4_kz2 | 20 / 20 | 3 / 20 |  |
| L5_np_signed | 19 / 20 | 0 / 20 |  |
| L6_q_par | 20 / 20 | 3 / 20 |  |
| L7_c0_align | 20 / 20 | 2 / 20 |  |
| L8_b_nowait | 0 / 20 | 0 / 20 |  | not caught
| L9_cota_word | 20 / 20 | 3 / 20 |  |
| L10_np_word | 20 / 20 | 3 / 20 |  |
| T1_stream_late_ack | 20 / 20 | 14 / 20 |  |
| T2_end_open | 14 / 20 | 10 / 20 |  |
| SU1_sort_c3 | 20 / 20 | 3 / 20 |  |
| SU2_sort_c1 | 20 / 20 | 3 / 20 |  |
| SU3_copy_shade | 20 / 20 | 0 / 20 |  |
| SU4_flat_ps | 17 / 20 | 2 / 20 |  |
| SU5_flat_never | 17 / 20 | 2 / 20 |  |
| SU6_nonfin_drop | 0 / 20 | 0 / 20 |  | not caught
| SU7_queue_guard | 20 / 20 | 3 / 20 |  |
| SU8_stage_half | 20 / 20 | 3 / 20 |  |
| SU9_out_wait | 20 / 20 | 4 / 20 |  |
| SU10_kernel_kind | 20 / 20 | 3 / 20 |  |
| SU11_half_free | 20 / 20 | 4 / 20 |  |
| SU12_word_lane | 20 / 20 | 3 / 20 |  |
| SU13_group_size | 20 / 20 | 4 / 20 |  |
| SU14_two_records | 0 / 20 | 0 / 20 |  | not caught
| SU15_stage_wait | 20 / 20 | 4 / 20 |  |
| FS2_side_late | 20 / 20 | 3 / 20 |  |

88 of 97 caught. Not caught: N2_buffer_hazard, N4_ftz_sign, N6_reuse_window, G5_outcode_eq, G8_region_end, G11_addr_adder, L8_b_nowait, SU6_nonfin_drop, SU14_two_records. SU6 needs a non-finite triangle and SU14 a polygon with d exactly 0; neither occurs in these frames.

### Wrong turns and findings (the set-up on the pool)

- **My first corner script named the cold corner `7_slow_1100mv_n40c`.** Quartus 17
  calls it `7_slow_1100mv_-40c`, so the TimeQuest script failed after the 100C half, and
  the fit script deleted the database before I saw the cold paths. Refitting lateack_s3
  cost a slot. `fit.sh` now keeps the database unless the −40C section is written.
- **The first set-up fit failed both corners on my own sort**: it compared the third y
  word straight off the projection output's M10K (−0.374 ns at −40C, −0.308 at 100C).
  The three y words are now registered before any compare, which costs one cycle a
  triangle and no frame-level cycles (VF3 p99 77.0% → 76.9%).
- **My first post-op fix put a 32-bit select on each of the three write-back paths per
  lane**, about +120 ALM in that fit (7,997 without pins), where the old code had one
  shared select. Post-op results only go to the lane registers (and MAME's shade to the
  side memories), so only those carry the select now.
- **The XOR alone did not clear k255.** The next fit's worst path came from the
  ambient byte through `vq_r`, a register shared across the main machine, not from the
  negation.
- **Quartus 17 would not elaborate the default build without `r3d_ucode_su.sv`.** The
  inactive generate branch's instance used implicit port connections (`.clk`), and
  Quartus wants the module declared to resolve them. Verilator accepted it. Explicit
  connections fixed it, and the viewer's and core's file lists need no change for the
  default build.
- **Duplicating the emitter's header and viewport code made FA7's and L1's patterns
  match twice**, and `faults.py` stops at an ambiguous pattern. The code is now shared
  (`e_hw`, `e_vw`). FA3b, FA7, L1, D1, D3, G6, G12 and L5 moved to the current lines,
  and the faults are the same. G12 now plants "the post-op's select left out of the
  lane-register write", the old "result mux" in its new place.
- **R65's measurement kernels were never checked for their results**, because the
  results were unused. Built, SETUP4 and SETUP2 give `rtl_tri::setup`'s words on 6.57 M
  records with no change to the rounding order.
- **The built form beats the model** by 1.8-3.1 points: there is no 18-cycle hold, the
  kernel is 5 cycles shorter, and the port overlaps the polygon machine.
- **72,162 flat triangles in VF3's 400 frames** (3.1% of records) take the flat path,
  so its substitution is not a corner case.

### Open after the set-up

- Daytona 2 is 117.6% at p99 with the set-up (93.9% without).
- The port is about 50 ALM over R60's estimate. Staging in `pqa` alone (with the
  address's spare bit) would take the shared write data's second source away, an
  estimated 20-40 ALM.
- SU6 (the non-finite drop) and SU14 (the second record of a polygon with d exactly 0)
  are not reached by these frames: no non-finite triangle, and no d of exactly 0.
