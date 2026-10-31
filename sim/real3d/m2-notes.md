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
