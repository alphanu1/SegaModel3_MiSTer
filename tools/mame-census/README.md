# MAME census for gates M0-A1 and M0-B

A patch to MAME 0.289 (`f34f0250`) that counts what a Model 3 game does, so
the design study works from measurements instead of guesses.

- **CPU (M0-A1):** every executed PowerPC instruction by opcode class, SPR
  traffic, MSR translation state, PC region (RAM / fixed CROM / banked CROM),
  code footprint, distinct 32-byte I-cache lines per frame, single vs double
  FP, and whether each double result is exactly representable as single.
  Optional `M3IDLE` PC ranges count busy-wait loops per frame.
- **Real3D (M0-B):** highest word and distinct words written to display-list,
  culling and polygon RAM, and per frame: models, polygons (and how many come
  from VROM), new and reused vertices, polygon fetch bytes, triangles after
  clip, summed triangle area (overdraw), texture uploads; texture-sheet tiles
  written, every texel the renderer reads, distinct 4x4 blocks per frame, and a
  4-way texel cache with 32-byte lines simulated at 8, 16, 32, 64 and 128 KiB.

## Build

```
git clone --local /path/to/mame mame289-m3 && cd mame289-m3
git checkout f34f0250
git apply ../../tools/mame-census/mame0289-model3-census.patch
make SUBTARGET=m3 SOURCES=src/mame/sega/model3.cpp TOOLS=0 NOWERROR=1 -j20
```

Build under `sm3-builds/`, never in this tree.

## Run

```
M3CENSUS=$PWD/vf3 \
M3IDLE=1E928-1E930,1E97C-1E984,1E9DC-1E9E4,1E9FC-1EA04,2518-251C \
../mame289-m3/m3 vf3 -rompath /home/ben/roms/Model3 \
    -video none -sound none -nothrottle -seconds_to_run 360 -skip_gameinfo \
    -cfg_directory cfg -nvram_directory nvram -autoboot_script play.lua
```

Run from a scratch folder under `sm3-builds/`: MAME writes `cfg/` and
`nvram/` there, and `nvram/` is ROM-derived. `play.lua` coins up at frame
5200 (`M3_COIN_FRAME`), starts and mashes player 1 so the run covers gameplay.

Add `-numprocessors 1` when the texel cache matters: MAME's renderer is
multithreaded, and the cache model needs texels in drawing order. It runs at
about 30% speed that way.

Outputs: `<prefix>.txt` and `<prefix>_frames.csv` (CPU), `<prefix>_real3d.txt`
and `<prefix>_real3d.csv` (Real3D). The census callback costs about 20% of
MAME's speed; a 360 s run takes about 7 minutes.

The `M3IDLE` ranges are VF3's vblank and decrementer wait loops, found from
the hottest-PC table. Other games need their own.

## Supermodel replay (games MAME cannot run)

MAME stalls on Bass Fishing (I/O board error) and Daytona 2. Supermodel runs them,
but draws through OpenGL, so it cannot count texels. Instead Supermodel saves the
Real3D memories every N frames and MAME's renderer replays them with the census on.

Supermodel at `036af1c` (github.com/trzy/supermodel), with
`supermodel-real3d-dump.patch`; needs SDL2_net. Build with
`make -f Makefiles/Makefile.UNIX`.

```
env -u WAYLAND_DISPLAY SDL_VIDEODRIVER=x11 \
M3DUMP_DIR=$PWD/dumps_bassdx M3DUMP_EVERY=30 M3DUMP_START=1800 M3DUMP_EXIT=13800 \
xvfb-run -a -s "-screen 0 640x480x24 +extension GLX" \
    ./bin/supermodel /home/ben/roms/Model3/bassdx.zip \
    -no-throttle -no-vsync -no-sound -no-dsb -no-gpu-thread
```

Each dump is the 8C (4 MiB), 8E (1 MiB), polygon (4 MiB) and texture (8 MiB) RAM,
gzip level 1, 0.5–5 MB. Then replay one dump per MAME frame on the same set, which
supplies VROM and the stepping:

```
M3REPLAY=$PWD/dumps_bassdx M3CENSUS=$PWD/replay_bassdx \
../mame289-m3/m3 bassdx -rompath /home/ben/roms/Model3 -video none -sound none \
    -nothrottle -numprocessors 1 -seconds_to_run 120 -skip_gameinfo
```

During a replay the game's own frames are not drawn. Unknown texture types and
traversal addresses MAME would treat as fatal are counted and skipped instead.
Without input injection, Supermodel runs cover attract mode only. The RAM-footprint
lines in a replay's summary describe MAME's own game, not the dumps.

### Supermodel CPU census

The same patch counts every instruction Supermodel's PowerPC interpreter executes.
`M3CPU=<prefix>` writes `<prefix>_cpu_frames.csv` (instructions, idle, ROM-executed
per frame) and `<prefix>_cpu.txt` (hottest RAM PCs with their opcodes). `M3IDLE`
takes the same busy-wait ranges as the MAME census. Run with `-no-idle-skip`, or
Supermodel skips the wait loops and they are never counted. Supermodel renders every
frame, so its per-frame counts are the 60 Hz figures; MAME's run at VF3's 30 Hz.

Wait loops found so far: VF3 `1E928-1E930,1E97C-1E984,1E9DC-1E9E4,1E9FC-1EA04,2518-251C`;
Daytona 2 `4C754-4C760,4C73C-4C744,4C6CC-4C6D4,541C-5428`;
Bass Fishing Deluxe `79D30-79D38,79CB8-79CC0,79C64-79C6C,79CC4-79CCC,88A9C-88AA0,8AC20-8AC2C,8ABA0-8ABA4`
(vblank-flag polls, a decrementer poll, a timebase poll and two count-down loops, from a
6,000-frame attract run's hottest-PC table).

`M3SHOT_DIR` (with `M3SHOT_EVERY`, or `M3SHOT_LIST` for a comma-separated list of frame
numbers) saves numbered framebuffer grabs. They are taken
before the 3D is drawn, so they show the 2D layers only. Supermodel's BMP header has a
wrong file size; read them with `magick -define bmp:ignore-filesize=true`.

### CPI model

`M3CPI=1` (with `M3CPU` and `M3IDLE`) runs an in-order scalar 603e model over the busy
instruction stream, with every load and store address taken from Supermodel's memory
accessors. Each configuration (cache size, line fill, first-word latency, store buffer)
runs side by side and adds a `cyc_<name>` column per frame to `_cpu_frames.csv`.
Configurations are listed in `m3cpi_model::init()` in `ppc603.c`.

`M3CPI_FPU=1` (with `M3CPI`) adds an FPU sweep: 26 FPU latency options at 100, 115 and 120 MHz, plus a 603e reference
at 66 MHz and a split-latency grid at 120 MHz (below), each at 100, 115
and 120 MHz, all on one shared 16 KiB 4-way I and D cache pair with store buffer and early
restart (the R22 bold row). Cache hits and misses do not depend on timing, so they are
resolved once per instruction; each option keeps its own scoreboard, and a frame's
instructions are replayed through the options on worker threads at the end of the frame.
Memory-side costs scale with the clock, rounded up: first word 12/14/15 cycles, uncached
load 20/23/24; posted uncached stores stay 2, and FP latencies stay in cycles. Columns are
`cyc_f<MHz>_<option>`; options are listed in `m3cpi_model::init_fpu()`.
`M3CPI_FPU_ONLY=<text>[,<text>...]` keeps only the options whose name contains one of the texts (for example
`est` for the table-estimate `fres`/`frsqrte` options, `meas` for the built FPU's measured
latencies: FMA at L = 12 or 8, divider fdiv 38, fdivs 24, fres 24, frsqrte 97, all blocking).
`ref66*` is the 603e at 66 MHz: singles and double adds 3 cycles pipelined, double multiplies
2-1-1, fdivs and fres 18 and fdiv 33 blocking the FPU, frsqrte 3 pipelined, FPSCR
instructions wait for the FP pipeline, divw 37; memory costs scaled to 66 MHz (first word
8, uncached load 14), or kept at the 100 MHz cycle counts in `_slow`; `_dual` adds the
603e's dispatch of two instructions a cycle to different units (IU, LSU, FPU, SRU, BPU).
`sp_s<Ls>_d<Ld>[n]_q<97|24|est>` runs at 120 MHz only: single-precision ops (opcode 59)
Ls, double arithmetic, frsp and fctiw Ld (`n`: issuing every Ld cycles), the built
divider, and frsqrte on the divider or as an Ls estimate.
`rsq_l12_q<Q>` (120 MHz) is the unified L = 12 FPU with the built divider and frsqrte at Q
cycles. `lh_q<24|60>_ld<1|2>[_ic]` (120 MHz) adds the built caches' costs: `ld2` a load hit
that holds MEM a second cycle (one cycle per word loaded, load-use one longer), `_ic` a
2-cycle redirect for every taken branch, one extra cycle for each sequential fetch into a
new line, and miss first words at 16 (D) and 15 (I) cycles.
`ref66_dual_ic[_slow]` adds the 603e's branch folding (User's Manual 6.4.1.1): branches take
no dispatch slot, a correctly predicted taken branch costs nothing, a misprediction
refetches 2 cycles after its condition is ready, and `bclr`/`bcctr` wait for LR/CTR.
`dt_<u12|s4d12|s6d12>_<q36|q97|qest>_<ld1|ld2>_ic` (120 MHz) is the decision grid: unified
L = 12 or singles at 4/6 with doubles at 12, frsqrte on the divider or a pipelined estimate
at 12, both load-hit models, built I-side costs. It runs at 100, 105, 110 and 120 MHz; the
built caches' miss latency L (12 cycles at 120 MHz) scales with the clock, rounded up
(10, 11, 11, 12), with first words at L + 3 (I) and L + 4 (D).
`_ld1b` is the built load path: load-use distance 3, a load hit outside the predicted way
(the set's most recently used way) 2 cycles more, FP loads holding WB one cycle; the run's
`_cpu.txt` gives the shared D-cache's load hits in and outside the predicted way.
`lv_<ld1b|ld1>_<r2|r1|b64|b256>_<n1|n0>_<w2|w1>_<d0|da|db>` prices throughput levers on
`s4d12_qest` with the built caches: load use 3 or 2 (way-miss and FP-load costs kept),
taken-branch redirect 2 or 1 cycles or free on a hit in a direct-mapped 64- or 256-entry
BTB (target must match; a misprediction still costs 2), sequential new line 1 or 0, way
miss 2 or 1, and single issue, dual issue to two different 603e units (`da`), or dual issue
of an FP op with a non-FP op (`db`). All 96 run at 100 MHz; at 110 MHz the single-lever
options and the `_b256_n0_w1_` sets. `_cpu.txt` gives taken branches and BTB hits.
`l6_<lever row>[_nowb]` (100 and 110 MHz) is the built fast single path: singles and the
frsqrte estimate at latency 6, issuing only when no FP pipeline result is due at t + 6 or
later (`_nowb` drops that rule), doubles pipelined at 12. `dn<16|24|32>_<lever row>` drops the
pipelined double FMA: doubles, frsp and fctiw iterate through the single unit, blocking the FP
pipeline for 16, 24 or 32 cycles. FP operands are
decoded exactly here (the R22 columns keep their original decode), so `f100_base` repeats
R22's latencies on the exact decode.

### FP operand census

`M3FP=<prefix>` (with `M3CPU` in Supermodel; with `M3CENSUS` in MAME, same patch file
`m3fp_census.h` in both) writes `<prefix>_fp.txt` and `<prefix>_fp_frames.csv`. Per FP
opcode: input and result classes (zero, denormal, below single range, infinity, quiet and
signalling NaN), single-format representability of every operand of single-precision and
estimate instructions, and for single arithmetic the emulator's result against the
double-then-float model and against an exactly once-rounded single (computed from the
double operands with error-free transformations; `mp_single()`, built with `M3FP_MPFR`,
is the MPFR version it was checked against: 4 M random cases, all four rounding modes, no
difference). Also: double `fmadd`-family results against a fused double, FPSCR
control byte after every `mtfsf`, `mtfsfi`, `mtfsb0` and `mtfsb1`, the FPSCR seen by `mffs`,
`mcrfs` fields, MSR FE0/FE1 over every instruction, `fctiw(z)` inputs, the producer of each
non-single operand, the consumers of `frsqrte` and `fres` results to three steps, and the
code around every FPSCR instruction site.

Caveats. `fctiw(z)` results are an integer bit pattern, so they count as denormal results.
Both emulators compute `frsqrte` and `fres` exactly, where the 603e returns an estimate, so
an operand produced by them counts as not single-representable here although the
hardware's estimate may be. Supermodel does not set the FPSCR sticky exception bits, so the
values `mffs` reads show FPRF only. MAME's DRC calls the census before each instruction,
so a result is read at the next instruction's call; an interrupt taken between the two
leaves the destination stale, which is the likely source of MAME's rare
more-than-one-ulp differences that Supermodel does not show.

## Real3D triangle lists (M2 golden data)

`mame0289-real3d-tris.patch` applies on top of `mame0289-model3-census.patch` and
writes, during an `M3REPLAY` run, one triangle-list file per replayed dump: every
triangle after clip, divide and viewport as the rasteriser receives it, with the
viewport, model, polygon (header words, clip-space vertices before clip) and early-exit
records around it. The format is documented in `sim/real3d/r3d_trilist.h`.
`M3TRIS_VROM=<file>` also saves MAME's 64 MiB interleaved VROM image once, for the
reference model; it is ROM-derived and stays under `sm3-builds/`.

```
M3REPLAY=$PWD/dumps_vf3 M3CENSUS=$PWD/r3d/vf3/census M3TRIS=$PWD/r3d/tris_vf3 \
M3TRIS_VROM=$PWD/r3d/vrom_vf3.bin \
../mame289-r3d/m3 vf3 -rompath /home/ben/roms/Model3 -video none -sound none \
    -nothrottle -numprocessors 1 -seconds_to_run 120 -skip_gameinfo
```

400 VF3 dumps take 77 s and 1.5 GB; 400 Bass Fishing Deluxe dumps 0.8 GB. Then
`make test_real3d R3D_GAME=vf3` diffs the C++ model (`sim/real3d/`) against them.

`supermodel-frame-grab.patch` (on top of `supermodel-real3d-dump.patch`) saves the
finished frame, 3D included, for the frames listed in `M3SHOT3D_LIST` into
`M3SHOT3D_DIR`. Read the BMPs with `magick -define bmp:ignore-filesize=true`.

## Real3D frame images (M3 golden data)

`mame0289-real3d-frames.patch` applies on top of the census and triangle-list
patches. During an `M3REPLAY` run, `M3FRAMES=<dir>` writes MAME's 3D layer for each
replayed dump as `<dump>.3d` (496 x 384 little-endian ARGB words, alpha 0 where no
triangle wrote). With `M3TILES=<dir>` holding Supermodel's tile-generator state for
the same frames, it also loads that state for one call of `screen_update_model3` and
writes the composite as `<dump>.full`; MAME's own tile state is restored afterwards.
Run `M3TRIS` in the same replay: MAME reads an uninitialised vertex slot on some VF3
polygons (`sim/real3d/m2-notes.md`), so triangle lists from another run can differ
from what that run drew.

`supermodel-real3d-frames.patch` applies on top of `supermodel-real3d-dump.patch` and
`supermodel-frame-grab.patch`. `M3TILE_DIR` saves the tile generator's VRAM (1.125 MiB,
bus byte order) and its 64 registers, gzip, on the dump frames; `M3NO2D` clears the
frame but draws no tile layer, so `M3SHOT3D_LIST` grabs show the 3D alone. Supermodel
runs are deterministic: a fresh run with an empty `NVRAM/` reproduced all 800 census
dumps byte for byte.

```
M3REPLAY=$PWD/dumps_vf3 M3CENSUS=$PWD/c M3TRIS=$PWD/rast/tris_vf3 \
M3FRAMES=$PWD/rast/mame_vf3 M3TILES=$PWD/rast/sm_vf3_full/tiles \
../mame289-rast/m3 vf3 -rompath /home/ben/roms/Model3 -video none -sound none \
    -nothrottle -numprocessors 1 -seconds_to_run 120 -skip_gameinfo
```

`make test_real3d_rast` and `make compare_real3d_sm` read them (`sim/real3d/m3-notes.md`).

### BTB designs, dual-issue variants and the branch trace (`supermodel-lv2.patch`)

`supermodel-lv2.patch` applies on top of `supermodel-real3d-dump.patch`. On the `lv_`
base (load use 3, redirect 2, new line 1, way miss 2) it adds, at 100 MHz:
- `lv2_<bt>_<d>_<f>`: `bt` is `r2` (no BTB), `b256` (as `lv_`), `bC` or `bD`, the two BTB
  designs of `rtl/cpu/ppc_btb.sv` (256 entries, tagged with MSR[IR, PR]; written for taken
  branches the static rule predicts taken and for taken bclr / bcctr; `bC` looked up a
  request ahead, a hit free, emptied on translation changes; `bD` looked up at the
  branch's own request, a hit 1 cycle; neither looks up a branch fetched right after a
  redirect). `d` is the pairing: `d0` single issue, `da` any two units, `di` a
  single-cycle integer op with another unit, `dn` the same writing no CR or XER, `dl`
  integer with load / store, `dc` the second a load, store or branch, `dm` the second a
  branch, `ds` a store or branch writing no GPR, `dq` a load, store or branch, `db` either
  op a branch, `dw` a single-cycle integer op with an op writing no GPR; `dm`, `ds`, `dq`
  and `dw` read at most three GPRs between them. `f` is the fetch model: `fi` bubbles at
  issue (as `lv_`), or `f1q4`, `f2q4`, a stream of 1 or 2 words a cycle into the core's
  4-entry queue, redirects counted from the redirecting instruction's arrival (fetch) or
  issue (EX).
- `lv2s_<bC|bD><128|64>_n1w2`: the smaller tables. `lv2x_*` (110 MHz): single issue with
  `n0w1` or `n1w2` beside each BTB.
- `lv3_` / `lv3x_`: `bCy`, `bDy`, the designs with the direction from the entry (every
  taken branch written; a hit predicts taken; a hit not taken invalidates).
- `_cpu.txt` gives each design's lookups, hits, right and wrong.

`M3BRTRACE=<file>` (with `M3BRT_FROM`, default 6000, and `M3BRT_N`, default 300 frames)
writes a 16-byte record per busy branch, translation change and refetching op (format in
`ppc603.c`, `trace()`); `_cpu.txt` then gives the `lv_` 256-entry BTB's hits over the
window from empty, which `make test_btb BTB_TRACE=... BTB_R52=...` must reproduce.

### The D-30 set as built (`supermodel-lv4.patch`)

`supermodel-lv4.patch` applies on top of `supermodel-lv2.patch`. It models the fetch of
`rtl/cpu/ppc_core.sv` per option, in the replay, so the BTB and the direction can depend
on timing:
- `lv4_<b0|c64>_<f1|f2>_<s|la|dy|lady>_<s4|l6>` (100 MHz; `_b0_f1_s_` and `_c64_f2_la_`
  also at 110): no BTB or design C with 64 entries and the y-bit rule (entries only for
  branches the rule predicts taken, bclr / bcctr included); one or two words a request
  (two only inside a line; the request after a redirect target is one word in `lv5_`);
  the static rule, the 603e's CR look-ahead (`la`: a branch whose CR and CTR writers have
  all committed when its word arrives is resolved there), R61's direction from the entry
  (`dy`), or both; singles at 4 (R61's base) or the built fast path at 6 with its
  in-order rule (`l6`). A line crossing pays 1 only after a jump to words 5-7 (one word a
  request) or 3-7 (two).
- `lv5_<b0|c|cx>_<f1|f2>[_la|_dy]_<s4|l6>`: as `lv4_`, two words only on sequential
  requests, and with `cx` the request after a fetch-side redirect (soft, BTB, a failed
  check) looked up from its target, as the RTL does (`ppc_btb` `CURPC`). In `lv5_` the
  `cx` test read the stream's break flag, which is set at every redirect target, so
  `cx` never applied and equals `c`.
- `lv6_<b0|cx>_<f1|f2>[_la|_dy]_<s4|l6>`: `lv5_`'s options with `cx` as built.
- `_cpu.txt` gives each option's lookups, hits, fetch corrections, register targets
  refetched, branches resolved at fetch, flushes and new-line cycles.

Run with `M3CPI_FPU_ONLY=lv4_,ref66_dual` (or `lv5_`, `lv6_`); `sm3-builds/lv2runs/lv4_table.py`
tabulates them as `lv2_table.py` does.

## Tile generator dumps (M0-E)

Two more instrumentation patches save the tile generator's state for the M0-E
bench (`sim/tilegen`). Both write, per chosen frame, `fNNNNNN.vram` (F1000000-F111FFFF,
0x120000 bytes in bus byte order) and `fNNNNNN.regs` (F1180000-F11800FF, bus order).

- `mame0289-tilegen-dump.patch`, on top of the census patch (`sm3-builds/mame289-tg`, a
  copy of `mame289-m3` so only `model3_v.cpp` rebuilds). The dump is taken inside
  `screen_update_model3`, from the state `draw_layer` drew, and adds `fNNNNNN.tile`: the
  below-3D and above-3D passes on cleared bitmaps (496 x 384 ARGB each, alpha 0 = not
  drawn), so the reference renderer is checked against MAME's own `draw_layer`. A Lua
  dump is not enough: `model3_vid_reg_r` does not return the scroll registers, a Lua write
  tap on the 64-bit space aborts on data with bit 63 set ("integer value will be
  misrepresented"), and a debugger watchpoint instead runs VF3 at under 1 frame a second.

```
M3TG_DIR=$PWD M3TG_START=600 M3TG_EVERY=60 M3TG_EXIT=9600 \
../../mame289-tg/m3 vf3 -rompath /home/ben/roms/Model3 -video none -sound none -nothrottle \
    -seconds_to_run 400 -skip_gameinfo -cfg_directory cfg -nvram_directory nvram \
    -autoboot_script ../../census/play.lua
```

- `supermodel-tilegen-dump.patch` (`sm3-builds/supermodel-tg`, a clean clone at `036af1c`):
  state at the start of the active display, `fNNNNNN.surf` (both of `CTileGen`'s draw
  surfaces, 512 x 384 RGBA, bottom then top) at its end, and `fNNNNNN.info` saying whether
  VRAM or the registers changed in between. Same `xvfb-run` command as above with
  `M3TG_DIR`, `M3TG_START`, `M3TG_EVERY`, `M3TG_EXIT`.

Dumps are ROM-derived and stay in `sm3-builds/tilegen/`.

## 53C810 capture (M0-F)

Two patches log what the games ask of the 53C810 (`sim/scripts`): every register access
(width, value, PC), PCI configuration access, start, executed SCRIPTS instruction, memory
move (source, destination, length) and interrupt change, one line each, prefixed by frame
and time. `sim/scripts/cap2vec.py` turns a log into replay vectors.

- `supermodel-scripts-log.patch` (`sm3-builds/supermodel-scripts`, a copy of
  `sm3-builds/supermodel` with its census patches, so only three files rebuild). Also saves
  the 1 KiB of code around the first access from each PC as `<log>.code_<addr>.bin`, for
  `bin/ppcd`. The files are CRLF, as upstream's are.

```
env -u WAYLAND_DISPLAY SDL_VIDEODRIVER=x11 M3SCRIPTS_LOG=$PWD/sm_vf3.log M3DUMP_EXIT=4000 \
xvfb-run -a -s "-screen 0 640x480x24 +extension GLX" \
    ../supermodel-scripts/bin/supermodel /home/ben/roms/Model3/vf3.zip \
    -no-throttle -no-vsync -no-sound -no-dsb -no-gpu-thread
```

- `mame0289-scripts-log.patch`, on top of the census patch (`sm3-builds/mame289-scripts`, a
  copy of `mame289-m3`). MAME logs byte accesses only; one time stamp is one CPU access.

```
M3SCRIPTS_LOG=$PWD/mame_vf3.log ../mame289-scripts/m3 vf3 -rompath /home/ben/roms/Model3 \
    -video none -sound none -nothrottle -seconds_to_run 70 -skip_gameinfo \
    -cfg_directory cfg -nvram_directory nvram
```

Logs and vectors hold RAM contents (the SCRIPTS words) and stay in `sm3-builds/scripts_cap/`.

## System bus trace (stage A, `sim/system`)

`supermodel-bus-trace.patch` applies on top of `supermodel-tilegen-dump.patch`
(`sm3-builds/supermodel-sys`, a clean clone at `036af1c`). `M3BUS_TRACE=<file>` writes
every CPU and 53C810 access outside work RAM, one 24-byte record each (frame, PC,
address, kind, size, master, data), and events for the IRQs, the ping-pong flip and the
frame start; CROM reads are left out unless `M3BUS_CROM=1`. The 53C810's moves run
inside the CPU's DCNTL write, so they are logged by master, not by nesting depth.
`sim/system/bustrace.py` summarises, dumps and compares traces; the system bench writes
the same format. The edited sources keep their CRLF line endings, or the patch becomes
a whole-file diff.

```
env -u WAYLAND_DISPLAY SDL_VIDEODRIVER=x11 \
M3BUS_TRACE=$PWD/../sys/sm_bus.bin M3TG_DIR=$PWD/../sys/sm_tg M3TG_EVERY=30 M3TG_EXIT=1500 \
xvfb-run -a -s "-screen 0 640x480x24 +extension GLX" \
    ./bin/supermodel /home/ben/roms/Model3/vf3.zip -no-throttle -no-vsync -no-sound -no-dsb -no-gpu-thread
```

Start from no `NVRAM/` folder: the trace is then a fresh boot, as the core's.

## Stage B oracle: Real3D memories and finished frames (`sim/system/notes.md`, Stage B)

`supermodel-stageb-dump.patch` applies on top of the bus trace and tile dump patches
(`sm3-builds/supermodel-sb`, a copy of `supermodel-sys`). At the bus-trace frames in
`M3SB_LIST` (comma-separated, the trace's own numbering) it writes `r3d_<frame>.bin`
into `M3SB_DIR`: 8C 4 MiB, 8E 1 MiB, polygon RAM 4 MiB and texture RAM 8 MiB, raw, as
that frame is drawn. It also writes `full_<frame>.bmp`, the finished frame with its 3D.
Run as the system trace above, with `-no-gpu-thread`, adding
`M3SB_DIR=... M3SB_LIST=...` and, for the tile state at the same frames,
`M3TG_START=249 M3TG_EVERY=25` (a tile dump is one frame behind: f001399 is trace
frame 1400). The dumps are ROM-derived and stay in `sm3-builds/sb_oracle/`.
