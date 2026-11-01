# Handoff

## Current state (2026-11-01, night)

The study's §15 R83–R94 and D-37–D-42 carry every finding and decision; this is the summary.

- **VF3 runs on the MiSTer with 3D** (R94). The integrated build at 90 MHz boots, shows the warning and SEGA logo,
  then draws the attract fights textured over the tile layers: Aoi, Kage, Pai, Jacky, Lau and Lion on the beach
  stage. A few glitches are untriaged (flat-coloured suits, a full-screen blur, particle sprays).
- **Stage A** (R85, R92): the CPU, the bus (MPC105, system registers, IRQs), the 53C810, JTAG, I/O stubs, the tile
  generator, the MRA loader (`mra/`, board path `_Arcade/_Model3/`) and a hex debug overlay with CPI on the UART.
  The attract runs with tiles only at 100 MHz.
- **Stage B** (R89, R94): Real3D command and status, memory windows written to both copies with deferred runs,
  texture upload from a DDR3 ring, the compositor and a 7-port DDR3 arbiter. Exact against Supermodel on VF3's
  trace.
- **CPU fixes found on the board:**
  - misaligned word accesses (R85/R86, D-39);
  - integer load-with-update with rD = rB (R91).
- **CPU additions:** the D-cache snoop port (R90, not wired yet). PVR stays 0x00060101 until the bus snoops (D-41).
- **Rasteriser:** blocks 8–9 (R87), tuned (R88). VF3 p99 is 82% of a frame with nearest mip.
- **Builds:** multicorner timing (D-35); one seed for test fits.
- **Area:** 41,346 ALM (99%) without sound or I/O. The full release needs about 2,500+ ALM freed (R81, R93, R94).
- **Timing:** the integrated build closes only at 90 MHz; the CPU spike runs 99–108 MHz at 9.09 ns.

### Next
- **Frame rate: 14.5 fps on the board** (R95). Half the CPU's cycles wait on data memory: write-through RAM stores
  (32% of cycles, D-41) and uncached CROM reads (18%). Wire the snoop and move to PVR 0x00060103 first.
- **Limb trails** in attacks, and some polygons out of place (R95). Check against Supermodel's frames
  (`sm3-builds/sysb/runp/`, frames 1300 and 1390).
- Everything at 110 MHz if it closes, after a timing pass; otherwise only the CPU at 110.
- **Bass Fishing Deluxe** (`bassdx`, R96): shows a still 2D water screen and never any 3D (the
  Real3D counter says 359 draws begun), then freezes
  with the CPU looping in the boot ROM; the last exception was an alignment interrupt (0x600). Screen grab
  `sm3-builds/board/bassdx_frozen.png`; bench image `sm3-builds/bass_sdram.bin`. MRA in `mra/`, uncommitted.
- On the board: `_Arcade/_Model3/` has the 3D build and "stage A, 2D only" (`M3stageA.rbf`, 9ae84cd8), for
  recording progress.
- Triage the 3D glitches against Supermodel: run the bench chain to the fights (`sm3-builds/sysb/ext2.sh`).
- An area pass with targets per block: the viewer read adapters (~960), stage B, mip, CPU additions, M10K trades,
  and a smaller 68000 before sound.
- Wire the snoop (`m3_bus`), then PVR 0x00060103.
- Timing back to 100 MHz, then 110 (D-38). The `lmw` held base (R91).

## State on 2026-10-30

No RTL yet. The work so far is the design study
(`docs/model3-step10-design-study.md`) and the measurements behind it.

Decided: all in fabric (D-1), 3D layer in DDR3 (D-2), clocks 100/100/50 (D-3), ROMs
and board state in SDRAM (D-4), VF3 is the bring-up title (D-5), polygon RAM and the
texture FIFO in DDR3 with the 3D layer (D-6).

Gates done:

- **M0-G** — measured budget from the Model 2 fits (s1633 main, s1629 2C). Everything
  but the CPU is 29K–37K ALM; the CPU has about 3.5K–12K. Study 8.1, R10.
- **M0-A1** (VF3 only) — MMU on with software data-TLB reloads, real double-precision
  FP, code only from RAM, p99 1.07 M busy instructions a frame. Study section 9, R12.
- **M0-B** (VF3 only) — Real3D RAM footprint 3 MiB, geometry fits two multipliers,
  overdraw p99 8.3× needs two pixels a clock at 50 MHz. Study section 9, R13.

Tooling: `tools/mame-census/` (patch to MAME 0.289, play script, run instructions).
The MAME build lives in `sm3-builds/mame289-m3` and the run outputs in
`sm3-builds/census`, both git-ignored. ROMs: `/home/ben/roms/Model3/` (vf3 verified).

- **Texture census** (VF3 only) — sheets 7.5 of 8 MiB written; texture-cache fills
  94 MB/s p50, 200 MB/s p99 with a 64 KiB cache. More than one SDRAM carries. Study
  section 9, R16, open question 4.

- **Z pre-pass model** — texture fills with a 64 KiB cache drop to 59 MB/s p50,
  163 MB/s p99. Typical frames fit one 120 MHz SDRAM; heavy frames need textures on a
  second SDRAM (D-9). Study R17.

- **Other sets** — VF3 Team Battle (MAME) matches VF3. Bass Fishing Deluxe through
  Supermodel replay (attract): a third of VF3's texture traffic, 24 K polygon peak.
  Replay validated against MAME on VF3. Daytona 2 (Step 2.1) stuck before 3D in both
  emulators without input. Study R18.

Supermodel clone at `sm3-builds/supermodel` (036af1c) with the dump patch from
`tools/mame-census/`; run it under Xvfb with `SDL_VIDEODRIVER=x11` and
`WAYLAND_DISPLAY` unset, or OpenGL fails to start.

- **CPU at 60 Hz and Daytona 2** — Supermodel's interpreter now counts instructions
  per frame (`M3CPU`, `M3IDLE`). VF3 busy p50 567K, p99 1.15M per frame (R19). Daytona 2
  (link set to Single in Supermodel's test menu, saved in its NVRAM) busy p99 760K,
  3D inside a 50 MHz frame at p99, 168 MiB of non-sound ROM (R20, D-10).

- **CPI model** — in-order scalar 603e at 100 MHz: CPI 1.52 (VF3), 1.79 (Daytona 2), no
  frame over budget with 16 KiB 4-way caches, a store buffer and early restart. Study R22,
  D-15. SloPPC603 reviewed as a reference only (R21, D-13). One core for every step,
  driven by the game ID (D-14).

Decisions added: D-7 (SDRAM up to 120 MHz, own clock domain), D-8 (sound may be left
out of first builds), D-9 (Real3D memory placement as a compile-time parameter for a
second SDRAM).

## CPU (M0-A2), 2026-10-30

- `rtl/cpu/`: in-order integer core, checked against `sim/cpu/ppc_iss.h` (`make test_cpu`,
  20,000 seeds clean). Standalone fit 2,513 ALM, 77.2 MHz (R23). Next: timing pass to
  100 MHz (forwarding selects in ID, CR0 in MEM), then branch prediction, MMU, caches, FPU.
- CPU bring-up build (R24): `Model3.sv` + `rtl/dbg/m3_cputest.sv` run a generated program
  at 75 MHz and report cycles, stall counters and a retire signature on the debug UART.
  Build: `make cputest_hex test_cputest`, then `quartus_sh --flow compile Model3`
  (write `build_id.v` first if only running analysis). Board: `tools/cputest-capture.sh`,
  then `tools/cputest-compare.py capture.txt`. Run on the board since (R25, R26, R28,
  R32).

## What remains

- Bass Fishing gameplay (needs input injection in Supermodel) and the `getbass`
  I/O CPU path.
- VF3's heaviest stage (Sega Retro quotes 32 K polygons; the scripted play peaked at
  16 K).
- M0-C on the board: Model 3's DDR3 traffic (3D layer + polygon RAM, 72–100 MB/s) on
  the Model 1 DDR3 path; SDRAM arbiter budget.
- M0-A2: PowerPC 603e core spike to D-15, written from scratch with the 8.4 area
  techniques; SloPPC603 at `tools/sloppc603-ref` as a reference only.
- Daytona 2 in gameplay (input injection in Supermodel), and the Step 2.x deltas (D-10).
- Dual-SDRAM memory map once the modules arrive (D-11).
- M0-D (sound) into a top level. M0-E and M0-F are done standalone (R77, R80).
- Rasteriser blocks 8–9 finished; the viewer moved onto `r3d_rast`; the frame in real time.
- M1: the CPU, the bridge, board RAM, SCRIPTS and the tile generator in one top level.
- The tile stencil's bit order, from cabinet footage of VF3's portrait wipes.

## Findings this session, including the wrong turns

- 2026-11-01 (night): VF3 crashed twice on CPU gaps that the core benches never generated: a misaligned word load
  (R85), and lbzux with rD = rB (R91). Both were found from the board overlay plus the system bench; the
  decrementer interrupt seen before the second crash was a red herring.
- 2026-11-01: a build with only 1 ps of hold miss died at boot, while builds with -0.9 ns of setup slack ran.
  Hold is now checked on every fit.
- 2026-11-01: the 100 MHz integrated build (-2 ns) was dead on the board; at 90 MHz the 3D runs.
- 2026-11-01: the first DDR3 probe builds touched the port at PLL lock, before Main releases
  the FPGA ports. That wedged the port across core reloads until a power cycle. Four rounds of
  diagnosis (owner queue, terminator, late take) ran on a dead port and were wrong (R71, R72).
- 2026-11-01: every project timed the slow 100C corner only. D-32's geometry builds missed
  100 MHz at −40C unseen (R73, R79); fixed exactly in R76.
- 2026-11-01: R67's texture fills came from MAME's lists, which draw back-facing
  double-sided polygons at level 0. With the geometry's own LOD, fills are about a third
  (R75).
- 2026-11-01: the geometry dropped a model when a memory acknowledge came late. Every
  earlier bench acknowledged within 3 cycles (R73).
- 2026-11-01: the tile palette needs 52 M10K, not 32 (R77).
- 2026-11-01: VF3 never auto-starts SCRIPTS (R80).
- 2026-11-01: D-30's direction rule follows the 603e manual. R61's direction from the BTB
  entry is not the chip's behaviour (R78).
- Paper budget was 8K–23K ALM too pessimistic (R10).
- ROMs in DDR3 was proposed and rejected (R5, D-4).
- Real3D RAMs measured at 3 MiB of 7–10 MiB of windows; VF3 then fits SDRAM exactly.
- DDR3 was first ruled out for polygon RAM on a 100 MB/s budget (R14), then put back
  on the evidence of Model 1, which runs the same pattern at 60 Hz (R15).
- MAME renders VF3 at 30 Hz; the cabinet is 60 Hz. MAME's cadence is an artefact.
- The MAME census callback needs its opcode slot inside the PPC core state struct: the
  x64 recompiler can only address memory near its cache (`offset_from_rbp`).
- The texel-cache simulation needs `-numprocessors 1`, so the renderer runs in one
  thread and the cache sees texels in order.
- 2026-10-31: the standalone spike hides memory paths. Both the instruction side (R28)
  and the data side (R32) missed in the full build through the harness's M10K. The
  caches confirm it from the inside (tag M10K output about 2.3 ns late, R36).
- 2026-10-31: my first −200 ALM estimate for one subtract order measured −22 ALM
  and cost 1 ns (R33).
- 2026-10-31: I briefly believed MAME draws VF3 2× magnified. Supermodel frame grabs
  refuted it (R35).
- 2026-10-31: MAME's fnmsubs gives the wrong zero sign, and both emulators' double FMA
  family is unfused. The manual wins; expect 2–13% of double FMA results to differ from
  Supermodel (R30).
