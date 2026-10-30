# Handoff

## Current state (2026-10-30)

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
  then `tools/cputest-compare.py capture.txt`. Built and timing-clean; **not yet run on
  the board.**

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
- M0-D (sound), M0-E (tile generator), M0-F (53C810 SCRIPTS).

## Findings this session, including the wrong turns

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
