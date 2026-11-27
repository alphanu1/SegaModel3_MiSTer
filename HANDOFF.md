# Handoff

## Update (2026-11-27, evening): where everything is

`main` takes branch `dirn`: the branch-direction table (D-90, R209) on, `M3_DIRN=1024` in the qsf. One M10K, +91
ALM; -3.36% of Daytona 2's race frame and -3.21% of Sega Rally 2's start line on the replay bench, results unchanged.
Board test dirn140_s3 (RBF 37541dd3, dual, 140 MHz, hold +0.066 ns, setup -4.03 / -3.89 ns): Daytona 2 about the
same as all1 and its course-guide attract right; Sega Rally 2 no start-line slowdown, one or two in the race, mostly
through water; Scud Race the same; VF3 good. Calibration SDRAM1 6 1 35 20, SDRAM2 7 1 36 22 (SDRAM2's window 13
steps wider than on c5_s2's boot). Not a release (D-87).

The study now carries R201 (the CPU kit), R203 (the calibration margin) and R206 (one calibration engine), which went
into `main` with all1 and calib-shared without their entries, each with its board result.

| Work | Where | State |
|---|---|---|
| Everything above | `main` | board-tested |
| CPU rebuilt for 140 (D-89, R208 draft) | sm3-builds/cpu-fable (NOTES §6-12, proto/ipipe6-9) | integer pipeline with way prediction (route B: 91-92% of loads hit the MRU way) fits 153-160 MHz alone, hold positive; the in-chip probe of v9 running; then the I-cache front end |
| Real3D area, second pass (R207 draft) | sm3-builds/area-r3d2, patches/area-r3d2-R4-dba80c1.patch | -270 to -400 chip ALM, 0 frame cycles, 0 pixels wrong; DSP 87 -> 106 of 112 pushes 11-12 blocks to the far columns and makes them the Real3D's worst path; packing lone small multipliers two to a block proposed, not built |
| CPU / Real3D area, first passes (R204, R205 drafts) | sm3-builds/area-cpu, area-r3d | zc2 and A2 on hold |
| 8-stage single FPU (R202 draft) | sm3-builds/sfpu | bench-proven; +3.75% / +3.44% cycles, needs rebasing |
| TLB misses made cheaper (D-88) | not started | after the CPU rebuild's current steps |
| M3_R3EMPTY, M3_R3PF | in `main`, switched off | need area |

Open from the board: Sega Rally 2's slowdowns going through water, Real3D or CPU, to be measured on replay frames
captured there.


## Update (2026-11-27): where everything is

`main` takes branch `all1` (one board test, all1_s2, RBF c26b7cad, dual, 140 MHz): the CPU kit (conversions on their
own port, the double unit's iterative front, the FP load word forwarded, fcmpu released early, bypassed doubles: about
-3.1% / -4.7% of the race / start-line frame on the replay bench), the SDRAM calibration with hostile traffic, four
sweeps and a 12-step early margin, and the contributor's changes behind switches (M3_DMADIR and M3_SND_WD13 on;
M3_R3EMPTY, M3_R3PF, M3_R3LATE off). Board: VF3 solid 60; Daytona 2's course-guide attract right and 70-80% of track 1
at 60; Sega Rally 2 no start-line slowdown, one slowdown about halfway; Scud Race mostly 60, the bridge start line 30.
Calibration on Auto: SDRAM1 +6 (width 36), SDRAM2 +12 (width 22). Setup -4.25 / -4.11 ns: not a release (D-87).

| Work | Where | State |
|---|---|---|
| Everything above | `main` | board-tested |
| Area pass: CPU / Real3D / system side (D-87) | sm3-builds/area-cpu, area-r3d, area-sys | in progress |
| 8-stage single FPU, no early forward (R202 draft) | sm3-builds/sfpu, patch against c5d4783 | bench-proven; +3.75% / +3.44% cycles, needs rebasing |
| M3_R3EMPTY, M3_R3PF | in `main`, switched off | need area |
| FP store slot (kit item 1e), load-use 3 | not built | |

Rule: board-proven work goes to `main` on its own branch with a `--no-ff` merge; anything waiting for the board sits on
a pushed branch off `main`; nothing that has passed stays only in sm3-builds.

## Update (2026-11-26): where everything is

`main` now carries the 140 MHz work that has run on my board: the system side with automatic SDRAM read calibration
(R198, D-83) and the Real3D rework (R197), merged from `work-140`. The tree builds at 140 MHz (PLL at 840 MHz). Its
RTL is the combo7 tree that ran VF3, Daytona 2 and Sega Rally 2 on Auto calibration (combo7b_s2; SDRAM1 +3 / 36,
SDRAM2 +2 / 35). Timing is not closed (about -4.3 ns, CPU first, then Real3D): no release until it is (D-81).

| Work | Where | State |
|---|---|---|
| 140 system side + calibration, Real3D rework | `main` | board-run at 140 |
| CPU conversion port + iterative double front + step-1 guard (df4, R200, D-86) | branch `cpu-kit` (on `main`) | bench-proven; board test at 130 pending (df4_s1) |
| CPU kit items 1b-1e | sm3-builds/kit (in progress) | |
| 140 single-precision FPU | sm3-builds/sfpu (in progress) | |
| The contributor's changes (D-84, D-85) | sm3-builds/picks (in progress) | |
| Findings only: 130 closure (R194), ISA census (R195), CPU at 140 (R196), fork review, independent review | study + sm3-builds/{tc130,isa,cpu140,fork-review,review140} | |

Rule from here: every agent's work is rebased onto `main` before it is tested on the board, and goes in on its own
branch with a `--no-ff` merge once the board passes it. Nothing lives only in sm3-builds once it has passed.

## Update (2026-11-25)

`main` takes branch `sr2-dec-readout`: the frame-time overlay (M3_FBUSY_BUILD only) shows the last value the CPU
wrote to DEC, row 6's third value, latched once a second (`cmdec_s1`, eeec1604; one flop in ppc_core, 33 in
m3_frame_busy). Release builds are unchanged.

Board (cmdec_s1, Sega Rally 2, a full lap): the game writes 0x00043711 every frame, the same as a clean boot on the
bench, so the board does not calibrate short (D-78 corrected). With the CPI items and PRX the start line holds
51-58 fps but for one second at 38; its long frames (up to 1.6 vblank outside the DMA calls) are unexplained, with no
Real3D drop or hold. The two race slowdowns (FPS 29-50) are the Real3D dropping 7-28 frames a second while every CPU
frame is under 0.9 vblank. Readings in `sm3-builds/c135/NOTES.md`.

Under way (D-80): closing the setup slack at 130 MHz in `sm3-builds/tc130/` (about -3.8 ns over some 15,500
endpoints), then 135 and 140 again. Model3.rbf on the board is cmdec_s1.

## Update (2026-11-20)

`main` takes branch `cpu-r3d-130` with a `--no-ff` merge: the RTL of `cxp130_s1` (ac2e1b55), board-tested.
- **R191:** r3d_mem's 8E copies by four-word group (v3) and polygon RAM writes gathered into DDR3 bursts; row 6 of the
  frame-time overlay now reads DD SS WWWW (frames dropped at a flip, swaps, longest swap wait) and the DMA total.
  v5 (posted store writes) stays out: with the bursts it still made five polygon-RAM-heavy frames slower (D-48).
- **R188 / R192:** the CPI items. **R193:** PRX (an RPX access pre-reads its set while its page is pending).
- Model3.qsf: UNFORCE_MERGE_PLL_OUTPUT_COUNTER on pll_0002, so the two SDRAM clocks stay separate counters.

Board (cxp130_s1, dual, 130 MHz, 3125 / 2885 ps, with M3_FBUSY_BUILD): VF3 full 60, no lockup; Scud Race good (its
usual short 30 at the start of track 1); Daytona 2's course-guide attract correct, and about 8-10% of track 1 now
holds 60. 41,128 ALM, setup -3.494 / -3.600, hold positive, HDMI -0.20. Model3.rbf on the board is cxp130_s1.

Findings (study D-76 to D-79, R191-R193):
- Daytona 2's race is the CPU's: real race frames (749 k instructions a loop) run 1.098-1.117 vblank on the bench,
  the board 1.01-1.10; the Real3D holds it only 0.005-0.013. Late loops are slow motion (4C76C counts overruns, no
  catch-up). Nothing affordable reaches -10%: dual dispatch could take at most 2.8-4.9% for 1,500+ ALM.
- Sega Rally 2's start line is not speed. The game measures vblank-to-flip once at boot (6AC20-6AD00) and uses it as
  the decrementer reload (4C21A8); a short value gives exactly the board's 2.00-vblank frames. A clean boot on the
  bench measures 0x43711 (right). Next: show the live mtdec value on the overlay (build `cmdec_s1` under way).
- 135 MHz is dropped (D-79): at 135 the first SDRAM reads only at 2778 ps, and there Daytona 2's course-guide attract
  goes wrong (c135p, c135z). Builds and notes in `sm3-builds/c135/`. The course guide is the check for wrong results
  at any new clock.
- No second geometry engine (D-77).

Still open: the board's short SR2 calibration, Daytona 2's last few percent, Real3D speed in SR2's heavy views, timing
slack at 130, the old single SDRAM module (D-73). No releases until the speed is right.

## Update (2026-11-19)

`main` takes branch `sr2-r3dmem` with a `--no-ff` merge: D-74's three changes and the frame-time measurement.
- **R189, r3d_mem:** a re-opened 8E group joins the open run, the age flush waits for a free clock (it also closes a
  word dropped from both copies), and the deferred copy pauses for waiting writes. +9 ALM.
- **b6 (R188, R179):** the reciprocal seed tables in M10K. About -100 ALM, +2 M10K (536 / 553).
- **R190, area:** the second SDRAM told it carries only work RAM, the edge walker and FP divider remainder at 54 bits,
  the DDR3 arbiter's one-hot select. -131 ALM by entity, no M10K.
- **m3_frame_busy** with Daytona 2's decrementer-paced frame and the DMA-wait split, and the qsf switch
  `M3_FBUSY_BUILD` (commented out) for a release build with overlay rows 6-7.

Board: `cd74_s1` (743bfb4c) is Model3.rbf: this tree with M3_FBUSY_BUILD, dual, 130 MHz, 3125 / 2885 ps. 41,076 ALM,
setup -3.828 / -3.803, hold positive, HDMI +0.09 / +0.17. VF3, Scud Race, Daytona 2 and Sega Rally 2 run with no
lockup. No release RBF of this tree is fitted yet (u_sdr2's refresh path moved 0.3-0.6 ns in R190's fit: two seeds).

Sega Rally 2 over a lap (`sm3-builds/d74/NOTES.md`): DMA wait 0.00-0.22 vblank (was 0.61-0.81), most of the lap
57-58 fps. Left: the start line (frames pinned at 2.00, the host held outside the DMA calls) and two stretches
(the game loop at 60, the Real3D dropping frames, FPS 19-43). Daytona 2 unchanged: CPU-bound at 1.01-1.10 vblank.

Next (D-75): a fair boost to the Real3D, for Daytona 2 as well as SR2. First split where SR2's and Daytona 2's heavy
frames go (geometry, rasteriser, texture path, held host access), then price the speed-ups. The CPI items (R188)
stay in `sm3-builds/cpuspeed/` until that split says how much of Daytona 2's frame is the CPU.

Before the 2026-11-19 power outage the session's scratch files went to `sm3-builds/session-save-2026-11-19/`; the
board script is now `sm3-builds/board-tools/b.sh`.

## Update (2026-11-17)

`main` is the `--no-ff` merge of `sdram-130` (ae56d77): 130 MHz on one or two SDRAM modules. `git revert -m 1` on the
merge undoes it. Nothing from the CPU speed pass is committed; it is all in `sm3-builds/cpuspeed/`.

**Model3.rbf on the board is `cfb3_s1`** (15fab5a4), a measurement build: dual, 130 MHz, 3125 / 2885 ps, without the
CPI items. Row 7 of the overlay times each frame against the vblank (one vblank = 227B units of 256 clocks) and
splits the longest into Real3D DMA wait and the rest; row 6 has the DMA wait's range and total. VF3's frame count is
wrong in it (stale 4.85 vblank); its pacing needs its own rule. `c130ddpa_s1` is the board-tested release build.

Board readings (readings by the second in `sm3-builds/cpuspeed/NOTES.md`):

| Game, scene | FPS | Frame (vblank) | DMA wait | Rest |
|---|---|---|---|---|
| Daytona 2, track 1 | 27-31 | 1.01-1.10, peaks 1.24-1.31 | 0.03-0.04 | 0.99-1.16 |
| Sega Rally 2, racing | 57-58 | 0.6-0.9 | 0.08-0.35 | 0.57-0.70 |
| Sega Rally 2, start line | 28-29 | 1.29-1.79 | 0.61-0.81 | 0.70-0.82 |
| Sega Rally 2, slowdowns | 28-29 | 1.1-1.41 | 0.60-0.73 | 0.63-0.67 |
| Sega Rally 2, single 2.0 frames | | 2.00-2.06 | 0.31-0.41 | 1.63-1.69 |

- **Daytona 2's race is CPU-bound, just over a vblank.** The loop is paced by the decrementer, not the vblank (R159,
  R160), and a frame runs from the decrementer event that starts the loop to the pacing call at 4C6FC. Holding 60
  through the lap needs about 10% off the CPU's frame, about 25% at the worst spots. "Rest" still includes cache
  misses, SDRAM waits and held Real3D accesses; splitting it is the next measurement.
- **Sega Rally 2 is not CPU-bound** (my 2026-11-16 note above was wrong for it). Its CPU work stays under 0.82 vblank;
  every slow frame is over because of waits in its Real3D DMA calls (6F468, 6F50C, polling C200000C bit 7), or, in
  the single 2.0 frames, a hold outside them (likely a Real3D store held by r3d_cmd's sw_wait or full write FIFO).
  Supermodel's DMA is instant, so this is our RTL. The bench investigation is in `sm3-builds/sr2dma/`.
- **CPI items 1-2 plus b6** (`csb_s1`, d05c0942, 41,097 ALM): test_cpu 2,075,079 cycles against 2,119,480; Daytona 2's
  attract loop 3.2-3.7% shorter (p50 0.850 to 0.820 vblank). On the board Daytona 2 still runs at 30, as the
  readings above predict. The R188 draft is in `sm3-builds/cpuspeed/R-entry.md`.
- The first measurement build counted no Daytona 2 race frames: it ended frames at the boot's vblank poll at 4C6CC,
  which the race never runs. Its bench check had covered boot frames only.
- The second SDRAM module gives Daytona 2 almost nothing (attract loop 0.868 vblank at single 120 scaled to 130,
  against 0.850-0.884 dual).

Still open: the old single module (D-73), a power-up read-phase sweep (no decision), the area pass, and the Daytona 2
frame split.

## Update (2026-11-16)

One squashed commit on branch `sdram-130` (ae56d77, merged into `main` with `--no-ff` on 2026-11-17) takes everything since 718e9e4 that the board build
has. That is `sm3-builds/ddio/ddio-718e9e4.patch` (md5 99698145…) at 130 MHz with 3125 / 2885 ps, plus the study's
D-59 to D-73 and R169 to R187 (there is no R181).

**130 MHz is back on the new SDRAM modules** (R187). The SDRAM clock pins come from DDIO cells, each clocked by its own PLL output. The SDRAM interfaces are
reported in every fit (`sdram_io.txt`). SU_PAIR and fix B are in.
- **Model3.rbf on the board: `c130ddpa_s1`** (2629cdbc), dual, 130 MHz, outclk_2 3125 / outclk_3 2885 ps. No lockup in
  VF3 (20 minutes of attract plus an in-game round), Scud Race or Daytona 2. Frame rate: Sega Rally 2 is mostly 30 in
  game and 60 in attract, Scud Race has small drops to 30, Daytona 2 is 60 in attract and mostly 30 on track 1. Those
  are CPU-bound; the next speed step is the CPU (the cpi3 items need about 110 ALM freed). Wrong for Sega Rally 2:
  see 2026-11-17.
- `c130ddpp_s1` (3365 / 3125) locked up in VF3. It is the same fit as (a), so the lockup was the SDRAM read timing,
  not a clk_sys path. `c125ddpp_s1` (3500 / 3250 at 125) passed. At 125, 4000 ps does not work on the dual board
  (D-72). The plain-DDIO builds put the second SDRAM about there and were not tried.
- `c130ddpb_s1` (280a590d, 3365 / 2885, only SDRAM2 moved) also runs VF3, so the 130 MHz failure was the second
  SDRAM's read. (a)'s 3125 ps stays the first SDRAM's default: its eye centre is nearer c125ddpp's. Model3.rbf on the
  board is (a) again.
- **Deferred (D-73): the old single module.** I am not stripping the MiSTer down to swap it back yet. The build
  for it is `c130sd_s1` (ade245af, single, 130, 3125 ps). If VF3 locks there, the power-up read-phase sweep is next.
- The plain-DDIO builds (`c125dd_s1`, `c130dd_s2`, `c125sd_s1`, `c125dd35_s1`) are reference only.
- Under way: the CPU speed pass in `sm3-builds/cpuspeed/`. It measures Daytona 2's race frames against the vblank, then
  cpi3's items with the fetch fix and a zero-cycle area pass to pay for them.

## Update (2026-11-14), not committed

Nothing new is committed or pushed since 718e9e4. The study's working tree carries D-59 to D-69. The R-entries
below were drafts in `sm3-builds/` then; they are in the study since 2026-11-16.

**The board** (MiSTer Pi R2, now 192.168.1.128):
- Two new 128 MB SDRAM modules are fitted. The second sits where the analog board was; the analog board is off.
- SW3 is ON and SW1 is OFF (SW1 had been routing HDMI audio to the user port).
- MemTest256, built from source in `sm3-builds/memtest256`, reads 256 MB = 128 + 128. Slot 1 passes at 153 MHz, slot 2 at 157 MHz.
- arcsnd is not in `user-startup.sh` and has to be started by hand after a boot.
- `Model3.rbf` is `d130_s2_3728ac8d`.

**130 MHz broke with the new modules, and the second SDRAM floating was the main cause** (`sm3-builds/cl3/NOTES.md`):
- so130 (CL2) froze when the 3D started. b10 at 120 MHz worked.
- CL3 (`cl3_130_s2`) changed nothing. That was a wrong turn: the SDRAM reads fine at CL2.
- `d130` (so130 built as a dual-SDRAM core with the module held deselected) runs Scud, Daytona 2, Bass Fishing,
  Spikeout, Lost World and Sega Rally 2 at 130. VF3 still freezes as its 3D starts; the cause is not found.
- c125 (125 MHz everywhere) works.
- A single build on a board that has the second SDRAM is therefore unsafe at 130 (D-67 amended).

**Decisions D-62 to D-69, in the study table:**
- D-62: sound into the FPGA if it fits, plus the audio-filter patch.
- D-63: built for the second SDRAM.
- D-64: a Step 1.x / 2.x split, Model3s1x and Model3s2x. It waits on the sound board.
- D-65: the wide micro-ROM for m3_s68k.
- D-66: the CPU's work RAM goes on the second SDRAM.
- D-67: single builds use arcsnd, dual builds the FPGA, one MRA per game. Amended.
- D-68: until the sound board fits, dual builds keep arcsnd and stay one core.
- D-69: the target is a single core and a dual core at CPU 125 / rest 130, from a second PLL, with the RPE bug fixed first.

**Drafts, all to revisit:**
- R174, `cpi3`: CPI items 2 and 4 are built. Daytona 2 is −3.1% busy; it does not fit.
  The VF3 2-word DMA difference is the game's own stfs/lfs round trip, not a core bug.
- R175, `tclose3`: timing round 2, b10. Mean TNS is −11%; b10 is on the board.
- R176, `snd`: FPGA sound priced; the opcode census; the filter patch is −470 ALM in its block.
- R177, `s68k`: m3_s68k at 1,153 ALM, lock-step with fx68k on all ten programs.
- R178, `snd2`: the FPGA sound board built and benched. It is about +5,750 ALM net and does not fit.
- R179, `area3`: b4 is about −790 ALM needed. It also found the RPE bug.
- R180, `split`: the Step 1.x / 2.x measurement, still running.
- R182, `csplit2`: the split on b10. `cs2f_120_s1_c62b7c40` is CPU 120 / rest 130, dual, with the filter patch. It is not yet on the board.

**Open:**
- The RPE bug (`test_cpu --mis` seed 9). The first fix job was stopped by a safety filter before it finished.
- VF3 at 130.
- The FPGA sound board's room, about 2,500-3,500 ALM short per Step core.
- The one-sample SCSP engine mismatch at extreme latency.

## Update (2026-11-12)

`main` is e8a06ac, pushed: a `--no-ff` merge of `spikeout` (87e7f16), on top of 18026dd (README and LICENSE).

**Done:**
- **Spikeout Final Edition** (study R170; notes and tools in `sm3-builds/spikeout/`). Its banked CROM is 112 MiB, so
  the set needs 135 MiB of the 128 MiB SDRAM: CROM3 (banks 12-13, 16 MiB) now lives in DDR3 at 0x3E000000, read by the
  new `m3_xcrom` on the loader's DDR3 port (D-42 amended). The 315-5881 key comes from configuration bytes 22-25
  (Daytona 2's 29250E16 when a set gives none); Spikeout's is 29236FC8. The compressed mode is still not needed (0
  compressed words in 8,000 frames). arcsnd needed no change. Board, 130 MHz build `so130_s2` (RBF 6bbed34b, hold
  positive at all corners): boots to the attract at 58 fps. `main` takes it at 120 MHz; no 120 MHz fit of it yet.
  - Open: a solid black floor-shaped quad under the fighter in the attract's tatami-room scene, probably the shadow
    drawn opaque; not yet compared with Supermodel on that scene. Play, controls and sound not yet reported.
  - `m3_xcrom`'s select path is -2.37 ns at 130 MHz (inside the CPU's slack); register it when closure reaches it.
  - Wrong turns: a split across the picture in the first photo was the capture card (60 fps), not the core; the
    `tb_m3_sec` bench took the Real3D DMA's CROM reads as security-board accesses (22,932 false differ), fixed.
- **Sega Rally 2's board test** at 130 MHz passed in full, and the arcsnd fork's `model3` branch is one squashed
  commit, 85ce77d, with `releases/arcsnd_20261112.zip` (binary 748320bb); tested on the board from the zip. The
  release's version stamp (007847a) names the pre-squash commit; the source is identical.
- **Clock-split test** (draft R169 in `sm3-builds/csplit/`, not in the study; RBFs `cs110_s2_14b58fc0`,
  `cs120_s2_55bbd389`): the CPU at 110 or 120 MHz with the rest at 130 did not give VF3's attract a full 60; the full
  130 build does. So the CPU's own speed matters, which led to D-59.
- **D-59 and D-60** are in the study's working tree, **not committed**: D-59, close timing at 130 or better the CPU's
  CPI, and the CPI is to be bettered regardless; D-60, start with CPI items 1 and 2, the area from the core's own
  modules at zero cycle cost (D-57 stands). They go in with R169 and R171.
- **CPI study** (draft R171, `sm3-builds/cpi/NOTES.md`): CPI 2.92 on Daytona 2's attract, 2.69 on VF3. 60 fps at 120
  MHz needs about 8% (race median) to 14% (race p90, city/replay shots) fewer CPU cycles; VF3 has a few percent margin.
  Ranked plan: store misses allocating without a fill (VF3 -5.3% est.), MMU replay without refetch (Daytona 2 -2.5%),
  dynamic branch direction (reopens D-30), one-cycle stfs/fcmpu. A second posted line write in the SDRAM port gained
  nothing and corrupted data (`tree_pw2`, not kept): the data wait is the port busy, not a shallow buffer.

**Running:** CPI items 1 and 2 (`sm3-builds/cpi2/`): build, benches, the area account, one 120 MHz fit, draft R172.

**Board** (192.168.1.105 on 2026-11-12; DHCP moved it): `Model3.rbf` is Spikeout's `so130_s2` (6bbed34b), which runs
every game at 130 MHz. The full 130 build adbc4c0e is at `/media/fat/model3_backup/` and
`_Arcade/cores/Model3.rbf.f130sr2`. arcsnd release 20261112 runs by hand.

**Still queued:** the black quad in Spikeout, timing closure, Daytona 2's race, the Bass Fishing hiss, the 93 ms
watchdog; Lost World's guns from a Sinden, a mouse and a stick (D-61); edge smoothing as a post-pass on the finished
picture (FXAA-like: 3-4 line buffers, about 4-5 M10K and a few hundred ALM; no room yet). Done since: the arcsnd PR is
open (ppriest/MiSTer_ArcadeSoundServer#2, one commit 6c551fd, with the interrupt move for the Sinden driver's USB load).

## Paused work and queue (2026-11-10, night)

Everything is paused; Sega Rally 2 is done (below). Each workstream's notes live in its `sm3-builds/` folder (git-ignored); the
study's R-entries carry the findings. To pick a stream up, read its folder's notes first.

**Done 2026-11-11: Sega Rally 2** (study R168; notes in `sm3-builds/srally2/`). It halted at boot (black screen) on
the PVR: it needs version 7, so Step 2.x now reads 0x00071202 and Step 1.x keeps 0x00060103 (D-41 amended). On the
board at 130 MHz (`sm3-builds/sr2_130p_s2_adbc4c0e.rbf`, hold positive at all corners) with arcsnd ad1a76a0: boots,
attract, race, music, controls and the FPS counter's 1 s default all fine. `main` (merge 4834f66 of `srally2`,
c684fb5) takes the change at **120 MHz**; the 130 MHz PLL/SYS_HZ hunks stay out (−3.7 ns setup, a test clock). No
120 MHz build with the PVR change has been fitted yet. The fork's `model3` has 479da3e (README: the DSP in the FPGA
and what a core needs) and 625699c (all 63 Games.xml sets claimed; DSB2's wait loops found by pattern, Sega Rally 2's
DSB2 time 1.40 s -> 0.33 s for a 121.7 s race, output unchanged).
- Wrong turns: my first board build (`sr2_130_s2`, 26e6cafb) lacked the PVR fix and gave the black screen; I first
  suspected the MRA and the banking, but the bench showed the LED-code halt at frame 75. The C1 53C810 probe and the
  93C46 DO difference (R168 §3, §3a) show nothing on the board.

**Paused, in order of priority:**
1. **Timing closure** (the top priority; ahead of speed and area). Census done: `sm3-builds/tclose/CENSUS.md` and
   `tclose/cen/`. 110 MHz at every corner looks reachable; 120 needs LAB headroom and the clock split. Two fix streams
   were started and stopped early:
   - non-CPU (`sm3-builds/tc2`): the sound DSP's ring addresses pipelined (free: about 35% spare clocks a sample),
     MAX_FANOUT on sys_top's `gp_outr[17..20]`/`rack` in the qsf (no sys/ edit), `tq_full` registered, the colour
     operand fed `texel ^ x` and the texel differences registered, the steering and FPS overlay outputs registered;
   - CPU C1/C4 (`sm3-builds/tc3`): `fpx_cond` and the other `ser_wait` sources from next values, the BAT/TLB enables a
     cycle ahead, the fetch-queue compares from the one-hot `o`; cycle-identical as R140.
   Next after those: CPU C2/C3/C5 (I-cache `rd_a`, `da_av`, D-cache tag compare), geometry G2/G3 (`rst_geo` fan-out,
   `xw_data` select), then the DSP anchors (f4b) for a few tenths.
2. **Clock-split test** (branch `clksplit`, created from 8ea120d, empty): the CPU on its own clock at 110 MHz, the rest
   at 120 and at 130 (D-52; CDC at the CPU's m_* port, snoop, ext_int, reset; outclk_3; set_clock_groups in our sdc).
   At 130 MHz everything runs faster together, so the board result (R167: Daytona 2's attract and VF3 at 60) cannot
   say whether the CPU or the Real3D/SDRAM made the difference. These builds would.
3. **Daytona 2's race** stays at 30 even at 130 MHz (attract is at 60). Map it like R160 (`sm3-builds/d2race`, just
   started): CPU, geometry and rasteriser per frame at 120 and 130, the game's pacing in a race, Supermodel's race
   frame rate. Then the same for a cloth-heavy VF3 fight, to see which block's speed gave VF3 its 60.
4. **CPU re-pipelining study** (desk only): a 603e pipeline laid out for about 150 MHz on its own, with forwarding and
   prediction redesigned so cycles stay as they are (D-57). Other cores close CPUs at 145-150 MHz, including Namco
   System 22/23 at 99.9% full; ours was built for cycle count first and retrofitted for timing.
5. **CPU memory:** reads going ahead of the CPU's posted writes (about −1.6% of Daytona 2's loop, bench first), and the
   second SDRAM as the CPU's own memory when the modules arrive (−4 to −6% est.; it costs area: a second controller).
6. **Sound:**
   - Bass Fishing hiss after a direct game change: arcsnd fix in `sm3-builds/sound/bass` (md5 40d24879, stray session
     removed, session number in the tag), not yet board-tested or committed; the likely root cause is the sound DSP's
     failing timing at 120, which item 1's DSP pipelining addresses. The bridge side (+12 ALM) needs a decision.
   - The bridge's 93 ms watchdog reboots the sound board if a live server stalls that long: raise to 1-2 s.
   - The arcsnd PR to ppriest: draft `sm3-builds/arcsnd_pr_draft.md`; the README additions are now committed on the fork
     (479da3e), and the PR should also say every set is claimed (625699c). Ben approves the PR text before it is opened.
7. **Area:** the device is full (4,191 / 4,191 LABs, about 620 ALM free). The only large item left is the CPU's MLAB
   files into M10K (about 380 ALM, 7 M10K), which needs a speed proof first.

## Update (2026-11-10)

`main` is ad35700, pushed: a `--no-ff` merge of `area`, which carries `timing-120` (`git revert -m 1 ad35700` undoes
the batch). The tree is board build `next2_s3`'s RTL (RBF 4c02ce46), on the board now: **120 MHz** (PLL and SYS_HZ),
41,286 ALM (98.5%), 4,191 / 4,191 LABs, 528 M10K, 90 DSP; setup −3.08 / −3.23 ns at slow 100 / −40 °C, hold positive
at every corner (least +0.016 ns), HDMI −0.31 ns. The study's R138-R166 and D-50 to D-58 carry the detail.

- **In since 2026-11-08:** timing rounds (R138-R140, R146-R148: geometry MAC stage); area passes (R151, R153, R155,
  R157, R163, R164: storage into M10K, the MPC106 file in M10K with a reset sweep, the interconnect, the FPU);
  `MISTER_DUAL_SDRAM` off while the analog board is fitted (R150); player controls with Template e901a9f and Model 2's
  steering options (R154, R166); Scud Race's HUD (the tile generator's bus timing, R152); the Real3D DMA bursts and
  the pair queue (R144, R149, R156, R165); sound (below); the FPS counter averaged over 5 s.
- **Sound (D-51, D-53, D-55; R142, R145, R159, R161):** the 68000, the SCSP slots and the Digital Sound Board run on
  the HPS in arcsnd (my fork alphanu1/MiSTer_ArcadeSoundServer, branch `model3`, 7ccac34; PR to ppriest not opened,
  draft in `sm3-builds/arcsnd_pr_draft.md`). The SCSP DSPs and the bridge are in the core (`rtl/sound`). Output +12 dB
  fixed, OSD Volume on top. On the board with arcsnd 35c3d966 (run by hand from `/media/fat/arcsnd_test/`): VF3 seven
  stages clean at 57-58 fps, Scud Race and Daytona 2 (DSB music) clean, Lost World's quiet stretches are its own attract
  (Supermodel agrees, R166 notes).
- **Decisions:** D-57 no area change may cost the CPU a cycle; D-58 no behaviour removed because the games in hand do
  not use it; D-54 no `sys/` edits, FPS counter kept, no per-Step cores.
- **Open:**
  - **Timing closure is the next priority** (ahead of speed and area): a census of `next/tree` at 110 and 120 MHz is
    running in `sm3-builds/tclose`. The sound DSP (about 340-450 failing endpoints) can take pipeline registers at no
    game cost; the rasteriser front's next family is `tq_full`.
  - A 130 MHz test build is fitting (`sm3-builds/f130`), to see what the CPU's speed does to Daytona 2's frame rate.
  - Bass Fishing hisses after a direct game change (not through the menu). A stray session of the old set is fixed in
    arcsnd (`sm3-builds/sound/bass`, md5 40d24879, not yet tested, not committed); the likely cause is the sound DSP's
    failing timing at 120 MHz.
  - The bridge's 93 ms watchdog reboots the sound board if a live server stalls that long: raise it to 1-2 s.
  - Scud Race's polygons broke under the first server's word writes; not reproduced on any bench (R162). The board test
    that would separate VROM damage at load from DDR3 starvation is in R162.
  - Daytona 2's 30 fps stretches are the CPU's loop just over a vblank (R160); the second SDRAM as the CPU's own memory
    is the strongest lever. The device is full: the CPU's MLAB files into M10K (about 380 ALM) are the only large area
    left, and need a speed proof.
  - Daytona 2's car shadow (R137) and Lost World's fog (R134) as before.

## Update (2026-11-08, later)

The study's R134-R137 and D-49 carry this. The tree is build `c7_s1`'s RTL (RBF be782d4a): 41,091 ALM (98%), 347
M10K, 88 DSP; setup −2.59 / −2.55 ns, HDMI met, hold least +0.084 ns.
Committed and pushed as 71e669c (2026-11-08). Timing closure is under way in `sm3-builds/tc` (CPU paths in
`tc/cpu`, everything else in `tc/r3d`; the census of all 3,267 failing endpoints in `tc/cen_100.txt`, `cen_n40.txt`).

- **Lost World's foliage (R134):** an L8 texel of 0xFF is transparent, as Supermodel. The green boxes are gone.
- **Texture decode follows Supermodel (D-49, R135):** formats 1-6 under contour, texture inversion, and the glass
  (texture-alpha kind) blended by the polygon's translucency. Daytona 2's windows are right on the board.
- **The polygon store (R136):** the old 1 MiB list regions dropped the newest records, which were the player's car
  in Daytona 2 and Scud Race. A pass's two lists now share an 8 MiB region, run lists 8,190 a bin above VROM, and an
  overflow counter on overlay row 2. About 3× the heaviest of 5,143 sampled attract frames; there is no hard bound.
- **Board (`c7_s1`):** Daytona 2 and Scud Race right; windows right.
- **Not taken:** `TAOP`, Supermodel's opaque pass for texture alpha (R137): closer pictures, but Scud Race's worst
  frame from 91% to 108%.
- **Open:**
  - Daytona 2's car shadow pokes out past the front-left fender; on the real cabinet it lies under the car.
    Supermodel is wrong too (R137). Being worked in `sm3-builds/shd`.
  - Lost World: leaves with too little fog in the T-rex scene (R134), board only.
  - Timing: setup −2.59 ns at 110 MHz. A release needs every corner closed.

## Update (2026-11-08)

The study's R128-R133 and D-48 carry this round. The tree is build `c3_s1`'s RTL (RBF 9919dc9a), apart from
comments renumbered to R121 / R124 / R133.

- **D-48:** no area change may cost a game any cycles, and none that could make one slower. Room for sound has to
  come from cycle-free changes.
- **What `c3_s1` adds to `vr3`:**
  - the rasteriser's overlapped front, the filter's duplicate arithmetic, one MPC106 register read and the dead SDRAM
    VROM reader removed (R131, with `PB` = 2 under D-48);
  - the CPU's TLB ways in M10K and the FP double crossing a page from a BAT block split, as 4.5.6.1.1 has it (R129);
  - Lost World: the serial status 0x00, the geometry hold fix and its MRA (R132);
  - Lost World's I-cache: the CPU board's DIP bank, per game, on the OSD's DIP Switches page (R133).
- **`c3_s1`'s fit:** 40,826 ALM (97%), 347 M10K, 87 DSP; setup −1.85 / −1.61 ns, HDMI −0.14 / −0.28 ns, hold least
  +0.059 ns.
- **Board (`c1_s1`, the same without R133; then `c3_s1`):**
  - VF3, Scud Race, Daytona 2: pictures clean, frame rates unchanged.
  - Bass Fishing: a solid 58.
  - Lost World: the intro draws its 3D and attract holds 58. SW1 On from the OSD takes the 3D away again, so the
    switch reaches the game.
- **Area left:** about 1,100 ALM free against 3,890-4,525 for sound and I/O. Cycle-free candidates not yet built:
  the two-pass sound-chip ALU (about −900), the system selectors (−200 to −400), the Step 1.x / 2.x RBF split (−724).
- **Not merged:** R130's geometry rank 1 (a standalone Fmax drop) and R129's iterative double front (under 0.1% of a
  vblank), both out under D-48. Their work is in `sm3-builds/ar_geo` and `ar_cpu`.
- **Open:**
  - Lost World: some foliage cards in attract show a green box where they should be cut out (one texture or
    polygon type; other foliage in the same frame is right). Being worked in `sm3-builds/lwtx`.
  - Bass Fishing's water shadows and underwater tint; Scud Race's luminous polygons; the lemans24 download and the
    missing parent sets.
- **Tests on the merged tree:** `lint_sys` the same nine warnings; `test_loader_sw` (new, 13 checks) PASS.

## Update (2026-11-07, later)

- **The page-crossing fix is merged (R124, D-39 amended).** The tree is `xp1_s2`'s RTL (RBF 8851ce80).
  - Scud Race no longer freezes.
  - Bass Fishing boots and runs its attract at 30-60 fps. Its open faults: the water shadows, and some underwater
    objects not tinted blue.
  - VF3 and Daytona 2 show no regression.
- **`du1` is merged (R126):** the data micro-TLB's LRU replacement, Daytona 2 work CPI −1.4%. The tree is `du1_s1`'s RTL
  (RBF 9385d044), and it works on the board.
- **Area (R125):** about 700 ALMs left to recover. Sound and I/O need an area pass first.
- **The VROM overlap is merged (R127).** The tree is `vr3_s1`'s RTL (RBF 0fa26a16) apart from comments renumbered
  R124 → R127. VROM sits in DDR3 in pair order (`VROM_LIN`), with up to four geometry requests in flight (`PIPE`).
  It also fixes Daytona 2's texture-port uploads, which read half their words from the wrong VROM block.
  - Scud Race's long view mostly holds 58, dipping to 30 for about a second.
  - Bass Fishing mostly holds 58.
  - VF3 shows no regression. Daytona 2's frame rate is unchanged (the rasteriser limits it), and no texture change
    is visible by eye.
  - `test_xmem` (12 runs) passes on the merged tree; `lint_sys` shows the same nine warnings as before.
- **Area:** the census (`sm3-builds/area127`, draft to become R128) is not merged. A deeper pass is next.
- **Still running:** Lost World (`sm3-builds/lw15`).

## Current state (2026-11-07)

The study's §15 R119–R123 and D-47 carry this round's findings. The tree is build `r7j1`'s RTL (RBF 6c89cad1),
byte for byte, apart from the `M3_RELEASE` line that `Model3.qsf` keeps commented out for test builds.

- **What `r7j1` is:**
  - `r7f1`'s CPU;
  - Step 1.5 for Scud Race (R119);
  - the fetch-side LR/CTR look-ahead and a 4-entry instruction micro-TLB (R121);
  - a 32 KB I-cache, `ISB` on in `Model3.sv` (D-47);
  - the scan-out buffer fix (R123).
- **`r7j1`'s fit:** 110 MHz, release, setup −2.72 / −2.77 ns, hold least +0.100 ns, 41,161 ALM, 336 M10K.
- **Board, `r7j1`:**
  - VF3's tearing is gone.
  - Scud Race and Daytona 2 run at their earlier frame rates.
  - Every game's 3D is one frame sooner than before (R123).
- **Board, `r7i1`:** Daytona 2 holds 58 through one attract scene and 29 elsewhere. VF3 is about as before.
- **Scud Race:** attract runs at 57-58 with slow long views. It freezes at Supermodel frame 10,730 on an alignment
  exception: a page-crossing misaligned `lwzx` at 0x2DF34, EA FF292FFE (R122). D-39 traps that case and the 603e
  evidently does not.
- **Sega Bass Fishing:** its crash at frame 797 matches the same page-crossing loads (R122). Not proven on the board.
- **Lost World (`lostwsga`):** bring-up in progress in `sm3-builds/lw15`.
- **Data side (R120):** FSH and SLD are not kept. There are no CPU-side data levers above 0.5%.

### Next

1. The page-crossing misaligned split in the CPU (`sm3-builds/xpage`, build `xp1`). Then the board on Scud Race
   past three minutes and on Bass Fishing, and an R-entry amending D-39.
2. Lost World's report and board test.
3. Speed: the Real3D's VROM requests overlapped in `m3_xmem`, or a VROM cache (R100, R119). That is where Scud
   Race's long views and Daytona 2's geometry wait.
4. Step 1.5 luminous polygons (R119).

## State on 2026-11-06, evening

The study's §15 R105–R118 and D-44 to D-46 carry every finding; this is the summary. The tree is build `r7f1`'s RTL
(RBF 5c456258): byte for byte, except the `M3_RELEASE` line `Model3.qsf` keeps commented out for test builds.

- **Board, `r7f1` (110 MHz, release, setup −2.11 ns, hold positive at every corner):** VF3 correct, with more
  moments at 58 fps mid-scene than any build before. Not yet seen on Daytona 2.
- **Board, `r7c2`:** the FPS counter reads Daytona 2 at a steady 28-29 (a frame every second vblank of 57.52 Hz)
  and VF3 mostly 29, with some scenes at 58. Daytona 2 "running really well".
- **Clock: 110 MHz (D-44).** It does not close: −1.8 to −2.1 ns at the slow corners, TNS −450 to −930 ns. Test builds
  run on the board; a release needs a whole-chip timing pass.
- **Area (release): 39,758 ALM (95%), 324 of 553 M10K.**
- **New in the tree since `r7b2`:**
  - R115: FP result bypass, FP load write-back, targeted micro-TLB emptying, no extra FP wait.
  - R117: integer load-use 2, from the predicted way's word in the D-cache's lookup cycle.
  - R118: FP early forward (dependent single op at 5 cycles, not 6) and relaxed WAW.
  - The FPS counter (`rtl/dbg/m3_fps.sv`, OSD "FPS counter", off by default).
  - On the system bench since R107's baseline, VF3's busy cycles a vblank are down about 12%. R117 + R118 alone:
    VF3 busy / work CPI −5.7% / −4.4%, Daytona 2 −2.2% / −1.7%.
- **Wrong turns:**
  - `r7c1`'s FBYP paths looked like the slack's cause; `r7c2` showed placement moves them, and `r7d1` (FBYP off) was
    slower on the board.
  - My first combined system bench crashed: a cleaned-away `ckpt/` folder, not the tools.
  - A menu command once queued behind a core load and would have quit VF3; checking `/tmp/CORENAME` first now.

### Next
- Fetch side (`sm3-builds/fe119`) and data side (`sm3-builds/st119`) speed work, both running. The data side
  first fixes `r7f1`'s new worst family (R117's early word into R118's early forward, −2.11 ns).
- Step 1.5 / Scud Race bring-up (D-46, `sm3-builds/s15`), running.
- Count Daytona 2's third 53C810 wait (2B64-2B7C) in R104's windows (R117).
- Review later: occasional horizontal tearing across VF3's stage and fighters on `r7f1` (some scenes only; may not
  be new). Suspects and next steps in the study under R118 ("Board, open").
- A whole-chip timing pass at 110 MHz for a release build.
- Deferred: the iterative double FMA (about 1,350–1,750 ALM); room for sound (about 4,500 ALM, D-45) and I/O.

## State on 2026-11-06 (r7b2)

The study's §15 R105–R115 and D-44 carry every finding; this is the summary. The tree is build `r7b2`'s RTL
(RBF 66e258c3), with the clock at 110 MHz in `rtl/pll/pll_0002.v` and `Model3.sv`. `r7b2` was a release fit: its
build script turns on the `M3_RELEASE` line that `Model3.qsf` keeps commented out for test builds.

- **Board, `r7b2` (110 MHz, release, setup −1.83 ns, hold positive at every corner):** VF3 runs at 60 fps in some
  scenes and 30 in others. It is the first build with any scene at 60, but VF3 is not at 60 throughout. Daytona 2
  runs at 30 fps or over the whole time. Judged by eye; no frame count.
- **Board, `r7a1` (R111):** Daytona 2 and VF3 both look right. The skid marks, shadows and smoke are correct (R106's
  texture-ring and three-bit alpha fixes).
- **Clock: 110 MHz (D-44, superseding D-43's 120).** R113 priced 120 MHz for the CPU at 3-4% more instructions a
  second at best, and possibly fewer, so the speed now has to come from cycles at 110.
- **Area (release): 38,870 ALM, 324 M10K.**
- **In the tree:**
  - R107's SDRAM options (BXOR, L16, LW1, PAIR, PW).
  - R108's CPU timing restructurings, cycle-identical.
  - R109's Real3D pipelining, pixel-identical.
  - R110's area changes.
  - R111's SDRAM-port line buffers in MLABs.
  - R112's six memory options (`BUS_FAST`, `BUS_R3P`, `SDR_FREQ`, `SDR_FWD`, `SDR_FDSP`, `SDR_PW_DLY` 8). On the
    bench they take busy cycles a frame down 6.3% on VF3 and 12.5% on Daytona 2.
  - R114's ordering fix: with R3P, every access to a target other than the SDRAM waits for the posted host-port
    writes, as on the MPC105 (manual 8.1.2.2). It costs 0.2% of VF3's busy CPI.
- **Wrong turns (recorded in the study):**
  - R3P as R112 built it let palette, PCI config, I/O, IRQ, 53C810 and tilegen accesses overtake up to 7 posted
    words. The bus bench's new order check reproduced it, and R114 fixed it.
  - `r7b1` was launched before that check and was stopped in the fitter.
- **Not merged: R115's four CPU cycle changes:**
  - What they are: the FP result bypass, FP load write-back, the targeted TLB flush and dropping the extra FP wait.
  - Gain: busy CPI −3.1% on VF3 and −1.9% on Daytona 2; work CPI −3.8% and −5.2%.
  - Cost: +366 ALM, no M10K, CPU spike within seed noise.
  - Location: they live in `sm3-builds/cpi112`, measured on the R107-era CPU tree.
- **Poll loops:** VF3 spends 17.1% of its busy cycles in a byte-poll loop at 61DC4–61DD0, outside R104's wait
  windows; Daytona 2 spends 7.6% at 52A0–52C4. With VF3's loop counted as waiting, a frame every vblank needs a
  9–18% cycle cut, not 25%.

### Next
- Make the system bench's checkpoints load into the R108+ CPU (PDW 75→76, new registers). Until then the system
  bench measures the R107-era CPU, not the one that is built.
- Merge R115's four changes, bench on the merged tree, then a 110 MHz test fit.
- Trace what the two poll loops wait for; if it is an interrupt or a DMA, add them to the bench's wait list.
- Load-use 3→2 through the D-cache answering a cycle earlier (R56 cost 6.9 MHz done in the core), and the earlier
  redirect.
- Deferred: the iterative double FMA (about 1,350–1,750 ALM); the five-seed SDRAM spike sweep; frame-busy counts
  nothing on Daytona 2.
- Room for sound (about 4,500 ALM) and I/O (about 1,000).
- Review later: occasional horizontal tearing across VF3's stage and fighters on `r7f1` (some scenes only; may not be new). Suspects and next steps in the study under R118 ("Board, open").
