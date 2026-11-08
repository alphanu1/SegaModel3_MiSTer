# Handoff

## Update (2026-11-08, later)

The study's R134-R137 and D-49 carry this. The tree is build `c7_s1`'s RTL (RBF be782d4a): 41,091 ALM (98%), 347
M10K, 88 DSP; setup −2.59 / −2.55 ns, HDMI met, hold least +0.084 ns.

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
