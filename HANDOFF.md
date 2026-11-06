# Handoff

## Current state (2026-11-06, evening)

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
