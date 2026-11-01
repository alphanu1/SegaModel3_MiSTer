# CPU working notes

Findings and wrong turns from CPU work that have not yet gone into the study. The
study (`docs/model3-step10-design-study.md`) is the record; these notes are where I keep
the detail behind an R-entry, and the turns that did not work.

## D-30: BTB, two-word fetch and the 603e's branch look-ahead (2026-11-01)

Built in R61's order of gain: the BTB first, then two-word fetch, then direction. Each
is a parameter of `ppc_core` (`BTB`, `FETCH2`, `LA`; `ppc_icache` `PRE`, `FETCH2`;
`ppc_cpu` `BTB`, `FETCH2`), on by default.

### What I built

- **BTB, design C** (`ppc_btb.sv`, 64 entries, two M10K, `CURPC = 1`, `TD = 2`). An entry
  is keyed by the request before the branch's request, and holds the target, the
  target's physical page and I-cache way, which word of its request the branch is, and
  the key request's size. B's request carries the cache's pre-read of T; T's request is
  answered in one cycle when the stored way's tag is the stored page.
  - Every prediction is checked as B's word arrives (registered, acted on the next
    cycle: `bmis_q` discards T's answer and fetch moves to what the word asks).
  - A bclr / bcctr target is checked in EX; a difference refetches from MEM through
    the resync path (`m.rtm`, target in `m.sprv`). One register prediction is in
    flight at a time (`rp_v`, `rp_tgt`), so the queue carries no targets.
  - The y bit is honoured: entries are written only for branches the PEM 4.2.4.2 rule
    (or, with `LA`, the 603e's resolved direction) predicts taken.
  - The table empties on mtsr, mtsrin, tlbie, tlbli and BAT writes, not on tlbld
    (data side only), since the cache trusts the stored page.
- **Beyond R61's design C:** the address the coming request will carry is presented to
  the table even when it is a fetch-side redirect's target (a soft redirect, B's T, a
  failed check's correction). `CURPC` makes the BTB compare the request's own address,
  so a wrong guess costs only the lookup. Only the request after a hard (EX or MEM)
  redirect is still not looked up.
- **Two-word fetch.** The I-cache's data arrays became an even and an odd bank per way
  (two 512 x 16 lanes each: the same 16 M10K), read at their own indices, so a word and
  the next in its line come out together. Only the fast path answers two (`i_two`
  offered from a register, `i_w2` asked), never across a line, so never across a page.
  The queue takes a second word at counts 0-2; its ring keeps the rule that its slots
  are written by the count alone (at count 2 a pair fills both, and a pop turns the
  ring). The predecode moved into `ppc_pdw.sv`, instantiated twice.
- **Direction: the 603e's own rule.** 603e manual 2.3.4.4: the branch processor "scans
  the execution pipelines to determine whether an instruction in progress may affect
  the particular CR bit. If no interlock is found, the branch can be resolved
  immediately ... If an interlock is detected, the branch is ... predicted using static
  branch prediction" (PEM 4.2.4.2, the y bit). So a bc, bclr or bcctr whose CR and CTR
  have no writer in the queue, EX or MEM is resolved from the committed registers as its
  word arrives; otherwise the y bit decides. The scan is per register, not per CR bit.
  R61's "dynamic direction" (the BTB entry's history overriding the y bit) is not the
  603e's behaviour and is not built; it stays a model option (`_dy`).

### Wrong turns and bugs found

- **A failed check met a hard redirect.** The first --loops run at wait 1 (seed 19,
  real mode, a rewritten branch) retired the wrong PC: B's check failed in the same
  cycle as an older isync's EX redirect, and the registered correction then overrode
  the hard redirect. The correction is now cancelled by a same-cycle hard redirect.
- **The bench's obj folders picked up an old bench.** Verilator's generated make found
  `sim/cpu/tb_ppc_core.cpp` in a copy of the tree beside the obj folder (`sm3-builds/d30/`),
  so a BTB-off comparison ran the old generator. Bench builds now go where no tree copy is.
- **The fault list went into VFLAGS.** My first faults.py edit matched the end of the
  VFLAGS line, not of FAULTS, and four parallel runs then collided in the work folder.
- **A seed fit picked up a later spike top.** The spike script copied the spike project
  when each seed started, so a seed that waited for a Quartus slot got the next
  addition's ports. The project is now snapshot with the RTL.
- **Pairs across a page.** The core bench offered two words anywhere; a pair crossing a
  page gave the second word the first's translation (seed 26, mode 2). The core now asks
  for two only inside a line, as the cache offers them.
- **Grouping changed under the keys.** With two-word fetch, whether a request is one or
  two words depends on queue room, so the request after a key is not always the one the
  entry was written for: 4,150 failed checks in 200 seeds. The entry now also holds its
  key request's size and is used only when that matches (failed checks back to the
  rewritten-code ones).
- **The look-ahead check fired on bcctr.** EX's assertion (a branch resolved at fetch must
  not mispredict) fired on a bcctr resolved taken: without a BTB target fetch cannot
  follow it, so only branches fetch followed are marked.
- **Quartus 17 rejected a struct field before its declaration** (`q_pd.w_cr` in the
  look-ahead scan); Verilator accepted it. The predecode structs are declared earlier.
- **Timing, first BTB spike:** the late kill (EX redirect) sat in the enables of the
  prediction's 54 payload flip-flops, and the register-target compare sat behind EX's
  LR forward. The payload now loads on any capture (p_v alone says it is live), and the
  compare is made on both forward sources and chosen by the forward select.
- **Entry width.** 56 + 24 bits needs three M10K; leaving PC[29:28] out of the tag
  (`TD = 2`) fits two. An alias only costs a failed check.
- **The model's redirect-target lookup never applied.** `lv5_`'s `cx` tested the
  stream's break flag, which is set at every redirect target, so `cx` equalled `c`.
  `lv6_` has it right.
- **The soft allocation's key read a cycle late** took the next request's address
  (f_rsp_pc moves as the next request goes out), +1.9% bench cycles, no wrong result:
  the key is now registered in the redirect's own cycle.
- **A guessed lookup address from the static rule** (to keep the CR look-ahead off the
  BTB's address) cost +2% bench cycles in random code, where the look-ahead resolves
  many branches; reverted, the path left as it was (-0.66 to -0.68 ns, not the worst).
- **I replaced a full-build tree while its build was starting** (the queue script had
  reached it): that run failed on the deleted tree and was queued again.
- **The cached full builds' worst paths moved with each fix** (they swing about +-0.5 ns
  by seed, as R70 found): round 1 the arriving word through the look-ahead into the
  BTB's read address (fixed: the soft-redirect lookup is guessed from the static rule,
  `SOFTLU`); round 2 the D-cache's `x_v` into the queue's count copy and the two-word
  decision into the cache's bank index (fixed: the count for both pop values, the pop
  choosing last; the bank indices of nx + 1 and nx + 2 from registers); round 3 the
  first word's full predecode feeding the second word's look-ahead into the queue's
  predicted bit (fixed: a shallow superset of CR / CTR writers from the raw word,
  checked against the predecode in simulation).
- I briefly used `pgrep -f` in a wait loop (read-only); stopped and replaced.

### Measured

Bench cycle totals (R70's regression, same seeds; the core bench offers two words on
75% of cycles, so its two-word gains are larger than a real cache gives; `sub` and
`lsub` run the real caches):

| Run | R70 | + BTB | + two-word | + look-ahead |
|---|---|---|---|---|
| w1 | 792,096 | 789,942 | 786,012 | 778,708 |
| w4 | 1,385,186 | 1,325,275 | 1,169,799 | 1,164,062 |
| w9 | 2,594,506 | 2,463,713 | 2,062,148 | 2,064,795 |
| dl1 | 3,431,454 | 3,283,704 | 2,884,585 | 2,869,617 |
| sub (caches, wait 1-80) | 16,339,058 | 16,214,401 | 16,203,339 | 16,186,194 |
| lp4 (--loops) | - | 3,580,430 | 3,184,396 | 3,156,204 |
| lsub (--loops, caches) | - | 12,418,870 | 12,383,411 | 12,355,123 |

Board harness: P09 0CA977DF throughout. P00 no cache 000011EA -> 000011EA -> 000011EA ->
000011DA (4,586 -> 4,570); cached 00001E5E -> 00001E5C -> 00001E41 -> 00001E34 (7,774 ->
7,732). The board program is straight-line (each branch runs once), so the BTB cannot
show there; only the look-ahead and two-word fetch do.

CPU spike, 8.333 ns, seeds 1 / 2 / 3 (R70: 108.3 / 108.6 / 110.1 MHz, 11,086-11,090 ALM):

| Form | Fmax MHz | ALM | M10K |
|---|---|---|---|
| + BTB (first form) | 107.97 / 105.98 / 104.84 | 11,075-11,105 | 2 |
| + two-word, BTB timing fixes 1-2 | 110.89 / 103.23 / 102.83 | 11,806-11,833 | 2 |
| + look-ahead, fixes 3-5, R70's MLAB patch | 105.81 / 108.53 / 108.01 | 11,839-11,871 | 2 |

I-cache alone (seed 1, I/O registered): 626 ALM 131.9 MHz; with the pre-read port 765
ALM 116.9 MHz; with two-word answers too 918 ALM 123.6 MHz; 18 M10K in all three.

CPI model (`tools/mame-census/supermodel-lv4.patch`, `lv6_` options; 18,900 frames a
title at 100 MHz; frames lost against `ref66_dual_ic`, VF3 / Daytona 2 / Bass Fishing):

| Option | Singles at 4 (R61's base) | Built fast path, 6 |
|---|---|---|
| core before D-30 (`b0_f1`) | 360 (334 / 16 / 10) | 373 (343 / 20 / 10) |
| + BTB (`cx_f1`) | 22 (4 / 11 / 7) | 42 (16 / 19 / 7) |
| + two-word (`cx_f2`) | 17 (3 / 8 / 6) | 37 (12 / 18 / 7) |
| + look-ahead (`cx_f2_la`), as built | **15 (3 / 6 / 6)** | **34 (10 / 17 / 7)** |
| R61's entry direction instead (`cx_f2_dy`) | 10 (3 / 6 / 1) | - |
| as built at 110 MHz | 0 | 3 (3 / 0 / 0) |

CPI as built, singles at 4: 1.757 / 2.100 / 2.031 (R61: 1.726 / 2.055 / 1.986, 3 frames).
The difference from R61 is what the model now charges that R61's did not: 64 entries
and the y-bit rule, the first request after a redirect target is one word, refetching
ops (rfi, sc, isync, mtmsr, MMU writes) pay a redirect, and only the request after a
hard redirect is not looked up.

Full builds, release form, 100 MHz, first round (SOFTLU as first built; slack ns, seeds
1 / 2 / 3): no cache +0.017 / -0.988 / -0.337 at 17,248-17,267 ALM, 123 M10K, 30 DSP
(seeds 2 and 3 fail only on the harness ROM's address through fetch translation); cached
-0.294 / -0.444 / -0.465 at 19,206-19,256 ALM, 159 M10K (the arriving word through the
look-ahead into the BTB's read address).

Full builds, release form, 100 MHz, all rounds (slack ns, seeds 1 / 2 / 3):

| Round | RTL | No cache | Cached |
|---|---|---|---|
| 1 | as built, with the spike's timing fixes and R70's MLAB patch | **+0.017** / -0.988 / -0.337 | -0.294 / -0.444 / -0.465 |
| 2 | + soft-redirect lookup on the static guess (`SOFTLU`), allocation key registered | -0.143 / -0.226 / -1.652 | -1.074 / -0.058 / **+0.196** |
| 3 | + queue count pop-last, two-word bank indices from registers | -0.376 / -0.824 / -0.460 | -0.567 / -0.359 / -0.782 |
| 4 | + the second word's look-ahead from raw fields (cached only) | - | -0.454 / -0.703 / -0.455 |

No cache 17,184-17,267 ALM, 123 M10K; cached 19,181-19,290 ALM, 159 M10K; 30 DSP (R70:
16.75K and 18.07-18.14K ALM, 121 and 157 M10K). Every no-cache failure is the harness
ROM's address through fetch translation (a path the core with its I-cache does not
have); the cached failures move with the seed (the queue pop from the D-cache's ready,
the D-cache's own request registers, the arriving word into pt_q), +-0.5 ns as R70 found.
Closing builds: no cache round 1 seed 1 (md5 3aaaa6ac...), cached round 2 seed 3 (md5
a1fe9f93...).

Parameters: with `BTB = 0`, `FETCH2 = 0`, `LA = 0` the core bench gives R70's cycle totals
exactly (w1 792,096, w4 1,385,186, w9 2,594,506), so the additions come out cleanly.

Planted faults (each a copy of the tree; caught if any bench run fails):

| Set | Caught | Missed |
|---|---|---|
| BTB and pre-read port (24) | 16 | btb_no_flush, btb_ctx (no code remapping or context change at one address in the generator), btb_rp_gate, btb_rteq_sel (rare cases), btb_key, btb_pa_ea, btb_soft_dup (cost only cycles), ic_pre_haz |
| Two-word fetch (19) | 17 | f2_mark (a junk second word behind a fault marker, never run), ic2_phit (the core never asks for two at word 7) |
| Look-ahead (10, with la_may) | 9 | la_q3 (queue entry 3 rarely holds a writer when a branch arrives) |
| Existing set (84) | 75 | as R70: pd_ctr_bclr, ffw_unit, qf_m_hold, icbi_set, rt_kind_fub, replay_kill_wb, dbat_run, fq_ring_ptr, wb_live_clr |

Generator: `--loops` adds counted loops (bdnz), LR loops, subroutines called from several
sites (bl, bctrl, conditional returns with y = 1) and, in real mode, code rewritten in a
loop (a branch or a blr, with dcbst / sync / icbi / isync); a `--loops` run of 100 seeds
or more fails unless every BTB path was taken. The board program never uses it.

## D-36: two-word fetch off by default (2026-11-01)

`FETCH2` now defaults to 0 in `ppc_core`, `ppc_icache` and `ppc_cpu`; `BTB = LA = 1` stay.
Nothing else sets it (the spike tops and `m3_cputest` take the defaults; `pair_spike`'s
`FETCH2` is `ppc_pair_proto`'s own). It stays a parameter.

### Regression, `FETCH2 = 0`, `BTB = LA = 1` (never benched on its own before)

Run from a snapshot of `rtl/` with only the three defaults changed (`sm3-builds/d36_rtl`,
`d30_reg_src.sh`), so the bench saw what now ships. Everything passes:

| Run | D-36 cycles | R78 (all three on) | BTB only (R78's column) |
|---|---|---|---|
| w1 | 782,564 | 778,694 | 789,942 |
| w4 | 1,318,268 | 1,163,629 | 1,325,275 |
| w9 | 2,464,137 | 2,064,684 | 2,463,713 |
| dl1 | 3,267,615 | 2,869,099 | 3,283,704 |
| sub (caches, wait 1-80) | 16,198,764 | 16,186,230 | 16,214,401 |
| lp4 (--loops) | 3,570,653 | 3,156,852 | 3,580,430 |
| lsub (--loops, caches) | 12,363,713 | 12,355,144 | 12,418,870 |

All 1,080 seeds at wait 1-9, 300 at `--rdy 25`, 300 at `--dlat 2`, 300 at `--dlat 1`, 300
subsystem seeds, the four `--loops` runs (coverage check included), `test_cache`,
`test_fpu_fast`, `test_fpu_reg`, `test_btb` (64 24 0) and `test_icache_pre` pass. The core
bench's big losses (w9 +19%) are its 75% two-word offer, which no real cache gives; with
the real caches (`sub`, `lsub`) the cost is 0.08% and 0.07%.

Board harness (`make test_cputest`, `test_cputest_cached`): P09 0CA977DF, P01 00000661 in
both. P00 no cache 000011DA (as R78: the uncached ROM never offers two words), cached
00001E4E (R78 00001E34, +26 cycles of 7,732, +0.34%). The ROM images are unchanged.

### Planted faults in this configuration

| Set | Caught | Missed |
|---|---|---|
| BTB and pre-read port (24) | 16 | the same eight as R78: btb_rteq_sel, btb_rp_gate, btb_soft_dup, btb_key, btb_no_flush, btb_pa_ea, btb_ctx, ic_pre_haz |
| Look-ahead (10) | 8 | la_q3 (as R78); la_w1, which is now dead code (`la2` is gated by `LA && FETCH2`) |

So of the 9 look-ahead faults that still apply here, 8 are caught.

### CPU spike, 8.333 ns, seeds 1 / 2 / 3

| Form | Slow 100C MHz | Slow -40C MHz | ALM | M10K |
|---|---|---|---|---|
| R78 (`d30a3`, all three on) | 105.81 / 108.53 / 108.01 | 107.11 / 109.40 / 107.89 | 11,839-11,871 | 2 |
| D-36 (`d36sp`) | 102.54 / 108.89 / 109.16 | 102.21 / 107.75 / 109.27 | 11,444-11,550 | 2 |

-321 to -427 ALM in the core alone (the cache's +153 for two-word answers is not in the
spike). R78's spike RTL predates the full-build rounds 2-4, so this is not a like-for-like
pair; a FETCH2 = 1 spike of today's RTL would be. Seed 1's worst path is EX -> MEM
(`op_x` -> `m.rp`), not a fetch path.

### CPI model

The frames-lost figures in `lv6_` had `cx_f1_la` at singles-at-4 and 100 MHz only: 18 lost
(VF3 3 / Daytona 2 8 / Bass Fishing 7) against 15 with two-word fetch, so FETCH2 = 0 costs
3 frames in 56,700 there. I added `_cx_f1_la_l6` and 110 MHz for `_cx_f1_la_` to
`supermodel-lv4.patch`. Both edits are on existing lines, so the hunk counts stand and
the CRLF endings are kept. Runs are in `sm3-builds/lv2runs/d36/` (`run_d36.sh`), with
`cx_f2_la` alongside as the check that R78's 15 / 34 / 0 / 3 come back.

### Wrong turns

- My first edit of `supermodel-lv4.patch` went through Python's text mode and turned its
  CRLF lines into LF (a 36 KB diff). I restored it from the copy and redid it in binary.
- My first regression waiter grepped for "rror" and counted the 16 "PASS: 0 error(s)"
  lines as failures. I read the output by hand: there are no failures.
- I started the spike before the regression's last unit benches had finished (the core
  and subsystem benches had passed). The snapshot RTL is the same as what ships.

### Full builds, release form, 100 MHz, multicorner (D-35)

CPU-clock setup slack in ns. Hold is positive at every corner on every seed (smallest
+0.059, fast -40C, no cache seed 1).

| Build | Seed | Slow 100C | Slow -40C | Fast 100C | Fast -40C | ALM | M10K | RBF md5 |
|---|---|---|---|---|---|---|---|---|
| No cache | 1 | +0.008 | -0.013 | +4.430 | +5.299 | 17,214 | 123 | 29c7f0d0... |
| No cache | 2 | -0.039 | -0.116 | +4.133 | +5.136 | 17,222 | 123 | dd385ab5... |
| No cache | 3 | -0.223 | -0.288 | +4.372 | +5.282 | 17,235 | 123 | 5e4ebe7f... |

None of the three uncached builds closes at every corner. Seed 1 closes at slow 100C and
misses by 13 ps at -40C. Under R78's single-corner analysis it would have counted as
closing. Seed 2's worst path is the harness ROM's `i_data` into the fetch queue head, a
path the cached core does not have. Seed 3's is the queue head into `qe_e`.

Area: 17,214-17,235 ALM against R78's 17,184-17,267, so no saving uncached. `m3_cputest`
ties `i_two` to 0 there, so Quartus had already removed the two-word logic in that form.
The saving shows only in the cached build.
| Cached | 1 | -0.177 | -0.203 | +4.246 | +5.169 | 18,735 | 159 | e71ee5c3... |
| Cached | 2 | -0.117 | -0.243 | +4.067 | +5.122 | 18,680 | 159 | 060746af... |
| Cached | 3 | -0.232 | -0.231 | +4.171 | +5.198 | 18,694 | 159 | 61620789... |

The cached rows belong in the table above. Cached area is 18,680-18,735 ALM against R78's
19,181-19,290: -446 to -610 ALM (about -520 mid to mid) at the same 159 M10K. That is less
than R81's -880, which came from the spike and cache deltas added up. R81's projection
without two-word fetch moves from about 40,560 to about 40,920, still under 41,000 but
with about 80 to spare instead of 440.

None of the three cached builds closes. All miss by 0.12-0.24 ns at slow 100C and by
0.20-0.24 ns at -40C. The worst paths are not fetch paths: the DTLB MLAB's read address
(seed 1), `op_y` (seed 2) and the D-cache's `r_a` (seed 3). They move with the seed, as
in R70 and R78. Hold has margin at every corner.

### CPI model, D-36 configuration (`lv6_`, 18,900 frames a title, against `ref66_dual_ic`)

| Option | 100 MHz, singles at 4 | 100 MHz, built fast path at 6 | 110 MHz, s4 / l6 |
|---|---|---|---|
| `cx_f2_la` (R78, as rerun) | 15 (3 / 6 / 6) | 34 (10 / 17 / 7) | 0 / 3 |
| `cx_f1_la` (D-36) | **18 (3 / 8 / 7)** | **35 (10 / 18 / 7)** | **0 / 3** |

The rerun gives R78's `cx_f2_la` figures exactly, so the added options change nothing
else. With FETCH2 off, CPI as built at s4 is 1.766 / 2.117 / 2.037 (R78 1.757 / 2.100 /
2.031). The cost is 3 frames at singles-at-4 and 1 at the built fast path, all of them
Daytona 2 and Bass Fishing, and none at 110 MHz. Against the full builds, two-word fetch
was about 520 ALM cached for 1-3 frames in 56,700.

## 100 MHz at every corner after D-35 / D-36, and misaligned accesses (2026-11-01)

The goal was that both release forms close 100 MHz at every corner on at least 2 of 3 seeds, with
CPI and function unchanged. The forms are the uncached harness and the cached ppc_cpu. Midway the
long-term target moved to 110 MHz (D-38), so I chose fixes that take levels out, not ones that only
scrape past 10 ns. Then VF3 crashed at the SEGA-logo wipe on a misaligned lwzx, so misaligned
accesses went in as required behaviour (603e 2.2.3, 4.5.6), with no parameter.

### How I built and measured

- Every full build comes from a frozen copy of the D-36 release tree (`sm3-builds/tc_base`, from
  `d36full_s1`), with rtl/cpu and rtl/dbg swapped in (`tc_full.sh`, `tc_batch.sh`, `tc_round.sh`).
  During the day the live Model3.sv moved to the system top (the M3_CPUTEST macro, a third PLL
  output). The frozen tree keeps the harness form R83 measured, so every slack here compares
  like for like with R83, and every board RBF here is that same harness form.
- First I rebuilt R83's cached seeds 1-3 from the frozen tree, keeping the db. Seed 1's RBF md5 came
  out identical to R83's (e71ee5c3...): a rebuild reproduces a fit exactly.
- `wp.tcl` reports the worst 400 paths and 12 full paths at both slow corners. `tc_q2.sh` queries
  any from/to pair at -40C from a kept db.
- Regression: `tc_reg.sh` (R70/R78's set, from `d30_reg_src.sh`) on frozen copies of sim/ and the
  Makefile, compared line by line with D-36's output by `tc_cmp.sh`: cycle totals, BTB counters,
  unit benches, board P lines. `tc_reg2.sh` adds 15 `--mis` runs (below).

### Timing fixes, rounds 1-3 (regression identical to D-36 after each)

| Path (where it failed) | Fix | Levels |
|---|---|---|
| h0 -> qe_e (D-36 uncached s3, -0.22) | ID's own rd from predecode (`pd.rda`, `id_rdf`), not the decoder | 8 -> about 4 |
| harness ROM address (uncached, all seeds) | the ROM in 1,024-deep blocks (`max_depth`): bits 14-12 select from a register | harness only |
| DTLB MLAB -> sl_tm_q / sl_dsi_q (cached s1, -0.20) | the outcomes per way, the hit choosing last (`sl_pf`, `sl_wf`) | -2 |
| h0 -> ex_u.imm (cached s2, -0.12) | the branch displacement from the raw word (`ex_bd`); ex_u.imm is trimmed | -5 |
| h0 -> ex_fsfld | mcrfs from the raw word | -3 |
| op_y -> D-cache r_a (all three cached seeds, -0.25) | the micro-TLB compare laid out by hand (keep), and the page's AND-OR in two levels | 8 -> 5 |
| D-cache r_a -> x_ph, x_v -> d_rdata, x_v -> d_rdy_q | the set compares, the store-buffer compares and the tag compare for both candidates, x_v (and a copy of it, `x_vc`) choosing last | -1 to -2 |
| I-cache tag -> pend / nx (cached s3, -0.17) | each way against the page, the stored way choosing last | -1 |
| p_tgt / f_pc -> I-cache pa (cached round 1, -0.73) | pa loads whenever no request is pending, not only as one starts | -1 |
| monitor rt_gpr_val_q -> sd_q, line -> u_data_q | the data CRC as a flat XOR matrix; the printer's word registered | harness only |
| op_x -> w_be (harness) | lanes and enable registered apart | harness only |
| fc_q / fb_q -> FMA esum (cached r2 s2, uncached r3 s1) | frB's and frC's raw exponents into one ternary add (xe(e) = e' - 1023) | 4 chains -> 1 |
| sfma r4_q.lz -> r5_q.bnu2 (uncached r1 s1) | bnu0/1/2 each on its own add, tiny / far choosing last | -2 chains |
| m.result (m_z) -> m.cr, -> redir_k_q | the CR logic bit and fetch's redirect registers for both values of m_z, m_z choosing last | -2 |
| m.cmp_we -> op_a (uncached r1 s3, -0.52) | mfcr's CR mask folded a cycle early (`mf4c`, `mf4o`) | -2 |
| h0 -> ex_mfv (cached r2 s1, -0.57) | the move-from value as one AND-OR under selects from the predecode, MEM's forward last | 7 -> 5 |
| wk_act / fb_v -> spr_mem rdaddr_reg | the SPR array's read address through a LUT of its own | Quartus no longer copies h0 into the array |
| m.spr~DUPLICATE -> spr_mem write address: hold -0.085 (cached r2 s1) | the write address through a LUT | hold fixed |
| i_data -> h0 SPR index (uncached r1 s3) | the SPR index as one flat case | 6 -> about 3 |
| h0 -> GPR file -> op_y | a registered copy of the head's register fields (`r_rf`, preserve) | shorter route |
| m.duwi -> pend (cached r2 s3, -0.28) | the FP scoreboard's set gate from the kill's two products | 4 -> 2 |
| op_y -> DBAT -> xl_rp -> D-cache d_valid (cached r3 s3, -0.95) | the DBAT compare laid out by hand (`db_cf`, `db_ok`), checked on 20M cases with any BL | 5 -> 4 |

Every restructure has a Verilator check against the form it replaced.

### Wrong turns

- My first launch of a round ran from the wrong directory. A backgrounded `cd ... && ... &` ran the
  cd in its subshell only, so the next lines expanded `$PWD` to the repository root. Three builds
  then started with a missing RTL path, and the round's scripts were written to the root. I killed
  them before they finished, deleted their folders, moved the scripts into sm3-builds and
  relaunched with absolute paths. No result came from those builds.
- The first `ex_bd` had the b / bc select inverted (`q_hi[27]`). The new check caught it in the
  first seed.
- The early-compare check had to be qualified by EX's valid bit, not only its rd_we: with
  `id_rdf`, qe_e is only meaningful for a valid EX uop that writes a GPR.
- The D-cache's per-candidate group vector was declared 56 bits for 28. Lint caught it.
- `cross` is a SystemVerilog keyword (Verilator refused it); the signal is `mis_x`.
- **Flat queue entries (round 2), reverted in round 3.** I wrote entries 0 and 1 as one six-input
  LUT per bit to take the arriving word's predecode through one level. That put the pop (EX's hold)
  through a LUT in front of every entry bit. The pop's paths (ex_valid, fb_v, wk_act, r_v -> h0/h1)
  then led three of the six round-2 builds, and the fetch queue grew by about 45 ALM. Only the
  register copy for the GPR addresses was kept.
- Round 1's cached builds got worse (-0.57 to -0.87), not better. The x_v select in front of the
  D-cache tag compare (x_v drives about 200 loads) and the fetch translation into the I-cache's pa
  surfaced once op_y -> r_a was gone. Round 2 fixed both.
- The full builds still swing by about +-0.4 ns by seed and by round, as R70 found. The same path
  (p_tgt -> I-cache pa) read +0.13 ns in one fit and -0.59 ns in another, with near-identical RTL.

### Misaligned accesses (VF3's crash at the SEGA-logo wipe)

VF3 does `lwzx r4,r3,r9` at an address 2 mod 4, from a pointer table in CROM (DBAT2). Earlier, at
frame 291, it stores and loads back words at offsets 1 and 2 in write-through work RAM. The core
forced every access aligned, so the pointer came back wrong and a later lhau took a DSI. The
603e does misaligned integer accesses in hardware (2.2.3: "The 603e provides hardware support for
some misaligned memory accesses ... Any memory access that crosses an alignment boundary must be
broken into multiple discrete accesses").

What I built (ppc_core.sv, "Misaligned accesses"):

- **The split.** An integer halfword or word crossing a word is split in EX. In its first cycle
  (`mis_hold`, EX holding) the word at EA goes out on the lanes from EA on, as a uop of its own
  into MEM with no effect but the access (`m.sp0`). In the next cycle (`ex_sp1`) the uop itself
  goes out at EA + 4 (ea_x's + 4) on the remaining lanes.
- **Stores.** Both words take one byte rotation of the value; each word has its own lanes.
- **Loads.** The first word is kept as it leaves WB (`mis_w0`) and merged with the second
  (`ld_mrg`). If the second word's answer reaches WB, WB holds one cycle and registers the merge.
  If it came while the uop was in MEM, the merge goes straight into wb_result.
- **What the split covers.** D, X, update and byte-reversed forms; the D-cache and the
  single-access path alike. A halfword at offset 1 is one access on lanes 0110.
- **Interrupts.** Once the first word has gone (`ex_s0ok`), an interrupt waits for the next
  instruction. The instruction completes and is never half done.
- **Cost.** One cycle per crossing access. Aligned code is untouched: with the FP-double page rule
  (below) taken out, w1/w4/w9/--loops/subsystem give D-36's cycle totals exactly (782,564;
  1,318,268; 2,464,137; 3,570,653; 16,198,764).
- **The alignment exception** (0x600; SRR0 the instruction; DSISR by table 4-13, X form for opcode
  31, else D form; DAR the EA) is taken before any access for:
  - lmw / stmw (at step 0), lwarx and stwcx. not word-aligned. For lmw / stmw DAR = EA + 4, as
    4.5.6.2 says: "alignment exceptions for load or store multiple instructions store that
    address value + 4 into the DAR";
  - FP loads and stores not word-aligned (4.5.6);
  - with MSR[DR] = 1, a halfword or word crossing a page, and an FP double crossing a page
    (below). With DR = 0 a page crossing is split.

**The page rule, and where the manual is ambiguous.** 603e manual (Rev 1), 4.5.6.1.1, verbatim:

> 4.5.6.1.1 Page Address Translation Access
> A page-address translation access occurs when MSR[DR] is set, SR[T] is cleared and there is not a
> match in the BAT. Note the following points:
> - The following is true for all loads and stores except strings/multiples:
>   - Byte operands never cause an alignment exception.
>   - Half-word operands can cause an alignment exception if the EA ends in 0xFFF.
>   - Word operands can cause an alignment exception if the EA ends in 0xFFD-FFF.
>   - Double-word operands cause an alignment exception if the EA ends in 0xFF9-FFF.
> - The dcbz instruction causes an alignment exception if the access is to a page or block with the
>   W (write-through) or I (cache-inhibit) bit set in the TLB or BAT, respectively.
> ...
> Finally, note that when the 603e is in page address translation mode, there is no special handling
> for accesses that fall into BAT regions.

The text is ambiguous in two ways:

- **BAT hits.** The first sentence excludes them from the access type. The last sentence can be read
  either way: BAT-region accesses get the same check, or they get no special (alignment) handling.
- **"Can cause" versus "cause".** Halfwords and words "can cause" the exception; doubles plainly
  "cause" it.

I first built the stricter reading (every DR = 1 crossing traps), then chose between the two.

**D-39 (my decision).** A crossing inside one DBAT block is split. The trap is taken only when
the access is page-translated (DR = 1, no BAT hit) or when the crossing leaves the BAT block. The
reasoning:

- Model 3's boot ROM treats any alignment exception as fatal, so no game relies on the trap.
- If the chip traps there, games never do it, and the split never fires.
- If the chip splits, games that do it keep working.

How it is built:

- **RTL** (`db_stay`, `db_leave`). The block ends at EA's last word when EA[16:2] and EA's bits
  under the hitting BAT's BL are all ones. The second word's EA + 4 goes through the same DBAT
  compare in its cycle, so its physical address is the BAT's.
- **ISS** (`dbat_of`, `pg_cross`). The same rule, with "leaves" meaning that EA + size - 1 is not
  in the block EA hits.
- **SR[T] = 1** is checked as a page access, so a direct-store segment gives the alignment
  exception, not the DSI. FP doubles follow the same rule.
- **Bench.** `--mis` adds crossings at a page boundary inside DBAT0's block (vd2 + 0x1000 k - 1..3)
  and at its end (vd2 + 0x20000 - 1..3). In 200 seeds at wait 4: 139 crossings with DR = 1 done
  inside a BAT block.
- **Faults.** `d39_leave` (never leaving) and `d39_stay` (never staying) are both caught.

**The emulators** (the manual wins, D-17; this is a manual-over-emulator point):

- **MAME** (ppcdrc.cpp, PPCCAP_MISALIGNED on the 603e):
  - never traps a big-endian misaligned halfword or word; it splits it, each part translated, so a
    page crossing never traps;
  - traps an 8-byte access only when not word-aligned;
  - traps lmw / lwarx misaligned (read32align);
  - splits a misaligned FP single instead of trapping it, against 4.5.6.
- **Supermodel** has no MMU and no alignment exception. Read32 at addr & 3 becomes two Read16.

Under either emulator, a page-crossing misaligned access with DR = 1 is done in hardware. VF3's
census shows none, so the choice matters only for other games.

**The FP double rule changes the default bench.** The bench made doubles across a page to exercise
the second word's replay. Those now take the alignment exception, which shows as 24 -> 317
alignment exceptions in w1 and slightly different cycle totals in the translated seeds. With that
rule taken out (a test copy, `sm3-builds/tc_cmpw1/nofp`), every default run gives D-36's cycle totals.

**Verification:**

- **ISS (ppc_iss.h).** Misaligned accesses byte for byte, the trace seeing each word a split reads,
  and the exceptions as above, checked against the manual's text. Counters: `mis_n`.
- **Generator (`--mis`, tb_ppc_core.cpp):**
  - halfwords and words of every form, 1-3 bytes past a word, doubleword, line or page boundary,
    plus an in-word halfword at offset 1;
  - lwarx / stwcx. / lmw / stmw / FP misaligned;
  - VF3's store-then-load-back at offsets 1-3, inside a doubleword and across a line, D and X form.
  - Under `--mis` the decrementer period is at least 600 ticks and external interrupts come every
    150-750 cycles. The first, because seed 55071 at wait 80 livelocked: DEC reload 50 ticks, shorter
    than its handler, the model doing the same. The second, so that an interrupt meets a split's
    second cycle.
  - 100 seeds at wait 4: in a word 520, across a word 615, a doubleword 340, a line 747, a page (DR
    = 0) 136, stores 1,008; alignment exceptions 1,773.
- **Regression (`tc_reg2.sh`).** The R70/R78 set plus 15 `--mis` runs: waits 1-9 (80 seeds each),
  `--rdy 25`, `--dlat 1` and `2`, the subsystem with caches, and `--loops` on the core and the
  subsystem. All pass. Board harness P00 000011DA / 00001E4E, P01 00000661, P09 0CA977DF, unchanged.
- **Planted faults.** 18 for the split, merge and exception, and 2 for D-39 (faults.py, benches
  `corem` / `subm`).
  - All 20 are caught. In particular `mis_w0_fwd` is caught by corem (subm misses it), and
    `mis_irq` is caught once the `--mis` interrupt rate went up. It was missed at the old rate:
    the window is one cycle.
  - In development the bench found two real bugs. First, MEM's merge read mis_w0 in the cycle the
    first word was still leaving WB (now `mis_w0_m`). Second, an interrupt taken in the second
    cycle made the first word twice (now `ex_s0ok`).
  - It also found a generator bug of mine: lhz / lha were treated as stores and given src()
    registers, which overwrote the base r2.

### Full builds, release harness form, 100 MHz, multicorner

CPU-clock setup slack in ns: slow 100C / slow -40C. The fast corners kept more than 3.9 ns, and hold
is positive everywhere except one fit, round-2 cached seed 1: -0.085 ns on spr_mem's write address,
fixed in round 3. Area is ppc_core (uncached) or ppc_cpu (cached) in ALM.

| Round (RTL) | No cache s1 / s2 / s3 | ppc_core | Cached s1 / s2 / s3 | ppc_cpu |
|---|---|---|---|---|
| R83 (D-36) | +0.008 / -0.013, -0.039 / -0.116, -0.223 / -0.288 | 10,391-10,408 | -0.177 / -0.203, -0.117 / -0.243, -0.232 / -0.231 | 11,947-12,001 |
| 1 | +0.180 / -0.177, **+0.346 / +0.486**, -0.522 / -0.262 | 10,327-10,341 | -0.709 / -0.572, -0.590 / -0.520, -0.774 / -0.872 | 11,918-11,948 |
| 2 | -0.108 / -0.159, -0.418 / -0.226, -0.141 / -0.106 | 10,388-10,425 | -0.569 / -0.358, -0.094 / -0.012, -0.467 / -0.277 | 12,083-12,142 |
| 3 | +0.114 / -0.051, **+0.063 / +0.039**, **+0.265 / +0.321** | 10,362-10,391 | -0.376 / -0.419, **+0.185 / +0.066**, -0.826 / -0.952 | 12,006-12,049 |
| 4 (+ misaligned) | +0.028 / -0.301, **+0.154 / +0.299**, -0.358 / -0.215 | 10,696-10,736 | -0.440 / -0.438, -0.838 / -0.886, -0.217 / -0.464 | 12,309-12,417 |
| 5 (+ trims) | **+0.053 / +0.095**, -0.078 / -0.113, +0.052 / -0.074 | 10,630-10,676 | -0.200 / -0.184, -0.868 / -1.034, -0.508 / -0.495 | 12,277-12,319 |

- Round 3, the timing work without misaligned support, met the goal uncached (2 of 3 seeds at every
  corner) at the same area as D-36. Cached it reached 1 of 3, at about +60 ALM.
- Misaligned support costs about +250 ALM uncached and +300 cached. It also put new paths into
  EX's hold. Round 5's cached worst, ex_u.op -> redir_hard_q (-1.03), was the split's hold: the
  byte offset came from the ALU's sum, behind its subtract logic. It now comes from the operands'
  own two bits (`boff = op_x[1:0] + op_y[1:0]`, one LUT; round 6).
- Other round-5 paths: icache i_data -> h0 (predecode), z_act / dq_v -> redir_a_q, FMA out_res ->
  ex_ffw, d_valid -> wb_result (the split merge into the MEM-answered load register).

Closing builds for a board check (round 3, before misaligned support):

| Build | Slack 100C / -40C | RBF md5 |
|---|---|---|
| `sm3-builds/tc3_s2` (no cache) | +0.063 / +0.039 | a1c84e05bca87aa4a0f5b04279414a7d |
| `sm3-builds/tc3_s3` (no cache) | +0.265 / +0.321 | 5955e806f59993a96e7aa113eff525af |
| `sm3-builds/tc3c_s2` (cached) | +0.185 / +0.066 | 5747ada7f622f67335981208cc27eab4 |
| `sm3-builds/tc5_s1` (no cache, misaligned support) | +0.053 / +0.095 | 5903f1af61a1b7c5a9e71767cc6c3b76 |

### Area

- Synthesis estimates on the CPU spike: D-36 11,117 ALM / 15,434 ALUTs; now 11,283 / 16,012.
- The FMA exponent rewrite is about +25 ALM (11,258 without it).
- The hand-laid DBAT compare is about neutral (11,378 without it, at fewer ALUTs: within the noise).
- Fitted: uncached ppc_core +250, cached ppc_cpu +300 against D-36. Against R83's whole-core
  projection (about 80 under 41,000), the ceiling is now passed by about 220-250 ALM unless
  something else gives. Misaligned support is required behaviour. Dropping the FMA rewrite would
  save about 25; the rest needs a budget decision at study level.

Round 6 (the byte offset from the operands, before D-39 and the raw split-kind decode; RBF md5s in
`sm3-builds/tc6*/result.txt`):

| Form | s1 | s2 | s3 | Area |
|---|---|---|---|---|
| No cache | -0.132 / +0.017 | **+0.143 / +0.035** | +0.033 / -0.091 | ppc_core 10,605-10,650 |
| Cached | -0.207 / -0.531 | -0.200 / -0.257 | +0.091 / -0.147 | ppc_cpu 12,321-12,382 |

The cached misses are now the D-cache tag into x_v, x_v and m.valid into EX's redirect registers,
and op_y into the D-cache's posted answer.

### CPU spike at 9.09 ns (110 MHz), the final RTL

| Seed | Slow 100C | Slow -40C | ALM |
|---|---|---|---|
| 1 | 107.71 MHz | 108.83 MHz | 11,691 |
| 2 | 106.87 MHz | 105.76 MHz | 11,719 |
| 3 | 99.66 MHz | 100.72 MHz | 11,667 |

D-36's spike was 102.5 / 108.9 / 109.2 MHz (8.333 ns) at 11,444-11,550 ALM. What stands between the
core and 110 MHz, worst first:

- **MSR / interrupt state into EX's hold** (seed 3, -0.94 ns). msr_q -> irq_req -> ex_xcm -> mis_hold
  -> ex_hold -> the ID step counters, imm4_q and the redirect registers. The split's hold carries
  ex_xcm.
- **EX translation into MEM's control** (-0.1 to -0.7 ns). op_x -> micro-TLB / DBAT -> xl_rp, algn,
  mis_go -> m.rp, m.valid, m_rpq, m.algn, qf_m, ex_s0ok.
- **FMA r1a_q.nc -> r1_shc** (-0.27 to -0.37 ns): stage 1b's shift count.
- **fpscr_q -> qf_m / fr_at** (-0.18 ns), and **fr_at -> the fetch queue** (-0.45 ns): the FP wait
  into the pop.
- **wb_sp1 -> wb_result** (-0.14 ns): the split merge.
- From the full builds:
  - icache i_data -> h0 / h1, the predecode (-0.1 to -0.45);
  - D-cache tag -> x_v / dq_v / uq_v (-0.2 to -0.5);
  - fetch translation -> I-cache pa (-0.6 when it shows);
  - x_v / dq_v / z_act -> EX's redirect registers.

The fixes and their costs are listed in R86's open paths. They are not built.

## D-41: the D-cache snoop port, for 53C810 DMA with copy-back RAM (2026-11-01)

VF3 never flushes the D-cache before the 53C810 reads RAM, and with PVR 0x00060103 RAM is
copy-back. On the board the MPC105 snoops PCI master cycles and the 603e pushes a modified line
(603e 3.3.3, 3.6.2-3.6.8, table 3-6). I built that push into `ppc_dcache` behind `SNOOP`
(default 1 in `ppc_dcache` and `ppc_cpu`).

### The interface (`ppc_cpu`, unchanged from the request)

- `sn_req` level, held until `sn_ack`; `sn_addr[31:5]` the physical line, stable with it;
  `sn_inv` 0 for a DMA read, 1 for a DMA write. `sn_ack` is a one-cycle pulse.
- Read: a modified line is pushed and stays valid and clean (the caching-inhibited read row of
  table 3-6). Write: pushed if modified, then invalidated. Miss or clean: no bus traffic.
- `sn_ack` comes after the push's last beat is taken (MC_LINE_WR acknowledged, eight
  `m_wnext`), or a cycle after the decision when nothing is pushed.
- With HID0[DCE] = 0 nothing is snooped (3.2.3.2: "snooping is not performed when the data
  cache is disabled"). The RSV_TAG page (where invalid lines point) is never a hit.
- `SNOOP = 0`: `sn_ack` answers a cycle after `sn_req`, nothing is looked up, `res_kill` is 0.
- New verification outputs on `ppc_cpu`: `t_dbe`, `t_dwdata`, `t_drsv`, `t_dcnd`. New core
  ports: `d_resv`, `d_cond` (lwarx / stwcx. with `d_req`), `res_kill` (input).

### How it works

The 603e gives a snoop the tag port first and retries it (ARTRY) when it meets a fill, a
cast-out or a tag write (3.6.3, 3.6.8). Here:

- **SN_W** waits until the cache has nothing in flight: `quiet` (no DQ decision, miss, fill,
  cast-out copy, zeroing, write-back, single access, store buffer or tag write) and no sweep.
  It does not hold up the CPU while it waits.
- **SN_A** takes the read port for the snoop's set, as the cast-out copy does (RD_CP, the copy
  first). **SN_B** takes it again, so no CPU lookup runs while the snoop decides. In SN_B the
  set's tags are compared and registered (`sn_hv`), and the cache must still be quiet: the
  lookup that ran during SN_A may have left a store-buffer write, a miss or a single access.
  If it did, SN_B repeats (still holding the port, so no new work can start) until it is.
- **SN_C** decides through DQ's own outputs. DQ's address and tags were loaded with the
  snoop's in SN_B, when no lookup ran, and DQ is free. A hit uses dcbst's (read) or dcbf's
  (write) actions: `do_cp`, `do_tw`, `d_st` with the dirty bit cleared. The state is read on a
  second MLAB port (`st_sn = stm[dq_set]`). Nothing writes the set's state or tags between SN_B
  and SN_C.
- **SN_WB** waits for the push's last beat. Nothing else can start a fill, copy or write-back
  while it is out, so the next write-back is the snoop's.
- Stores are not posted in SN_A-SN_C: a posted store's lookup must come the next cycle and find
  what `base_ok` saw. The flash invalidate does not start in SN_A-SN_C either.

Every CPU access looked up before SN_A is in the push; every later one sees the result. The
CPU loses the two read-port cycles if it had a request then, and posting for three.

### Reservations

603e 2.3.6.1: a reservation is cleared by "an attempt by some other device to modify a location
in the reservation granularity (32 bytes)". I built:

- `rsv_q`: an lwarx was taken last edge (its line is then in `r_a`); `rv_pa` the latest lwarx's
  line.
- During a write snoop (`sn_kw`: from the cycle after `sn_req` is taken to the cycle before
  `sn_ack`), the reservation is lost (`rv_kill`) if `rv_pa` is the snooped line, or if an lwarx
  to that line is taken. It stays lost until the next lwarx.
- `res_kill = rv_kill || (rsv_q && sn_kw && r_a line == sn_addr)`. The combinational term is
  needed: an lwarx taken the cycle before SN_A is looked up before the snoop, and a stwcx.
  right behind it in EX would otherwise succeed through MEM's bypass, with its store parked
  until after the snoop.
- In `ppc_core`, `res_in` is gated by `res_kill`, so the stwcx.'s store and its CR0 agree.

An lwarx taken during the window to the same line is cleared even if its own lookup came after
the snoop. That is conservative: the write can be ordered after that lwarx.

### Verification

**D-cache bench** (`tb_dcache.cpp`, arg 6 = 1 reads, 2 reads and writes, `SNOOP_HOSTILE=1`):
- A DMA master snoops at random gaps (40% back to back), mostly straight after a store, an
  allocation or a cast-out of the line, and sometimes the reservation's line.
- At `sn_ack` it reads memory. Each byte must be the line's value after every request answered
  by the cycle `sn_req` rose, or a value a request unanswered then (or made before `sn_ack`)
  gave it.
- A write snoop is followed by the master writing some words of the line.
- `res_kill` is checked every cycle against a model of the rule.
- Write snoops of a clean line are started 0-4 cycles before a flash invalidate. Some lwarx are
  made, as inhibited loads, to a write snoop's line inside its window.
- Without arg 6, the output is byte-identical to the baseline RTL and bench on seeds 1-8.

**Subsystem bench** (`tb_ppc_core.cpp` with `--snoop 1|2`):
- A shadow of memory takes every store request in order, and the same bracketed check is made
  on every snoop.
- After a write snoop the master writes the line. In real-mode programs, a data line untouched
  for 400 cycles gets new words, which the model's memory takes too. Otherwise the line is
  written as the answered requests left it.
- A stwcx. that stores while the rule says the reservation is lost fails the seed.
- A stwcx. that fails where the model would succeed is accepted only if the rule cleared that
  reservation since its lwarx, and the model's reservation is then cleared.
- Lines ever accessed cache-inhibited are not checked: an inhibited access to a resident line
  is boundedly undefined (3.6.4.1), and the generator maps frames with random WIMG.

### Results

**Regression with `sn_req` tied low.** The R70/R78/R86 set (`tc_reg2.sh`), with `SNOOP = 1` and
the harness tying the port off, is identical line for line to `tc_r11_reg.out` (R86's final
RTL): every cycle total, unit bench and board P line. Board harness in simulation: P00 000011DA
uncached, 00001E4E cached, P01 00000661, P09 0CA977DF. The D-cache bench without snoops is
byte-identical to the baseline on seeds 1-8.

**D-cache bench, snoops on** (100,000 ops a seed, read and write snoops):
- 96 of 96 runs pass: latencies 1-1, 1-4, 4-9, 12-15, 20-40, 40-80, with and without
  `SNOOP_HOSTILE`, 8 seeds each.
- A seed at 12-15 makes about 14,000 read and 4,700 write snoops, about 40% back to back. About
  400 come within 3 cycles of a store to the line, 2,100 within 24 of its allocation, 440
  within 24 of its cast-out, and 3,100 during a miss to another line. About 470 start 0-4
  cycles before a flash invalidate, 2,400 lwarx are made to a write snoop's line in its window,
  and about 1,000 reservations are cleared.
- Snoop latency is 50-90 cycles on average, because this bench keeps the cache busy.

**Subsystem bench, `--snoop 2`:**
- All pass: 100 seeds at wait 80, 100 at wait 6, 60 with `--loops` at wait 20.
- Seeds 60001-60100 at wait 80: 65,313 read and 48,323 write snoops; mean 14.8 cycles, worst
  994; 3.2 M bytes checked; 6,697 DMA writes that change data; 87 stwcx. failed by a snooped
  write.

**CPU cost of a snoop** (200 seeds, `--snoop 1` against none, same programs):
- +96,479 cycles for 71,851 snoops at wait 4, 1.34 cycles a snoop.
- +143,513 for 211,173 at wait 80, 0.68 cycles a snoop.

VF3's p99 frame moves about 169 KB (R80), 5,280 lines, which comes to about 7,100 cycles, about
0.4% of a 100 MHz frame.

**Planted faults:**
- D-cache bench, 14 of 14 caught:
  - no push;
  - ack as the push starts;
  - ack after the first beat;
  - store buffer not waited for;
  - no invalidate;
  - reservation not cleared;
  - an lwarx in the window keeps its reservation;
  - the combinational clear a cycle late;
  - posting while the port is held;
  - one-cycle port hold;
  - no recheck in SN_B;
  - `tw_last` ignored;
  - the RSV page looked up;
  - the sweep starting during the decision.
- Subsystem (`subs` in sim/cpu/faults.py), 7 of 9 caught:
  - caught: the core ignoring `res_kill`, reservation not cleared, no push, early ack, no
    invalidate, posting while the port is held, one-cycle hold;
  - missed: the late combinational clear and the store-buffer race. Both need a one-cycle
    alignment the random programs rarely make, and the D-cache bench catches both.

**Fits, seed 1 each** (same tree, R86's release harness form):

| Build | Slow 100C | Slow -40C | Hold | ALM |
|---|---|---|---|---|
| CPU spike, 9.09 ns (ppc_core) | 108.27 MHz | 108.68 MHz | - | 11,681 (R86 seed 1: 107.71 / 108.83 MHz, 11,691) |
| Cached harness, `SNOOP = 0` | -0.295 ns (TNS -1.755) | -0.628 ns | +0.099 ns worst | 19,137; ppc_cpu 12,350.5; ppc_dcache 842.2 |
| Cached harness, snoop driven | -0.610 ns (TNS -10.444) | -0.525 ns | +0.068 ns worst | 19,213; ppc_cpu 12,415.5; ppc_dcache 949.8 |

- **Area.** The snoop build drives the port from a 16-bit LFSR snooper in the harness (fit only,
  never on the board). The snoop costs about +108 ALM in ppc_dcache and +65 in ppc_cpu, since
  ppc_core moved by -49 (fitter noise). M10K is unchanged at 159; the state's second read port
  is MLAB.
- **Timing.** The snoop build's worst paths are not snoop paths. They are `d_valid` →
  `wb_result` (R86's split merge), `d_rdy_q` → the fetch queue head, the queue's `cnt` →
  `qf_m`, and `op_x` → `r_post`. Only the last passes through logic I changed (`base_ok` gains
  `!sn_np`). R86 measured ±0.4 ns between seeds on these paths. One seed each does not show a
  real loss or gain; a three-seed batch would.

### Wrong turns

- **Designs I did not build.**
  - Injecting the snoop into R or X as a dcbst/dcbf: it needs a mux on `d_addr` → `r_a`, a
    path that failed in R86.
  - Holding the CPU off with `d_rdy`: it adds a term to the `x_v` → `d_rdy_q` path, and costs
    the CPU every cycle the snoop waits.
  - A separate engine with its own tag compare and decisions: the second tag read and the
    state's read-modify-write race with the lookup stage.

  The two-cycle port hold reuses the copy's existing RD_CP path and DQ's outputs instead.
- **Bench, read snoops.** At first the reference did not make a read-snooped line clean. A later
  dcbi of that line then threw away data the RTL had already pushed. The reference now cleans
  the line as the snoop starts, and the CPU side keeps stores, dcbz and dcbi off the line until
  `sn_ack`.
- **Bench, expected value.** A DMA read's expected value was the cache's copy whenever the line
  was resident. After an inhibited store to a resident line (603e 3.6.4.1's boundedly undefined
  case) a clean copy differs from memory, and the snoop correctly does not push it. The
  expected value is now the copy only if modified. Inhibited accesses to a read-snooped line
  inside its window had the same problem, and are now kept off it.
- **Bench, reservations.** I took a failing stwcx. for a core reservation bug and instrumented
  the core. It was the bench: the first record of that stwcx. was a translation replay that
  ended in a DTLB store miss, an exception in its place, and I had checked its CR0. lwarx
  retirements were miscounted the same way. Both now count only records the model completes
  (probed on a copy of the model).
- **Fault script.** My first patch to sim/cache/faults.py failed silently: its anchor was not
  the end of the list. The run that followed was the old list; I stopped it.
- **Redundant logic, found by equivalent mutants.** `!s_v` in both snoop conditions and the
  `sn_rdv` register were redundant, so I removed them: a pending state write lands before SN_C
  reads it, and `quiet` already means the copy does not own the port. "No wait in SN_W" is also
  equivalent apart from performance (SN_B rechecks), so it is not in the fault list.
- **Bench, flash invalidate.** `s_sweep` was missed until the bench started the snoop 0-4
  cycles before the flash invalidate. Started with it, the sweep always began first.
- **Faults list.** Four of sim/cache/faults.py's older faults no longer apply (`d_stale_tag`,
  `d_stale_dq`, `d_byp_load`, `d_byp_dce`), because R86 restructured `blk` and the bypass. That
  was already so before this work.

### Open

- **The MPC105's snoop transaction type for a PCI read is not known here.** A global
  non-inhibited read makes the 603e invalidate as well as push (3.6, table 3-6). The interface
  keeps the line valid and clean (the caching-inhibited row), as requested. Both are coherent;
  only the hit rate afterwards differs. The MPC105 manual would settle it.
- **The I-cache is not snooped** (603e 3.1). A DMA write into code needs icbi, as on the chip.
- **Fits:** a three-seed timing batch of the cached form, and the system build with the bus
  driving the port.

## R91: integer loads with update write rA from step 0's EA (2026-11-01)

VF3's crash at about 21 s (R91) was `lbzux r4,r6,r4` at 0x4D040. The decoder cracked the integer
update loads into step 0 (load rD) and step 1 (`OP_ADD rA = rA + rB`, or `rA + d`). Step 1 read rB
again. When rD = rB, step 0 had already written rD, so step 1 stalled on its own load and then added the
loaded byte. PEM 8 (lbzux) gives `EA <- (rA) + (rB); rD <- MEM(EA, 1); rA <- EA`. PEM 4.2.3.2 names only
rA = 0 and rA = rD as invalid forms, so rD = rB is valid.

### The fix (ppc_core.sv, `id_xuea`)

Step 1 of lbzu, lhzu, lhau, lwzu and their x forms now takes rA's value from step 0's EA, never from the
register file:
- **x.** EX's sum as step 0 leaves EX comes in through the existing `x_ex` forward (`id_xuea &&
  ex_valid`). This is the path an FP double's step 1 already used.
- **y.** The immediate 0. `id_xz` forces the base to 0 and `id_yreg` is cleared.
- **No wait.** Step 1 reads no register, so the load-use wait is masked for it. Without the mask it
  would wait on its own load and lose the EA, since step 0 would have left EX.
- **Recognition.** `id_xuea = id_s1 && crack == 1 && !fdx && !fpw && !ifault`, all from predecode flops
  already in the queue. It adds no register.
- **Why step 0 is always in EX then.** An add makes no access, so step 1 never resumes from a replay.
  When ID holds step 1, EX holds its own step 0 or nothing, and with the wait masked it is never nothing.
  Two Verilator checks back this up: step 1 leaves ID only behind its own step 0 in EX, and `id_xuea`
  agrees with the raw word.
- **The decoder** now describes step 1 as `a_zero`, immediate 0. The core's datapath takes its operand
  selects from the predecode, not from these fields.
- **Exceptions.** A DSI, DTLB miss, alignment exception or replay on step 0 kills step 1 behind it, so rA
  is untouched, as before.
- **The misaligned split (R86).** EX holds step 0 for both cycles and `r_sum` stays the EA (the + 4 is
  only in `ea_x`), so step 1 gets the EA.
- **Timing.** The only change is that an update load with rD = rB no longer waits for its own load.
- **The invalid form rA = rD.** rA now ends as the EA, as in Supermodel (`ppc_lbzux`) and MAME
  (`updateaddr`). Before, it was the loaded value + rB.
- **Build 5.** The same change, as `sm3-builds/fix_lux/build5.patch`, applies to build 5's CPU (ppc_core
  md5 6c015e30 -> 61f0ea75). With it the system bench can resume from a checkpoint.

### The other multi-uop and update forms, checked for the same hazard

| Instruction | Uops | Verdict |
|---|---|---|
| lbzu lhzu lhau lwzu (D) | load; rA = rA + d | Safe in valid forms (rA != rD), since step 1 read only rA. Now from the EA anyway. |
| lbzux lhzux lhaux lwzux | load; rA = rA + rB | **The bug** when rD = rB. Fixed. |
| stbu sthu stwu (D, X) | one uop, rA written in it | Safe: rS, rA and rB are read once in ID. rS = rA stores the old rA. The split's first cycle writes no GPR (R86). |
| lfsu lfsux | load; rA = rA + d / rB | Safe: frD is an FPR, so rA and rB are unchanged when step 1 reads them. |
| lfdu lfdux | high word; low word at EA + 4 (held); rA = rA + d / rB | Safe for the same reason. |
| stfsu stfsux | one uop | Safe. |
| stfdu stfdux | high word, rA = EA; low word at the held EA + 4 | Safe: step 1 reads neither rA nor rB. |
| stmw | one store a word | Safe: no GPR is written. |
| lmw | rA + d + 4n read each step | Valid forms are safe. **Invalid form: rA in rD..r31** (PEM 8: "invalid"). See below. |
| lwarx, stwcx., the byte-reversed forms, cache ops | one uop | Safe. |

**lmw with rA in the range is not fixed.** Each step reads rA again. Once rA has been loaded, the later
words come from the new base. Supermodel and MAME both compute the EA once, from the original rA. MAME
also skips writing rA. The PEM calls the form invalid (boundedly undefined), so by the rule that emulator
behaviour is followed where the manual says undefined, the core should keep the original base. I have
not changed it:
- Compilers do not emit this form.
- A replay part way through an lmw re-reads rA from the file, so the fix needs a held base and touches
  `imm4_q`'s path.

It is a study item.

**R91's note on the FP update loads is not quite right.** It says their last step "writes rA from the
held address". It writes rA = rA + d / rB, read again, and that is safe only because the earlier steps
write an FPR. Only stfdu's second word uses the held EA.

### The ISS and why the regression missed it

- **The ISS** (`ppc_iss.h`) is right. It reads `a = gpr[ra]`, `b = gpr[rb]` and `s = gpr[rd]` before any
  write, and writes rD, then rA = EA. lmw computes the EA once.
- **The generator** never made rD = rB:
  - Update loads always went through base r6.
  - X forms used r3 as rB, and `dst()` leaves out r1, r2, r3, r6 and r9.
  - The aligned path in `mem()` had no X-form update loads at all; only `mis_mem()` did, with rD forced
    off r3 and r6.
  - So no program ever had rD = rB, rA = rB, rS = rA or rS = rB on an update form.
- **The new `--alias` option** (tb_ppc_core.cpp `alias_mem`, and the FP update forms in `fp_mem`) gives
  a third of the memory ops these shapes:
  - loads with rD = rB (rB random, rA = EA - rB) and rA = rB (EA = 2 x rA);
  - stores with rS = rB, rS = rA (D and X) and rS = rA = rB;
  - two rD = rB loads back to back through r6, then an add of both;
  - D-form updates back to back, then an X form from the updated r6;
  - a quarter of them misaligned (R86's split).
- **The reproducer.** Each `--alias` program ends with it for all eight integer update loads, and the
  bench checks rD and rA in both the RTL and the model against fixed values:
  - Real mode runs the literal case: `lis r4,1; li r6,0x10; lbzux r4,r6,r4` with the byte at
    0x10010 = 0x55, giving r4 = 0x55 and r6 = 0x10010.
  - Translated programs go through 8 seg 6 pages of their own (`Layout::vrep`, PP = 2 under both
    VSIDs), each after a tlbie, so step 0 takes a DTLB miss and the handler's rfi runs the instruction
    again.
  - In the core bench, a translated run shows the lbzux recorded first as the miss, then the 0x1100
    handler, then the completed instruction with r4 = 00000055 and r6 = 60350010.
- **Unchanged without `--alias`.** Default programs are identical, because the option draws no random
  numbers when it is off.

### Verification

- **Reproducer**, programs of the reproducer alone (length 1), modes 0-3, on the core bench (no cache)
  and the subsystem bench (caches): all pass. Pre-fix RTL (`sn_reg_rtl/cpu`): every mode fails at the
  lbzux, on both benches.
- **Regression** (`ue_reg.sh`: `tc_reg2.sh`'s R70/R78/R86 set with the live sim, plus 10 `--alias`
  runs): every run of the old set identical to D-41's `sn_reg1.out` line for line: cycle totals, BTB
  counters, unit benches. Board harness P00 000011DA / 00001E4E, P01 00000661, P09 0CA977DF, unchanged;
  the harness program has no update load with rD = rB, so its signatures could not move. The 10 alias
  runs (waits 1 / 4 / 9, `--rdy 25`, `--dlat 1`, `--mis`, `--loops`, the subsystem at 80 and with
  `--mis`, 1,500 seeds in all) pass. Per 150-seed core run they complete about 3,100 loads with rD = rB,
  2,000 with rA = rB, 860 stores with rS = rB, 860 with rS = rA and 1,700 stores with rA = rB, and check
  2,400 reproducer values. The same runs on the pre-fix RTL fail in their first seed.
- **Build 5** (`fix_lux/rtl_cpu_b5`, core bench): the reproducer in modes 0-3 and `--alias` at waits 1 /
  4 / 9 and with `--mis` pass; unpatched build 5 fails all of them. w1 / w4 / w9 / `--rdy 25` / `--dlat 1`
  / `--loops` / `--mis`, 60 seeds each, give the unpatched cycle totals exactly (w4 624,286, w9
  1,179,179). Not run there: the subsystem bench, whose live build needs D-41's snoop ports.
- **Quartus 17** `--analyze_file` of the changed files and a full analysis and elaboration of ppc_cpu:
  clean, live and build 5.
- **Planted faults** (sim/cpu/faults.py, benches `corea` / `suba`): 5 of 5 caught, each by corea (and the bug by suba too):
  - `upd_rb_reread` (the bug put back, its raw-word check removed with it): corea and suba, both by the
    state compare at an lhaux with rD = rB;
  - `upd_luse` (step 1 waits on its own load, so its EA has left EX): corea, by the new check that
    step 1 leaves ID only behind its step 0;
  - `upd_imm` (y left at the form's immediate) and `upd_xex` (x not from EX's sum): the state compare,
    first at the 603e-style TLB miss handler's own `lwzu r1,8(r2)`;
  - `upd_yreg` (y from rB again): the state compare.
- **Fits** (CPU spike, 9.09 ns, seed 1, ppc_core): 

  | RTL | Slow 100C | Slow -40C | ALM | Registers |
  |---|---|---|---|---|
  | D-41 (`sn_spike_s1`) | 108.27 MHz | 108.68 MHz | 11,681 | 10,189 |
  | first form (predecode bit, `ex_ue0`) | 107.79 MHz | 108.06 MHz | 11,574 | 10,238 |
  | final form (no new register) | 99.14 MHz | 99.67 MHz | 11,610 | 10,187 |

  The final form's worst path is `op_y` -> `ea_x` (Add19) -> the DBAT compare `db_ok` -> `m_rpq`, EX's
  translation and replay decision, with 1.6 ns of routing on its first hop. It goes through none of the
  logic I changed (that is all in ID). The next ones are `op_y` -> the fetch queue's `cnt` and
  `redir_a_q` -> the BTB, also untouched. So I read it as placement, but one seed does not show that,
  and a 9 MHz swing is more than the +-0.4 ns R86 measured. A three-seed batch of the final form should
  settle it before a release build.

### Wrong turns

- **My first version added state.** It used a predecode bit (`pd.uea`, PDW 75 -> 76, one more bit per
  queue entry), a flop `ex_ue0` (EX holds that step 0), and y from `fimm_q` set to 0 as step 0 issued.
  It passed the full regression identical to D-41 and gave a CPU spike of 107.79 / 108.06 MHz, 11,574
  ALM, 10,238 registers (D-41: 108.27 / 108.68 MHz, 11,681 ALM, 10,189 registers). The system bench then
  needed the fix as a patch to build 5 with no new registers, so its checkpoint keeps the same register
  set. All three can go: the kind comes from existing predecode flops, EX's being valid is enough (only
  the instruction's own step 0 can be ahead of its step 1), and the immediate 0 is a constant. I
  changed the live tree to that form so build 5 and live carry the same logic.
- **My first reproducer used the ordinary seg 6 data pages.** It took a DSI on the store: those pages
  have random PP and 1 in 10 have no PTE. Hence `vrep`.
- **A stale build.** I rebuilt only the core bench after moving the reproducer to `vrep`. The subsystem
  bench then failed in translated modes with the old addresses, and I took that for a cache problem
  for a few minutes.
- **A generator bug of mine.** In the back-to-back case, `r2 = r1 + 1` when the two picks matched. From
  r1 = 0 that gives r1, a base register. A clobbered r1 then sent a later dcbst to a non-word-aligned
  EA, and R86's Verilator check "the shared merge against ld_align for an unsplit load" fired. That
  check does not leave out cache ops, so it is a false positive for a legal dcbst / dcbf / dcbt at any
  EA in the block. The CPU was not at fault. I fixed the generator. The check still needs `!cop` (MEM
  does not carry the kind today); I noted it as open rather than change R86's check here.

### Open

- lmw with rA in the range (above).
- R86's merge check and cache ops at a non-word-aligned EA (above).
- Board: VF3 past 21 s with the fix. Desk-only so far.
