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
