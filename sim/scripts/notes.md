# 53C810 SCRIPTS (M0-F) — notes

My working notes for the standalone 53C810: what the games ask of it, what each emulator
does, where they disagree, what the RTL is built to, and every wrong turn. Study sections:
§5.2 (the bridge and the 53C810), §4's memory map (C0000000 on Step 1.x, C1000000 for
Lost World), M0-F, D-14 (SCRIPTS behind a compile-time parameter), D-19 / D-25 (Supermodel
over MAME where they differ), D-27 ("everything else" 3,500 ALM; I/O, i8251, NVRAM, RTC and
SCRIPTS estimated at 500–1,500), R71 / R73 (late acknowledges matter).

The authority for the chip is the LSI53C810A Technical Manual, Version 2.1, March 2001
(DB14-000168-00), the same document Supermodel cites. My copy is in
`sm3-builds/manuals/lsi810a/` (docs.broadcom.com/doc/12353257). Where an emulator
disagrees with it, I follow the manual and record the difference here.

## Files

| File | What |
|---|---|
| `rtl/io/m3_scripts.sv` | The block: PCI configuration, register window, SCRIPTS fetch and decode (MMOV, INT), memory move through a 16-word MLAB FIFO, on the CPU's memory-port protocol |
| `sim/scripts/m3s_ref.h` | Reference model in three forms: HW (the manual, what the RTL is built to), SM (Supermodel), MAME |
| `sim/scripts/m3s_vec.h` | Vector reader and the bench memory (unwritten words read as a hash of their address) |
| `sim/scripts/cap2vec.py` | Capture log → replay vectors |
| `sim/scripts/tb_ref.cpp` | Replays a capture through the reference, compares every read, interrupt level, instruction and move |
| `sim/scripts/tb_m3_scripts.cpp` | Verilator bench: RTL against the HW reference on the captures and synthetic programs, hostile memory |
| `sim/scripts/faults.py` | Planted faults |
| `quartus/m3_scripts/` | Standalone fit: wrapper, sdc, `analyze.sh`, `constfold.sh` + `cf_top.sv`, `fit.sh`, `batch.sh`, `variants.sh`, `summary.sh` |
| `tools/mame-census/supermodel-scripts-log.patch`, `mame0289-scripts-log.patch` | The capture instrumentation (README, "53C810 capture") |

Make targets: `scripts_vec`, `test_scripts_ref`, `lint_scripts`, `test_scripts`, `faults_scripts`.

## Captures (`sm3-builds/scripts_cap/`, git-ignored)

The logs hold RAM contents (the SCRIPTS words), so they and the vectors stay in `sm3-builds/`.

| Capture | Emulator | Length | Chains | Steps (= moves) | Bytes moved |
|---|---|---|---|---|---|
| `sm_vf3` | Supermodel 036af1c + patch, attract | 4,000 frames | 8,983 | 109,433 | 263,438,852 |
| `sm_bassdx` | Supermodel, Sega Bass Fishing Deluxe attract | 6,000 frames | 57,720 | 79,708 | 116,454,240 |
| `sm_vf3tb` | Supermodel, VF3 Team Battle attract | 4,000 frames | 8,973 | 143,515 | 290,680,508 |
| `mame_vf3` | MAME 0.289 + census + patch, attract | 70 s | 5,729 | 63,741 | 159,094,548 |
| `mame_vf3_play` | MAME, `play.lua` (coin at frame 5,200, fights) | 200 s | 15,964 | 219,104 | 504,927,716 |

A chain is one DSP write and the steps that follow it. Get Bass itself (`getbass`) is not on
disk; I captured its parent `bassdx`, which Supermodel runs (MAME stalls on its I/O board).

Per-frame load, the frames with any move:

| Capture | Bytes a frame: median / p99 / max | Moves a frame: median / max | Lengths |
|---|---|---|---|
| VF3 (SM) | 85,848 / 174,768 / 386,016 | 44 / 133 | 4 B – 256 KiB; 73% in 1–2 KiB |
| Bass (SM) | 1,784 / 174,772 / 262,144 | 8 / 133 | 33,605 of the 79,708 are 4-byte moves |
| VF3 TB (SM) | 90,156 / 174,768 / 386,016 | 47 / 134 | as VF3 |
| VF3 play (MAME) | 46,992 / 174,768 / 262,144 | 18 / 102 | as VF3 |

Destinations are all Real3D: 98 (polygon RAM, 61% of VF3's bytes), 8E (display list), 94
(texture FIFO), 8C (culling RAM), 90 (VROM texture port, 4 bytes), 9C (buffer config, 12
bytes), and for Bass 88 (the command port). Sources are work RAM. **Every move in every
capture is word-aligned at both ends with a length that is a multiple of 4**, and the
driver makes that so (below).

## What the games do

VF3, VF3 TB and Bass Fishing run one Sega driver: the same code at VF3 0x61B9C–0x61EB8, VF3 TB
0x672B4–0x6759C and Bass 0x886E4–0x88A00, the register base the only difference. I read it with Supermodel's
`ppcd` from the code windows the patch saves.

1. **Boot:** DMODE = 0xC1 (burst 16, **MAN set**), DIEN = 0x08 (SSI only). PCI: command 0x0006
   (memory space, bus master), latency timer 0xFF, BAR1 = C0000000 (VF3, VF3 TB) or
   F9000000 (Bass).
2. **Program:** built in RAM with `stwbrx` (the chip is little-endian): `k` times
   [`C0000000 | words*4` (MMOV), source, destination], then `98080000 00000001`
   (INT, jump-if-true, no test, vector 1). The count is formed by
   `rlwimi r2,r5,2,0x00FFFFFC`, a word count times four, so a sub-word move cannot occur.
3. **Start** (0x61C4C): read ISTAT; if DIP, read DSTAT to clear it. Write DSP with one
   `stwbrx` (a 32-bit access). Read DCNTL, OR 0x14 (SSM | STD), write it. Spin 0x20
   timebase ticks (about 1.9 µs).
4. **ISR** (0x61E14, called from the external-interrupt dispatcher at 0x19A4 for every
   interrupt, hence the many ISTAT reads of 0): read ISTAT; return unless DIP. Read DSTAT;
   return unless SSI. Read DSP (`lwbrx`), compare with the last start + 12; on a mismatch
   count an error and resynchronise. Read the word at DSP **as data**: if it is 0x98080000
   (the INT), set the done flag and return; otherwise DCNTL |= 0x14, spin, return.

So the program is a list of memory moves, each run by one single step from the ISR, and the
INT is a terminator the CPU reads. It is never executed.

**Distinct opcodes executed, over 615,501 steps in five captures: one, DCMD 0xC0 (MMOV,
no-flush clear).** Present but never executed: INT 0x98080000. §5.2's list (block move,
JUMP, CALL, RETURN, register read/write, INT) is a superset; nothing outside it occurs, and
of it only the memory move is used. No SCSI phase, no block move in the SCSI sense, no
register read/write instruction, no transfer of control.

Registers the CPU touches: ISTAT (R, and W only behind an INTF test that never passes),
DSTAT (R), DSP (R/W, 32-bit), DMODE (W), DIEN (R/W), DCNTL (R/W). Accesses are 8-bit except
DSP. MAME's 53C810 aborts on any other register, and never did.

## VF3's single-step quirk — resolved

Supermodel's note says VF3 writes DSP, "triggering automatic code execution because
MAN=0", then sets single step and expects an interrupt; and that Scud Race needs the
auto-start. **The captures do not show it.** VF3 writes DMODE = 0xC1 once at boot (frame
62), so MAN is set, and in both emulators no DSP write ever starts SCRIPTS (0 automatic
starts in all five captures). Every step is DCNTL = SSM | STD with MAN set, which is the
manual's own single-step procedure (5-46: "set the START DMA bit to restart execution of
SCRIPTS after a single step interrupt"). The interrupt VF3 expects is the SSI after one MMOV.

The note fits a build in which the DMODE write was lost. With MAN clear, a DSP write
free-runs the MMOV and then the INT, which halts with DSP still at the INT (Supermodel
leaves it there). The following DCNTL step then runs the INT a second time, and the ISR
finds DSP equal to the INT's address, which it expects. That is the "invalid instructions
hit each time" and the 2016 change 8f87bb1 ("do not increment DSP when executing interrupt
instruction. Fixes invalid polygons in Virtua Fighter 3"). Even the oldest version in the repository (the 2011 import) stores DMODE,
so I cannot say from history why MAN was clear. I did not build old Supermodel to find out.
Scud Race is Step 1.5 and not captured.

**What the hardware form does, then:** nothing special. MAN is honoured. A DCNTL write
with STD runs from DSP; with SSM it stops after one instruction with SSI, DIP and the
interrupt (DIEN.SSI), and DSP points at the next instruction (+12 after an MMOV). A DSTAT
read clears SSI, DIP and the interrupt. STD clears at the interrupt. After an INT or an
illegal instruction the processor is halted until DSP is written (manual 6-31). The INT is
never reached in these games.

## Where MAME and Supermodel disagree

| # | What | MAME 0.289 | Supermodel 036af1c | Manual | Built | Seen by these games |
|---|---|---|---|---|---|---|
| 1 | ISTAT after a single step | 0x03 (DIP and SIP); SIP is never cleared, so 0x02 ever after | 0x01 | SSI is a DMA interrupt: DIP | 0x01 | Read; the driver tests DIP only |
| 2 | DSTAT read | 0x08: no DFE, never cleared | 0x88; clears SSI | DFE (bit 7) set when the FIFO is empty; reading clears the interrupt bits and DIP | 0x88; clears SSI, SIR, IID | Read; the driver tests SSI only |
| 3 | Interrupt release | at an ISTAT read; the line is a pulse (`m_scsi_irq_state` cleared as it is set) | at the DSTAT read | latched until the status register is read (DSTAT) | DSTAT read | ISR runs either way |
| 4 | Interrupt routing (§5.2 item 3) | ORed into the 603e's external interrupt, outside the board's IRQ enable | bit 0x100 of its IRQ controller, named NMI, outside the enable mask, driving the same external interrupt input | — | an `irq` output | **The same in effect**: unmaskable by the board, on the 603e's one external interrupt (the 603e has no NMI pin). The difference is a name |
| 5 | DCNTL.STD after the interrupt | stays set (0x14) | stays set (0x14) | "remains set until an interrupt occurs" | cleared (reads 0x10) | Read; the driver ORs 0x14, so the write is the same |
| 6 | DCNTL decode | steps on SSM **or** STD (`dcntl & 0x14`); never free-runs from DCNTL | steps on both; runs on STD alone | STD starts; SSM alone does nothing | as the manual | No: always 0x14 |
| 7 | DSP after INT | past it | at it | past it ("points to the next instruction") | past it | No: INT never runs |
| 8 | A step while halted | refused | runs | DSP must be written first | refused | No |
| 9 | DSP write with MAN clear | runs on, ignoring SSM | runs on, ignoring SSM | starts; SSM stops it after one | honours SSM | No: MAN is set |
| 10 | Memory move | handed to the Real3D DMA callback, Real3D destinations only, others dropped; DSPS, TEMP, DBC untouched | copies any memory, words then bytes; DSPS, TEMP end at source and destination + n, DBC 0 | a copy; the byte count decrements to zero; DSPS a holding register | a copy; Supermodel's end values | Not read |
| 11 | Register window | mapped at a fixed address per machine | C0 only once the BAR says C0; C1 and F9 always | BAR1 | BAR1 and command.MEM, out to the bridge | VF3 C0000000, Bass F9000000 |
| 12 | Step timing | about 40 µs between steps (the ISR waits for the next timeslice) | none (the ISR runs inside the DCNTL write) | — | the move's own time | neither is hardware timing |

Both emulators agree on what the games depend on: the instruction stream, the moves, DSP
after each step, the DIP and SSI bits they test. All five captures replay in either
emulator's form with identical instructions and moves.

## The reference (`m3s_ref.h`)

Three forms on one model, so each difference above is a switch I can show is understood:

| Replay | SM form | MAME form | HW form (what the RTL is) |
|---|---|---|---|
| `sm_vf3`, `sm_bassdx`, `sm_vf3tb` | exact: 0 read, interrupt, instruction or move mismatches in 1,842,858 accesses | — | DCNTL reads 0x10 for 0x14 (rows 5), 332,653 reads; nothing else |
| `mame_vf3`, `mame_vf3_play` | — | exact: 0 mismatches in 7,604,482 accesses | rows 1, 2, 3, 5: ISTAT 0x01/0x00 for 0x03/0x02, DSTAT 0x88 for 0x08, the line released one read later, DCNTL 0x10 |

`make test_scripts_ref` runs all of these.

## The RTL (`rtl/io/m3_scripts.sv`)

- **SCRIPTS:** fetch two words at DSP (DSP advancing per word, as the manual says), decode,
  a third for MMOV. MMOV moves words; INT in its unconditional form halts with SIR (INT_OP,
  default on). Everything else, an INT with a carry, data or phase test or INTFLY, a zero
  count, and a move whose source, destination or count is not a word multiple raises IID
  and halts. The chip itself moves bytes at matched alignment; the Sega driver cannot ask for it.
- **The move:** DSPS and TEMP are the source and destination pointers, DBC counts the words
  still to write, `rd_left` the words still to read. A 16-word FIFO in MLAB sits between.
  With the port idle the next transaction is, in order: a line write (destination
  line-aligned, eight or more words to write and in the FIFO), a single write (destination
  not line-aligned, or under eight words left, and the FIFO not empty), a line read (source
  line-aligned, eight or more to read, the FIFO at most half full), a single read. Since the
  words to write are always those to read plus the FIFO's, one of these is always possible
  until the move is done.
- **Memory port:** `ppc_cache_pkg`'s protocol (the CPU's memory bus): single reads and
  writes, eight-beat line reads and writes, one transaction at a time, request and fields
  held until the acknowledge, beats after it. Line reads only at line-aligned addresses, so
  word 0 is the critical word. `m_req` is low for at least one cycle between transactions,
  so a rising-edge port (`m2_sdram`) sees each one.
- **Register window:** one access per `s_req`, held to a one-cycle `s_ack`; big-endian lanes;
  the chip's registers little-endian (register byte N in lane 3 − (N & 3)). ISTAT, DSTAT,
  DSP, DMODE, DIEN, DCNTL; TEMP, DBC/DCMD and DSPS read back with RD_ALL. A DSP write while
  running is ignored. DSTAT.DFE reads 1: the FIFO is empty whenever DSTAT may be read.
- **PCI configuration:** ID 0x00011000, command [2:0], latency timer, BAR1 [31:8]; `win_en`
  and `win_base` out for the bridge's decode.
- **Interrupt:** `irq` = latched (SSI & DIEN.SSI | SIR & DIEN.SIR | IID & DIEN.IID), cleared
  by the DSTAT read, masked at the pin by DCNTL.IRQD.
- **Software reset** (ISTAT.SRST): operating registers to their defaults, the FIFO emptied; a
  transaction in flight completes on the port and its data is dropped.
- **Parameters:** ENABLE (D-14; 0 leaves a window that reads 0, configuration all ones, no
  memory traffic), LINE, INT_OP, RD_ALL.

## Bench

Every CPU access goes to the RTL and the HW reference. The bench lets the RTL run to idle
before the next access, as the emulators do, then compares every read, the interrupt
line, and the memory traffic since the last access: the word addresses read and the
(address, data) words written, in order. Source data is a hash of the address, so a wrong
address shows as wrong data. The memory checks the protocol (fields held to the
acknowledge, the request dropped after it, no request while beats are due, line addresses
aligned).

Synthetic programs cover what the games never do: driver-style chains of 1–6 moves of
4 B–16 KiB at random word alignments; free runs started by a DSP write with MAN clear,
ending in INT, with and without SSM; illegal instructions (near misses of MMOV and INT and
every other class); zero counts and misaligned moves, then the halt released by a DSP
write; a software reset in mid-move; byte-wide DSP writes in random order; DIEN masking;
IRQD; RD_ALL reads.

| Profile | Captures | Synthetic | Errors |
|---|---|---|---|
| quick: acknowledge 0–2 | all five, every chain (97,369 chains, 615,501 moves) | 2,000 | 0 |
| acknowledge 0–400, BUSY 30% | every 40th chain | 1,000 | 0 |
| acknowledge 100–400, BUSY 60%, beat gaps 50% | every 60th | 500 | 0 |
| acknowledge 0–3, beat gaps 90% | every 20th | 1,000 | 0 |
| acknowledge 0–400, BUSY 30%, ISTAT polled while running (DIP must stay 0) | every 40th | 1,000 | 0 |
| LINE = 0 build, acknowledge 0–400, BUSY 30% | every 40th | 1,000 | 0 |
| RD_ALL = 1 build | — | 1,500 | 0 |
| INT_OP = 0 build | every 40th (SM) | 1,500 | 0 |
| ENABLE = 0 build | window, configuration, port idle | — | 0 |

Against the captures themselves the RTL's reads differ only in DCNTL.STD (row 5) for
Supermodel's, and rows 1, 2 and 5 for MAME's.

## DMA rate

Measured on the bench with every VF3 and Bass chain, acknowledges 6–10 cycles late and
beats back to back (about what an SDRAM behind a bridge gives at 100 MHz):

| Build | Cycles a word | Rate at 100 MHz | VF3 per frame: median / p99 / max |
|---|---|---|---|
| LINE = 1 | 4.66 | 85.8 MB/s | 1.02 / 1.97 / 4.34 ms |
| LINE = 0 | 21.06 | 19.0 MB/s | 4.52 / 9.18 / **20.27 ms** |

The hardware's rate is not documented. `[EST]` A memory move on a 32-bit, 33 MHz PCI bus reads
and writes every word, so 66 MB/s is the ceiling. With burst 16, the MPC105's read
latency and the four-clock fairness delay between bursts, I put it nearer 40 MB/s. That
gives VF3's p99 frame (174,768 B) 2.6–4.4 ms and its largest (386,016 B) 5.8–9.7 ms. Line
bursts keep the block faster than that ceiling. Without them the largest frame takes longer
than a frame (17.4 ms), so LINE stays on. The rate depends on the real port's latency, which
the bridge does not have yet.

## Faults

`sim/scripts/faults.py`: each fault is a text substitution in a copy of the RTL. Each is
tried on the captures (every 400th chain, quick memory), then on 1,500 synthetic programs
behind a memory with acknowledges 0–400 cycles late and BUSY 30%. **40 of 40 caught**, 21
on the captures and 19 on the synthetic programs. Those 19 are the
paths the games never take: auto-start, INT, halts, masking, illegal instructions,
moves over 256 KiB.

| Fault | What | Caught on |
|---|---|---|
| D1 | opcode decode: MMOV ignores the reserved bits 29:25 | synthetic |
| D2 | opcode decode: MMOV taken from bits 31:30 only (load/store too) | synthetic |
| D3 | opcode decode: RETURN decoded as INT | synthetic |
| D4 | opcode decode: INT's carry test not refused | synthetic |
| D5 | opcode decode: INT taken when its jump-if-true bit is clear | synthetic |
| L1 | move length: one word fewer read than written | captures |
| L2 | move length: the read count's top byte dropped | synthetic |
| L3 | move length: a line write with four words left | captures |
| L4 | move length: a zero count not refused | synthetic |
| A1 | address increment: a line advances seven words | captures |
| A2 | address increment: always one word | captures |
| A3 | address increment: a read advances the destination | captures |
| A4 | address increment: DSP not advanced by a fetch | captures |
| A5 | address increment: line read taken at an unaligned source | synthetic |
| S1 | DSP auto-start: ignores DMODE.MAN | captures |
| S2 | DSP auto-start: never | synthetic |
| S3 | DSP auto-start: on byte 2C instead of 2F | synthetic |
| S4 | DSP auto-start: the run ignores SSM | synthetic |
| T1 | single step: never stops | captures |
| T2 | single step: SSM read from the wrong bit | captures |
| T3 | single step: STD starts a halted processor | synthetic |
| T4 | single step: STD not cleared by the interrupt | captures |
| T5 | single step: a DSP write does not release the halt | synthetic |
| C1 | status: a DSTAT read leaves SSI set | captures |
| C2 | status: a DSTAT read leaves IRQ asserted | captures |
| C3 | status: ISTAT.DIP misses SSI | captures |
| C4 | status: DSTAT's SSI and SIR swapped | captures |
| C5 | status: the interrupt ignores DIEN | synthetic |
| C6 | status: DCNTL.IRQD does not mask the pin | synthetic |
| C7 | status: INT does not set SIR | synthetic |
| C8 | status: an illegal instruction does not halt | synthetic |
| P1 | port: m_req not dropped at its acknowledge | captures |
| P2 | port: a line counted as seven beats | captures |
| P3 | port: a single write pops the FIFO before its acknowledge | captures |
| P4 | port: the move asks again before the port is idle | captures |
| P5 | port: busy falls with the last write still open (R73 bug 2) | synthetic |
| P6 | port: a fetch issued with the last beat of the one before (no idle cycle) | captures |
| G1 | config: BAR1 bits 31:24 from the wrong lane | captures |
| G2 | config: the window enabled by the I/O bit | captures |
| G3 | config: reset does not clear the register window's DMODE | synthetic |

Two earlier forms of L2 and C4 could not be caught, because they changed logic that never
mattered (wrong turns). Two misses on the way came from the bench's coverage: D2 with
uniformly random illegal words, and L2 with no move over 2.4 KiB in the synthetic set.

## Fit

Quartus 17.0, 5CSEBA6U23I7, 10 ns, every port but `clk` a virtual pin (`quartus/m3_scripts/batch.sh`),
the default build (ENABLE, LINE, INT_OP on; RD_ALL off). ALM is the block's own row
(`|m3_scripts:u_sc`), without the virtual pins' registers (about 145 ALM: the ENABLE = 0 build
is 146 in total and 1 in the block).

| Seed | ALM (block) | ALM (with virtual pins) | Registers | M10K | DSP | Fmax slow 100C | Fmax slow −40C |
|---|---|---|---|---|---|---|---|
| 1 | 260.0 | 405 | 342 | 0 | 0 | 164.3 MHz | 169.4 MHz |
| 2 | 258.5 | 403 | 342 | 0 | 0 | 159.6 MHz | 165.9 MHz |
| 3 | 258.0 | 403 | 342 | 0 | 0 | 156.4 MHz | 161.6 MHz |

All three close at both corners with over 3.6 ns to spare. The worst paths run from `rd_left`
or `cnt` through the choice of the next transaction to `m_cmd`, `m_addr` and DBC. The FIFO is
an `altdpram` in MLAB (LUT RAM), so 0 M10K. Against D-27: SCRIPTS takes about 260 of the 500–1,500
estimated for I/O, i8251, NVRAM, RTC and SCRIPTS together.

### Per technique (study 8.4), seed-1 fits

The FIFO, parameter and LINE variants were measured on the build before the adder change
(273 ALM), so their deltas are against that one.

| 8.4 technique | In m3_scripts | Measured |
|---|---|---|
| Small storage out of flip-flops | the 16 × 32 FIFO in MLAB | in flops: 447 ALM, 850 registers, against 273 / 342: **−174 ALM** |
| Per-game logic behind compile-time parameters | ENABLE (D-14) | ENABLE = 0: **1 ALM** |
| | RD_ALL off: TEMP, DBC/DCMD and DSPS not readable (no game reads them) | RD_ALL = 1: 307, so **−34 ALM** off |
| | INT_OP (INT decode, never run by these games) | INT_OP = 0: 271.5, so INT costs **1.5 ALM**; kept |
| Duplicated arithmetic built once | one adder for DSP, DSPS and TEMP on the port address | **+16 ALM** against three incrementers (275 / 259); not used |
| Idle registers shared | DSPS and TEMP are the move's pointers, DBC its write count, as the manual has them | 259 against 255.5 with two extra pointers: +3.5 ALM, within seed noise, and 60 fewer registers; kept |
| No general ALU (as planned) | none built: increments and compares only; no read/write-register ALU, no transfer-control compare, no SCSI | — |
| Redundant logic out | the end-of-move test and a live DFE (found by faults L2, C4) | 275 against 273: no change, as Quartus had removed both |
| (rate, not area) | LINE: eight-word bursts | LINE = 0: 259, so lines cost **+14 ALM** for 4.5 times the DMA rate |

Register-file, FP-pool, converter-sharing and MLAB/M10K-trade rows have nothing to act on here.

## Wrong turns

- **I first expected an auto-start quirk to model.** Supermodel's note and §5.2 said VF3
  starts SCRIPTS with the DSP write. The capture shows MAN set and every step manual. I had
  planned a special case; none is needed.
- **The capture converter attributed interrupt releases to the wrong access.** Both
  emulators log a release inside the read that causes it, before that read's own line. My
  first converter gave it to the access before. I rewrote it to give a release followed
  by a read with the same time stamp to that read.
- **Supermodel's PCI write folds dwords:** `(pciReg >> 2) & 0x3C` maps dwords 0–3 to 0 and
  4–7 to 4, so its log cannot say which dword was written. MAME's log can (BAR1 = dword 5,
  command = 1, latency = 3). The converter maps Supermodel's by value.
- **My Python edits of Supermodel's sources turned CRLF into LF**, and the first patch was
  9,736 lines of whole-file churn. I restored CRLF; the patch is 263 lines.
- **MAME 0.289 has no `first_screen()`**; the first build failed. `screen_device_enumerator`.
- **The bench carried its memory from one capture file into the next**, and Bass failed 46
  times on words VF3 had installed. A bench bug, not the RTL's.
- **DFE read the live FIFO count**, and a software reset in mid-move left words behind, so
  DSTAT read 0x00 where the manual's reset value is 0x80. The reset now empties the FIFO. Fault
  C4 then showed that DFE is never 0 at any legal DSTAT read, so it is now a constant.
- **The read data returned every lane**, and the bench compared lanes the CPU had not read.
  The bench now masks them; on the bus they do not matter.
- **Fault D2 (MMOV decoded from bits 31:30, so load/store too) slipped through twice.**
  Uniformly random illegal words almost never hit 0xE0/0xE1. With near-miss templates, 400
  programs still gave about 2.5 hits; 1,500 catches it.
- **Fault L2's second form (the read count's top bits dropped) was missed** at first: the
  synthetic moves stopped at 2.4 KiB and the sampled captures skipped VF3's 256 KiB moves.
  One synthetic move in 40 is now 64 KiB–1 MiB, and the fault runs' hang timeout went from
  3 M to 60 M cycles so a 1 MiB move behind a slow memory is not mistaken for a hang.
- **The bench's polled-ISTAT check was too strict.** It required DIP 0 for as long as the RTL
  ran. That holds only if nothing was pending at the start: a byte-wise DSP write with MAN
  clear can auto-start, stop with SSI, and be started again by STD with the SSI still
  unread. The check now applies only when no interrupt was pending. A bench bug; the RTL
  was right.
- **Hang faults made the bench time out at every following access**, and an orphaned bench
  from a fault run I had stopped ran on for six minutes. The bench now stops after 20 errors.
- **The redundant end-of-move test (fault L2 first form) could not be caught**: with the
  port idle the move always has something to issue until it is done. I removed the test;
  the fit did not change (Quartus had already removed it).
- **The shared adder lost.** I built §8.4's "arithmetic built once": DSP, DSPS and TEMP all
  advancing through one adder on the port address. Three incrementers measured 16 ALM
  smaller (seed 1: 259 against 275), because a constant increment packs into the register's
  own carry chain and the shared form adds a mux and fanout. Using DSPS and TEMP as the
  move's pointers rather than adding two (the "idle registers shared" row) measured 3.5 ALM
  *larger* than separate pointers, which is within seed noise, for 60 fewer registers. I kept the reuse.
- **Quartus 17 rejects a bare `if` generate at module scope**; it needs `generate`/`endgenerate`.
  **iverilog rejects an enum assigned through `?:`**; both are now `if`/`else`.

## Open

- The real port: which bridge port the block sits on, and its latency into Real3D memory.
  The DMA-rate table assumes acknowledges 6–10 cycles late.
- The hardware DMA rate is an estimate. Cabinet footage of a heavy VF3 scene would show
  whether a slower DMA (LINE = 0) ever drops a frame on the real board; nothing here needs it.
- Lost World's C1000000 window: not captured (not on disk); BAR1 covers it if Lost World
  writes its BAR as the others do.
