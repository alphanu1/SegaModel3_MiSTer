# The Model 3 system, stage A (study D-37)

The first real top level: the 603e, the MPC105's map, board RAM and the ROMs in
SDRAM from the MRA, interrupts, the 53C810, the tile generator and video out. The
Real3D is a status-handshake stub, and there is no sound and no cabinet I/O. The pass
mark is VF3's warning and SEGA logo screens, which are tile layers (R84).

## Files

| File | What |
|---|---|
| `rtl/sys/m3_system.sv` | Everything below the framework: the bench runs it whole |
| `rtl/sys/m3_bus.sv` | Arbiter (CPU, 53C810), the Step 1.0 decode, system registers, IRQs, PCI configuration, the Real3D stub, MIDI stub |
| `rtl/sys/m3_sdr_port.sv` | The caches' protocol onto one `m2_sdram` port, both byte forms |
| `rtl/sys/m3_jtag.sv` | The Real3D JTAG chain, ten devices, serial over an MLAB |
| `rtl/sys/m3_loader.sv` | ioctl to SDRAM, the MRA's stream as it comes |
| `rtl/io/m3_io.sv` | 315-5649, inputs idle |
| `rtl/io/m3_93c46.sv` | Model 2's 93C46 with a write-busy time added |
| `Model3.sv` | The framework's `emu`; the CPU test is behind `M3_CPUTEST` |
| `mra/Virtua Fighter 3 (Japan, Revision D).mra` | Written by `sim/system/m3_mra.py`; on the board it goes in `/media/fat/_Arcade/_Model3/`, the core at `/media/fat/_Arcade/cores/Model3.rbf` |
| `sim/system/*` | Benches, the bus-trace tool, the MRA writer and packer |
| `quartus/system/*.sh` | Analyse and fit scripts |

Benches (`make -f sim/system/system.mk ...`):

| Target | Checks |
|---|---|
| `test_jtag` | `m3_jtag` replaying the 25,511 TCK/TMS/TDI writes of Supermodel's boot: 12,755 TDO reads identical |
| `test_io` | `m3_io` replaying the trace's 47,339 315-5649 accesses: every read identical (two classes handled, below) |
| `test_sdr_port` | `m3_sdr_port` on the real `m2_sdram` and the SDRAM at its pins: 200,000 random transactions, both byte forms, every critical word, with a contending reader; 0 mismatches |
| `system` | The whole system bench, `Vm3_system` |

## The oracle: a Supermodel bus trace

`sm3-builds/supermodel-sys` is a clean clone of Supermodel `036af1c` with the tile
dump patch and a bus trace (`tools/mame-census/supermodel-bus-trace.patch`,
`M3BUS_TRACE`): every CPU and 53C810 access outside work RAM, CROM reads left out
unless `M3BUS_CROM=1`, one 24-byte record each. `sim/system/bustrace.py` summarises,
dumps and compares two traces. The bench writes the same format, so the RTL's stream
of I/O accesses lines up against Supermodel's in order. Reads that repeat (polls)
are collapsed, since how often a game polls is timing.

VF3 from a fresh boot, 1,500 frames, 14.5 M records:

| Region | What VF3 does |
|---|---|
| MPC105 F8FFF0xx | 23 writes in frame 1, never read |
| PCI configuration | device 14 (53C810) only: one ID read, BAR1 = C0000000, latency timer, command |
| 315-5649 | the EEPROM bit-banged; the inputs polled each frame |
| System registers | CROM bank, IRQ enable 6E, IRQ status read, and JTAG: 12,755 TCK cycles at frames 211-232 and a few later |
| RTC | two control registers written and read back as 0 (with the battery bits) |
| Real3D | status read (FFFFFFFD / FFFFFFFF, the line-of-sight words 0), command port 1,140 writes, 9C configuration by DMA |
| 53C810 | single-stepped MMOV from frame 233: 8C, 8E, 94, 98, 9C, 90 |
| Tile generator | VRAM 764 K writes, palette 11 K, registers; IRQs acknowledged at F1180010 one bit at a time |
| MIDI | 69 writes, control 27 / 06 around each burst, 39 IRQ 0x40 |

## Where the emulators disagree, and what I built

| Point | MAME | Supermodel | Built |
|---|---|---|---|
| IRQ 0x02 | line 384 | start of line 383 | Supermodel |
| IRQ 0x0C / 0x0D | 0x0C every line | 0x0D once, at the end of blanking | Supermodel |
| Write to F0100018 | re-evaluates only | clears the bits written | Supermodel (VF3 never writes it) |
| Real3D status | every bit flips on every read | FFFFFFFD, bit 1 ping-pong, flipped at the line in tile register 08 | Supermodel |
| JTAG | a 46-bit IR and two recognised patterns | the ten devices, IEEE 1149.1 | Supermodel's devices |
| EEPROM busy after a write | never | 5 reads | the datasheet: 2 ms (`T_WP`) |
| EEPROM DO with CS low | 1 (pull-up) | 0 | MAME's 1; the game polls inputs so and ignores bit 5 |
| EEPROM data shift | falling SK, a dummy 0 first | rising SK, no dummy bit | MAME's (Model 2's copy, board-proven there); VF3 reads identical words either way |
| DIP bank (port G) | FF (all OFF) | 7F (an Ocean Hunter workaround) | by MRA (`<switches>`, index 254, the OSD's DIP Switches page): 7F unless set; Lost World FF, since it reads SW1 ON as "leave the I-cache off" (study R133); the other games 7F |
| MIDI IRQ 0x40 | 1 ms timer while control bit 5 | in blanking, at most 128 a frame | TxRDY: 320 us (one byte at 31.25 kbaud) after the last byte, while bit 5 |
| PCI read of an absent device | 0 | 0 | FFFFFFFF (the PCI master abort); VF3 reads none |

## SDRAM layout and byte order

| Byte address | What | Form |
|---|---|---|
| 0x0000000 | banked CROM, 64 MiB (8 banks of 8 MiB) | word |
| 0x4000000 | VROM, 32 MiB (stage B) | four 8 MiB blocks of four ROMs |
| 0x6000000 | fixed CROM, 2 MiB | word |
| 0x6200000 | backup RAM, 128 KiB | word |
| 0x6400000 | work RAM, 8 MiB | word |
| 0x6C00000 | tile VRAM, 1 MiB | byte |
| 0x6D00000 | free, 19 MiB (stage B: culling RAM) | |

Word form: the 32-bit value the PowerPC reads at A is the little-endian word at
SDRAM A. That is how MAME's `ROM_LOAD64_WORD_SWAP` files land with a plain 64-bit
interleave, how Supermodel holds RAM, and what the Real3D reads. Byte form: the
tile generator's VRAM is a byte image of the bus, as `m3_tilegen` expects. The fixed
window's low 6 MiB is CROM0 (MAME's `ROM_COPY`), so it is decoded, not stored.

VROM needs sixteen ROMs interleaved a 16-bit word each; an MRA interleave goes to 64
bits, so it is four blocks of four, and the Real3D's VROM reader permutes the address
(word i = {g, b, h} at {b, g, h, 00}). Stage B adds that.

`m3_mra.py pack` builds the image the MRA describes; I checked it against MAME's
region built from `ROM_START(vf3)`: the fixed window and both CROM0 banks match, and
the first words at FFF00100 are the MPC105 set-up the trace shows.

## Findings and wrong turns

- My trace patch first logged nothing from the 53C810: its moves run inside the CPU's
  write to DCNTL, and my nesting guard dropped them. The DMA is now logged by master.
- The first JTAG plan was one flip-flop shift register per device (about 470
  registers). The serial form over an MLAB matched the trace first time.
- The first I/O replay failed on 2,773 reads: Supermodel's EEPROM reports busy for five
  reads after every CS fall and reads 0 with CS low. Only the busy after a write is
  hardware behaviour (the datasheets' ready/busy status); I built that.
- The whole-system bench first ran at 78 kHz. gprof put 55% of it in the FPU's
  leading-zero tree, which Verilator turns into loops over 256-entry arrays. The bench
  uses an equivalent scan (`ppc_fpu_lzc_sim.sv`, checked against the RTL on 400,000
  inputs, both register forms) and runs at 260-300 kHz with four threads.
- VF3's boot ROM waits for timebase bit 24, 58 frames at 16.5 MHz, before it copies
  640 KiB of code from FFF10000 to RAM and jumps to 100. `--tb-fast` counts the
  timebase every clock until that loop is left; the bench's only departure from the
  board.
- The first boot run ended in the boot ROM's fatal-exception handler (FFF00000, a
  program check at 0x100): RAM held zeros where the boot copy had put its code. The
  bus took a request again in the cycle its acknowledge went out: a posted SDRAM write
  returns the bus to idle at its acknowledge, the master drops its request only the
  cycle after it sees it, so the write was done twice and the duplicate's acknowledge
  answered the CPU's next store, which was never done. A request whose acknowledge is
  out this cycle is no longer a candidate. `test_bus` (two masters against a reference)
  was written after this; with the old grant it fails (5 bad reads, 1 bad byte in
  20,000 transactions), with the new one it passes 100,000.
- VF3's BATs (logged from the retire trace): DBAT0 RAM 0-8 MiB and DBAT1 at 1 MiB are
  WIMG 1011 (write-through, coherent, guarded), DBAT2 the CROM at FF800000, DBAT3
  F0000000-FFFFFFFF WIMG 0111 (cache-inhibited). HID0 ends at C001 (both caches on).
  So the 53C810 always reads current data from SDRAM, and the cache needs no snoop.
  The cost: every store to RAM is an SDRAM write, and `m2_sdram` writes 16 bits a
  request, so a store takes the port for about 18 clocks. The board's CPI counters
  will show what that costs; a 32-bit write in one transaction is the fix if it does.
- Backup RAM fill: MAME's NVRAM default is FF, Supermodel's fresh state 0. VF3 tests
  the backup RAM word by word at boot and restores what it found. I fill 00 so the
  trace lines up with Supermodel's; the real board's power-on contents are unknown.
- Checkpoints: the bench is built `--savable`; `--save` / `--restore` carry the model,
  the SDRAM, the tile state and the cycle count. A run from frame 20 restored to frame
  30 gives the same 144,457 records, field for field, as a straight run.

## The debug overlay (hex, top left, OSD "Debug overlay")

| Row | Word 0 | Word 1 | Word 2 | Word 3 |
|---|---|---|---|---|
| 0 | CPU PC at the last vblank | IRQ state, IRQ enable, CROM bank register, ping-pong | frame | 53C810 interrupts (high half) |
| 1 | 88xxxxxx frames submitted | 8Cxxxxxx culling words (low) | 8Exxxxxx culling words (high) | 98xxxxxx polygon words |
| 2 | 94xxxxxx texture FIFO words | 90xxxxxx VROM port words | unmapped accesses (high half) | last unmapped address |
| 3 | tile register 20 | 60 | 64 | 68 |
| 4 | 6C | 08 (ping-pong line) | tile overruns (high), port A | 9C000000 / 9C000008 (low halves) |
| 5 | bytes loaded | flags: SDRAM ready, ROM loaded, loader overflow, CPU bus wait over 0.1 s | longest CPU bus wait (clocks) | CPI x 256, last second |
| 6 | cycles, last second | instructions | data memory wait | fetch waiting |
| 7 | redirects | load-use | mul/div wait | decode starved |
| 8 | first exception entry after reset: vector | SRR0 | SRR1 | MSR after entry |
| 9 | that entry's record: PC | its instruction | fatal handler: the PC before FFF00000 | SRR0 there |
| 10 | the entry before the most recent (at the fatal: the last before it): vector | SRR0 | SRR1 | instruction |
| 11 | entries counted | last DAR read by mfspr | last DSISR read by mfspr | the most recent entry's vector |

Rows 10 and 11 freeze when the boot ROM's fatal handler (FFF00000) is reached. The last
word off the table is the build tag (3A000006 = stage A, build 6).

Rows 5-7's performance numbers come from `m3_cpu_mon` (`CPI_MON`), which also prints
them once a second on the debug UART (ttyS1, 115200): eleven lines `P nn value`,
00 cycles, 01 instructions, 02 uops, 03 decode starved, 04 load-use, 05 mul/div,
06 data memory wait, 07 redirects, 08 fetch waiting, 09 retire signature, 0A seconds.
`tools/uart_capture.py` captures them.

## Results

**The warning screen, in simulation (2026-11-01).** The bench reached VF3's warning
screen at its frame 300 (Supermodel's 306-330; the bench runs 23-36 frames ahead,
`--tb-fast` and the CPU's own pace). From frame 300 to 319 the tile generator's state,
1 MiB of VRAM, the 128 KiB palette and the registers it uses (20, 40, 44, 60-6C), is
byte for byte Supermodel's dump of the stable warning screen (frames 330-930), and the
video output of the RTL for frame 310 equals the tile generator reference render
(`tg_tool ppm`) of that state on all 190,464 pixels. The run took three checkpointed
segments of 12-16 minutes each (0-110, 110-220, 220-320).

**The bus against Supermodel, region by region, to frame 320.** Identical: MPC105 (23),
PCI configuration (20), tile VRAM (24,160 writes so far), palette, tile registers up to
the per-frame ones, Real3D configuration (3 DMA writes), MIDI, 53C810 set-up. The
differences are timing, each explained: port B bit 5 read with the EEPROM's CS low (the
pull-up choice above), the IRQ status read (IRQ 0x40 comes at TxRDY here, in blanking
there), the Real3D status's ping-pong phase at one read, and one per-frame tile
register write landing either side of a frame boundary. A whole-stream compare stops at
the first interrupt that lands at a different instruction, so `bustrace.py regions`
compares each region's own stream.

**On the board (seed 1, b155c886; `/media/fat/_Arcade/cores/Model3.rbf`, loaded
through `/media/fat/_Arcade/_Model3/Virtua Fighter 3 (Japan, Revision D).mra` with
`vf3.zip` in `/media/fat/games/mame`).** VF3 boots: the warning screen, then SEGA
Presents with Insert Coin(s) and Credit 0/2, tile layers clean, about 75 s after the
load starts (the MRA load itself takes most of that). Then it stops: overlay row 0
shows PC FFF00048, the boot ROM's fatal-exception handler, where the cabinet starts the
logo wipe. CPU performance from the UART before the stop: CPI 1.80 in the attract loop
(55.5 M instructions a second at 100 MHz), load-use 36.5% of cycles, fetch waiting
20.8%, data memory wait 5.4%, decode starved 2.5%; 1.48-2.25 during the boot seconds.
R83's CPI model gives VF3 1.766.

**The 53C810's DMA against Supermodel, to frame ~480.** Word for word identical in
every destination window: 231,224 culling RAM (8C) writes, 109,596 to 8E, 344,548 to
polygon RAM, 258 to the texture FIFO, the VROM port and the three 9C configuration
words. The register-level difference is R80's: a read of DCNTL after a step gives 0x10
here (the manual: STD clears) and 0x14 in Supermodel.

**Backup RAM.** The per-frame stream diverges at our frame 282: a word at F00DC038 is
a counter VF3 adds 0x100 to every frame from boot (an operating-time count, I
think). It started 23 frames earlier here, so when the game walks its records the
values differ and so does the path. Timing, not a fault.

**Fit (release form, 100 MHz, multicorner).** Build 1, seeds 1/2/3: slow 100C -0.900 /
-0.882 / -0.259 ns, slow -40C -0.771 / -0.928 / -0.323 ns, hold positive everywhere.
Every failing path is inside `ppc_cpu`. Build 2 (the exception capture), seed 3: slow
100C -0.176 ns (98.3 MHz), slow -40C -0.152 ns (98.5 MHz), fast +4.352 / +5.290, hold
+0.148 / +0.104 / +0.098 / +0.003; the only failing paths are the CPU monitor's
retire-signature CRC, now off in the system (`SIG = 0`).

## Area (build 1, seed 3, the fitter's "ALMs needed")

| Block | ALM | Registers | M10K | DSP | §8.4 techniques used |
|---|---|---|---|---|---|
| `m3_bus` | 504 | 841 | 0 | 0 | one decode for both masters; one register-read mux; line transactions to internal targets as beats through the same single-word path; tile register shadow in MLAB |
| `m3_sdr_port` | 219 | 435 | 0 | 0 | one port for both byte forms (a lane swap); posted writes |
| `m2_sdram` (2 ports) | 371 | 860 | 0 | 0 | reused unchanged |
| `m3_jtag` | 173 | 98 | 0 | 0 | ten devices' shift registers, instructions and mode words in one MLAB word each, one device a clock (about 470 flip-flops as a plain chain) |
| `m3_io` with `m3_93c46` | 166 (115) | 216 | 0 | 0 | the EEPROM array in MLAB (Model 2's) |
| `m3_scripts` | 226 | 414 | 0 | 0 | R80's, unchanged |
| `m3_tilegen` | 512 | 911 | 54 | 0 | R77's, unchanged |
| `m3_loader` | 83 | 223 | 3 | 0 | FIFO in M10K |
| `m2_video_timing` | 15 | 27 | 0 | 0 | |
| system glue | 544 | | 0 | 0 | mostly debug: the overlay's value mux and register copies, the CPI divider, watchdogs |
| debug: `m3_cpu_mon` | 500 | 978 | 0 | 0 | `CPI_MON = 0` drops it |
| debug: `m3v_overlay` | 75 | 27 | 0 | 0 | `OVERLAY = 0` drops it |
| CPU (`ppc_cpu`) | 12,401 | 12,440 | 38 | 9 | |
| **`m3_system`** | **15,788** | 18,259 | 95 | 9 | |
| Whole design | 21,755 | | 151 | 30 | |

Against D-27's "everything else" (3,500 for tile generator, SDRAM and DDR3
controllers, PCI bridge, I/O, SCRIPTS, Real3D command and texture upload): the
stage-A blocks without debug come to about 2,300-2,400 ALM (bus 504, SDRAM port 219,
`m2_sdram` 371, JTAG 173, I/O 166, SCRIPTS 226, tile generator 512, loader 83, video
timing 15, and the non-debug part of the glue). Still to come in that budget: the DDR3
master (`m3_ddr3p`, about 625) and the Real3D command and texture-upload path. R81
projected about 2,900 for the whole row.

## The logo crash: a misaligned word load (2026-11-01)

On the board, builds 1 and 2 both show the warning and the SEGA logo, then stop where
the cabinet starts the wipe. Build 2's capture: vector FFF00300 (DSI), SRR0 00054B08,
SRR1 0000B032, MSR 00001000, reached through the game's RAM vector at 0x300, which
branches to the boot ROM's fatal handler.

0x54B08 is `lhau r3,2(r19)` in a function (0x54A88) that walks a list of halfwords;
r19 is its argument. Its callers (0x55134-0x55170) load the pointer with
`lwzx r4,r3,r9`, r9 = FFFD4FD2 and r3 = 4 x index: a word load from fixed CROM at an
address 2 mod 4. `ppc_core` forces misaligned accesses aligned (its header, R31's
still-to-do list), so it reads 2FD2FFFD where the table holds FFFD2FD2; every entry of
the table comes out as halves of two neighbours. 2FD2FFFB is in no BAT and in no PTE, so
the game's TLB-miss path ends in a DSI. The 603e does a misaligned integer access inside
a page in hardware (two bus accesses); only lmw/stmw, lwarx/stwcx., FP accesses that are
not word-aligned, dcbz to W or I pages and little-endian cases take the alignment
exception (603e manual 4.5.6). The fix is in `ppc_core`, and is reported before
touching `rtl/cpu`, as the stage's rules ask.

The bench's misaligned-access detector (the retire trace and shadow GPRs) found none in
the first 60 frames, so nothing earlier depended on it.

## After the misaligned fix: rebench, builds 3-6 and the s21 crash (2026-11-01)

**The rebench from frame 0 with the CPU's fix.** The five misaligned accesses at frame
291 now load and store the right values (the stores prove it: the bench's checker shows
a split load's first-uop value in its load column, not the final one). At frame 948 the
`lwzx` that crashed reads FFFD4A72, with no DSI, and the logo's wipe runs. The bus and
DMA streams match Supermodel region by region through the wipe.

**Build 3 (3 seeds, multicorner).** Seed 1 (slow -0.069 ns, fast -40C hold -1 ps) died
at boot on the board. I bisected it with a build of the round-6 RTL (sysd_r6) and a
90 MHz build of the final RTL (sysd_90). Both boot normally, so the failure belonged to
that one fit and was a timing or hold problem, not logic. Since then I check hold at
every corner before a board build.

**sysd_r6, sysd_90 and build 5 on the board.** All three behave the same way. They boot
through the warning and SEGA Presents and run the attract loop at CPI 1.80 (s6-s16),
then 2.3-2.7 (s17-s20). From s21 they sit in the fatal loop: a DSI with SRR0 0004D09C
reaches the boot ROM's DSI vector FFF00300 and then FFF00000. 0x4D09C is
`lwzx r25,r4,r21` in a loop that copies banked CROM (FF0xxxxx) to RAM around 0xB4004. A
static check of the game's own page attributes (its per-64 KiB table at FF801F00, read
by its own TLB-miss handler) gives every banked-CROM page 3F: valid, PP 3, WIMG 1111. A
correct address through that handler should not end in a DSI, so the suspect is the
address or the path to it. The bench run to the crash will settle which.

**The s21 cause: lbzux with rD = rB (bench, 2026-11-01).** The bench reaches the same
fatal at its frame 1158: SRR0 0004D09C, with a DEC entry and then a DTLB load miss as the
last two entries, as build 6's rows 10-11 show on the board. The decrementer is not
involved. The copy routine at 0x4D014 looks up a block (start k, count n) in a byte table
at 0x7C74C with `lbzux r4,r6,r4`. `ppc_decode` cracks it as step 0 load into rD and
step 1 `rA = rA + rB`, and step 1 reads rB again after step 0 has overwritten it. So r6
became 0x1A + 0x2C = 0x46 instead of 0x7C766. The count `lbz r23,1(r6)` then read RAM 0x47
(00) instead of 03. With a count of 0, the `addic. r23,r23,-1` / `bne` loop never stops.
It copies a 128 KiB block every 2 frames until r21 wraps past FFFFFFFF to 001E0D7C. The
next load misses at EA 00200000, and the game's table marks RAM above 2 MiB invalid
(attribute 00), so its handler takes the DSI path. The first wrong count was at bench
frame 960. The PowerPC manual allows rD = rB; only rA = 0 and rA = rD are invalid forms. I
found it with the new bench options: a value trigger on 4D170 (r23 = FFFFFFFF), then a PC
watch on the routine's set-up from checkpoint c920, then a trigger on the lbzux's second
record.

**Builds 6 and 7.** Build 6 (e31c24d9: the new latch, build 5's CPU) showed the crash's
entries on the board: row 10 a DEC at SRR0 0004D0A4 inside the copy loop, row 11 the
DTLB load miss after it and 0x34DC entries in all. Build 7 (9ae84cd8: build 6's system,
build 5's CPU and the update-load fix, fix_lux/build5.patch) fitted with
slow 100C -0.286 / +0.240, slow -40C -0.331 / +0.185, fast 100C +4.044 / +0.128 and fast
-40C +5.041 / +0.073 (setup / hold, system clock), 22,325 ALM. A Verilator checkpoint
cannot be restored into a changed model: each one carries a layout check value, and the
fix also changes the temporaries Verilator saves, so the bench for the fix runs from
frame 0 (run7). For that bench I turned a Verilator-only check in `ppc_core` (the ld_align
merge compare, which fires falsely for cache ops at unaligned addresses) into a
message.

**The exception latch, wrong twice.** Build 2's first-entry latch fired on code that ran
on into a vector address (0x16FC -> 0x1700). Build 5 asked for new SRR0/SRR1 values on
the vector's first record, and on the board row 8 stayed at zero. The core retires the
instruction an exception replaces as a record of its own, and that record already
carries the new SRR0/SRR1 (`ppc_core` `tr_srr0`). So the change is always on the record
before the vector's, and the latch could never fire. Build 6 takes the trigger from the
core's own taken-exception events on that record: `ev_mmu[3:0]` (interrupt, DSI/ISI,
ITLB miss, DTLB miss) and `rt_irq`. The vector is the PC of the next record. For the
kinds with no exported strobe (program, alignment, sc, FP unavailable, trap), the
trigger is an SRR0/SRR1 change on the record itself, with mtspr to either excluded. A
60-frame bench run with a Verilator-only log (`+define+M3_XLOG`) logged 164 entries:
38 at 0x500, 18 at 0x900, 11 at 0x1100, and 97 at 0x1200. Many 0x1200 entries were
back-to-back repeats of the boot's clearing loop (0x1620 `stwu`) with identical
SRR0/SRR1, so an SRR-change test would have missed them. A complete strobe would need
two CPU ports (exception taken and the vector, from `m_xcp`/`m_trap` and `m_xvec`).

**PVR (D-41).** VF3 sets W in its BATs and RPA when the PVR revision is below 0x0103. It
flushes nothing before the 53C810's DMA (three dcbf and one dcbi in the whole run, none
on the DMA path), so copy-back caching is only safe once the D-cache snoops the DMA's
RAM writes. `m3_system`'s `PVR` parameter stays at 0x00060101 until that snoop exists
and is proven. The live `ppc_cpu` now has the snoop port; `m3_system` ties it off
(`sn_req = 0`) until the 53C810's RAM writes drive it.

## Stage B wiring plan (the Real3D live; not built, not fitted)

What stage A already gives stage B: every Real3D write arrives at the bus decoded by
window (`m3_bus`, X_R3D), the 53C810's DMA moves the same words as Supermodel's, the
9C configuration registers are kept (and moved by the JTAG Mercury instruction), and
the JTAG mode-word updates come out of `m3_jtag` (`mw_we`, `mw_dev`, `mw_val`: block
culling is Mercury bit 2, the sun clamp Mars bit 18).

1. **Write paths (the first thing to build, and measured first).**
   - Culling RAM 8C (VF3 1.87 MiB of 4) and 8E (108 KiB of 1) go to SDRAM (D-6) at
     0x6D00000-0x72FFFFF, word form, so the geometry reads them as the viewer does.
   - Polygon RAM 98 (1.005 MiB) goes to DDR3 (D-6) through `m3_ddr3p`'s write port.
   - The 53C810 writes these as 8-word lines, the CPU as single words. `m2_sdram`
     writes 16 bits a request, about 9 clocks each: a word is 18 clocks, a line 144.
     Supermodel's trace puts 2,400 culling words a frame on average through boot and
     attract, and R80's largest VF3 frame far more, so `m2_sdram` needs a write burst
     (eight 16-bit columns from one request, p_long for writes) before culling RAM can
     live in SDRAM. The same burst halves the CPU's write-through stores (every store
     to RAM is an SDRAM write, VF3's DBATs being WIMG 1011).
2. **Ping-pong.** The Real3D draws frame N while the CPU writes N + 1; writes to 8E
   and polygon RAM after the flip must not change the frame being drawn (Supermodel's
   update buffers). I will double the two regions instead (8E 2 x 1 MiB in SDRAM,
   polygon RAM 2 x 1.125 MiB in DDR3), swapped at the flip; low culling RAM 8C is
   not double-buffered in Supermodel either. The status bit already flips at the line
   in tile register 08.
3. **Frame start.** Supermodel draws when both the command port (88) and the tile
   generator's register 0C write have come in a frame; stage A counts both. The
   sequencer is the viewer's (`m3v_core`): f_begin to `r3d_rast`, start to `r3d_geo`
   at the vertical blank after both, f_end after the geometry's last record.
4. **Textures.** The texture FIFO (94) and the VROM port (90: a VROM word address,
   the block copied from VROM into the FIFO, Step 1.0's single-word form) feed an
   upload engine that decodes the FIFO's headers and writes texels into the two
   2048 x 1024 sheets in DDR3 in `r3d_rast_tcache`'s line order (the viewer's
   loader's tiling, D-33). Uploads run after the command port write, before the
   flip, as Supermodel's FlushTextures.
5. **VROM reads** for the geometry: the MRA's four-block layout means VROM word i =
   {g, b, h} is at 0x4000000 + {b, g, h, 00}; the geometry's VROM reader
   (`m3v_sdr_rd` today) takes that permutation.
6. **DDR3** only after hps_io's first command and RESET low for 100 ms (R71, R72), as
   the viewer does; `f_begin` is held until then.
7. **Compositing.** The tile generator's line buffer already gives drawn and above
   per pixel: above tiles over everything, below tiles where the 3D layer did not
   draw, the 3D layer from `m2_fb_read` scan-out otherwise.
8. **Status.** Once the Real3D draws, the status word keeps FFFFFFFD/ping-pong, and
   tot_clks counts 33.33 MHz ticks of the frame's drawing (study 5.3 risk row). VF3
   only ever saw FFFFFFFD/FFFFFFFF in Supermodel's trace.

Area against the ceiling (R81's projection, without sound): stage A's 21.8 K plus the
geometry about 7.7 K, the rasteriser about 7.9 K, `m3_ddr3p` about 0.6 K, upload and
glue about 0.5 K: about 38.5 K of 41 K, with the CPI monitor and the overlay (0.6 K)
removable. Sound (about 4.4 K) does not fit beside it without the R81 savings.
M10K: 151 now; the geometry 43 and the rasteriser's 75 or more bring it to about
270-300 of 553.

Order of work: the `m2_sdram` write burst (bench, then the board's CPI figure), the
culling and polygon RAM write paths with their own unit benches against Supermodel's
trace, then the geometry against R35's model on frames captured from the trace,
then the rasteriser, the uploads and the compositor.

**Misaligned accesses VF3 makes** (bench, retire trace and shadow GPRs; word-aligned
doubles not counted): frames 0-520, five, all at frame 291 just before the warning
screen, all D-form word accesses to work RAM with r30 = 000A0000: `lwz r31,0x401(r30)`
at 0x55D60 (twice), `stw` to the same at 0x55D90, `lwz r31,0x402(r30)` at 0x55DA0 and
`stw` at 0x55DD0. It looks like a check of misaligned access (store, load back at
offsets 1 and 2). Forced alignment makes those stores write the wrong bytes, silently.
At the logo: the X-form `lwzx` from fixed CROM at 2 mod 4 that crashes. The checks ran
from checkpoints with a bench built from the same RTL (`sm3-builds/sys/objt2b`), which
restores the original chain's checkpoints exactly (144,435 records, field for field).

**The SEGA logo, in simulation, and the crash reproduced.** Resumed from checkpoints in
segments of about 100 frames, the bench reached the logo at its frame 947: the tile
state (VRAM, palette, registers) equals Supermodel's stable logo (its frames 960-1020)
byte for byte, and the RTL's video output equals the reference render on all 190,464
pixels. At frame 948 the bench takes the board's crash: `lwzx` at 0x55138 reads 4 bytes
at FFFD5006 and gets 4A72FFFD (the ROM holds FFFD4A72 there), `lhau` at 0x54B08 takes a
DTLB miss, the game's miss handler (0x1100) finds no PTE and takes its DSI path (0x1040,
SRR1 0000B032), rfi to 0x300, then the boot ROM's fatal loop: SRR0 and SRR1 as the
board's overlay showed. Over the whole run, frames 0-948, VF3 made seven misaligned
integer accesses: the five at frame 291 and these two.

# Stage B: the Real3D fed live (D-37 stage B), standalone blocks

Built as separate modules, benched and fitted on their own; not yet in
`m3_system` (stage A owns it until its rebench and fit are done). The
integration steps are at the end of this section.

## Files

| File | What |
|---|---|
| `rtl/real3d/r3d_cmd.sv` | Host port for every Real3D window, the status packet, the ping-pong flip, the frame sequence, four layer buffers |
| `rtl/real3d/r3d_mem.sv` | 8C / 8E / 98 writes into SDRAM and DDR3; 8E and polygon RAM double-buffered |
| `rtl/real3d/r3d_texup.sv` | Texture FIFO (94) and VROM port (90) into a DDR3 ring; the upload decode into the texture sheets |
| `rtl/video/m3_compose.sv` | 3D layer scan-out (`m2_fb_read`) and the mix with the tile generator's pixel |
| `rtl/mem/m3_ddr_arb.sv` | `m3v_ddr_arb` with a byte enable a master (the windows and the store write 32-bit words) |
| `sim/real3d/r3d_host_tb.sv`, `tb_r3d_host.cpp` | Bench: the trace through `r3d_cmd` + `r3d_mem` + `r3d_texup` on the real `m2_sdram` and `m3_ddr3p` |
| `sim/video/m3_compose_tb.sv`, `tb_m3_compose.cpp` | Bench: the compositor on stage A's video timing and `m3_ddr3p` |
| `sim/real3d/faults_sb.py` | Planted faults for all four blocks |
| `quartus/stage_b/*` | One-seed standalone fits, the virtual-pin calibration, the constant-fold check |
| `tools/mame-census/supermodel-stageb-dump.patch` | Supermodel hooks for the oracle below |

Makefile: the block that starts `# ==== D-37 stage B: the Real3D fed live`
(`lint_sb`, `test_sb_host`, `test_sb_compose`).

## The oracle

`sm3-builds/supermodel-sb` is `supermodel-sys` (trace patch and tile dump)
plus `supermodel-stageb-dump.patch`: at the listed bus-trace frames
(`M3SB_LIST`, the trace's own frame numbers) it saves the Real3D memories as
the frame is drawn (`r3d_<f>.bin`: 8C 4 MiB, 8E 1 MiB, polygon RAM 4 MiB,
texture RAM 8 MiB) and the finished frame with its 3D (`full_<f>.bmp`). Run
from no NVRAM, 99 frames (every 25th from 250, and every texture-upload
frame), in `sm3-builds/sb_oracle/`. Its bus trace is byte for byte
`sm3-builds/sys/sm_bus.bin` over that file's whole length, so the dumps belong
to the trace the benches replay. The tile dumps (`M3TG_START=249`,
`M3TG_EVERY=25`) are one frame behind: tile dump f001399 is trace frame 1400.

## What VF3 does to the Real3D (trace, frames 1-1500)

| Window | Writes | Who | When |
|---|---|---|---|
| 84 status | 32,094 reads, all from PC 0x64244 | CPU | all nine words in a row, through the SDK's little-endian copy (0x63F68 calls 0x64230, `lwbrx`) |
| 88 command | 1,140 (one a frame from 234, except loading frames) | CPU | 723 in vertical blank, 374 before the flip; 5 after it, all at boot |
| 8C | 236,768 | 53C810 | boot loads; from frame 300 only 5,544, all in vertical blank |
| 8E | 3,298,907; 521-8,542 a frame | 53C810 | from frame 300 all before the flip, 89% in active display |
| 98 | 7,750,586; 19,348 a frame in the fights, up to 50,500 | 53C810 | from frame 300 99.9% in vertical blank |
| 94 | 1,922,706: 43,692 words in each of 45 frames | 53C810 | vertical blank; frame 252 and every third frame 967-1118 |
| 90 | 1, frame 304 (VROM word 0xFEAAF8) | 53C810 | after the flip |
| 9C | 3, frame 233: 0x7FFC, 4, 0x20000 | 53C810 | |

Every frame with a command-port write also writes tile register 0C. 0C itself
comes in vertical blank (729), before the flip (374) and after it (92).

- **The same addresses every frame.** From frame 600, polygon RAM and 8E are
  rewritten at the same words in consecutive frames (overlap 100% and 99.9%).
  VF3 does not double-buffer in software.
- **Runs.** A frame's writes are few contiguous runs: at most 37 in polygon
  RAM, 6 in 8E, 32 in 8C. The longest run is 96,504 words.
- **A true ping-pong is wrong for VF3.** I modelled writes going to one copy
  only, with the copies swapped at each flip. Against Supermodel's single
  memory, polygon RAM differs at 1,236 of 1,500 flips: models written once
  sit in one copy. 8E differs at 303 flips, by 15-1,609 words.
  - Supermodel's notes give strong evidence that the hardware's 8E is two
    memories and that games write such data twice.
  - So the design follows Supermodel's emulated behaviour, which draws
    memory as at the flip.

## The design

- **When a frame is drawn.** It is drawn at its ping-pong flip:
  - only when the command port and tile register 0C have both been written
    since the last flip (Supermodel's DrawFrame condition);
  - only when the Real3D can take it. Otherwise the frame is dropped and the
    last image stays.
  - Supermodel instead renders every frame from memory, submitted or not.
    For VF3 the difference is the loading frames without a command-port
    write (127 of 1,267). Not measured on images.
- **Memory as at the flip (`r3d_mem`).**
  - Writes after the flip must not reach the frame being drawn. VF3
    rewrites polygon RAM in the next vertical blank, about 34% of a frame
    after the flip, while the geometry needs up to 77% (R79).
  - So 8E and polygon RAM have two copies. Bank R is the geometry's; every
    write goes to W at once, and to R at once unless R is being read or
    writes to R are pending.
  - A pending write leaves only its address, as a run (the open run, plus a
    256-entry FIFO in one M10K). Once the geometry is done, each run is
    copied W to R. W always holds the newest data, so the copy order does
    not matter.
  - The banks swap at the flip, only when nothing is pending.
  - At the flip the host port stops taking accesses until the write it
    holds has gone in. Otherwise a write the bus issued before the flip
    could land after the swap.
  - 8C is one copy, as in Supermodel. Where Supermodel and this design can
    differ: VF3's 8C writes in vertical blank, which Supermodel shows from
    the next frame. Here the geometry may see them if it is still running.
    None were seen to matter.
- **Textures (`r3d_texup`).**
  - The FIFO and VROM-port words go into a 256K-word ring in DDR3. A
    command-port write flushes up to there; a draw marks the flush point.
  - The upload decodes up to the mark, after every earlier frame has been
    drawn (it would change their textures). The frame's rasteriser f_end
    waits for it.
  - Between frames, flushed words are uploaded at once, so the ring never
    fills.
  - The decode is Supermodel's UploadTexture and StoreTexture. Its tile
    tables reduce to two closed forms (in the file's header): no 64-entry
    tables, no tile register file.
  - Mip origins: 2048 - 2048 / 2^i is formed as 0 - (1 << (11 - i)) on 11
    bits; the constant-fold check agrees.
- **VROM.** A VROM word i = {g, b, h} is at 0x4000000 + {b, g, h, 00} in
  stage A's MRA layout, as stage A worked out. VF3's one VROM upload (frame
  304) decodes byte for byte as Supermodel's. So the permutation, and the
  word's byte order (the same little-endian word as the FIFO's), are right.
  - Supermodel's lower 32 MiB VROM mirror holds only 3,200,000 bytes: it
    copies `32*100000`, not `32*0x100000`. VF3's address 0xFEAAF8 uses the
    upper copy.
  - Here bit 23 of the address is ignored (a mirror). A game that uses the
    lower half above 3.2 MB would differ from Supermodel there, and
    Supermodel would be the wrong one.
- **The status packet.**
  - Default: Supermodel's 0xFDFFFFFF with the ping-pong bit, line-of-sight
    words 0, the rest FFFFFFFF. VF3 runs on it in Supermodel.
  - `TOT = 1` reports gp_done, rend_done and tot_clks (the last frame's
    drawing in 33.33 MHz ticks): the study's risk row. It is off, because
    a real count could change what VF3 does and no game has been seen to
    need it. Folds away when off.
- **The 9C configuration.** It stays in `m3_bus` with the JTAG. Its only
  use in Supermodel is the update-buffer sizes, which this design does not
  need.
- **Block culling.** The Mercury's mode word bit 2 is taken at the draw and
  hides that frame's 3D (Supermodel's SetBlockCulling).
- **Compositor.**
  - A pixel is: an above tile; else the painted 3D; else a below tile; else
    black. This is Supermodel's bottom surface, 3D, top surface.
  - The 3D is read by the tile generator's own read strobe: ready two
    cycles after it, the tile pixel at three.
  - Four layer buffers: shown, waiting for the vertical blank, and the two
    the store's banks can be drawing.

## Benches and results

**`test_sb_host`**: the whole 1,500-frame trace through the three blocks on
the real `m2_sdram` and `m3_ddr3p`, DDR3 BUSY 20% and latency 8-40, a load
generator on port 0. Checked:
- every status read;
- at each listed frame's draw, bank R of 8E (1 MiB) and polygon RAM
  (4 MiB), and again when the geometry stand-in ends;
- at the frame's f_end, all 8 MiB of texture RAM;
- at the frame's end, 8C, except words 0-0x1FFFF, which Supermodel uses for
  its update buffers.

| Run | Frames | Draws checked | Result |
|---|---|---|---|
| Real frame length (1,738,525 cycles), geometry 20-90% of a frame, replay 20-70% | 1-1500 | 43 draws (42 again at the geometry's end, with 8E / polygon writes landing during it), 42 texture sheets (42 again at the replay's end), 98 8C | all identical; 32,094 status reads identical; 5,502 runs deferred, 11,119 copy transfers |
| Compressed frames (200,000 cycles) | 1-1125 | 31 draws (and 30 at the geometry's end), 30 texture sheets, 83 8C | all identical |
| Compressed, geometry 150,000-400,000 cycles (spans the next frame's writes, so the deferral and the copies run) | 1-1500 | 27 draws, 27 at the geometry's end, 27 texture sheets, 27 again at the replay's end, 98 8C | all identical |
| Synthetic uploads after frame 300: 2,000 random textures | | 793 flushes, each sheet compared | all identical |

- **Textures at the replay's end.** Each listed frame's sheets are checked
  again when its stand-in replay ends, so an upload landing under an earlier
  frame shows.
- **Synthetic uploads.** VF3 uploads one format only: 44 textures of type 0,
  16-bit, 256 x 256 with mip levels, one a flush, and one type 0x80 (32 x
  32, skipped). The synthetic mode drives random textures through the host
  port: types 0, 1, 2 and 0x80; both depths; every byte select; 32-256 a
  side; both pages; one to four a flush. They are checked against a
  transcription of Supermodel's StoreTexture and UploadTexture in the bench.

8C words 0-0x1FFFF: 47,727 differ from frame 275 on. These are Supermodel's
update buffers, filled by the boot loads after the flip (frames 244-298).

**`test_sb_compose`**: 2 frames, 380,928 pixels a case; tile pixels from
Supermodel's two tile surfaces (R77: the tile RTL equals them).

| Case | Against Supermodel's order | Against its finished frame |
|---|---|---|
| Frame 1400, 3D from its grab, buffer 1 | 0 differ | 98.33% within 9 a channel |
| Frame 1500, buffer 2, BUSY 50%, latency 13-120 | 0 differ | 97.36% within 9 |
| Frame 1000 (logo), 3D off, buffer 3 | 0 differ | 91.88% within 9 (the rest is its black edge) |
| Random layers on frames 1400 and 1375, buffers 0 and 3 | 0 differ | |

No line arrived late after the warm-up frame. The grab is not exact against
the surfaces anywhere: Supermodel's GL path converts 5-bit colour its own way
(off by 8-9 a channel) and its grab has a black edge. So the exact check is
against its composite order, and the grab is a measurement.

**Planted faults** (`python3 sim/real3d/faults_sb.py`): 32 faults, one edit
each, 27 caught.

| Fault | What it breaks | Result |
|---|---|---|
| C1 | status: ping-pong bit inverted | caught (status reads) |
| C2 | swap before the held write goes in | caught (frame 968 textures) |
| C3 | draw without tile register 0C | missed: equivalent for VF3, every submitting frame writes 0C |
| C4 | bank R not held during the geometry | caught (frame 1125, geometry end: 796 8E, 11,050 polygon words) |
| C5 | upload while an earlier frame is drawn | caught (a synthetic frame not drawn) |
| C6 | no mark at the draw | caught (synthetic) |
| C7 | 8E not mirrored to 1 MiB | missed: equivalent, VF3 writes 8E below 0x1B014 only |
| M1 | no deferral while R is read | caught (as C4) |
| M2 | a run's last beat not copied | caught (a later draw) |
| M3 | 8E copy skips pairs | caught (frame 1275: 6 8E words) |
| M4 | polygon RAM lanes swapped | caught (frame 325: 64,854 words) |
| M5 | culling RAM halves swapped | caught (8C: 146,333 words) |
| M6 | runs never extend | missed: performance only. Before the deadlock fix it hung the bus. |
| M7 | swap with copies pending | caught (frame 1125) |
| M8 | a closed run is lost | caught (frame 969: 63 8E words) |
| T1-T6, T8-T12 | texel halves, 8-bit bytes, mip origin, row parity, VROM permutation, size formula, no fresh fetch, type 1, tile end, buffer overrun, page | all caught (synthetic, T5 by frame 325) |
| T7 | upload past the mark | missed: needs a flush during an upload; neither VF3 nor the synthetic mode makes one |
| T13 | the header's words left in `have` | missed: the fetch always runs far enough ahead; a starved DDR3 port would show it |
| P1-P4 | 3D over above tiles, the 3D a pixel late, buffer high bit, show_3d ignored | all caught |

## Fits (one seed, 10 ns, every port a virtual pin; slow 100C / -40C)

| Block | ALM fitted | Registers | M10K | Fmax 100C / -40C | Virtual pins | ALM less pins (est.) |
|---|---|---|---|---|---|---|
| `r3d_cmd` | 250 | 171 | 0 | 231.9 / 239.8 MHz | 335 | ~160 |
| `r3d_mem` | 486 | 331 | 3 | 151.0 / 150.9 MHz | 379 | ~385 |
| `r3d_texup` | 762 | 504 | 2 | 128.0 / 123.2 MHz | 340 | ~670 |

`r3d_texup`'s first fit (three seeds, before my one-seed rule) was 968-974
ALM at 97.7-104.7 MHz; seed 1 missed 100 MHz at both corners.
| `m3_compose` (with `m2_fb_read`) | 224 | 125 | 5 | 185.7 / 180.5 MHz | 194 | ~175 |
| `m3_ddr_arb`, 7 ports | 696 | 8 | 0 | 194.4 / 193.1 MHz | 882 | ~460 |

The virtual pins are calibrated by `vp_cal`, 170 inputs registered to 170
outputs: 175 ALM, of which the registers need 85. So 340 pins cost about
90 ALM, 0.265 a pin. The arbiter is nearly all pins and multiplexers; the
viewer already carries one (`m3v_ddr_arb`, six ports).

## Findings and wrong turns

- **Bench artifacts, three.** I pulsed line 0 at IRQ 0x0D as well as at the
  flip, so before VF3 sets the flip line (0 at reset) the bit toggled twice:
  74 status reads wrong. `m3_ddr3p`'s asynchronous reset left one random
  command on the pins at cycle 0. And the stand-in rasteriser took a random
  f_end from the registers' power-up state during reset, so the RTL's
  in-flight count wrapped. Each fixed in the bench, none in the RTL.
- **A held write could land after the swap.** Found in review, not by the
  bench: a write the bus posted before the flip could still be in
  `r3d_cmd` when the flip came. The host now waits at the flip until it has
  gone in.
- **The upload decode first compared against the live flush pointer.** In
  the compressed bench an upload took several frames: about 4.4 ms at three
  cycles a texel and a write a beat. It kept swallowing later frames'
  textures and never finished; textures were wrong from frame 1012. And when
  frames were not drawn, flushes piled up in the 256K-word ring. Fixed with
  the mark, and with uploads between frames.
- **`r3d_texup`'s first draft** had variable part-select writes (against the
  rules), mip origins from 2047 instead of 2048, a header word read a cycle
  early, a 9-bit tile counter that wraps at 512, and a fetch window that
  deadlocked after a restart. All found in review before the first build.
- **`r3d_texup` missed 100 MHz** (97.7 / 95.1 MHz) at 971 ALM. The path ran
  from the tile size through the word count into four 19-bit window
  compares. Map-only probes put the window logic at 232 ALM, the VROM path
  at 80 and the counters at 18. The window became a signed count of words
  held from the decode's position (the decode only moves forward), and the
  texture start, tile end and window start registers went. Result: 762 ALM,
  128.0 / 123.2 MHz, every bench still identical.
- **A deadlock in `r3d_mem`, found by a planted fault.** Fault M6 (runs
  never extend, meant as performance only) hung the bus. Every deferred
  write became its own run, and the 256-entry run FIFO filled. The waiting
  write then held `w_valid` high, and copies started only with no write
  waiting, so nothing drained. VF3 makes at most 43 runs a frame and never
  fills it, but a game writing scattered words during the geometry would.
  Copies now go first while the FIFO is full.
- **A bench race made three faults look caught.** The synthetic mode
  snapshot the sheets after a drain that did not wait for a flushed but
  unstarted upload. Faults that only changed timing (C4, M1, M3) then
  failed on textures. The drain now waits for it, and those three became
  misses.
- **The bench never let a listed frame's geometry overlap the next frame's
  writes.** Before its 8C check, the bench waited for `r3d_mem` to report
  its banks in step. That only happens once the geometry has ended and the
  copies are done. So every listed frame was checked with no writes during
  its geometry, and "no deferral" (M1, C4) passed.
  - I first blamed still scenes rewriting the same values. Not so: the
    fights change about 11,300 polygon and 2,600 8E words a frame.
  - I extended the run to frame 1500 and lengthened the geometry; neither
    helped.
  - A count of the writes during checked geometries showed zero. The 8C
    check now waits only for the 8C write itself.
  - Since then: the real-time run deferred 5,502 runs with 11,119 copy
    transfers, bank R stayed unchanged at all 42 geometry ends, and M1 and
    C4 are caught.
- **A second bench race.** The synthetic compare fired at the first frame
  end after synthetic mode began, which could belong to a trace frame still
  in flight. It now fires only after the bench's own flip, once nothing is
  in flight.
- **The tile dump's surfaces are not Supermodel's screen colours.** At frame
  1400 only 1.9% of top-surface pixels equal the grab. Its GL composite
  converts colour differently and adds a black edge. I first suspected the
  frame offset and the byte order; both were right.
- **The compositor first read the 3D one pixel late.** `hcnt` moves on the
  edge after the pixel enable, and the tile pixel comes three cycles later.
  Found in review; the position is now latched with the tile generator's
  read. Planted as fault P2.

## Not done

- **Fog wiring (block 5).** The viewport records' fog already reaches
  `r3d_rast_fogvp` inside `r3d_rast` (the rasteriser's work). Scroll fog
  (`sf_*`) is a frame input, and render_d25's rule needs viewport node
  words 0x00 (priority), 0x20, 0x22-0x25 and the rectangle, which the
  geometry's viewport record does not carry. Two ways:
  - `r3d_geo` adds them to its viewport record (in the geometry, owned by
    nobody now);
  - a small reader walks the viewport list in culling RAM at the draw.
  Until then `sf_*` should be tied off (no scroll fog).
- **The `m2_sdram` write burst.** It is not built; `m2_sdram` is in stage
  A's fit. The cost without it: an 8E word is four 16-bit SDRAM writes
  (both copies), about 40 clocks. VF3's peak 8,542 words a frame is then
  about 340,000 clocks, 20% of the SDRAM port; its typical 521, 1.2%. The
  bus is not stalled meanwhile beyond the one held write, but the 53C810's
  DMA runs at that rate.
- **Images.** No frame has been drawn by the live chain. That needs the
  geometry and the rasteriser in the system.

## Integration (after stage A's rebench and fit)

1. **`m3_bus`.** Route `X_R3DST` (84000000-3F reads) and `X_R3D` writes
   other than 9C to a new host port:
   - out: `h_go` (one strobe), `h_we`, `h_addr`, `h_wdata = wd_le`;
   - in: `h_rdy`, `h_rv`, `h_rdata` (CPU byte order).
   - A new state waits for `h_rdy` before the strobe; a read then waits for
     `h_rv`. Writes are posted. Keep 9C as it is.
   - Remove `pp`, `pp_line`, `pp_on` and the X_R3DST read; the bus's
     `pingpong_o` becomes `r3d_cmd`'s `pingpong`. The trace's event 2
     becomes `r3d_cmd.flip`.
   - Line-transaction beats to X_R3D already come one at a time (B_BEAT);
     each becomes one strobe.
2. **`m3_system`, the blocks.**
   - Instantiate `r3d_cmd` with `tg_reg_*` from `m3_bus`, `ln_start` and
     `ln` as given to `m3_bus`, and `mw_*` from `m3_jtag` (now
     unconnected).
   - Wire `r3d_cmd`'s `m_*` to `r3d_mem`'s `w_*`, `swap`, `r_busy`,
     `bank` and `can_swap`.
   - Wire its `t_*` to `r3d_texup`: `t_valid`/`t_ready`/`t_vrom`/`t_data`/
     `t_flush`/`t_pend`, `t_mark` to `mark`, `t_mpend`, `t_go` to `go`,
     `t_busy` to `busy`.
3. **SDRAM.** Two more `m2_sdram` ports (NP 2 to 4, plus the geometry's two
   read ports: 6 in all):
   - `r3d_mem`: p_long 0, writes 16-bit, reads 64-bit;
   - `r3d_texup`: reads only, `p_we` 0.
   - Layout (stage A's free area, unchanged MRA): 8C at 0x6D00000, 8E
     copies at 0x7100000 and 0x7200000 (the `r3d_mem` defaults), VROM at
     0x4000000.
4. **DDR3.** `m3_ddr3p` (NP 1, the texture cache's pipelined port) behind
   `m3_ddr_arb` with port 0 `m3_compose` and then, in any order: the
   rasteriser's store (`s_*`, be 0x0F/0xF0), its layer (`l_*`, be FF), the
   geometry's polygon reader (`m3v_ddr_rd`, be FF), `r3d_mem`, `r3d_texup`.
   - DDR3 map (64-bit words from the core's base):

     | Word | What |
     |---|---|
     | 0x000000-0x07FFFF | layer buffers k = 0-3 at 0x20000 k |
     | 0x100000, 0x180000 | polygon RAM copies |
     | 0x200000 | the texture ring (0x20000 beats) |
     | 0x400000 | the rasteriser's store: records, two 32 MiB banks to 0xBFFFFF (`store_base`; study R136) |
     | 0xC00000 | texture sheets (`tex_base`) |
     | 0x1000000 | the extra memory (VROM, R100), 64 MiB |
     | 0x1800000 | the store's run lists, 32 MiB to 0x1BFFFFF (`RUNO` 0x1400000 from `store_base`; study R136) |

   - **R71:** gate the arbiter's `m_req` (and `m3_compose`'s `ddr_ok`) on
     hps_io's first command plus `RESET` low for 100 ms, as `m3_viewer`
     builds `ddr_ok`. Every DDR3 client then waits behind it.
5. **The geometry and the rasteriser.**
   - `r3d_cmd.g_start` starts `r3d_geo` (SETUP_OUT, LOD_WORD).
     `g_busy = geo_busy || g_valid`.
   - The geometry's records go straight to `r3d_rast`'s `i_*`.
   - Culling port: `m3v_sdr_rd` with words below 0x100000 at
     0x6D00000 + 4a, and words 0x800000-0x83FFFF at the 8E copy
     `r3d_mem.bank` selects. VROM port: `m3v_sdr_rd` with the {b, g, h}
     permutation above. Polygon port: `m3v_ddr_rd` with `base` the polygon
     copy `bank` selects.
   - `r3d_cmd.f_begin`/`f_end` to `r3d_rast`'s; `f_ready` back;
     `fb_base = 0x20000 * f_buf`; `r_done = done`, `r_busy = busy`; `sf_*`
     tied to no scroll fog.
6. **Video.** Replace stage A's
   `if (o_v) px <= o_drawn ? {o_r, o_g, o_b} : 0` with `m3_compose`:
   - `hcnt`, `vcnt`, and `sc_rd` as given to the tile generator;
   - `show_buf` and `show_3d` from `r3d_cmd`;
   - its `px` feeds the overlay as `px` did.
7. **Overlay.** `r3d_cmd`'s `n_draw`/`n_drop` and `r3d_mem`'s
   `n_runs`/`n_copy` can replace row 1's counters.

## Stage B integrated (2026-11-01)

The six steps above, done in `m3_bus`, `m3_system`, `Model3.sv` and `files.qip`:

- **`m3_bus`.** Status reads (84000000-3F) and window writes other than 9C go
  to `r3d_cmd`'s host port. States B_R3D and B_R3DR are added; `h_go` is
  asserted combinationally only in a cycle with `h_rdy`, so a port closed at
  the flip cannot lose a word. The bus's own ping-pong bit, flip line and
  status read are gone, and the window counters count at the strobe. 9C
  stays here with the JTAG.
- **`m3_system`.**
  - SDRAM ports 0-5: bus, tile generator, culling reads, VROM reads,
    `r3d_mem`, `r3d_texup`.
  - DDR3 through `m3_ddr_arb` (6 ports) onto `m3_ddr3p`, whose pipelined
    port is the texture cache. Both requests are gated on `ddr_ok`.
  - `r3d_geo` with SETUP_OUT and LOD_WORD; `r3d_rast` with MIP and scroll
    fog tied off.
  - `m3_compose` in place of the tile-only pixel.
  - Overlay row 1: frames submitted, drawn and dropped; geometry error bits,
    late layer lines, textures uploaded; runs deferred and copies. The build
    word reads 3B000001.
  - The flip is a bench event (kind 2).
- **The geometry's VROM port is `r3d_vrom_rd`, not the viewer's
  `m3v_sdr_rd`.** With the MRA's four-block layout only VROM word pairs are
  contiguous, so each SDRAM read gives two words. This costs VROM bandwidth
  against the viewer's contiguous image.
- **`Model3.sv`.** The DDRAM pins go to the system. `ddr_ok` is the viewer's
  R71 gate: hps_io's io_enable seen, then RESET low for 100 ms.
- **`files.qip`.** The stage B files, and `AUTO_SHIFT_REGISTER_RECOGNITION
  OFF` (R64, the geometry's delay pipes), which `Model3.qsf` lacked.
- **Synthesis of the release form:** about 40,430 ALM estimated, 86 DSP,
  2.36 Mbit of block memory. No warnings in the new code.

**Bench.** `tb_m3_system.cpp` gains the DDR3 model (in the checkpoints),
`ddr_ok`, and `--r3d` / `--r3d-at`: a draw's memories and its layer. Built
in `sm3-builds/sysb` (`make -f sim/system/system.mk SYS_OBJ=sm3-builds/sysb/obj
SYS_THREADS=4 system`), run as `Vm3_sb` at 208 kHz with four threads.
- Stage A's checkpoints cannot be restored into this model, so the chain
  runs from boot (`sm3-builds/sysb/chain.sh`).
- The first chain segment was killed by SIGTERM five minutes in. Stage A
  also runs a `Vm3_system`, and stopping that by name takes mine too, so
  mine now runs under another name.

**Bus against Supermodel through frame 300 (bench) / 330 (Supermodel).**
Every Real3D DMA stream is identical through the new write path: 231,224 8C
words, 344,548 polygon RAM words, 258 texture FIFO words, the VROM port
write, the 9C words. The status reads differ once in phase (read 1,260),
as stage A's did.

**First chain (the CPU before R91), to frame 1153.** It boots through the
warning and SEGA logo screens, which compose correctly over the 3D.
- **The composite against Supermodel's grabs** (`sim/system/sb_grabcmp.py`,
  the nearest Supermodel frame in content): 90.8-91.9% of pixels equal and
  91.9-93.0% within 9 a channel at bench frames 950-1150. The rest is the
  grab's black edge and its GL colour conversion, the same figure the
  standalone compositor bench gave.
- **The layer against the RTL-form model.** At bench frame 1121 a scene is
  drawn under the logo (7 viewports, 18 triangles). `sim/system/sb_ref`
  runs the geometry model and render_d25 on the memories that draw read
  (fog on, nearest mip with the geometry's B, scroll fog off): 0 of 190,464
  pixels differ, 185,504 painted in both.
- **Its textures were noise, and the texture data itself was wrong.** The
  texture FIFO's DMA stream differs from Supermodel's from word 81,072 at
  frame 968 (0F32CF29 against 0F320F31). So the CPU built different
  texture data in work RAM, and the uploader and rasteriser drew it
  faithfully. The other Real3D streams are identical as far as the run
  went (polygon RAM 427,759 words, 8E 351,406).
- At frame 1153 that chain took the boot ROM's fatal handler: the
  21-second crash, which R91 fixes (integer update loads with rD = rB).
  The fixed-CPU chain (`chain2.sh`, `run2/`) is rerunning from boot.
- **Supermodel's 3D first shows at frame 1280** (a wipe through 1290), so
  about bench frame 1255.
- **The fixed-CPU chain (`run2/`) to frame 1400:** no crash. The 3D
  layer is exact against `sb_ref` (render_d25) on frames 1272, 1302 and
  1350: 5,860 triangles, 11 viewports, 0 of 190,464 pixels differ.
- **The composite does not match Supermodel past frame 1330.** Bench
  1240-1330 match Supermodel's logo frames (about 91% of pixels equal).
  From 1340 to 1390 the bench still shows the full SEGA logo on white,
  with a fight drawn in the 3D layer underneath, so the best match is
  always Supermodel's logo (frame 1190-1242, the window's edge). The
  compositor is not at fault: the tile layer covers the 3D because the
  game has not wiped the logo away. The bus traces show why:
  - Supermodel writes tile register 0x6C every frame from 1259 (the wipe,
    changing values) and the priority word 0x20 = EA050000 at 1275.
  - The bench writes the priority word at 1269, 0x6C twice (1270, 1272),
    then nothing more to the tile registers through 1400.
  - Supermodel writes about 19,300 polygon RAM words every frame from
    before 1240. The bench starts at 1260 and writes about 6,800 a frame
    (one full set every third frame, R95's rate). Its hot code is
    different too (0x23Exx, 0x482xx against Supermodel's idle loop at
    0x19xx).
  The game runs about a third as fast on the bench, and the logo wipe
  appears to wait on that. The board (R95) does get past the logo, so
  this is the slow CPU, not a hang. A frame-by-frame grab comparison
  past 1280 needs Supermodel run at the bench's game rate, or a check
  by game frame rather than video frame.

**Fits (one seed, release form, multicorner; the live CPU with R91, PVR
0x00060101, snoop tied off).**

| Build | Clock | Setup slow 100C / -40C | Setup fast 100C / -40C | Hold (four corners) | ALM | M10K | DSP |
|---|---|---|---|---|---|---|---|
| `sysb_s1` (89f65a14) | 100 MHz | -2.018 / -1.793 ns | +3.194 / +4.268 ns | +0.250 / +0.162 / +0.133 / +0.087 | 41,138 | 300 | 86 |
| `sysb90_s1` (5f02194c) | 90 MHz | +0.047 / -0.412 ns | +4.765 / +5.881 ns | +0.207 / +0.149 / +0.134 / +0.117 | 41,346 | 300 | 86 |

- **`sysb_s1` on the board**: dead from s1, CPI 21.5 every second,
  no screen (`sm3-builds/board_sb1`). As expected at -2 ns.
- **Its worst paths:**
  - the scan-out reader's request through `m3_ddr_arb`'s combinational
    grant into `m3_ddr3p`'s command queue and the f2sdram registers
    (-2.0 to -0.9 ns);
  - then, at 98% full, about 1 ns lost everywhere: the CPU's D-cache
    -1.26, the geometry -1.05, the texture cache -1.02, `r3d_texup`'s jump
    compare -0.99.
- **For `sysb90_s1`:**
  - the arbiter now presents everything from its registered owner, a cycle
    a transaction;
  - `r3d_texup`'s end and jump tests are registered (states U_H0S,
    U_JMPS);
  - both benches still pass: 500 synthetic textures, and 34 draws with
    their geometry-end and replay-end rechecks to frame 1500.

**Area, the top 15 blocks by ALMs needed (`sysb_s1`):** `ppc_cpu` 12,515
(38 M10K, 9 DSP), `r3d_rast` 8,142 (102, 45), `r3d_geo` 7,762 (43, 11),
`ascal` 2,112, `audio_out` 856, `m2_sdram` 599, `m3v_ddr_rd` 593,
`m3_tilegen` 562 (54 M10K), `pll_hdmi_adj` 510, `osd` 492 and 474,
`r3d_texup` 478, `m3_bus` 470, `m3v_sdr_rd` 370, `m3_cpu_mon` 359.
- The stage B blocks in the system: `r3d_cmd` 78, `r3d_mem` 277,
  `r3d_texup` 478, `m3_compose` 96, `m3_ddr_arb` 67, `r3d_vrom_rd` 70,
  `m3_ddr3p` 340, and the viewer's read adapters `m3v_ddr_rd` 593 and
  `m3v_sdr_rd` 370: about 2,370 in all.
- The two adapters keep word queues and 24-bit range compares in registers,
  and are the cheapest stage B area to win back.

## The D-cache snoop wired, PVR 0x00060103, write bursts (R101, 2026-11-02)

The fix for R95: VF3 at 14.5 fps on the board, half the CPU's cycles waiting on
data, and in the bench two-thirds of that wait on single write-through stores to
work RAM (about 25,000 a frame, each two 16-bit SDRAM requests).

**The snoop (D-41, R90's "to finish D-41").** `m3_bus` now drives `ppc_cpu`'s
`sn_*` port:
- every 53C810 access to work RAM (00000000-007FFFFF) is snooped first, one snoop
  an access: sn_addr is its line, sn_inv is 1 for a write (LINE_WR, WR);
- the access is not granted until sn_ack; the CPU keeps the bus meanwhile, so the
  D-cache's push (an MC_LINE_WR) goes through;
- from sn_ack (the same cycle) to the 53C810's grant the CPU is not granted, so a
  write that invalidated a line lands before the CPU can fill it again, and a read
  follows the push;
- no snoop is started while the 53C810 owns the bus (its request can still be up
  with the access already granted).
ROM, VRAM and the I/O space are not snooped. `m3_system`'s PVR default is now
0x00060103: VF3 maps RAM copy-back.

**Write bursts (the pending "`m2_sdram` write burst").**
- `m2_sdram` has a `WB` port mask (default none: the controller as it was). A WB
  port's write is a burst of two columns, or eight with p_long, from one dispatch:
  S_WR walks the columns as S_RD does, and column k's word and mask are read live
  from `wb_din` / `wb_be` while `wb_col` = k.
- `m3_sdr_port`: a single write (MC_WR) is one two-column burst instead of two
  requests. A cast-out (MC_LINE_WR) takes its eight beats into the line buffer
  (`bd`, shared with fills), one every four clocks, and writes them as two
  eight-column bursts, the first once beats 0-3 are in. Before, it was sixteen
  requests, one beat at a time.
- Posting: a single write was already acknowledged at once (one deep, in order).
  A cast-out is now posted too: the bus is free after the eighth beat. Nothing
  passes a posted write: the port takes the next transaction (of either master)
  only when the last is written, so a read after a write always sees it. I did not
  let reads pass posted writes; the MPC105's rules for that are not checked.
- The beat period of four is one clock more than the bus's own line writes
  (B_BEAT, three): the master sees m_wnext a clock after s_wnext and shows the next
  beat the clock after that.

**Benches.**
- `tb_m3_bus` had not run since stage B: the Real3D host port (`h_rdy`) was left
  unconnected in `m3_bus_tb.sv`, so the first window write hung. The bench top now
  has a host port that is always ready and answers a status read with FFFFFFFD.
  Its end-of-run drain also counts the last JTAG, I/O and tile register strobes
  (two seeds failed on a strobe that came after the main loop).
- New in `tb_m3_bus`: master 0 keeps a six-line copy-back cache of 32 lines
  (00012000-000123FF) and answers the snoop as `ppc_dcache` does (quiet first,
  push a dirty line, then sn_ack; a write snoop invalidates). The 53C810 reads and
  writes those lines, single and line. After a write snoop the CPU refills the
  line at once, in the sn_ack cycle, to race the DMA. Checked: no 53C810 write
  lands while the line is held or being filled; a 53C810 read returns the line as
  it was at the snoop; every CPU load hit returns the coherent value. Uncached
  reads follow a write to the same address half the time (about 4,300 a run).
- 40 seeds x 100,000 transactions pass (each about 22,500 snoops, 1,400 pushes,
  1,300 invalidates). `tb_m3_sdr_port` passes 5 seeds x 200,000.
- Planted faults, each caught on three seeds: no snoop at all (about 1,450 stale,
  3,800 reads before the push); every snoop a read (sn_inv 0: 1,500 stale); the
  CPU not held after sn_ack (2-3 stale a run, only through the same-cycle refill);
  a cast-out's first burst started after one beat (10,000 read mismatches). Started
  after two beats it is not caught and is harmless: `m2_sdram` reaches column 4
  later than beat 2 arrives. The first-burst rule waits for four.
- The core (200 seeds), CPU subsystem (200 seeds, latency to 80) and cache
  benches (8 + 8 seeds) pass; nothing in `rtl/cpu` changed.

**The system bench (`run3`, `chain3.sh`, from boot, built from `sm3-builds/pw/src` = HEAD + these four
files).** It reaches frame 1,400 with no fault. The 53C810's Real3D streams match the write-through bench
(`bustrace.py compare run2 run3`): texture FIFO 1,922,706 words, 8C 236,768, 8E all of run2's 689,282; polygon RAM
to word 861,686 (one word's low 24 bits differ at a different game frame, then agree again). Fight frames 1,210-1,390:
CPI 4.04-4.08, data wait 40.3-41.1% (all RAM: about 5,850 line fills and 4,390 cast-outs a frame, no single
writes), 0.5 frames submitted a frame. Study R101 has the table against run2.

**`--prof` additions (C++ only, so the model and the run3 checkpoints are unchanged; in `sm3-builds/pw/src`, and as
`sm3-builds/pw/tb_m3_system_prof_r101.patch` against the tree's bench, which R100 has changed meanwhile; not yet applied).** Each 10-frame window now
also prints:
- the longest CPU bus wait (`m_req[0]` to `m_ack[0]`, the overlay's `cpu_wd`), and a "stall" line with the bus state
  for any wait over 20,000 clocks;
- cycles by the last retired PC in VF3's wait loops (1E928, 1E97C, 1E9DC, 1E9FC, 2518) and the rest, with busy
  instructions a frame;
- frames submitted and drawn, the geometry's busy share, and the rasteriser's frame cycles (mean and max).

From `run3/c1300`:

| DDR3 model | rasteriser frame (clocks) | submitted / 10 frames | wait loops | busy instructions a frame | longest bus wait |
|---|---|---|---|---|---|
| 12-40 clocks, 10% busy (default) | 637,000 | 5 | 3.6% (1E9FC) | 390,000 | 16,500 |
| `--ddr-lat 40 120 --ddr-busy 30` | 820,000-842,000 | 3-4 | 21% (1E9FC 18.6, 1E9DC 2.6) | 273,000-281,000 | 32,900 |

Frames 0-200, 300-400 and 700-800 had no wait over 20,000 clocks. The board's 353,831 is not reproduced.

**Fits and the board (study R101).** `sysb90pw_s1` booted black on the board: the burst mux on `sd_dq_o` stopped it
packing into the I/O cells ("cannot simultaneously use clear and load signals"), so the ROM was written corrupt.
Column data now goes through `din_r`. Then:

| Build | Note | ALM | M10K | Setup (slow 100C / -40C) | Hold (4 corners) |
|---|---|---|---|---|---|
| `sysb90pw_s1` | DQ not packed: black on the board | 41,546 | 300 | +0.140 / -0.031 | all positive |
| `sysb90pw2_s1` | no fit (4,200 of 4,191 LABs) | - | - | - | - |
| `sysb90pw3_s1` | `CPI_MON` = 0; 20 fps on the board, textures and tiles corrupt | 40,977 | 300 | -0.386 / -0.308 | +0.143 / +0.160 / +0.134 / +0.100 |
| `sysb90pw4_s1` | with `snap_r98` | 39,180 | 321 | +0.205 / +0.260 | +0.210 / +0.148 / +0.092 / +0.038 |
| `sysb90pw5_s1` (ccd782df) | with `snap_r98`, overlay rows 12-15, `DDR_MAXQ` 8 | 40,815 | 321 | -0.436 / -0.225 (D-cache `mq_v`) | +0.155 / +0.043 / +0.124 / +0.014 |
| `sysb90pw6_s1` (530a14f4) | pw4 without the write bursts (HEAD's `m2_sdram`, `m3_sdr_port`), snoop and PVR 1.3 kept | 39,183 | 321 | -0.210 / +0.069 | +0.251 / +0.172 / +0.133 / +0.117 |
| `sysb90pw6_s2` | the same, seed 2 | 39,164 | 321 | -0.459 / -0.410 | +0.003 / -0.028 / +0.049 / +0.027 |
| `sysb90pw8a2_s1` (f5864d55) | pw6, `ALLOW_SYNCH_CTRL_USAGE OFF` on `sd_a[*]`: every SDRAM pin packed | 38,972 | 321 | -0.607 / -0.318 | +0.134 / +0.026 / +0.102 / +0.007 |
| `sysb90pw8b_s1` (5f2fcec3) | pw6 with PVR 0x00060101 (control) | 39,107 | 321 | -0.102 / -0.144 | +0.155 / +0.164 / +0.130 / +0.076 |
| `sysb90pw9_s1` (11335745) | pw8a2 + write bursts + overlay rows 12-15 + `DDR_MAXQ` 8; every SDRAM pin packed | 40,748 | 321 | -0.270 / -0.460 | +0.124 / +0.102 / +0.122 / +0.078 |

Every fit copy was HEAD plus these four files (`sm3-builds/pw/fit_patch*.sh`). None includes the framework edits
that were later reverted, or R100's changes; pw4 and pw5 include `snap_r98`.

## Daytona 2 (Step 2.1) on one SDRAM (study R100, 2026-11-02/03)

The census, the placement and the parameter scheme are in the study's R100; this is the
bench side.

**Files.**

| File | What |
|---|---|
| `rtl/sys/m3_loader.sv` | Downloads by ioctl index: 0 SDRAM, 1 the extra memory, 3 SDRAM at 0x7300000, 2 the NVRAM (backup RAM, then the 93C46), 4 the game configuration (step, flags, CROM bank map) |
| `rtl/sys/m3_xmem.sv` | VROM in the extra memory: the geometry's port (a request's 1-5 beats issued at once on a pipelined port) and r3d_texup's |
| `rtl/sys/m3_sec.sv`, `rtl/sys/m3_5881.sv` | The security board: RAM, registers, the stream; the 315-5881 cipher, generated by `sim/system/gen_m3_5881.py` from MAME's tables |
| `rtl/io/m3_scripts.sv` | `R3DMA`: the Step 2 Real3D DMA device as a register front end on the 53C810's move engine |
| `rtl/sys/m3_jtag.sv` | `jstep`: the Step 1.5 / 2.0 / 2.1 ID codes and the 17-device chain |
| `rtl/io/m3_io.sv` | `EE_LOAD`: the 93C46's load port for the NVRAM |
| `sm3-builds/dt2/m3_bus_step2.patch` | The Step 2 map in `m3_bus` (MPC106, the security board, the bank map, 8 MiB fixed CROM), held outside the tree; `m3_system` connects it with `M3_STEP2_BUS` |
| `sim/system/m3_mra.py` | `daytona2`; `pack` writes every index (`IMG.iN.bin`); `nvram` converts a Supermodel NVRAM file |
| `mra/Daytona USA 2 - Battle on the Edge (Japan, Revision A).mra` | Indexes 4, 1, 3, 0 and `<nvram index="2">` |

**Benches.**

| Target / run | Result |
|---|---|
| `test_xmem` | 2 seeds x 200,000 requests (both VROM sizes, VF3's zero half, texture reads) against the layout rule: 0 wrong. Planted faults (a block bit, a word given before its beat, the zero rule) fail on 17,856 / 40,160 / 18,393 words |
| `test_sec` | `m3_sec` replaying Supermodel's 3,000 Daytona 2 frames (392,868 accesses): all 253,386 reads identical (128,114 data words). One key bit flipped: fails |
| `Vm3_jtag ... --step 3` | Daytona 2's 32,215 TCK writes: 16,107 TDO reads identical. With the Step 1.0 chain 3,483 differ; VF3's trace still passes |
| `make test_scripts` | the 53C810 regression with R3DMA in the RTL (step2 low): PASS |
| `sm3-builds/dt2/tools/sec_replay.py` | the Python model of the stream (`gen_m3_5881.block_decrypt`) against the same trace: 0 of 128,114 differ |

**The system bench** (`make -f sim/system/system.mk SYS_OBJ=sm3-builds/dt2/sysobj SYS_THREADS=4
BUS_RTL=sm3-builds/dt2/rtl/m3_bus.sv SYS_DEFS=+define+M3_STEP2_BUS system`; chain
`sm3-builds/dt2/chain.sh RUN FROM TO`). New options: `--cfg`, `--xmem`, `--fix8`, `--nvram`
(the other indexes, each a window through ioctl and checked, the rest put in straight), and
`--tb-fast-pc` (Daytona 2's boot ROM also runs FFF00198, VF3's wait-loop exit, at its first
PCI probe; with 0 the fast timebase ends at the first instruction in RAM). The DDR3 model now
covers beat 0x1000000 + 64 MiB (192 MiB of host memory, in the checkpoints).

**Wrong turns.**
- The boot first stopped at frame 1: it writes MCCR1 (MPC106 register F0) and reads it back
  to set bit 11. The bus kept no MPC106 registers (VF3 never reads the MPC105's back), so it
  read 0 and wrote 0x800. Supermodel keeps the bridge's registers as a byte array with reset
  values; the patch keeps them (an MLAB and a written bit each).
- `--tb-fast` ended at frame 1 because FFF00198 is VF3's loop exit but Daytona 2's PCI probe.
- My first Supermodel runs failed: the run directory needs `Assets` (the crosshair bitmap),
  and two `xvfb-run -a` started together collided on display :99.

**On the board (study R101):** pw4 and pw5 are black too, though pw4 closes every corner. `sysb90_s1` and all of
these leave `SDRAM_A[0-4]` and `[6-9]` unpacked, with no SDRAM I/O constraints in `Model3.sdc`. Model 2's
`Model2.sdc` (its R57) constrains that interface; the adapted block is `sm3-builds/pw/Model3_sdram_io.sdc`, not applied.
pw6 takes the write bursts out to separate that cause from placement.

**The board's corruption was the address pins (R101).** pw8a2 (every SDRAM pin packed, no bursts) runs clean at
15.3 fps. pw8b (PVR 1.1, address pins unpacked as `sysb90_s1`) is dead at boot. `Model3.qsf` now sets
`ALLOW_SYNCH_CTRL_USAGE OFF` on `m2_sdram`'s `sd_a[*]` only; on the whole instance it unpacks `dq_r` instead.

**VROM placement (2026-11-04).** `m3_system` `VROM_LOC` (0 SDRAM, 1 DDR3, 2 the second SDRAM, not built; default 1).
With 1 the loader routes the index-0 image's VROM range (0x4000000, 32 MiB) to the extra memory, and VF3's VROM is
read through `m3_xmem`, as Daytona 2's. The bench puts a VF3-size image's VROM into both memories and checks that the
download's VROM window lands where the build places it (`loader: VROM window ... in DDR3 / in SDRAM`). `--vrom-x N`
sets the readers' choice after a restore with a `VROM_LOC 0` build (`-GVROM_LOC=0` in `SYS_DEFS`), so one checkpoint
measures both placements: `sm3-builds/dt2/vf3_measure.sh`, results in `sm3-builds/dt2/vf3a/RESULT.txt`.

**Daytona 2 to frame 1,300.** At bench frame 1,304 the composite shows the attract's race demo with textures under
"INSERT COIN(S)" (`sm3-builds/dt2/shot/f001304.png`); the tile layer matches Supermodel's at 1,100 and 1,300.

**The Step 2 texture port (2026-11-05).** `r3d_texup` `STEP2_PORT` in `sm3-builds/dt2/handoff2` (patch and merge
note), connected in `m3_system` with `M3_STEP2_TEX`. The system build takes it with
`TEXUP_RTL=sm3-builds/dt2/handoff2/rtl/r3d_texup.sv`. `make -f sim/system/system.mk test_texup2` uploads random
textures through the FIFO and through the port into two instances and compares the sheets: 2 x 300 batches, 0 differ.
The Daytona 2 rerun with the port (`sm3-builds/dt2/run6`) uses the tile generator of the last commit
(`sm3-builds/dt2/tg_head`): the tree's `m3_tilegen` has gained ports (`p_long`, `p_lo`, `name_inv`, R103) that the
tree's `m3_system` does not connect yet.

**The loader bug of the VROM placement (2026-11-05).** `VROM_LOC 1` routed index 0's 0x4000000-0x5FFFFFF to DDR3
for Daytona 2 too, whose index 0 is banked CROM there; CROM slots 8-11 went over VROM blocks 0-1 (board: a hang
after a power cycle, garbage 3D otherwise). Fixed in `m3_loader` (only without index-1 VROM, flags bit 0);
`sm3-builds/dt2/handoff3`. The bench now sends index 1 through the loader at three places and index 0's window at
0x4000000 for every set, and fails if Daytona 2's CROM lands in DDR3 or overwrites VROM.
