# M2.5 viewer and the DDR3 probe: working notes

These are my notes for the work after R64-R68: the pipelined DDR3 master, the
DDR3 probe (first inside the viewer, then as its own small core), the viewer's
move to SETUP4, and textures. The study (D-33, D-34) decides; this file records
how I got the numbers and where I went wrong.

## 1. The pipelined DDR3 master (`rtl/mem/m3_ddr3p.sv`)

`m2_ddr3` takes one request at a time, so it cannot carry texture fills (R67).
`m3_ddr3p` is a clean superset of its contract (D-20):

- **Port L** is `m2_ddr3`'s consumer side exactly. Requests are held to ack, one
  at a time. The address and count are latched for the burst (Model 2 R362), a
  write's byte enables come from the source beat by beat (R661), and no request
  is taken in its own ack cycle (R660). The viewer's arbiter and every existing
  client sit on it unchanged.
- **Pipelined ports P** take a read into a two-entry command queue (`p_ack`
  means taken into the queue, not answered). The data comes back later, in
  order, one `p_rvalid` a beat on the shared `dout`. With `p_we` a P request
  is a write burst instead, its beats on `p_din`, one a `p_wacc`.
- Reads are tracked from the bridge's take to their last beat in a 32-entry (or
  64) MLAB queue. Avalon returns read data in command order, so the queue's
  head names every beat's owner.
- The command queue's head drives the pins from registers, so `p_ack`
  depends on no pin and a command can still go every cycle.
- `PIPE = 0` takes a read only when none is queued or in flight, which is
  `m2_ddr3`'s behaviour, behind the same ports.
- Debug: every read's cycles from its command's first cycle on the pins to its
  first beat; for a write, to its first beat taken. This is Model 2 R351's
  measure. Also the reads the bridge holds now, and the most it has held. A
  client sees two cycles more than the pin figure (command register, `dout`).

Unit bench, `sim/mem/tb_m3_ddr3p.cpp`: port L does random reads and write
bursts, P0 does random reads, and P1 does reads and writes. Every beat is
checked by owner against a reference memory. Five DDR3 profiles were run,
including bridges that hold at most 1, 4 or 6 reads. Each ran 2 M cycles,
about 0.8 M transactions, with 0 errors.

**Wrong turn:** the unit bench failed on its first run with thousands of wrong
L beats. The master was fine. A Python edit had put a `//` comment
mid-line in the bench and swallowed `l_k = 0;`, so each L read was checked
from the last read's beat count.

## 2. The probe inside the viewer (`rtl/viewer/m3v_probe.sv`)

The first form of the probe: arbiter port 5 writes a pattern, the pipelined
port reads it, and the overlay gets eight more rows (4-11) in decimal digits. It
runs three cases: (a) idle with the scan-out paused (an OSD option), (b) idle
with the scan-out running, (c) drawing. The overlay layout is in the module's
header. I then built a standalone core first, so this form
has not been fitted. It stays in the viewer behind `PROBE`.

The viewer moved to SETUP4 at the same time: `m3v_setup` emits 4 header words
(word 3 the geometry's header word 3) and the planes 1/w, u/w, v/w, shade
(untextured: 1/w, shade). `m3v_raster` reads the header on tag 31 and the shade
plane as plane 3 (textured) or 1. The bench checks the SETUP4 record word by
word against `rtl_tri::setup`'s planes 0, 2, 3, 4 (textured) or 0, 1.

Bench, five frames (1800, 7800, 7860, 7920, 13770), probe running: bit-exact,
0 of 190,464 layer pixels differ on each, video output matching, with BUSY 20%
and latency 8-40.

**Open (found by the new bench options):** with the DDR3 model holding only
1 or 4 reads and the probe running, frame 1800 loses 8 of 7,621 triangles. The
geometry receives every polygon RAM word right (a new per-word check), so the
loss is upstream of the set-up but not in polygon RAM. Without the probe, or
without the read limit, the same frame is bit-exact. Not yet explained; next
on the list after the probe core.

## 3. The DDR3 probe core (`quartus/ddr3_probe`, `rtl/probe/`)

A small MiSTer core, release form as the viewer. A plain load runs every case
with no input, and the results are a table on screen and lines on the UART.

### Cases (automatic sweep)

| Rows | Reads | In flight | Background |
|---|---|---|---|
| 2-6 | random 4-beat lines in 8 MiB | 1, 2, 4, 8, 16 | none |
| 7-11 | the same | 1-16 | layer: 32-beat sequential writer + reader, 22 MB/s each |
| 12-16 | the same | 1-16 | layer + store: and a 32-beat sequential reader at 100 MB/s |
| 17 | sequential 4-beat | 16 | none |
| 18 | random 8-beat | 32 | none: the most reads the bridge holds |

Each case is a 1 s window (`WIN` cycles at the clock in use).

### Screen (496 x 384, 12 x 18 cells, glyphs 10 x 14)

Row 0 (yellow): `DDR3 PROBE PASS ppppp ERR eeeee CASE kk`. CASE is the case
running next. Row 1 is the header ` N BG LN    MB/S MIN MED   MAX AC BGM`, and
rows 2-18 are the table.

- **MB/S** is `ddd.ddd`, bytes in the window / 10^6.
- **MIN / MED / MAX** are cycles from the command's first cycle on the pins
  to its first beat. MED is the median of a 512-bin histogram; 511 means 511
  or more.
- **AC** is the most of the probe's reads the bridge held at once.
- **BGM** is the background's MB/s.

Row 19 is a legend. Row 20 (cyan) is the debug row:
`I<pattern words> R<reads issued> B<beats> Y<BUSY cycles> S<engine state>`,
every 0.1 s until a case ends and again whenever none has ended for 3 s.

In manual mode the screen shows the settings on row 3, the results on row 6,
and the same title and debug row.

Engine states (S): 0 waiting for the master to be idle, 1 writing the pattern,
2 window set-up, 3 window running, 4 draining, 5 median scan, 6 results out,
7 waiting for the fixed side to take them.

### UART (ttyS1, 115200 8N1)

- Start line: build date, DDR3 base, clock, mode, the status word and the bit
  map.
- `D T.. S.. I.. R.. B.. Y.. A.. O.. Q.. L.. K.. DROP..`: progress, as row 20,
  plus the reads at the bridge (A), the probe's in flight (O), the raw
  DDRAM_DOUT_READY count (Q), L-port read beats (L), and the master's queue
  state (K: bit 7 slot 1 valid, 6 slot 0 valid, 5 slot 0 a write, 4 port L
  busy, 3-0 reads queued or in flight).
- `C kk N nn BG b LN lll SEQ s CLK ccc MBS ddd.ddd MIN MED MAX AC BGM PASS ERR`
  per automatic case; `M DIR d ADDR a LN lll N nn BG bb CLK ccc MBS ...` per
  manual window.

A 1024-byte FIFO feeds the UART. A full FIFO drops the byte and counts it; the
probe never waits for the UART.

### Status bit map (OSD and `/media/fat/config/DDR3_probe.CFG`)

| Bits | Option | Values |
|---|---|---|
| 0 | reset | |
| 1 | mode | 0 automatic sweep, 1 manual |
| 2 | manual direction | 0 read, 1 write |
| 3 | manual addresses | 0 random, 1 sequential |
| 6:4 | manual burst, beats | 0..7 = 1, 2, 4, 8, 16, 32, 64, 128 |
| 9:7 | manual in flight | 0..5 = 1, 2, 4, 8, 16, 32 |
| 11:10 | manual background | 0 off, 1 layer, 2 layer + store |
| 13:12 | DDR3 clock | 0 100 MHz, 1 125, 2 85, 3 62.5 (VARCLK builds) |
| 14 | diagnostic | 1 skip the pattern (errors not counted) |
| 15 | diagnostic | 1 the probe's reads through port L, one at a time |
| 16 | diagnostic | 1 the pattern written a beat at a time |

Any change resets the measurement.

### What it should read if the bridge behaves like Model 2's figure

Model 2 R351 measured 13 cycles typical and 50 worst: single-word reads, one at
a time, at 100 MHz, through `m2_ddr3`, at an address past the end of the memory.
Robert Peip's MiSTerDDR3Test on this board (study R69) measured min 12-13, avg
15.2 random, and max 81-99. Bursts of 4 words gave 158.3 MB/s random, one at a
time.

The probe's N = 1 random line should land just under 158 MB/s, about 145-155.
My master adds about two cycles a read (command register, registered data). If
the bridge takes several reads, MB/S rises with N toward the port's ceiling
(800 MB/s at 64 bits and 100 MHz), and AC reads N. If it takes one at a time,
AC stays 1 and MB/S stays flat.

### Where the "100 MB/s usable" and Model 2's latency ruling came from

I traced both, because D-33 goes against them:

- **Model 2 R557's "DDR3 for textures: no"** was an answer to a question
  inside an SDRAM timing entry. It was argued, not measured: "the HPS DDR3
  path's latency is well above the SDRAM's, and the walk is latency-bound".
- **The only DDR3 latency measurement in Model 2 is R351:** 13 typical, 50
  worst, one word, one request at a time. Its addresses were past the end of
  memory, and the board was idle.
- **Model 2's one attempt at textures in DDR3 (R645)** mirrored the sheets
  with one read in flight per miss port, crossed by toggles, through an arbiter
  on `m2_ddr3`. It wedged the bus (R649) and was removed before anything was
  measured. R679 notes that it predates R660's request-taken-twice fault, a
  plausible cause.
- **The "about 100 MB/s usable" in study §7.3** has no measurement behind it
  that I could find. It is consistent with one request at a time.

So nothing so far measured a pipelined master on this board. The probe is the
first.

### The scaler's share

`ascal` sits on its own f2sdram port (vbuf, 128-bit) and always moves full
bursts. BLEN = N_BURST / N_DW * 8 = 2048 / 128 * 8 = 128 beats of 16 bytes,
2,048 bytes; `avl_burstcount` is fixed at BLEN. The input is 24 bits a pixel
(the `format` default, not overridden in sys_top).

For a 496 x 384 core:
- **write:** one burst a line (496 x 3 = 1,488 bytes, under one burst):
  384 x 2,048 x 57.52 = 45.2 MB/s;
- **read:** each input line once per output frame: 384 x 2,048 x 60 = 47.2 MB/s
  at a 60 Hz output (45.2 MB/s with the output locked to the core).

That is about 90-92 MB/s on the same DDR3 controller, in every case. The probe's
figures already include it; R67's model left it out.

### Board runs and what went wrong

**Build 1 (fit_s4-6, md5 8856515f / f28b25fe / f537998d):** hung at once.
After 25 s and 50 s the screen showed PASS 00000 and every number 000.

**Wrong turn (fit_s1-3):** the first batch failed timing at 88-90 MHz. The
text display divided the beam's x by 12 combinationally. I replaced the divide
with exact reciprocal multiplies, (x 683) >> 13 and (y 911) >> 14. I proved
them exact for every x < 2048 and y < 1024, and the screen came out
pixel-identical. Seeds 4-6 then closed at 117-120 MHz.

**Wrong turn:** the first fit of all failed in analysis. Quartus 17 rejects
`~25'(o)` inside a concatenation, which Verilator accepts. And the copied qsf
carried the viewer's `m3_viewer.sv` from after its SDC line, so `emu` was
declared twice.

**Build 2 (fit_s7-9, be6f8c3f on the board):**
- The DDR3 master's reset was split from the core's: the master now comes out
  of reset on PLL lock and stays out.
- Writes were cut to 32-beat bursts.
- The debug row and the UART were added.
- The bench now enforces the port's rules (any break fails it): RD and WE never
  together; an untaken command holds; a write burst keeps its address and
  count with no read until its last beat; burst counts 1-128. It also pulses
  resets with reads in flight.

On the board the UART showed the pattern complete (I = 1,048,576, 31,016 BUSY
cycles), then exactly one read issued and taken by the bridge (A01), and no
beat back for 46 s.

So the bridge takes the read and never answers. The bench's model answers
every read, so it cannot show this.

**Build 3 (diagnostic, VARCLK = 0) adds three status-bit discriminators:**
- skip the pattern (no writes before the first read);
- reads through port L (`m2_ddr3`'s one-at-a-time shape, which the viewer
  proved on the board);
- the pattern a beat at a time (Peip writes only single beats).

The UART also gains the raw DOUT_READY count (Q), so I can see whether beats
arrive at all and are misrouted, or never arrive.

**Wrong turn found by the reset bench:** in the two-clock form, the core's
reset also reset `cfg_tgl`, the PLL rewrite request. A toggle back to 0 looks
like a request, so a core reset could rewrite the PLL and reset the DDR3 master
in the middle of a write burst. The bench caught it as 5.4 M port-rule breaks.
The request and the record of the applied clock now see power-on reset only.

**Build 3 on the board (fit_s11, 8284efe3), one CFG bit at a time:**

| Setting | What the UART showed |
|---|---|
| default | pattern done; one read taken (A01); Q = 0 beats at the master, ever |
| bit 14, no pattern at all | the same: one read taken, Q = 0 |
| bit 16, single-beat pattern | six writes taken, then BUSY high for good |
| bit 15, reads through port L | the same as default: Q = 0 |

So the write side and my pipelined read path were both cleared. The port took a
few commands and never serviced them.

**The cause: Main releases the FPGA-to-SDRAM ports only after the core is running.**
`Main_MiSTer` `fpga_io.cpp` loads an RBF as `do_bridge(0)` (fpgaportrst = 0,
every FPGA port held in reset), `socfpga_load()` (the core starts and its PLL
locks), and only then `do_bridge(1)` (fpgaportrst = 0x3FFF). Nothing in the
fabric shows the release: sysmem's `reset_out` (the emu's RESET) is
`init_reset_n`, `hps_h2f_reset_n` and the core-reset request, not fpgaportrst.

The probe touched DDR3 within microseconds of lock, inside that window. A
command taken then is never answered, and the port stays wedged. The viewer
worked only because it touches DDR3 seconds later, after the files.

Not quite: its scan-out reader requests lines from lock. It survived, I
think, because the port refuses rather than takes commands for part of the
window. That is luck, not design.

Rule for every DDR3 client, the main core's included: start only after
hps_io has been spoken to (io_enable on HPS_BUS, which Main does after
`do_bridge(1)`), plus 100 ms. The probe core and the viewer now both gate on it.
The probe also keeps a 0 / 1 / 4 s extra delay (bits 18:17) to confirm the
margin on the board.

**Wrong turn, mine:** I read "RESET is sysmem's reset_out" and took the memory
rule (master out of reset on PLL lock and stays out) to be the whole story. The
rule is right for my own logic. It says nothing about the far side of the port,
which the HPS holds in reset after the core is already running.

**Build 4 (fit_s12, a31349d7), the port gate in:** the bridge answered. Q = 4,
exactly the probe's 4-beat line, but all four beats were credited to port L
(L = 4, B = 0), and the read never completed (A stayed 1). The read-owner queue's
head had read back as all zeros: owner 0 (port L) and burst count 0.

That queue was a 64-deep MLAB read asynchronously. Quartus 17 builds it as two
32-word halves with a read mux (altdpram, dpram_r8o1). Verilator cannot see how
the hardware is built, so the bench passed.

**Build 5 (fit_s14 / s15):** the queue is read synchronously. The register holds
the head, read a cycle ahead (the entry that will be the head next cycle), with
the write forwarded when it lands on that entry in the same cycle. The probe's
own 64-deep address FIFO got the same treatment. The unit bench passes with read
latencies of 1-3 cycles, which exercise the forwarding.

Rule, pending the board: no asynchronous read of a 64-deep MLAB. My own
asynchronous MLABs in the textured raster (32 and 8 deep) are now synchronous
too. Other blocks with asynchronous MLAB reads, all 32 deep or less, are
unchanged:
- `r3d_rast_tcache`: fill queue 8, beat buffer 32;
- `r3d_rast_interp`: delay line 16;
- `r3d_rast_band`: flush queue 8;
- `m3v_ddr_rd`: 16, board-proven in R64.

**Timing for a 125 MHz DDR3 clock.** At 100 MHz the engine's worst paths left
1.0-1.6 ns:
- the median scan: histogram read, a 32-bit add and a compare, 8.95 ns;
- the held-reads maximum: 8.4 ns;
- the pattern compare (an adder into a 64-bit compare).

All three are now pipelined, for the variable-clock build, which is timed at
125 MHz:
- the sum is compared a cycle later, so the bin is the one before;
- the maximum is taken from the registered count;
- the compare's operands are registered.

The median stays exact: at a fixed latency of 13, N = 1 and 2 read MIN = MED = 13.

**fit_s24 (VARCLK, cap 32, 64-entry register queue) missed 125 MHz** by
0.391 ns (TNS -2.3 ns); the fixed 100 MHz side passed at +0.881 ns. Every one
of the worst paths ran from the f2sdram block's DOUT_READY flop (FF_3804) into
`m3_ddr3p`'s queue head `rh`. The path was DOUT_READY -> r_last (the beat
count compare) -> rp_next -> the queue's 64:1 read mux -> rh. The late port
signal sat at the front of the mux tree instead of at its end.

The fix, for the register queue only (RQREG = 1):
- both candidates, the head staying (rq[rp]) and the next one (rq[rp+1]), are
  read from registers, each with its own forward of a read taken that cycle;
- r_last only picks between them in the last mux.

The probe now uses a 32-entry queue (QB = 5), enough for its top cap of 32.
The MLAB form (the viewer's) is left as it was, so the viewer's master is
unchanged logic.

Benches after the change:
- `tb_m3_ddr3p` at QB 5 and 6, MLAB and register queue, five profiles each;
  latency 1-1, 1-3 and 1-2 at 70% BUSY stress the forwards: all pass.
- A planted fault, the rp+1 forward removed, fails at latency 1-1.
- The probe bench at cap 32: five profiles pass, 0 errors, with 125 MHz at
  ratio 5/4 from the start and switched mid-run.

fit_s25 (seed 2, VARCLK, cap 32) closes:

| Item | fit_s24 | fit_s25 |
|---|---|---|
| DDR3 domain at 125 MHz, slow corner | -0.391 ns | +0.591 ns (135.0 MHz) |
| Fixed 100 MHz side | +0.881 ns | +1.570 ns |
| ALM | 9,679 | 9,554 |

Neither build has critical warnings. fit_s25's worst DDR3-side path is now the
probe's own: the arbiter's owner into the address FIFO head (`af_h`), not the
port. The RBF md5 is 107f7daf9a438d434199d97d5a54912d. It is not for the board
until the port has been power-cycled and the cap steps are proved.

**Build 5 on the board (fit_s14):** the same hang. The synchronous read did not
fix it.

**Build 6, a pair:**
- fit_s16: `m3_ddr3p` capped at 1 read, its queue in plain registers, the
  queue's fields on the UART (E).
- fit_s17: `m2_ddr3` itself, unchanged, behind port L.

Both hang. On s16 the queue entry is right (owner 1, 4 beats), and rbeat = 4.
The four beats arrived before the master had registered its take, so they were
counted against an empty head and given to port L. On s17 the four beats
arrived while `m2_ddr3` was still in D_ISSUE waiting for BUSY to fall; BUSY
never fell, and the master never saw its own take.

So both masters see the bridge accept the read later than it really did, or
never. That is not a master bug: something between the core and the HPS port
is out of step with the core. The lead is sysmem's `f2sdram_safe_terminator`.
While its reset (sysmem's `reset_out`, the emu's RESET) is high, it masks the
core's read and write, can present a latched read of its own, and passes
waitrequest straight through.

**Wrong turn (several builds):** I took the take as the master's only truth.
The bench's model answers exactly what the master presents, so it could never
show a port that accepts behind the master's back.

**Build 7 (fit_s18 / s19, the same pair) gates on RESET too:** DDR3 waits for
RESET to be low for 100 ms without a break, on top of the hps_io gate. New
pin-level counters in the emu:

| Field | What it counts |
|---|---|
| X | RESET's rising edges |
| Z | cycles since RESET's last edge |
| W | RESET at the first read taken (2: none yet) |
| P | cycles from the first RD on the pins to its take |
| V | cycles from the first RD to the first DOUT_READY |

V < P means data before the take. The debug line also goes out while the probe
waits at the gate, so a RESET held high shows on the UART.

**The finding behind all of it (01:5x):** the board's DDR3 port had been wedged
since the first probe builds, so every board result after them is tainted. The
evidence:
- fit_s18 showed X000 (RESET never rose) and W0 (RESET low at the take);
- its read was taken at once (P0), and V counted for ever, so it was never
  answered;
- its Q4 / L4 were four beats that arrived before our first read: data left
  over from an earlier core's read.
- Peip's DDR3Test, which ran at 158 MB/s on this board the night before, then
  showed 0 transfers.

The likely sequence: the first probe builds touched DDR3 at PLL lock, before
Main's `do_bridge(1)` released the port. That wedged the HPS SDRAM
controller's FPGA port. Main's `do_bridge(0)` / `(1)` on later core loads does
not clear it; only a power cycle does.

**Wrong turns this cost:** I chased the read-owner queue three times:
- the MLAB's asynchronous read;
- a synchronous read with forwarding;
- plain registers;
and the RESET terminator once, all on a port that could not answer. The fixes
made along the way are kept, because each removes a real hazard:
- the queue in registers or read synchronously;
- the two gates;
- the master out of the probe's reset.
None of them was the cause of the board hang.

**The rule for every DDR3 client (for the study):** no access until hps_io has
been spoken to (Main has released the port) and RESET has been low for 100 ms
without a break. RESET high closes the gate again. A port wedged by an early
access survives core reloads and needs a power cycle. Now in:
- the probe core (`ddr3_probe.sv`: ddr_ok holds the engine);
- the viewer (`m3_viewer.sv`: ddr_ok holds the scan-out's line requests, the
  in-viewer probe, the loader (as memory not ready) and the draw);
- `m3_ddr3p` never issues on its own.

The timing of the port's handshake was checked in fit_s18's database. BUSY
comes from the f2sdram block's `cmd_ready_1` flop into the take's enable
(`rq_wp`) on our clock: +3.2 ns setup, +2.19 ns hold. DOUT_READY comes from
`rd_valid_2`: +6.5 ns. The paths are timed, and nothing in `sys_top.sdc` cuts
them.

## 4. Textures (step 2)

The viewer's white-texel shader is replaced by the real blocks:
- `r3d_rast_interp` with its reciprocal;
- `r3d_rast_texel` twice (see below);
- `r3d_rast_tcache` (32 KiB) on `m3_ddr3p`'s second pipelined port;
- `r3d_rast_colour` (fog off);
- `r3d_rast_band` for the two band buffers, the blend and the overlapped flush.

The loader puts each frame's texture RAM in DDR3, tiled as the cache's lines.
Line {y / 4, x / 4} is four beats, row y % 4 a beat, texel x % 4 a lane, at
word TEX(k) + {y[10:2], x[10:2], y[1:0]}, with TEX(k) = 0xC00000 + 0x100000 k.
A row of four texels in the file is one beat, so the loader gathers them as it
does polygon RAM.

First bench run, frame 7860: 0 of 190,464 pixels differ against render_d25
with textures (level 0, fog off), the video output matches, 262,277 pixels
shaded by both. The image equals `png/f007860_d25tex.png` pixel for pixel.

### Benches (step 2)

All five frames (1800, 7800, 7860, 7920, 13770) are bit-exact against render_d25
with textures (level 0, fog off) under these DDR3 profiles:

| Profile | Probe | Result |
|---|---|---|
| BUSY 20%, latency 8-40 | on, video and overlay checked | every pixel, every set-up record |
| BUSY 50%, latency 13-120 | on | every pixel, every set-up record |
| BUSY 40%, latency 13-50, a bridge holding 16 reads | off | every pixel, every set-up record |
| fixed latency 60, a 16-read bridge | off | every pixel, every set-up record |
| BUSY 30%, latency 30-120, an 8-read bridge | off | frames 2-5 exact; frame 1800 loses triangles |

The frame 1800 loss is in `r3d_geo`. Every culling, VROM and polygon RAM word it
receives matches the files, and it emits 7,613 of 7,621 triangles. So its output
depends on when its polygon RAM data arrives (several hundred cycles late, with
gaps). It is the geometry owner's to look at.

Reproduce: the viewer bench `--no-loader --busy 30 --lat 30 120 --maxrd 8
--seed 3` on f001800, or the step-1 bench with the probe, `--busy 30 --lat 2 60
--maxrd 1 --seed 3`.

### Fit (sm3-builds/viewer/fit_s4-6, three seeds, before the RESET gate)

102.4 / 101.7 / 103.3 MHz, no critical warnings, 26,281-26,338 ALM, 183 M10K,
79 DSP. The worst paths are in `m3v_sdr_rd` (the viewer's temporary SDRAM
reader), not in the new blocks.

| Block (inside the viewer) | ALM | M10K | DSP |
|---|---|---|---|
| r3d_rast_interp (with its reciprocal, 134-149) | 1,140-1,155 | 0 | 15 |
| r3d_rast_tcache, 32 KiB | 859-874 | 42 | 0 |
| r3d_rast_texel, filter instance | 424-426 | 0 | 12 |
| r3d_rast_texel, address instance (filter pruned) | 149-151 | 0 | 0 |
| r3d_rast_colour, fog off | 59-60 | 0 | 9 |
| r3d_rast_band | 259 | 20 | 2 |
| m3v_raster's own (tables, rings, FIFO, serialiser) | 366 | 17 | 0 |
| m3_ddr3p (two pipelined ports, 32-entry MLAB queue) | 621-628 | 0 | 0 |
| m3v_ddr_arb (six ports) | 50 | 0 | 0 |
| m3v_probe (in the viewer) | 556 | 1 | 0 |

Against R64's build (fit_s1):
- **The raster** went from 3,290 ALM, 25 M10K, 13 DSP to 6,379-6,397 ALM, 91
  M10K, 40 DSP.
- **Freed by the replacement:**
  - the temporary shader with its colour band, flush and table: 535 ALM,
    13 M10K, 9 DSP;
  - the second Z walk on the shade plane (`u_zb`): 179 ALM, 2 DSP.
  - Together: 714 ALM, 13 M10K and 11 DSP.
- **The master and arbiter:** `m2_ddr3` and its arbiter were 80 + 384 ALM;
  `m3_ddr3p` and the six-port arbiter are 628 + 50 ALM.

### Refit with both DDR3 gates (sm3-builds/viewer/fit_s7-9)

This is the textured viewer with the hps_io and RESET gates on every DDR3
client, seeds 1 / 2 / 3. The gates cost nothing measurable.

| Seed | Folder | Fmax | Setup slack | Hold slack | ALM | M10K | DSP | RBF md5 |
|---|---|---|---|---|---|---|---|---|
| 1 | fit_s7 | 104.48 MHz | +0.429 ns | +0.167 ns | 26,276 | 183 | 79 | 382c2b8620566989e9af5ae4b2ae29eb |
| 2 | fit_s8 | 101.85 MHz | +0.182 ns | +0.128 ns | 26,331 | 183 | 79 | 3901706a0adcadc9ef7205bf2e55125f |
| 3 | fit_s9 | 103.20 MHz | +0.310 ns | +0.160 ns | 26,308 | 183 | 79 | 7bf3a478812d9445c28670626f8dccba |

No critical warnings, and no clock with negative slack in any corner. None of
these has been on the board. The board's port must be power-cycled first.

### Area techniques (study 8.4) in the new work

| Technique | Where | Effect |
|---|---|---|
| Storage out of flops | the per-pixel data from the interpolants to the band: three M10K rings by a 7-bit pixel number, so only the number rides through the cache and both pipelines | about 150 bits a pixel x ~40 stages of registers avoided, for 5 M10K; the cache's pixel FIFO stays at its 7-bit payload (2 M10K, not 5) |
| Storage out of flops | the triangle's planes and attributes: two M10K tables by tag, read once each (as the pixel enters the interpolants, and as it leaves them) | 343 + 56 bits a triangle in 11 M10K; the interpolants carry a 9-bit tag, not 64 bits of attributes |
| Narrower / exact | plane dx, dy stored as bits [30:0], the bits `r3d_rast_interp` uses | 111 bits a plane, not 129: 9 M10K, not 10 |
| Built once and shared | one zspan on the 1/w plane; the interpolants compute the shade, so R64's second zspan is gone | -179 ALM, -2 DSP |
| Built once and shared | `m3_ddr3p`'s one command queue and one read-owner queue serve port L and every pipelined port | |
| Time-shared | the probe: one sequencer, one converter (double dabble) and one read port of each memory for every test and every printed value; the probe core's reporter, one line printer for the screen and the UART | |
| MLAB / M10K trades | the band FIFO and the word delay line in MLAB, read synchronously after the board finding | |
| Parameters | `m3_ddr3p` PIPE (one at a time as `m2_ddr3`), MAXQ (the in-flight cap, stepped on the board), RQREG (queue in registers); the viewer's PROBE; the probe core's VARCLK and PROBE_M2 | |
| What didn't work | a split port in `r3d_rast_texel` would save the address instance's 150 ALM, but that file was being changed for the mip work at the time; the address stages are built twice instead | +150 ALM, recoverable |
| What didn't work | the probe's first text display divided by 12 combinationally (88-90 MHz); exact reciprocal multiplies fixed it | |

### Planted faults (sim/viewer/faults_viewer.py, frame 7860 unless noted)

R64's five white-shader faults went with the shader. Eight new ones are in the
textured raster, plus the texture tiling, the master and the earlier set:

| Fault | Where | Result |
|---|---|---|
| S1 a02 sign, S3 sort | set-up | caught (9,314 / 9,395 records wrong) |
| S2 flat ps | set-up, flat triangles | not caught, on 7860 (no flat triangles) or 1800 (6 flat, whose two vertices the fault confuses carry the same value): equivalent on these frames, as in R64 |
| T1 alpha order, T2 band skip | store | caught |
| R1 shade plane, R2 1/w exponent | interpolant inputs | caught (188,857 / 188,221 pixels) |
| R3 word lag | the cache's words to the filter, one cycle early | caught |
| R4 texture x / 32 bits | table 2 | caught |
| R5 contour Z | contour written without the alpha verdict | caught |
| R6 mirror u / v swapped | table 2 | caught |
| R7 ring C off by one | pixel place for the band | caught |
| R8 texture flag | the cache told no bank is used | caught |
| A1, A2, C1, L1 | readers, culling base, polygon RAM lanes | caught |
| L2 texture tile beat bits | loader | caught, after a bench fix (below) |
| M1 master beat count | `m3_ddr3p` r_last a beat early | caught: the viewer hangs (no frame in 15 min) |

18 of 19 caught; the miss (S2) is equivalent on these frames.

**Wrong turn (bench):** L2 was at first not caught. The loader check's windows
in texture RAM were rows 0 and 2047 only. A 4-row window from the first non-zero
byte overlapped row 0's boundary window and was dropped. In rows with y mod 4
of 0 or 3 the beat swap changes nothing. The texture window now covers rows 1-8
(16 KiB), and L2 shows 3,114 wrong words.

## 5. The geometry's set-up and the nearest mip level (R75, R76)

The viewer now takes its set-up from the geometry and samples the nearest mip
level:
- `r3d_geo #(.SETUP_OUT(1), .LOD_WORD(1))` sends one set-up record a
  triangle: header words 0-4 (word 0 with n = 3, word 4 the LOD word B), x1 y1
  x2 y2 x3 y3 sorted by y, det, then pdx, pdy, ps a parameter. That is 24
  words textured and 18 not. Viewport records (n = 0, 6 words) still come.
- `m3v_setup` is deleted. `m3v_store` takes the geometry's stream and builds
  its own word 0, `{9'd0, nw, ymax, ymin}`: ymin and ymax are m3v_setup's
  `ffloor` of y1 and y3, and the list is `{w0[1:0], w0[2]}`. It drops the
  viewport records.
- `r3d_rast_intake`, `r3d_rast_interp` and both `r3d_rast_texel` instances
  have `MIP = 1`. The intake's fifth header beat ({highest level, K}) goes into
  triangle table 1 beside the planes, so the interpolants get `i_lk` and
  `i_lvm` with the pixel. The level `o_lv` rides with the pixel to the first
  texel instance, and through ring A to the second.
- `rtl/real3d/r3d_ucode_su.sv` (the ROM with the set-up kernels) is in the qsf
  and the bench's file list.
- Both DDR3 gates (R71) are unchanged, and so are the frame files `vf3.v3v`
  and `vf3.v3f`: the loader and the memory map did not change.

**Done differently from the recipe I was given.** The recipe's record indices
(y1 and y3 at words 5 and 9, nw = 23 or 17) are for a 4-word header. With
LOD_WORD on, the header is 5 words. So the store reads y1 at word 6 and y3 at
word 10, and nw is 24 or 18. The header length is a store parameter (`LOD`).
nw is the number of words the store took, not a value chosen from the texture
bit; the bench checks it against 12 + 3 x planes.

**The overlay.** Row 4 was `{set-up triangles, set-up drops}`. It is now
`{triangles stored, 0}`. The geometry drops a non-finite triangle itself and
does not count it, so there is no drop count to show.

### First check: the geometry with both parameters on

R75 benched LOD_WORD, and R76 benched SETUP_OUT, but not both together. Before
touching the viewer I built `tb_r3d_geo` with `-GSETUP_OUT=1 -GLOD_WORD=1`
(`--setup-out --lod`):

| Run | Frames | Result |
|---|---|---|
| VF3, latency 12-20, 20% stall | 400 | exact, 2,313,148 triangles, p99 77.0% of a 100 MHz frame |
| Bass Fishing | 400 | exact, 1,389,845 triangles, p99 71.4% |
| Daytona 2 (step 2.1) | 400 | exact, 2,868,892 triangles, p99 117.7% |
| R73's hostile profiles 1, 2 and 5, each game | 20 each | 9 of 9 exact |

The LOD word adds about 0.1 points of frame load to R76's figures (76.9 /
71.3 / 117.6%).

### Bench (tb_m3v.cpp)

New checks:
- **Every word the geometry sends**, in order, against the geometry model with
  `lod_word` and `rtl_tri::setup`:
  - the header fields: pass, alpha, polygon, the ten texture and colour fields,
    the inverted-copy flag, fog intensity, layer;
  - word 3's top bits zero;
  - the LOD word against the model's B;
  - the 19 or 13 set-up words;
  - the viewport records' count (7 a frame) and length.
- **In the store,** besides the set-up words: the LOD word, nw, and the rows
  (floor(y1), floor(y3) clamped).
- **The reference** is `render_d25` with `mip = 1`, `lod = 1`, and
  `lodb` = B from the geometry model. `tr.poly` is the triangle's own index, so
  each triangle takes its own record's B. A double-sided polygon's two copies
  carry +B and -B, so B by polygon number would be ambiguous.
- `--busyrun N`: each time BUSY rises it stays up for 1 to N cycles. This gives
  the late acknowledges R73 found the geometry sensitive to; the bench's plain
  BUSY never holds a request more than a few cycles. With N = 0 the random
  sequence is the old one, so earlier runs reproduce.

### Results: all five frames, textured at the nearest level

Every profile: 0 of 190,464 layer pixels differ on each frame, every
geometry word and every stored record is exact, and the list counts match.

| Profile | DDR3 | Probe | Result |
|---|---|---|---|
| p0 | BUSY 20%, latency 8-40, loader windows, video | on | exact, video output matches, loader 0 wrong |
| p2 | BUSY 50%, latency 13-120, video and overlay | on | exact |
| p3 | BUSY 40%, latency 13-50, 16-read bridge | off | exact |
| p4 | latency fixed at 60, 16-read bridge | off | exact |
| p5 | BUSY 30%, latency 30-120, 8-read bridge, seed 3 (R74's failing case) | off | exact |
| h1 | late acknowledge: BUSY 30% in runs of up to 400 cycles, latency 30-120, 1 read held | off | exact; frames 1.8-2.3 s (7860: 2,222 ms), geometry phase up to 189 ms |
| h2 | late acknowledge: BUSY 10% in runs of up to 300 cycles, latency 1-200, 2 reads held | on | exact; the probe saw a first beat as late as 1,529 cycles |

### Frame time at 100 MHz

Against R74's build (level 0, `m3v_setup`), rebuilt from the same tree with
the old store, the old core wiring and MIP = 0, on the same p0 profile:

| Frame | R74: geometry phase / frame | Now: geometry phase / frame | Geometry held by the next stage, R74 / now |
|---|---|---|---|
| 1800 | 54.5 / 267.1 ms | 22.3 / 233.2 ms | 4.91 M / 0.63 M cycles |
| 7800 | 60.4 / 302.3 ms | 18.9 / 256.2 ms | 5.58 M / 0.21 M |
| 7860 | 65.3 / 330.7 ms | 18.7 / 280.5 ms | 6.11 M / 0.22 M |
| 7920 | 69.2 / 351.6 ms | 18.3 / 294.4 ms | 6.51 M / 0.24 M |
| 13770 | 13.6 / 64.5 ms | 6.8 / 58.8 ms | 1.08 M / 0.03 M |

- **The geometry phase is 2.0-3.8 times shorter.** R64 found the set-up held
  the geometry for 6.66 M of its 7.07 M busy cycles; now the store holds it for
  12% of its busy cycles on 7860, because it writes one 13-beat DDR3 burst a
  record.
- **The band phase barely moves.** On 7860 it is 261.8 ms, against 265.4 ms at
  level 0: the mip level's fewer fills save 1.4%.
- The frame is still 0.23-0.29 s, because every band replays every triangle
  (R64). That waits for D-20's bins.
- Slower memory stretches it: on 7860 it is 273.5 ms on p3, 415.1 ms on p4,
  495.0 ms on p5 and 536.5 ms on p2.

### Fit (sm3-builds/viewer/fit_s10-12, release form, both DDR3 gates)

`fit.sh` now starts each seed on its own as soon as a Quartus slot is free (any
quartus_map, _fit, _asm, _sta or _cdb counts), never while
`sm3-builds/QUARTUS_PRIORITY` exists. Before, it waited for three free slots at
once and starved behind other projects' jobs. After each flow it reads Fmax,
setup and hold at both slow corners, because the qsf has multicorner analysis
off. The database is kept if either corner's report is missing.

| Seed | Folder | ALM | M10K | DSP | Fmax 100C | Fmax -40C | Setup 100C | Hold, worst corner | RBF md5 |
|---|---|---|---|---|---|---|---|---|---|
| 1 | fit_s10 | 25,245 | 196 | 72 | 101.62 MHz | 100.68 MHz | +0.159 ns | **-0.012 ns** (-40C) | 5ab99bcd7c8cf74202e53f6ec2f37e3d |
| 2 | fit_s11 | 25,209 | 196 | 72 | **99.88 MHz** | **98.33 MHz** | -0.012 ns | +0.053 ns | 0ec1322efe177bf8f5436acbad968b24 |
| 3 | fit_s12 | 25,226 | 196 | 72 | 102.17 MHz | 104.66 MHz | +0.212 ns | +0.165 ns | 2aba8ce64c66bd1d0f5d723e29c1b4ce |

- **Seed 3 is the board candidate.** It meets both corners for setup and hold.
  Its RBF is saved as `sm3-builds/viewer/su_mip_s12_2aba8ce6.rbf`.
- **Seed 2 misses 10 ns** at both corners: -0.012 ns at 100C and -0.170 ns at
  -40C. The path is `r3d_rast_zband`'s a_y into `r3d_rast_zspan`'s output
  registers (p_z0, p_y, p_v0): the zband's row scoreboard compare gives a_go,
  then s_ready, then the viewer's pair_take, then zspan's p_ready and the enable
  of its wide output registers. That is a ready chain through three blocks with
  no register. Neither block changed in this work; the fitter placed it worse.
  A registered ready (a skid stage between zspan and zband) would take it out.
- **Seed 1 has a hold of -0.012 ns at -40C,** inside `r3d_rast_recip` (m6 to m7).
  The flow times only the slow 100C corner, so the fitter does not repair hold
  at -40C. R74's fits never looked at -40C, so I cannot say whether they had
  the same.
- The worst 100C paths in seeds 1 and 3 are the scan-out's and the store's
  DDR3 requests into the f2sdram block (+0.159 ns), and the front's span ends
  into the span queue's M10K (+0.212 ns).
- The one critical warning in seeds 1 and 3 is the framework's (`ascal` port
  `mode`, 4 elements to 5), which R74's fit_s7 has too. Seed 2's other two are
  its timing.

Against R74 (fit_s7-9: 26,276-26,331 ALM, 183 M10K, 79 DSP; 101.9-104.5 MHz at
100C): -1,031 to -1,122 ALM, +13 M10K and -7 DSP. The 100C Fmax fell by 2-3 MHz
a seed.

| Block (inside the viewer) | R74 seed 1 (fit_s7) | Now, three seeds | Change |
|---|---|---|---|
| `m3v_setup` | 1,787 ALM, 0 M10K, 7 DSP | deleted | -1,787 ALM, -7 DSP |
| `r3d_geo` (SETUP_OUT, LOD_WORD) | 7,391 ALM, 32 M10K, 11 DSP | 7,696-7,702 ALM, 43 M10K, 11 DSP | +305 to +311 ALM, +11 M10K |
| `m3v_store` | 1,093 ALM | 1,170-1,176 ALM | +77 to +83 ALM |
| `m3v_raster` in total | 6,289 ALM, 91 M10K, 40 DSP | 6,679-6,708 ALM, 93 M10K, 40 DSP | +390 to +419 ALM, +2 M10K |
| - `r3d_rast_intake` | 249 | 312-320 | +63 to +71 |
| - `r3d_rast_interp` | 1,152 | 1,124-1,153 | about 0 |
| - `r3d_rast_texel`, filter instance | 424 | 600-603 | +176 to +179 |
| - `r3d_rast_texel`, address instance | 155 | 294-296 | +139 to +141 |
| - the viewer's tables and rings | | | +2 M10K: table 1 at 364 bits (10 blocks), ring A at 83 (3) |

- **The set-up move** is -1,405 to -1,393 ALM, -7 DSP and +11 M10K (m3v_setup,
  the geometry and the store together). That is the -1,430 ALM, -7 DSP and +11
  M10K I was told to expect.
- **The nearest level** is +390 to +419 ALM and +2 M10K. R75 measured +263 to
  +273 ALM with one texel instance; the viewer has two, so +450 or so was
  expected. Both M10K are the viewer's own, and one of them is recoverable (P4,
  above).

### Planted faults (sim/viewer/faults_viewer.py, frame 7860)

S1-S3 went with `m3v_setup`. The set-up's own faults are now the geometry's
(R76: 88 of 97). New ones:

| Fault | Where | Result |
|---|---|---|
| G1 ymin from x1 | store | caught: 9,404 stored records wrong |
| G2 ymax from y2 | store | caught |
| G3 list from bit 3 | store | caught |
| G4 viewport records kept | store | caught: 3 lists miscounted, 7,283 records |
| G5 store told a 4-word header | core | caught |
| G6 nw one short | store | caught |
| G7 the inverted copy's B not negated | r3d_geo emitter | caught first by the geometry-word check: 1,349 words |
| P1 the LOD terms from header beat 3 | raster | caught |
| P2 level 0 into the address instance | raster | caught |
| P3 level 0 into the filter instance | raster | caught |
| P4 u, v [27:21] dropped from ring A | raster | **not caught: equivalent** (below) |
| P5 highest level one low | raster | caught |

The R74 set, unchanged, is all caught again: T1, T2, R1-R8, L1, L2, M1, A1,
A2, C1. R7 (ring C off by one) now hangs the band (the bench's frame
timeout) rather than giving wrong pixels. M1 hangs as before.

27 of 28 caught. The miss, P4, is equivalent at any level.

**P4 is equivalent, and it shows ring A is wider than it needs to be.** In
`r3d_rast_texel`'s mip form, the coordinate is (u >> L) >> 8 masked to the
level's side, (32 << w) >> L, doubled when mirrored. The shift and the mask
move together, so the highest u bit read is 12 + w + mirror, at most 20 for a
size code of 7. The fraction reads u bits up to 11 + w (L at most 4 + w). So
u[20:0] and v[20:0] are all the filter instance can use, at any level. Ring A
can stay at R74's 21 bits a coordinate, 69 bits in all, which fits two M10K
instead of three. A copy with the 69-bit ring is exact on all five frames
(BUSY 30%, latency 30-120, 8-read bridge). I found this after the fits had
started. The tree and the
fitted RBFs keep the 83-bit ring, which is what the benches passed; narrowing
it is the next change, and it saves 1 M10K.

### Area techniques (study 8.4) in this change

| Technique | Where | Effect |
|---|---|---|
| Built once and shared | the set-up runs on the geometry's pool (R76) | `m3v_setup`'s multiplier, adder and reciprocal gone |
| Storage out of flops | the LOD terms (21 bits) ride in triangle table 1 by tag, not with the pixel | 21 bits a triangle in M10K, instead of 21 bits a pixel through the interpolants |
| Narrower / exact | the level is 4 bits from the interpolants to both texel instances | |
| What didn't work | ring A at 83 bits; 69 is exact (P4) | +1 M10K, recoverable |
| What didn't work | table 1 at 364 bits is one M10K over 360 (nine blocks' width) | +1 M10K; no field found that can narrow by 4 bits exactly |

### Wrong turns

- I wanted to stop the first two fits, narrow ring A and rerun them, but could
  not stop them. So the batch stands on the 83-bit form, the tree matches the
  RBFs, and the narrowing is a follow-up.
- My first ring A width (u, v [27:0]) came from the texel block's port note
  ("u >> L taken 19 bits wide"), without working out which bits the masks
  leave. The planted fault P4 showed it.
- The recipe's word numbers assumed a 4-word header (above). I caught this
  from the intake's header (HW = 5 with MIP) before the first build.
