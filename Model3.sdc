derive_pll_clocks
derive_clock_uncertainty

# core specific constraints

# The CPU's reset crosses from the template's 20 MHz domain through a 3-flop
# synchroniser (cpu_rst_sync); the first stage is not a timed path.
set_false_path -to [get_registers {*cpu_rst_sync[0]}]

# The Model 3 system: the reset synchronisers take RESET and the PLL lock
# asynchronously; their first stages are not timed paths.
set_false_path -to [get_registers {*emu|mem_rst_s[0]}]
set_false_path -to [get_registers {*sys3|rst_s[0]}]

# ----------------------------------------------------------------------------
# The SDRAM interfaces (study R187)
# ----------------------------------------------------------------------------
# SDRAM_CLK is outclk_2 (clk_sys's rate, 0.5 ns before its 180 degrees: 3500 ps at 125 MHz) and SDRAM2_CLK outclk_3
# (0.75 ns before: 3250 ps), each from its pin's DDIO output register (m3_sdr_clk, u_sdclk1 / u_sdclk2 in Model3.sv),
# so each pin has a generated clock from its PLL output, with the path through the clock network, the DDIO and the
# output buffer as its latency. These always exist (they constrain no path by themselves).
#
# The I/O delays are Model 2's set (sega-model2-mister Model2.sdc, its R57: W9825G6KH-class parts):
#   input  max = tAC 5.4 + 1.0 PCB = 6.4 ns, min = tOH + PCB = 1.0 ns   (SDRAM_DQ / SDRAM2_DQ)
#   output max = tSU 1.5 ns, min = -tH = -0.8 ns   (address, bank, command, DQ, DQM, CKE)
# against each pin's own clock. Multicycles, as m2_sdram captures at CL 2:
#   - read: the READ leaves on a clk_sys edge (0), the device takes it at the pin's next rising edge (T / 2 less
#     the lead), drives the word tAC after the edge after that (1.5 T less the lead: the launch edge here), and dq_r
#     (the DQ pin's input register) takes it at clk_sys edge CL + rd_lat_sel - 1: 3 T for CL+2 (the OSD's default),
#     2 T for CL+1, 4 T for CL+3. From the launch that is setup multicycle 2, 1 or 3 (-end, on clk_sys). No hold multicycle: a burst
#     changes the word every clock, so the hold check stays one clock before the capture edge (the next word must
#     not reach dq_r before it), and setup + hold slack is the eye's margin.
#   - write and command: launched on clk_sys edge 0 and taken at the pin's next rising edge (T / 2 less the lead):
#     the default relation, no multicycle.
#
# Only for TimeQuest, and only when asked: `set m3_sdram_io 1` before read_sdc (fit.sh's sdram_io.tcl does this
# after every fit, at each read capture edge; `m3_sdram_rd_edge` picks the read multicycle, default 2 = CL+2).
# Not for the fitter: every SDRAM register sits in its I/O cell, so placement cannot move these paths, but the
# fitter's automatic delay chains would add input or output delay to meet Model 2's figures (its read eye is shut
# at 120-130 MHz on every build, the board-proven ones included: R184), which moves the board's capture point.
# Not in the flow's own timing report either: those figures' read hold would stand among clk_sys's paths and hide
# the core's own setup and hold.
set m3_exe ""
catch { set m3_exe $::quartus(nameofexecutable) }
if {![string equal $m3_exe "quartus_map"]} {
	# sys140: each SDRAM's DQ input registers are on a capture clock of their own (outclk_4 / outclk_5, m2_sdram
	# CAPCLK), so the read is timed from the pin clock to that clock; the capture edge is the same count of edges
	# after the launch as before (each capture clock's phase is under the pin clock's), so the multicycle stands.
	# Without a capture clock (no outclk_4 / outclk_5) the reads are timed to clk_sys as before.
	foreach {m3_n m3_pfx m3_ddio m3_k m3_kc} {sd1 SDRAM u_sdclk1 2 4 sd2 SDRAM2 u_sdclk2 3 5} {
		set m3_sdr_src [get_pins -nowarn -compatibility_mode "*|pll|pll_inst|altera_pll_i|*\[${m3_k}\].*|divclk"]
		set m3_prt [get_ports -nowarn ${m3_pfx}_CLK]
		# no DDIO (the CPU test build, or a single build's SDRAM2): no clock on that pin
		if {[get_collection_size $m3_prt] == 0 || [get_collection_size [get_cells -nowarn -compatibility_mode "*|${m3_ddio}|*"]] == 0} continue
		create_generated_clock -name ${m3_n}_pin -source $m3_sdr_src $m3_prt
		if {[info exists ::m3_sdram_io] && $::m3_sdram_io} {
			set m3_sys [get_clocks -nowarn "*|pll|pll_inst|altera_pll_i|*\[${m3_kc}\].*|divclk"]
			if {[get_collection_size $m3_sys] == 0} { set m3_sys [get_clocks {*|pll|pll_inst|altera_pll_i|*[1].*|divclk}] }
			set m3_nr 2
			if {[info exists ::m3_sdram_rd_edge]} { set m3_nr $::m3_sdram_rd_edge }
			set_input_delay -clock ${m3_n}_pin -max 6.4 [get_ports ${m3_pfx}_DQ[*]]
			set_input_delay -clock ${m3_n}_pin -min 1.0 [get_ports ${m3_pfx}_DQ[*]]
			set m3_out [get_ports -nowarn [list ${m3_pfx}_A[*] ${m3_pfx}_BA[*] ${m3_pfx}_DQ[*] ${m3_pfx}_nCS ${m3_pfx}_nRAS \
				${m3_pfx}_nCAS ${m3_pfx}_nWE ${m3_pfx}_DQML ${m3_pfx}_DQMH ${m3_pfx}_CKE]]
			set_output_delay -clock ${m3_n}_pin -max 1.5 $m3_out
			set_output_delay -clock ${m3_n}_pin -min -0.8 $m3_out
			if {$m3_nr > 1} { set_multicycle_path -setup -end $m3_nr -from [get_clocks ${m3_n}_pin] -to $m3_sys }
		}
	}
}

# sys140: the read-capture calibration moves each capture clock (outclk_4 / outclk_5) to its window's centre, held
# within the band m3_sdr_cal BAND_LO / BAND_HI sets: 8 PLL steps (8 x 148.8 ps = 1.19 ns) below the generated phase to
# 14 above (2.083 ns). The capture registers' transfers into clk_sys are analysed for that whole band, as uncertainty
# on them: a later capture clock launches later (setup, 2.083 ns), an earlier one earlier (hold, 1.190 ns). (The
# pins-to-capture-register reads are what the calibration measures; sdram_io.tcl's figures are at the generated
# phase.)
foreach m3_kc {4 5} {
	set m3_cc [get_clocks -nowarn "*|pll|pll_inst|altera_pll_i|*\[${m3_kc}\].*|divclk"]
	set m3_cs [get_clocks -nowarn {*|pll|pll_inst|altera_pll_i|*[1].*|divclk}]
	if {[get_collection_size $m3_cc] > 0 && [get_collection_size $m3_cs] > 0} {
		set_clock_uncertainty -add -setup -from $m3_cc -to $m3_cs 2.083
		set_clock_uncertainty -add -hold  -from $m3_cc -to $m3_cs 1.190
	}
}
# The calibration counts 48 PLL steps to a period (148.8 ps each), which holds only with the PLL's VCO at 840 MHz:
# the IP tool's physical parameters keep it there (rtl/pll.v). A PLL regenerated without them may run its counters at
# 420 MHz (steps of 297.6 ps, 24 to a period: the board's 99 / 18 on t6_s1, R198); say so loudly.
set m3_vco [get_clocks -nowarn "*|pll|pll_inst|altera_pll_i|*|vcoph\[0\]"]
if {[get_collection_size $m3_vco] > 0} {
	set m3_vp [get_clock_info -period $m3_vco]
	if {abs($m3_vp - 1.190) > 0.01} {
		post_message -type critical_warning "Model3.sdc: the PLL's VCO period is $m3_vp ns, not 1.190 (840 MHz): m3_sdr_cal's NSTEP 48 and the stepper's modulo do not match the PLL"
	}
}

# sys-timing: while the fitter runs only, 0.1 ns more hold uncertainty on the system clock and between it and clk_hps
# (tighter for the fitter, not for the sign-off): the fitter left same-clock hold paths at -0.005 to -0.054 ns at one
# corner in two fits of three (the rasteriser's queues), having padded them to within a few picoseconds of zero.
if {[string equal $m3_exe "quartus_fit"]} {
	set m3_hs [get_clocks -nowarn {*|pll|pll_inst|altera_pll_i|*[1].*|divclk}]
	set m3_hh [get_clocks -nowarn {*|pll|pll_inst|altera_pll_i|*[6].*|divclk}]
	if {[get_collection_size $m3_hs] > 0} { set_clock_uncertainty -add -hold -from $m3_hs -to $m3_hs 0.100 }
	if {[get_collection_size $m3_hs] > 0 && [get_collection_size $m3_hh] > 0} {
		set_clock_uncertainty -add -hold -from $m3_hs -to $m3_hh 0.100
		set_clock_uncertainty -add -hold -from $m3_hh -to $m3_hs 0.100
	}
}
