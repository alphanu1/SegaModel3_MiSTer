derive_pll_clocks
derive_clock_uncertainty

# The core PLL must match sys_top.sdc's clock groups (*|pll|pll_inst|altera_pll_i),
# or its clocks go untimed against the framework's.
set core_clks [get_clocks -nowarn {*|pll|pll_inst|altera_pll_i|*[*].*|divclk}]
if {[llength $core_clks] == 0} {
    post_message -type critical_warning "ddr3_probe.sdc: no core PLL clocks matched *|pll|pll_inst|altera_pll_i|*"
}

# The probe's reset synchroniser takes RESET from the framework; its first
# stage is not a timed path.
set_false_path -to [get_registers {*emu|rst_s[0]}]

# The DDR3 clock (pll_var, reconfigured at run time, timed at its compiled
# 125 MHz) is not in sys_top.sdc's groups: asynchronous to every other clock.
# Its crossings with the fixed clock are synchronisers and toggle handshakes
# (dp_core, dp_eng).
# (absent in a VARCLK = 0 build)
set var_clks [get_clocks -nowarn {*pll_var|pll_inst|altera_pll_i|*divclk}]
if {[get_collection_size $var_clks] > 0} {
    set_clock_groups -asynchronous -group $var_clks
}
# The reconfiguration's request crosses into CLK_50M by synchroniser
set_false_path -to [get_registers {*emu|cfg_block.tg[0]}]
set_false_path -to [get_registers {*emu|cfg_block.sel1[*]}]
set_false_path -to [get_registers {*emu|por_s[0]}]
