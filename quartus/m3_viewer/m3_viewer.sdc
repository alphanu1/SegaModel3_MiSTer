derive_pll_clocks
derive_clock_uncertainty

# The core PLL must match sys_top.sdc's clock groups (*|pll|pll_inst|altera_pll_i),
# or its clocks go untimed against the framework's.
set core_clks [get_clocks -nowarn {*|pll|pll_inst|altera_pll_i|*[*].*|divclk}]
if {[llength $core_clks] == 0} {
    post_message -type critical_warning "m3_viewer.sdc: no core PLL clocks matched *|pll|pll_inst|altera_pll_i|*"
}

# The viewer's reset synchroniser takes RESET from the framework; its first
# stage is not a timed path.
set_false_path -to [get_registers {*emu|rst_s[0]}]
set_false_path -to [get_registers {*emu|mem_rst_s[0]}]
