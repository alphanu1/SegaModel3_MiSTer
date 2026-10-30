derive_pll_clocks
derive_clock_uncertainty

# core specific constraints

# The CPU's reset crosses from the template's 20 MHz domain through a 3-flop
# synchroniser (cpu_rst_sync); the first stage is not a timed path.
set_false_path -to [get_registers {*cpu_rst_sync[0]}]
