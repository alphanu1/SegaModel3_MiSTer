# The CPU's standalone target (study D-3): 8.333 ns; the 10 ns runs edit this.
create_clock -name clk -period 8.333 [get_ports clk]
derive_clock_uncertainty
