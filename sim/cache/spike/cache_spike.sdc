# The CPU's standalone target, 120 MHz (study D-3)
create_clock -name clk -period 8.333 [get_ports clk]
derive_clock_uncertainty
