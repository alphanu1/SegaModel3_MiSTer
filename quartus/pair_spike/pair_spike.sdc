# The CPU standalone target (study D-3)
create_clock -name clk -period 8.333 [get_ports clk]
derive_clock_uncertainty
