# the sound engine on the 100 MHz CPU / SDRAM clock (D-3), its rates as enables
create_clock -name clk -period 10.000 [get_ports clk]
derive_clock_uncertainty
