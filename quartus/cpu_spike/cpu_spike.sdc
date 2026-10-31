# The integer core's standalone target is 120 MHz, not the 100 MHz it runs at
# (study D-3): the FPU, MMU and caches will cost timing, and the margin is kept
# for them.
create_clock -name clk -period 8.333 [get_ports clk]
derive_clock_uncertainty
