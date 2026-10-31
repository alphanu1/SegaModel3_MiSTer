project_open cpu_spike
create_timing_netlist -model slow
read_sdc
update_timing_netlist
report_timing -setup -npaths 40 -detail summary -panel_name worst -file worst_paths.txt
project_close
