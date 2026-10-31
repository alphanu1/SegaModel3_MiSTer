project_open cpu_spike
create_timing_netlist -model slow
read_sdc
update_timing_netlist
report_timing -setup -npaths 1 -detail full_path -to [get_registers {*m.result[31]}] -file path1.txt
project_close
