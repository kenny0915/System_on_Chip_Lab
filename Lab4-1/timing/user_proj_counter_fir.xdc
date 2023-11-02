create_clock -period 10.000 -name wb_clk_i -waveform {0.000 5.000} -add
set _xlnx_shared_i0 [get_ports -regexp -filter { NAME =~  ".*" && DIRECTION == "IN" }]
set_input_delay -add_delay 5.000 $_xlnx_shared_i0
set _xlnx_shared_i1 [get_ports -regexp -filter { NAME =~  ".*" && DIRECTION == "OUT" }]
set_output_delay -add_delay 5.000 $_xlnx_shared_i1
