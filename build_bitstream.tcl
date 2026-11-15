# Open the Vivado project
open_project ./ricscv.xpr

# Reset runs for a fresh build
reset_run synth_1
reset_run impl_1

# Launch Synthesis, Implementation, and Bitstream Generation
launch_runs impl_1 -to_step write_bitstream -jobs 8
wait_on_run impl_1

# Check status
set impl_status [get_property STATUS [get_runs impl_1]]
set impl_progress [get_property PROGRESS [get_runs impl_1]]
puts "=========================================================="
puts "Implementation Status: $impl_status, Progress: $impl_progress"
puts "=========================================================="

if {$impl_progress != "100%"} {
    puts "ERROR: Bitstream generation failed!"
    exit 1
}

puts "SUCCESS: Bitstream generated at [file normalize ./ricscv.runs/impl_1/fpga_top.bit]"
exit 0
