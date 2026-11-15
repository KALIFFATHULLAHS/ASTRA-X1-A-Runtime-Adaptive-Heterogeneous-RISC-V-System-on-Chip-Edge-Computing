@echo off
call D:\vivado\2025.1\Vivado\settings64.bat
call xvlog -i ricscv.srcs\sources_1\new ricscv.srcs\sources_1\new\riscv_core_tb.v
call xelab -debug typical -top riscv_core_tb -snapshot sim_snap
powershell -Command "(Get-Content xsim.dir/sim_snap/xsim_script.tcl) -replace '-autoloadwcfg ', '' | Set-Content xsim.dir/sim_snap/xsim_script.tcl"
call xsim sim_snap -R
