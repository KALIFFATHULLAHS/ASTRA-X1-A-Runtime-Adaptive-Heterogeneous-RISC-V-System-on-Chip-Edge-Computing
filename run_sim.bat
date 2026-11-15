@echo off
call D:\vivado\2025.1\Vivado\settings64.bat
call xvlog -prj sim_proj.prj
if errorlevel 1 exit /b 1
call xelab -debug typical -top riscv_core_tb -snapshot sim_snap
if errorlevel 1 exit /b 1
echo run -all > xsim.dir\sim_snap\xsim_script.tcl
echo exit >> xsim.dir\sim_snap\xsim_script.tcl
call xsim sim_snap -R
