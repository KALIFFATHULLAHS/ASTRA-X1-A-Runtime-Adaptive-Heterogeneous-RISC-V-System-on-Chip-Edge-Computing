@echo off
call D:\vivado\2025.1\Vivado\settings64.bat
call xsim npu_snap -tclbatch npu_script.tcl
