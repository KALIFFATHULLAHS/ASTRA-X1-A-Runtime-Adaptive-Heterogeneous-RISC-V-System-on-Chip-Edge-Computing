#!/usr/bin/env python3
r"""
ASTRA-X1 Raw RV32IMFX Assembly & CSR Programming Sketch
=======================================================
Demonstrates how a low-level systems programmer can emit native 32-bit
RV32IMFX instructions (`addi`, `mul`, `div`, `csrrs`, `sw`, `lw`) directly
inside `AstraApp` and execute them on the EDGE Artix-7 100T FPGA board.

Run on Hardware:
  & C:/Users/skali/AppData/Local/Microsoft/WindowsApps/python3.13.exe applications\my_first_asm_app.py
"""

import os
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tools import AstraApp, addi, mul, div, csrrs, NOP

app = AstraApp("Native RV32IMFX Assembly & CSR Sketch")
app.seven_seg_hex(0xA532)
app.println("\r\n=== HELLO FROM ASTRA-X1 RV32IMFX ASSEMBLY SDK! ===")

# 1. Execute Native RV32M Hardware Multiply & Divide in Assembly:
#    x24 = 25, x25 = 16 -> x2 = (25 * 16) / 4 = 100
app.emit(addi(24, 0, 25))
app.emit(addi(25, 0, 16))
app.emit(NOP); app.emit(NOP)
app.emit(mul(24, 24, 25))       # x24 = 25 * 16 = 400 (Hardware Wallace/DSP Multiplier)
app.emit(addi(25, 0, 4))
app.emit(NOP); app.emit(NOP)
app.emit(div(2, 24, 25))        # x2  = 400 / 4 = 100 (Hardware Radix-2 Divider)
for _ in range(36):
    app.emit(NOP)

app.print("1. [RV32M ASM] (25 * 16) / 4 = ")
app.jump(3, "PRINT_CELL"); app.emit(NOP); app.emit(NOP)
app.println()

# 2. Read RISC-V Machine CSR `mhartid` (0xF14) and `fcsr` (0x003)
app.emit(csrrs(2, 0xF14, 0))
app.emit(NOP); app.emit(NOP)
app.print("2. [CSR ASM] mhartid = ")
app.jump(3, "PRINT_CELL"); app.emit(NOP); app.emit(NOP)
app.println()

# 3. Print live board switches, L1 D-Cache hits & DAAQ energy saved
app.print_board_telemetry()

if __name__ == "__main__":
    app.build_and_run()
