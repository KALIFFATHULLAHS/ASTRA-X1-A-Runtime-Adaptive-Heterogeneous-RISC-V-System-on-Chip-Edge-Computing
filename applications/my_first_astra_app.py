#!/usr/bin/env python3
r"""
Example User Application for the ASTRA-X1 Heterogeneous RISC-V SoC
==================================================================
Shows how a developer writes a high-level program (just like an Arduino sketch
or ESP32 app) using `AstraApp` and runs it on the EDGE Artix-7 FPGA board!

Run on Hardware:
  & C:\Users\skali\AppData\Local\Microsoft\WindowsApps\python3.13.exe "d:\FINAL YEAR PROJECT\ricscv\applications\my_first_astra_app.py"
"""

import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "tools"))
from astra_sdk import AstraApp

# 1. Create a new ASTRA-X1 Application
app = AstraApp("Developer Custom Edge-AI & DSP Sketch")

# 2. Configure Runtime Power Management & Display "2026" on the 7-Segment Display
app.daaq_set_mode("ADAPTIVE")
app.seven_seg_hex(0x2026)
app.println("\r\n=== HELLO FROM ASTRA-X1 DEVELOPER SDK! ===")

# 3. Offload a 4x4 Matrix Multiplication to the Systolic NPU via Autonomous DMA
A = [
    [ 2,  1,  0, -1],
    [ 4,  3,  2,  1],
    [-2, -1,  0,  1],
    [10,  5,  2,  1]
]
B = [
    [ 3,  0,  1,  2],
    [ 1,  4,  0, -1],
    [ 0,  2,  5,  1],
    [-1,  1,  2,  3]
]
app.npu_matmul_4x4(A, B, use_dma=True, print_label="1. NPU + DMA 4x4 Matrix Output C =")

# 4. Run a 4-Lane 16-bit SIMD Vector Operation
app.simd_vector_op("ADD16", vec_a=(100, -50, 250, -120), vec_b=(25, 50, -50, 20),
                   print_label="2. SIMD 4-Lane ADD16 =")

# 5. Run an IEEE-754 Single-Precision FP32 Fused Multiply-Add (15.5 * 2.0 + 9.0 = 40.0)
app.fpu_compute("FMADD.S", a=15.5, b=2.0, c=9.0,
                print_label="3. FP32 FMADD.S (15.5 * 2.0 + 9.0) = ")

# 6. Read Live Physical Slide Switches, L1 D-Cache Hits & DAAQ Energy Saved
app.print_board_telemetry()

# 7. Compile to RV32IMFX Machine Code & Upload over USB-UART (COM3)
if __name__ == "__main__":
    app.build_and_run()
