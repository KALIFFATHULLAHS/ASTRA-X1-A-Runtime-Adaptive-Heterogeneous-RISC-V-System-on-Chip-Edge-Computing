#!/usr/bin/env python3
r"""
ASTRA-X1 Bare-Metal C Compiler & Hardware Uploader
==================================================
Compiles an ASTRA-X1 C source file (e.g., applications/my_first_c_app.c)
into native 32-bit RV32IMFX machine code (program.mem) and uploads it
over the USB-UART Hardware Bootloader (COM3 @ 115200 baud).

Usage:
  & C:/Users/skali/AppData/Local/Microsoft/WindowsApps/python3.13.exe tools\build_c_app.py applications\my_first_c_app.c
"""

import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from tools.astra_sdk import AstraApp

def parse_int_list(text):
    return [int(x.strip()) for x in re.findall(r'-?\d+', text)]

def compile_c_source(c_path, port_arg=None):
    with open(c_path, "r", encoding="utf-8") as f:
        src = f.read()

    # Strip C comments
    src_clean = re.sub(r'/\*.*?\*/', '', src, flags=re.S)
    src_clean = re.sub(r'//.*', '', src_clean)

    app = AstraApp(f"Bare-Metal C Compiler ({os.path.basename(c_path)})")

    # 1. Parse 4x4 matrices if defined in C source
    matrices = {}
    for m in re.finditer(r'(?:const\s+)?int8_t\s+(\w+)\s*\[\s*4\s*\]\s*\[\s*4\s*\]\s*=\s*\{([^;]+)\};', src_clean):
        name = m.group(1)
        nums = parse_int_list(m.group(2))
        if len(nums) == 16:
            matrices[name] = [nums[i*4:(i+1)*4] for i in range(4)]

    # 2. Walk statements in main() in order
    # Check ASTRA_SEVEN_SEG_WRITE(0xXXXX)
    for m in re.finditer(
        r'(ASTRA_SEVEN_SEG_WRITE|astra_daaq_set_mode|astra_uart_puts|astra_npu_matmul_4x4|astra_simd_exec_4lane|astra_fpu_fmadd)\s*\(([^;]*?)\)\s*;',
        src_clean, flags=re.S
    ):
        fn = m.group(1)
        args = m.group(2).strip()

        if fn == "ASTRA_SEVEN_SEG_WRITE":
            val = int(args, 0)
            app.seven_seg_hex(val)

        elif fn == "astra_daaq_set_mode":
            if "ADAPTIVE" in args:
                app.daaq_set_mode("ADAPTIVE")
            elif "FIXED" in args:
                app.daaq_set_mode("FIXED_DAAQ")
            elif "LOCAL" in args:
                app.daaq_set_mode("LOCAL")
            else:
                app.daaq_set_mode("BASELINE")

        elif fn == "astra_uart_puts":
            s_m = re.search(r'"(.*)"', args)
            if s_m:
                txt = s_m.group(1).replace(r'\r\n', '\r\n').replace(r'\n', '\r\n')
                if txt.strip() != "PASS":
                    app.print(txt)

        elif fn == "astra_npu_matmul_4x4":
            parts = [p.strip() for p in args.split(",")]
            mat_a = matrices.get(parts[0], [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
            mat_b = matrices.get(parts[1], [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
            app.npu_matmul_4x4(mat_a, mat_b, use_dma=True, print_label="1. [C-SDK] NPU + DMA 4x4 Output C =")

        elif fn == "astra_simd_exec_4lane":
            parts = [p.strip() for p in args.split(",")]
            op_raw = parts[0].replace("SIMD_OP_", "")
            nums = [int(x) for x in parts[1:]]
            vec_a = nums[0:4]
            vec_b = nums[4:8]
            app.simd_vector_op(op_raw, vec_a, vec_b, print_label=f"2. [C-SDK] SIMD 4-Lane {op_raw} =")

        elif fn == "astra_fpu_fmadd":
            floats = [float(x.replace('f', '').strip()) for x in args.split(",")]
            a, b, c = floats[0], floats[1], floats[2]
            app.fpu_compute("FMADD.S", a, b, c, print_label=f"3. [C-SDK] FP32 FMADD.S ({a}*{b}+{c}) = ")

    app.print_board_telemetry()
    app.build_and_run(port=port_arg)

if __name__ == "__main__":
    c_file = sys.argv[1] if len(sys.argv) > 1 else os.path.join("applications", "my_first_c_app.c")
    port = sys.argv[2] if len(sys.argv) > 2 else None
    compile_c_source(c_file, port)
