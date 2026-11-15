#!/usr/bin/env python3
"""
ASTRA-X1 Milestone 4: Bare-Metal GEMM Driver with Formatted Matrix C Printing
Streams the 4x4 matrix result computed by the NPU directly to UART.
"""

import sys
import os
import time

COM_PORT  = "COM3"
BAUD_RATE = 115200

def _le32(val):
    return [(val >> (8 * i)) & 0xFF for i in range(4)]

def lui(rd, imm_20bit):
    val = ((imm_20bit & 0xFFFFF) << 12) | ((rd & 0x1F) << 7) | 0x37
    return _le32(val)

def addi(rd, rs1, imm_12bit):
    imm = imm_12bit & 0xFFF
    val = (imm << 20) | ((rs1 & 0x1F) << 15) | (0 << 12) | ((rd & 0x1F) << 7) | 0x13
    return _le32(val)

def andi(rd, rs1, imm_12bit):
    imm = imm_12bit & 0xFFF
    val = (imm << 20) | ((rs1 & 0x1F) << 15) | (7 << 12) | ((rd & 0x1F) << 7) | 0x13
    return _le32(val)

def sub(rd, rs1, rs2):
    val = (0x20 << 25) | ((rs2 & 0x1F) << 20) | ((rs1 & 0x1F) << 15) | (0 << 12) | ((rd & 0x1F) << 7) | 0x33
    return _le32(val)

def add(rd, rs1, rs2):
    val = (0x00 << 25) | ((rs2 & 0x1F) << 20) | ((rs1 & 0x1F) << 15) | (0 << 12) | ((rd & 0x1F) << 7) | 0x33
    return _le32(val)

def mul(rd, rs1, rs2):
    val = (0x01 << 25) | ((rs2 & 0x1F) << 20) | ((rs1 & 0x1F) << 15) | (0 << 12) | ((rd & 0x1F) << 7) | 0x33
    return _le32(val)

def slli(rd, rs1, shamt):
    val = ((shamt & 0x1F) << 20) | ((rs1 & 0x1F) << 15) | (1 << 12) | ((rd & 0x1F) << 7) | 0x13
    return _le32(val)

def csrrs(rd, csr, rs1=0):
    val = ((csr & 0xFFF) << 20) | ((rs1 & 0x1F) << 15) | (2 << 12) | ((rd & 0x1F) << 7) | 0x73
    return _le32(val)

def lw(rd, offset_12bit, rs1):
    imm = offset_12bit & 0xFFF
    val = (imm << 20) | ((rs1 & 0x1F) << 15) | (2 << 12) | ((rd & 0x1F) << 7) | 0x03
    return _le32(val)

def lb(rd, offset_12bit, rs1):
    imm = offset_12bit & 0xFFF
    val = (imm << 20) | ((rs1 & 0x1F) << 15) | (0 << 12) | ((rd & 0x1F) << 7) | 0x03
    return _le32(val)

def sw(rs2, offset_12bit, rs1):
    off     = offset_12bit & 0xFFF
    imm_top = (off >> 5) & 0x7F
    imm_bot = off & 0x1F
    val = (imm_top << 25) | ((rs2 & 0x1F) << 20) | ((rs1 & 0x1F) << 15) | (2 << 12) | (imm_bot << 7) | 0x23
    return _le32(val)

def bne(rs1, rs2, offset_13bit):
    off     = offset_13bit & 0x1FFF
    imm_12  = (off >> 12) & 0x1
    imm_10_5= (off >> 5)  & 0x3F
    imm_4_1 = (off >> 1)  & 0xF
    imm_11  = (off >> 11) & 0x1
    val = (imm_12 << 31) | (imm_10_5 << 25) | ((rs2 & 0x1F) << 20) | \
          ((rs1 & 0x1F) << 15) | (1 << 12) | (imm_4_1 << 8) | (imm_11 << 7) | 0x63
    return _le32(val)

def beq(rs1, rs2, offset_13bit):
    off     = offset_13bit & 0x1FFF
    imm_12  = (off >> 12) & 0x1
    imm_10_5= (off >> 5)  & 0x3F
    imm_4_1 = (off >> 1)  & 0xF
    imm_11  = (off >> 11) & 0x1
    val = (imm_12 << 31) | (imm_10_5 << 25) | ((rs2 & 0x1F) << 20) | \
          ((rs1 & 0x1F) << 15) | (0 << 12) | (imm_4_1 << 8) | (imm_11 << 7) | 0x63
    return _le32(val)

def blt(rs1, rs2, offset_13bit):
    off     = offset_13bit & 0x1FFF
    imm_12  = (off >> 12) & 0x1
    imm_10_5= (off >> 5)  & 0x3F
    imm_4_1 = (off >> 1)  & 0xF
    imm_11  = (off >> 11) & 0x1
    val = (imm_12 << 31) | (imm_10_5 << 25) | ((rs2 & 0x1F) << 20) | \
          ((rs1 & 0x1F) << 15) | (4 << 12) | (imm_4_1 << 8) | (imm_11 << 7) | 0x63
    return _le32(val)

def bge(rs1, rs2, offset_13bit):
    off     = offset_13bit & 0x1FFF
    imm_12  = (off >> 12) & 0x1
    imm_10_5= (off >> 5)  & 0x3F
    imm_4_1 = (off >> 1)  & 0xF
    imm_11  = (off >> 11) & 0x1
    val = (imm_12 << 31) | (imm_10_5 << 25) | ((rs2 & 0x1F) << 20) | \
          ((rs1 & 0x1F) << 15) | (5 << 12) | (imm_4_1 << 8) | (imm_11 << 7) | 0x63
    return _le32(val)

def jal(rd, offset_21bit):
    off = offset_21bit & 0x1FFFFF
    imm_20    = (off >> 20) & 1
    imm_10_1  = (off >> 1)  & 0x3FF
    imm_11    = (off >> 11) & 1
    imm_19_12 = (off >> 12) & 0xFF
    val = (imm_20 << 31) | (imm_10_1 << 21) | (imm_11 << 20) | (imm_19_12 << 12) | ((rd & 0x1F) << 7) | 0x6F
    return _le32(val)

def jalr(rd, rs1, imm_12bit):
    val = ((imm_12bit & 0xFFF) << 20) | ((rs1 & 0x1F) << 15) | (0 << 12) | ((rd & 0x1F) << 7) | 0x67
    return _le32(val)

def jal_self():
    return _le32(0x0000006F)

WFI = _le32(0x10500073)
NOP = _le32(0x00000013)

def li32(builder, rd, val):
    val = val & 0xFFFFFFFF
    imm_hi = (val + 0x800) >> 12
    imm_lo = val - (imm_hi << 12)
    builder.emit(lui(rd, imm_hi & 0xFFFFF))
    builder.emit(addi(rd, rd, imm_lo & 0xFFF))

class ProgramBuilder:
    def __init__(self):
        self.instructions = []
        self.labels = {}
        self.fixups = []

    def label(self, name):
        self.labels[name] = len(self.instructions) * 4

    def emit(self, inst):
        self.instructions.append(inst)

    def branch(self, b_fn, rs1, rs2, target_label):
        idx = len(self.instructions)
        self.fixups.append(('B', idx, b_fn, rs1, rs2, target_label))
        self.instructions.append(None)

    def jump(self, rd, target_label):
        idx = len(self.instructions)
        self.fixups.append(('J', idx, jal, rd, target_label))
        self.instructions.append(None)

    def assemble(self):
        for item in self.fixups:
            if item[0] == 'B':
                _, idx, b_fn, rs1, rs2, target = item
                curr_addr = idx * 4
                target_addr = self.labels[target]
                offset = target_addr - curr_addr
                self.instructions[idx] = b_fn(rs1, rs2, offset)
            elif item[0] == 'J':
                _, idx, j_fn, rd, target = item
                curr_addr = idx * 4
                target_addr = self.labels[target]
                offset = target_addr - curr_addr
                self.instructions[idx] = j_fn(rd, offset)
        return self.instructions

def emit_sw_safe(p, rs2, offset, rs1):
    # 5 NOPs required because bram_imem rdata is un-stalled during 1-cycle Wishbone MEM stall
    p.emit(sw(rs2, offset, rs1))
    p.emit(NOP)
    p.emit(NOP)
    p.emit(NOP)
    p.emit(NOP)
    p.emit(NOP)

def emit_lw_safe(p, rd, offset, rs1):
    # 5 NOPs required because bram_imem rdata is un-stalled during 1-cycle Wishbone MEM stall
    p.emit(lw(rd, offset, rs1))
    p.emit(NOP)
    p.emit(NOP)
    p.emit(NOP)
    p.emit(NOP)
    p.emit(NOP)

def build_program(uart_delay=1500):
    p = ProgramBuilder()

    # Base addresses:
    # x11 = NPU Base: 0x4000_0000
    # x10 = Peripherals Base: 0xFFFF_0000
    # x16 = Data SRAM Base: 0x2000_0000
    p.emit(lui(11, 0x40000))
    p.emit(lui(10, 0xFFFF0))
    p.emit(lui(16, 0x20000))

    # Read ID Register (0x4000_0000)
    emit_lw_safe(p, 2, 0, 11)
    li32(p, 5, 0x41535452)
    p.emit(NOP); p.emit(NOP)
    p.branch(bne, 2, 5, "FAIL_PATH")

    # -----------------------------------------------------------------------
    # PART 1: PURE RV32IM CPU SOFTWARE 4x4 GEMM BENCHMARK (Data SRAM)
    # -----------------------------------------------------------------------
    mat_a = [
         1,  2,  3,  4,
         5,  6,  7,  8,
        -1, -2, -3, -4,
        10, 20, 30, 40
    ]
    mat_b = [
         2,  0,  1,  3,
         1,  2,  0, -1,
         0,  1,  2,  1,
         3,  0, -2,  1
    ]
    for i, val in enumerate(mat_a):
        p.emit(addi(3, 0, val & 0xFFF))
        emit_sw_safe(p, 3, 0x100 + i * 4, 16)
    for i, val in enumerate(mat_b):
        p.emit(addi(3, 0, val & 0xFFF))
        emit_sw_safe(p, 3, 0x200 + i * 4, 16)

    p.emit(addi(28, 0, 4)) # Loop bound = 4
    p.emit(NOP); p.emit(NOP)

    # Read mcycle before CPU software triple-nested GEMM loop
    p.emit(csrrs(24, 0xC00, 0))
    p.emit(NOP); p.emit(NOP)

    p.emit(addi(17, 0, 0)) # r = 0
    p.label("CPU_LOOP_R")
    p.emit(addi(18, 0, 0)) # c = 0
    p.label("CPU_LOOP_C")
    p.emit(addi(22, 0, 0)) # acc = 0
    p.emit(addi(19, 0, 0)) # k = 0
    p.label("CPU_LOOP_K")

    # Load A[r, k] from 0x2000_0100 + (r*4 + k)*4
    p.emit(slli(20, 17, 2))
    p.emit(NOP); p.emit(NOP)
    p.emit(add(20, 20, 19))
    p.emit(NOP); p.emit(NOP)
    p.emit(slli(20, 20, 2))
    p.emit(NOP); p.emit(NOP)
    p.emit(add(20, 20, 16))
    p.emit(NOP); p.emit(NOP)
    emit_lw_safe(p, 21, 0x100, 20)

    # Load B[k, c] from 0x2000_0200 + (k*4 + c)*4
    p.emit(slli(20, 19, 2))
    p.emit(NOP); p.emit(NOP)
    p.emit(add(20, 20, 18))
    p.emit(NOP); p.emit(NOP)
    p.emit(slli(20, 20, 2))
    p.emit(NOP); p.emit(NOP)
    p.emit(add(20, 20, 16))
    p.emit(NOP); p.emit(NOP)
    emit_lw_safe(p, 23, 0x200, 20)

    # Multiply-accumulate: acc += A[r, k] * B[k, c]
    p.emit(mul(21, 21, 23))
    p.emit(NOP); p.emit(NOP)
    p.emit(add(22, 22, 21))

    # k++
    p.emit(addi(19, 19, 1))
    p.emit(NOP); p.emit(NOP)
    p.branch(blt, 19, 28, "CPU_LOOP_K")
    p.emit(NOP); p.emit(NOP)

    # Store C_cpu[r, c] to 0x2000_0300 + (r*4 + c)*4
    p.emit(slli(20, 17, 2))
    p.emit(NOP); p.emit(NOP)
    p.emit(add(20, 20, 18))
    p.emit(NOP); p.emit(NOP)
    p.emit(slli(20, 20, 2))
    p.emit(NOP); p.emit(NOP)
    p.emit(add(20, 20, 16))
    p.emit(NOP); p.emit(NOP)
    emit_sw_safe(p, 22, 0x300, 20)

    # c++
    p.emit(addi(18, 18, 1))
    p.emit(NOP); p.emit(NOP)
    p.branch(blt, 18, 28, "CPU_LOOP_C")
    p.emit(NOP); p.emit(NOP)

    # r++
    p.emit(addi(17, 17, 1))
    p.emit(NOP); p.emit(NOP)
    p.branch(blt, 17, 28, "CPU_LOOP_R")
    p.emit(NOP); p.emit(NOP)

    # Read mcycle after CPU software GEMM -> x24 = cpu_cycles
    p.emit(csrrs(25, 0xC00, 0))
    p.emit(NOP); p.emit(NOP)
    p.emit(sub(24, 25, 24))
    p.emit(NOP); p.emit(NOP)

    # -----------------------------------------------------------------------
    # PART 2: ASTRA-X1 NPU SYSTOLIC ARRAY 4x4 GEMM BENCHMARK
    # -----------------------------------------------------------------------
    # Load Matrix A (4 words at 0x4000_0100 .. 0x4000_010C)
    li32(p, 3, 0x04030201)
    emit_sw_safe(p, 3, 0x100, 11)
    li32(p, 3, 0x08070605)
    emit_sw_safe(p, 3, 0x104, 11)
    li32(p, 3, 0xFCFDFEFF)
    emit_sw_safe(p, 3, 0x108, 11)
    li32(p, 3, 0x281E140A)
    emit_sw_safe(p, 3, 0x10C, 11)

    # Load Matrix B (4 words at 0x4000_0200 .. 0x4000_020C)
    li32(p, 3, 0x03010002)
    emit_sw_safe(p, 3, 0x200, 11)
    li32(p, 3, 0xFF000201)
    emit_sw_safe(p, 3, 0x204, 11)
    li32(p, 3, 0x01020100)
    emit_sw_safe(p, 3, 0x208, 11)
    li32(p, 3, 0x01FE0003)
    emit_sw_safe(p, 3, 0x20C, 11)

    # Set K Dimension = 4 (0x4000_0010)
    p.emit(addi(3, 0, 4))
    emit_sw_safe(p, 3, 0x010, 11)

    # Read mcycle right before NPU start
    p.emit(csrrs(25, 0xC00, 0))
    p.emit(NOP); p.emit(NOP)

    # Start Execution (0x4000_000C = 1)
    p.emit(addi(3, 0, 1))
    emit_sw_safe(p, 3, 0x00C, 11)

    # Poll STATUS (0x4000_0004) until Done (bit 1) == 1
    p.label("POLL_DONE")
    emit_lw_safe(p, 4, 0x004, 11)
    p.emit(andi(4, 4, 2))
    p.emit(NOP); p.emit(NOP); p.emit(NOP); p.emit(NOP)
    p.branch(beq, 4, 0, "POLL_DONE")

    # Read mcycle right after NPU Done -> x25 = npu_soc_cycles
    p.emit(csrrs(26, 0xC00, 0))
    p.emit(NOP); p.emit(NOP)
    p.emit(sub(25, 26, 25))
    p.emit(NOP); p.emit(NOP)

    # Read hardware NPU_REG_CYCLES (0x4000_0014) -> x26 = npu_core_cycles
    emit_lw_safe(p, 26, 0x014, 11)

    # -----------------------------------------------------------------------
    # PART 3: HARDWARE CROSS-VERIFICATION (C_CPU == C_NPU == GOLDEN)
    # -----------------------------------------------------------------------
    expected_c = [
        16,   7,  -1,   8,
        40,  19,   3,  24,
       -16,  -7,   1,  -8,
       160,  70, -10,  80
    ]
    for idx, exp_val in enumerate(expected_c):
        npu_off  = 0x40 + idx * 4
        sram_off = 0x300 + idx * 4
        emit_lw_safe(p, 2, npu_off, 11)
        emit_lw_safe(p, 4, sram_off, 16)
        li32(p, 5, exp_val)
        p.emit(NOP); p.emit(NOP)
        p.branch(bne, 2, 5, "FAIL_PATH")
        p.emit(NOP); p.emit(NOP)
        p.branch(bne, 4, 2, "FAIL_PATH")
        p.emit(NOP); p.emit(NOP)

    # Compute integer speedups on CPU:
    # x28 = cpu_cycles (x24) / npu_core_cycles (x26)
    p.emit(addi(28, 0, 0))
    p.emit(addi(20, 24, 0))
    p.emit(NOP); p.emit(NOP)
    p.label("DIV_CORE_LOOP")
    p.branch(blt, 20, 26, "DIV_CORE_DONE")
    p.emit(NOP); p.emit(NOP)
    p.emit(sub(20, 20, 26))
    p.emit(addi(28, 28, 1))
    p.branch(beq, 0, 0, "DIV_CORE_LOOP")
    p.emit(NOP); p.emit(NOP)
    p.label("DIV_CORE_DONE")

    # x29 = cpu_cycles (x24) / npu_soc_cycles (x25)
    p.emit(addi(29, 0, 0))
    p.emit(addi(20, 24, 0))
    p.emit(NOP); p.emit(NOP)
    p.label("DIV_SOC_LOOP")
    p.branch(blt, 20, 25, "DIV_SOC_DONE")
    p.emit(NOP); p.emit(NOP)
    p.emit(sub(20, 20, 25))
    p.emit(addi(29, 29, 1))
    p.branch(beq, 0, 0, "DIV_SOC_LOOP")
    p.emit(NOP); p.emit(NOP)
    p.label("DIV_SOC_DONE")

    # -----------------------------------------------------------------------
    # PASS PATH: Set LEDs = 0x5A, Print Matrix C + Hardware Cycle Benchmark
    # -----------------------------------------------------------------------
    p.label("PASS_PATH")
    p.emit(addi(6, 0, 0x5A))
    emit_sw_safe(p, 6, 4, 10) # dbg_led <= 0x5A

    # Subroutine call helper:
    # x6 contains char, jal x1, UART_SEND
    def call_send_char(ch):
        p.emit(addi(6, 0, ord(ch)))
        p.jump(1, "UART_SEND")
        p.emit(NOP); p.emit(NOP)

    def print_str(s):
        for ch in s:
            call_send_char(ch)

    print_str("\r\nC =\r\n")

    # Print 4 rows
    for r in range(4):
        print_str("[")
        for c in range(4):
            idx = r * 4 + c
            offset = 0x40 + idx * 4

            # Load actual computed cell from NPU memory into x2, then call PRINT_CELL
            emit_lw_safe(p, 2, offset, 11)
            p.jump(3, "PRINT_CELL")
            p.emit(NOP); p.emit(NOP)

            if c < 3:
                print_str(",")

        print_str(" ]\r\n")

    print_str("CPU SW Cycles : ")
    p.emit(addi(2, 24, 0))
    p.jump(3, "PRINT_CELL")
    p.emit(NOP); p.emit(NOP)

    print_str("\r\nNPU SoC Cycles: ")
    p.emit(addi(2, 25, 0))
    p.jump(3, "PRINT_CELL")
    p.emit(NOP); p.emit(NOP)

    print_str("\r\nNPU MAC Cycles: ")
    p.emit(addi(2, 26, 0))
    p.jump(3, "PRINT_CELL")
    p.emit(NOP); p.emit(NOP)

    print_str("\r\nMAC Speedup   : ")
    p.emit(addi(2, 28, 0))
    p.jump(3, "PRINT_CELL")
    p.emit(NOP); p.emit(NOP)
    print_str("x\r\nSoC Speedup   : ")
    p.emit(addi(2, 29, 0))
    p.jump(3, "PRINT_CELL")
    p.emit(NOP); p.emit(NOP)
    print_str("x\r\nPASS\r\n")

    p.emit(WFI)
    p.emit(jal_self())

    # FAIL PATH
    p.label("FAIL_PATH")
    p.emit(addi(6, 0, 0xEE & 0xFFF))
    emit_sw_safe(p, 6, 4, 10)
    print_str("FAIL\r\n")
    p.emit(WFI)
    p.emit(jal_self())

    # -----------------------------------------------------------------------
    # PRINT_CELL SUBROUTINE (Called via JAL x3, PRINT_CELL)
    # Formats signed 32-bit integer in x2 as 4-char right-aligned decimal
    # and transmits over UART using UART_SEND (which uses x1 as link reg).
    # -----------------------------------------------------------------------
    p.label("PRINT_CELL")
    lbl = "pcell"

    # Check sign
    p.emit(addi(12, 0, 0)) # is_neg = 0
    p.branch(bge, 2, 0, f"{lbl}_pos")
    p.emit(NOP); p.emit(NOP)
    p.emit(sub(2, 0, 2))   # x2 = -x2
    p.emit(addi(12, 0, 1)) # is_neg = 1
    p.label(f"{lbl}_pos")

    # Decompose into thousands (x27), hundreds (x13), tens (x14), units (x2)
    p.emit(addi(27, 0, 0)) # thousands = 0
    p.emit(addi(13, 0, 0)) # hundreds = 0
    p.emit(addi(14, 0, 0)) # tens = 0
    p.emit(addi(5, 0, 1000))
    p.emit(NOP); p.emit(NOP)

    p.label(f"{lbl}_k_loop")
    p.branch(blt, 2, 5, f"{lbl}_k_done")
    p.emit(NOP); p.emit(NOP)
    p.emit(sub(2, 2, 5))
    p.emit(addi(27, 27, 1))
    p.branch(beq, 0, 0, f"{lbl}_k_loop")
    p.emit(NOP); p.emit(NOP)
    p.label(f"{lbl}_k_done")

    p.emit(addi(5, 0, 100))
    p.emit(NOP); p.emit(NOP)

    p.label(f"{lbl}_h_loop")
    p.branch(blt, 2, 5, f"{lbl}_h_done")
    p.emit(NOP); p.emit(NOP)
    p.emit(sub(2, 2, 5))
    p.emit(addi(13, 13, 1))
    p.branch(beq, 0, 0, f"{lbl}_h_loop")
    p.emit(NOP); p.emit(NOP)
    p.label(f"{lbl}_h_done")

    p.emit(addi(5, 0, 10))
    p.emit(NOP); p.emit(NOP)
    p.label(f"{lbl}_t_loop")
    p.branch(blt, 2, 5, f"{lbl}_t_done")
    p.emit(NOP); p.emit(NOP)
    p.emit(sub(2, 2, 5))
    p.emit(addi(14, 14, 1))
    p.branch(beq, 0, 0, f"{lbl}_t_loop")
    p.emit(NOP); p.emit(NOP)
    p.label(f"{lbl}_t_done")

    # Digits to ASCII:
    p.emit(addi(27, 27, ord('0')))
    p.emit(addi(13, 13, ord('0')))
    p.emit(addi(14, 14, ord('0')))
    p.emit(addi(2, 2, ord('0')))
    p.emit(addi(5, 0, ord('0')))   # x5 = ASCII '0' for comparison

    # Setup 4 characters in registers x15 (c0), x8 (c1), x9 (c2), x2 (c3):
    p.emit(addi(15, 0, ord(' '))) # default c0 = ' '
    p.emit(addi(8, 0, ord(' ')))  # default c1 = ' '
    p.emit(addi(9, 0, ord(' ')))  # default c2 = ' '

    p.branch(beq, 12, 0, f"{lbl}_chk_pos")
    p.emit(NOP); p.emit(NOP)

    # --- NEGATIVE:
    p.branch(bne, 13, 5, f"{lbl}_n_h")
    p.emit(NOP); p.emit(NOP)
    p.branch(bne, 14, 5, f"{lbl}_n_t")
    p.emit(NOP); p.emit(NOP)
    # Neg units: "  -U"
    p.emit(addi(9, 0, ord('-')))
    p.branch(beq, 0, 0, f"{lbl}_emit4")
    p.emit(NOP); p.emit(NOP)
    # Neg tens: " -TU"
    p.label(f"{lbl}_n_t")
    p.emit(addi(8, 0, ord('-')))
    p.emit(addi(9, 14, 0)) # c2 = tens
    p.branch(beq, 0, 0, f"{lbl}_emit4")
    p.emit(NOP); p.emit(NOP)
    # Neg hundreds: "-HTU"
    p.label(f"{lbl}_n_h")
    p.emit(addi(15, 0, ord('-')))
    p.emit(addi(8, 13, 0)) # c1 = hundreds
    p.emit(addi(9, 14, 0)) # c2 = tens
    p.branch(beq, 0, 0, f"{lbl}_emit4")
    p.emit(NOP); p.emit(NOP)

    # --- POSITIVE:
    p.label(f"{lbl}_chk_pos")
    p.branch(bne, 27, 5, f"{lbl}_p_k")
    p.emit(NOP); p.emit(NOP)
    p.branch(bne, 13, 5, f"{lbl}_p_h")
    p.emit(NOP); p.emit(NOP)
    p.branch(bne, 14, 5, f"{lbl}_p_t")
    p.emit(NOP); p.emit(NOP)
    # Pos units: "   U" -> c0=' ', c1=' ', c2=' ' (already set)
    p.branch(beq, 0, 0, f"{lbl}_emit4")
    p.emit(NOP); p.emit(NOP)
    # Pos tens: "  TU"
    p.label(f"{lbl}_p_t")
    p.emit(addi(9, 14, 0)) # c2 = tens
    p.branch(beq, 0, 0, f"{lbl}_emit4")
    p.emit(NOP); p.emit(NOP)
    # Pos hundreds: " HTU"
    p.label(f"{lbl}_p_h")
    p.emit(addi(8, 13, 0)) # c1 = hundreds
    p.emit(addi(9, 14, 0)) # c2 = tens
    p.branch(beq, 0, 0, f"{lbl}_emit4")
    p.emit(NOP); p.emit(NOP)
    # Pos thousands: "KHTU"
    p.label(f"{lbl}_p_k")
    p.emit(addi(15, 27, 0)) # c0 = thousands
    p.emit(addi(8, 13, 0))  # c1 = hundreds
    p.emit(addi(9, 14, 0))  # c2 = tens

    # Emit the 4 characters: c0, c1, c2, c3
    p.label(f"{lbl}_emit4")
    p.emit(addi(6, 15, 0))
    p.jump(1, "UART_SEND")
    p.emit(NOP); p.emit(NOP)
    p.emit(addi(6, 8, 0))
    p.jump(1, "UART_SEND")
    p.emit(NOP); p.emit(NOP)
    p.emit(addi(6, 9, 0))
    p.jump(1, "UART_SEND")
    p.emit(NOP); p.emit(NOP)
    p.emit(addi(6, 2, 0))
    p.jump(1, "UART_SEND")
    p.emit(NOP); p.emit(NOP)

    p.emit(jalr(0, 3, 0)) # Return to caller: JALR x0, 0(x3)
    p.emit(NOP); p.emit(NOP); p.emit(NOP); p.emit(NOP)

    # -----------------------------------------------------------------------
    # UART SEND SUBROUTINE (Called via JAL x1, UART_SEND)
    # Transmits x6 over UART with 4 safety NOPs and 2500-cycle baud delay
    # -----------------------------------------------------------------------
    p.label("UART_SEND")
    emit_sw_safe(p, 6, 0, 10)     # SW x6 to UART TX (0xFFFF_0000)
    p.emit(addi(7, 0, uart_delay))# Delay ~4500 cycles (1500 = 0x5DC < 2048 so positive 12-bit imm!)
    p.label("UART_DLY")
    p.emit(addi(7, 7, -1 & 0xFFF))
    p.emit(NOP)
    p.branch(bne, 7, 0, "UART_DLY")
    p.emit(NOP); p.emit(NOP)
    p.emit(jalr(0, 1, 0))         # Return to caller: JALR x0, 0(x1)
    p.emit(NOP); p.emit(NOP); p.emit(NOP); p.emit(NOP)

    return p.assemble()

def main():
    target_port = sys.argv[1] if len(sys.argv) > 1 else COM_PORT
    uart_delay = 1 if target_port == "--fast-sim" else 1500
    instructions = build_program(uart_delay=uart_delay)
    print("=" * 65)
    print("  ASTRA-X1 MILESTONE 4: FULL SoC NPU GEMM TEST")
    print("=" * 65)
    print(f"Total instructions: {len(instructions)} words ({len(instructions)*4} bytes)")
    print("=" * 65)

    all_bytes = []
    for inst in instructions:
        all_bytes.extend(inst)

    base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    mem_path = os.path.join(base_dir, "program.mem")
    words = []
    for i in range(0, len(all_bytes), 4):
        b = all_bytes[i:i+4]
        words.append(f"{b[3]:02x}{b[2]:02x}{b[1]:02x}{b[0]:02x}")
    with open(mem_path, "w") as f:
        f.write("\n".join(words) + "\n")
    print(f"Written {len(words)} words to: {mem_path}")

    if target_port in ("--sim-only", "--fast-sim"):
        print("Simulation mode: program.mem generated.")
        return

    import serial
    print(f"\n=================================================================")
    print(f"  ASTRA-X1 MILESTONE 4 - HARDWARE UPLOAD (EDGE ARTIX-7 100T)")
    print(f"=================================================================")
    print(f"1. Please HOLD DOWN the RESET button (PIN M14 - Top Button) on your board.")
    input("   Press Enter in this terminal when holding reset...")

    print(f"Opening {target_port} at {BAUD_RATE} baud...")
    try:
        ser = serial.Serial(target_port, BAUD_RATE, timeout=1.0)
    except Exception as e:
        print(f"ERROR: Could not open {target_port}: {e}")
        return

    try:
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        print(f"Uploading {len(all_bytes)} bytes over UART to BRAM Instruction Memory...")
        ser.write(bytes(all_bytes))
        ser.flush()
        # Wait for physical UART wire transmission to finish before releasing reset
        time.sleep(len(all_bytes) * 10.0 / BAUD_RATE + 0.3)
        print("Upload complete!")
        print("--------------------------------------------------")
        print("2. Please RELEASE the RESET button now.")
        print("   Listening for execution response on UART...")
        print("--------------------------------------------------\n")

        output_buffer = []
        deadline = time.time() + 8.0
        while time.time() < deadline:
            if ser.in_waiting:
                chunk = ser.read(ser.in_waiting)
                text = chunk.decode('ascii', errors='replace')
                sys.stdout.write(text)
                sys.stdout.flush()
                output_buffer.append(text)
                if "PASS" in "".join(output_buffer) or "FAIL" in "".join(output_buffer):
                    deadline = time.time() + 0.3
                else:
                    deadline = time.time() + 1.5
            time.sleep(0.01)

        output_str = "".join(output_buffer)
        print("\n--------------------------------------------------")
        if "PASS" in output_str:
            print("\n>>> [SUCCESS] MILESTONE 4 HARDWARE VERIFICATION PASSED! <<<")
            print(">>> 1. NPU Slave 1 MMIO Address (0x4000_0000) verified.")
            print(">>> 2. 4x4 Signed Matrix A & B Scratchpad Buffers written & fed.")
            print(">>> 3. Autonomous Hardware Feeder FSM completed GEMM computation.")
            print(">>> 4. All 16/16 systolic output cells printed and verified bit-exactly!")
            print(">>> Check your board: LEDs MUST BE 0x5A (Binary 0b01011010)!")
        elif "FAIL" in output_str:
            print("\n>>> [FAILURE] Processor reported FAIL. At least one matrix cell mismatched.")
            print(">>> LEDs are displaying 0xEE (11101110).")
        elif not output_str:
            print(">>> [WARNING] No UART response received. Verify COM port and board power.")
        print("=" * 65)
    finally:
        ser.close()

if __name__ == "__main__":
    main()

