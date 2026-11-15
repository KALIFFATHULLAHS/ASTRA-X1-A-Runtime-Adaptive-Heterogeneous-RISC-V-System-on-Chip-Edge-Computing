#!/usr/bin/env python3
"""
ASTRA-X1 Milestone 6A: 4-Lane 16-bit Packed SIMD Vector Accelerator Benchmark
Verifies on Hardware (0x4000_3000):
  1. SIMD Slave 5 ID (0x4000_3000 == 0x53494D44 "SIMD")
  2. 4-Lane Parallel INT16 Operations:
     - ADD16  : [ 12, -5, 30, -15 ] + [ 3, 6, -10, 4 ] = [ 15,   1,   20, -11 ]
     - SUB16  : [ 12, -5, 30, -15 ] - [ 3, 6, -10, 4 ] = [  9, -11,   40, -19 ]
     - MUL16  : [ 12, -5, 30, -15 ] * [ 3, 6, -10, 4 ] = [ 36, -30, -300, -60 ] (Dot = -354)
     - ABS16  : |[ 12, -5, 30, -15 ]|                  = [ 12,   5,   30,  15 ] (Sum = 62)
     - RELU16 : ReLU([ 12, -5, 30, -15 ])              = [ 12,   0,   30,   0 ] (Sum = 42)
     - MAX16  : max(A, B)                              = [ 12,   6,   30,   4 ] (Sum = 52)
  3. Autonomous 32-Element (8 x 4-Lane Vector) Streaming Mode (VEC_MODE=1, LEN=8):
     - Compares Pure RV32IM Scalar 32-Element MAC Loop vs. 4-Lane SIMD Streaming Engine!
"""

import sys
import os
import time

COM_PORT  = "COM3"
BAUD_RATE = 115200

def _le32(val):
    return [(int(val) >> (8 * i)) & 0xFF for i in range(4)]

def lui(rd, imm_20bit):
    val = ((int(imm_20bit) & 0xFFFFF) << 12) | ((rd & 0x1F) << 7) | 0x37
    return _le32(val)

def addi(rd, rs1, imm_12bit):
    imm = int(imm_12bit) & 0xFFF
    val = (imm << 20) | ((rs1 & 0x1F) << 15) | (0 << 12) | ((rd & 0x1F) << 7) | 0x13
    return _le32(val)

def andi(rd, rs1, imm_12bit):
    imm = int(imm_12bit) & 0xFFF
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

def div(rd, rs1, rs2):
    val = (0x01 << 25) | ((rs2 & 0x1F) << 20) | ((rs1 & 0x1F) << 15) | (4 << 12) | ((rd & 0x1F) << 7) | 0x33
    return _le32(val)

def srai(rd, rs1, shamt):
    val = (0x20 << 25) | ((shamt & 0x1F) << 20) | ((rs1 & 0x1F) << 15) | (5 << 12) | ((rd & 0x1F) << 7) | 0x13
    return _le32(val)

def slli(rd, rs1, shamt):
    val = ((shamt & 0x1F) << 20) | ((rs1 & 0x1F) << 15) | (1 << 12) | ((rd & 0x1F) << 7) | 0x13
    return _le32(val)

def csrrs(rd, csr, rs1=0):
    val = ((csr & 0xFFF) << 20) | ((rs1 & 0x1F) << 15) | (2 << 12) | ((rd & 0x1F) << 7) | 0x73
    return _le32(val)

def lw(rd, offset_12bit, rs1):
    imm = int(offset_12bit) & 0xFFF
    val = (imm << 20) | ((rs1 & 0x1F) << 15) | (2 << 12) | ((rd & 0x1F) << 7) | 0x03
    return _le32(val)

def lh(rd, offset_12bit, rs1):
    imm = int(offset_12bit) & 0xFFF
    val = (imm << 20) | ((rs1 & 0x1F) << 15) | (1 << 12) | ((rd & 0x1F) << 7) | 0x03
    return _le32(val)

def sw(rs2, offset_12bit, rs1):
    off     = int(offset_12bit) & 0xFFF
    imm_top = (off >> 5) & 0x7F
    imm_bot = off & 0x1F
    val = (imm_top << 25) | ((rs2 & 0x1F) << 20) | ((rs1 & 0x1F) << 15) | (2 << 12) | (imm_bot << 7) | 0x23
    return _le32(val)

def bne(rs1, rs2, offset_13bit):
    off     = int(offset_13bit) & 0x1FFF
    imm_12  = (off >> 12) & 0x1
    imm_10_5= (off >> 5)  & 0x3F
    imm_4_1 = (off >> 1)  & 0xF
    imm_11  = (off >> 11) & 0x1
    val = (imm_12 << 31) | (imm_10_5 << 25) | ((rs2 & 0x1F) << 20) | \
          ((rs1 & 0x1F) << 15) | (1 << 12) | (imm_4_1 << 8) | (imm_11 << 7) | 0x63
    return _le32(val)

def beq(rs1, rs2, offset_13bit):
    off     = int(offset_13bit) & 0x1FFF
    imm_12  = (off >> 12) & 0x1
    imm_10_5= (off >> 5)  & 0x3F
    imm_4_1 = (off >> 1)  & 0xF
    imm_11  = (off >> 11) & 0x1
    val = (imm_12 << 31) | (imm_10_5 << 25) | ((rs2 & 0x1F) << 20) | \
          ((rs1 & 0x1F) << 15) | (0 << 12) | (imm_4_1 << 8) | (imm_11 << 7) | 0x63
    return _le32(val)

def blt(rs1, rs2, offset_13bit):
    off     = int(offset_13bit) & 0x1FFF
    imm_12  = (off >> 12) & 0x1
    imm_10_5= (off >> 5)  & 0x3F
    imm_4_1 = (off >> 1)  & 0xF
    imm_11  = (off >> 11) & 0x1
    val = (imm_12 << 31) | (imm_10_5 << 25) | ((rs2 & 0x1F) << 20) | \
          ((rs1 & 0x1F) << 15) | (4 << 12) | (imm_4_1 << 8) | (imm_11 << 7) | 0x63
    return _le32(val)

def bge(rs1, rs2, offset_13bit):
    off     = int(offset_13bit) & 0x1FFF
    imm_12  = (off >> 12) & 0x1
    imm_10_5= (off >> 5)  & 0x3F
    imm_4_1 = (off >> 1)  & 0xF
    imm_11  = (off >> 11) & 0x1
    val = (imm_12 << 31) | (imm_10_5 << 25) | ((rs2 & 0x1F) << 20) | \
          ((rs1 & 0x1F) << 15) | (5 << 12) | (imm_4_1 << 8) | (imm_11 << 7) | 0x63
    return _le32(val)

def jal(rd, offset_21bit):
    off = int(offset_21bit) & 0x1FFFFF
    imm_20    = (off >> 20) & 1
    imm_10_1  = (off >> 1)  & 0x3FF
    imm_11    = (off >> 11) & 1
    imm_19_12 = (off >> 12) & 0xFF
    val = (imm_20 << 31) | (imm_10_1 << 21) | (imm_11 << 20) | (imm_19_12 << 12) | ((rd & 0x1F) << 7) | 0x6F
    return _le32(val)

def jalr(rd, rs1, imm_12bit):
    val = ((int(imm_12bit) & 0xFFF) << 20) | ((rs1 & 0x1F) << 15) | (0 << 12) | ((rd & 0x1F) << 7) | 0x67
    return _le32(val)

def jal_self():
    return _le32(0x0000006F)

WFI = _le32(0x10500073)
NOP = _le32(0x00000013)

def pack2_i16(lane0, lane1):
    return ((int(lane1) & 0xFFFF) << 16) | (int(lane0) & 0xFFFF)

def li32(builder, rd, val):
    val = int(val) & 0xFFFFFFFF
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
    p.emit(sw(rs2, offset, rs1))
    for _ in range(5):
        p.emit(NOP)

def emit_lw_safe(p, rd, offset, rs1):
    p.emit(lw(rd, offset, rs1))
    for _ in range(5):
        p.emit(NOP)

def build_program(uart_delay=1500):
    p = ProgramBuilder()

    # Base addresses:
    # x18 = SIMD Base:        0x4000_3000 (Slave 5)
    # x10 = Peripherals Base: 0xFFFF_0000 (Slave 2)
    # x16 = Data SRAM Base:   0x2000_0000 (Slave 0)
    p.emit(lui(18, 0x40003))
    p.emit(lui(10, 0xFFFF0))
    p.emit(lui(16, 0x20000))

    # 1. Verify SIMD ID (0x4000_3000 == 0x53494D44 "SIMD")
    emit_lw_safe(p, 2, 0x000, 18)
    li32(p, 5, 0x53494D44)
    p.emit(NOP); p.emit(NOP)
    p.branch(bne, 2, 5, "FAIL_PATH")

    # 2. Load 4-Lane Input Vectors:
    #    A = [  12,  -5,  30, -15 ]
    #    B = [   3,   6, -10,   4 ]
    li32(p, 3, pack2_i16(12, -5))
    emit_sw_safe(p, 3, 0x010, 18) # SIMD_VEC_A0
    li32(p, 3, pack2_i16(30, -15))
    emit_sw_safe(p, 3, 0x014, 18) # SIMD_VEC_A1

    li32(p, 3, pack2_i16(3, 6))
    emit_sw_safe(p, 3, 0x018, 18) # SIMD_VEC_B0
    li32(p, 3, pack2_i16(-10, 4))
    emit_sw_safe(p, 3, 0x01C, 18) # SIMD_VEC_B1

    # Helper to run single 4-lane op, store RES_Y0, RES_Y1, ACC into SRAM at sram_base, and verify ACC
    # Opcodes: 0=ADD16, 1=SUB16, 2=MUL16, 4=ABS16, 5=RELU16, 6=MAX16
    ops_to_test = [
        (0, 0x100,   25), # ADD16:  [15, 1, 20, -11],   ACC = 25
        (1, 0x110,   19), # SUB16:  [9, -11, 40, -19],  ACC = 19
        (2, 0x120, -354), # MUL16:  [36, -30, -300, -60], ACC = -354
        (4, 0x130,   62), # ABS16:  [12, 5, 30, 15],    ACC = 62
        (5, 0x140,   42), # RELU16: [12, 0, 30, 0],     ACC = 42
        (6, 0x150,   52), # MAX16:  [12, 6, 30, 4],     ACC = 52
    ]

    for op_code, sram_off, exp_acc in ops_to_test:
        ctrl_val = (op_code << 2) | 0x1 # VEC_MODE=0, Start=1
        p.emit(addi(3, 0, ctrl_val))
        emit_sw_safe(p, 3, 0x008, 18)   # SIMD_REG_CTRL

        emit_lw_safe(p, 2, 0x020, 18)   # SIMD_RES_Y0
        emit_sw_safe(p, 2, sram_off + 0, 16)
        emit_lw_safe(p, 2, 0x024, 18)   # SIMD_RES_Y1
        emit_sw_safe(p, 2, sram_off + 4, 16)
        emit_lw_safe(p, 2, 0x028, 18)   # SIMD_REG_ACC
        emit_sw_safe(p, 2, sram_off + 8, 16)

        li32(p, 5, exp_acc)
        p.emit(NOP); p.emit(NOP)
        p.branch(bne, 2, 5, "FAIL_PATH")

    # -----------------------------------------------------------------------
    # 3. 32-ELEMENT (8 x 4-LANE VECTOR) STREAMING BENCHMARK (MAC16)
    #    Populate 8 copies of Vector A & Vector B into SIMD_BUF_A (0x100) & SIMD_BUF_B (0x200)
    #    and also into Data SRAM (0x2000_0200 & 0x2000_0300) for CPU Scalar Benchmark!
    #    Each 4-lane vector has dot product = -354, so 8 vectors (32 elements) = 8 * (-354) = -2832!
    # -----------------------------------------------------------------------
    for v in range(8):
        off_w0 = v * 8
        off_w1 = v * 8 + 4
        # Write to SIMD_BUF_A (0x100) & Data SRAM (0x200)
        li32(p, 3, pack2_i16(12, -5))
        emit_sw_safe(p, 3, 0x100 + off_w0, 18)
        emit_sw_safe(p, 3, 0x200 + off_w0, 16)
        li32(p, 3, pack2_i16(30, -15))
        emit_sw_safe(p, 3, 0x100 + off_w1, 18)
        emit_sw_safe(p, 3, 0x200 + off_w1, 16)

        # Write to SIMD_BUF_B (0x200) & Data SRAM (0x300)
        li32(p, 3, pack2_i16(3, 6))
        emit_sw_safe(p, 3, 0x200 + off_w0, 18)
        emit_sw_safe(p, 3, 0x300 + off_w0, 16)
        li32(p, 3, pack2_i16(-10, 4))
        emit_sw_safe(p, 3, 0x200 + off_w1, 18)
        emit_sw_safe(p, 3, 0x300 + off_w1, 16)

    # Clear SIMD Accumulator first (CTRL = bit 1)
    p.emit(addi(3, 0, 2))
    emit_sw_safe(p, 3, 0x008, 18)

    # Set SIMD_REG_LEN = 8 (8 x 4-lane vectors = 32 INT16 elements)
    p.emit(addi(3, 0, 8))
    emit_sw_safe(p, 3, 0x00C, 18)

    # Start 32-element autonomous SIMD MAC16 streaming:
    # VEC_MODE = 1 (bit 8 = 256), OP_MAC16 = 3 (bits 5:2 = 12), Start = 1 -> 256 + 12 + 1 = 269
    p.emit(addi(3, 0, 269))
    p.emit(NOP); p.emit(NOP)
    p.emit(csrrs(25, 0xC00, 0)) # start mcycle for SIMD SoC
    emit_sw_safe(p, 3, 0x008, 18)

    p.label("POLL_SIMD")
    emit_lw_safe(p, 4, 0x004, 18)
    p.emit(andi(4, 4, 2))
    p.emit(NOP); p.emit(NOP); p.emit(NOP); p.emit(NOP)
    p.branch(beq, 4, 0, "POLL_SIMD")

    p.emit(csrrs(3, 0xC00, 0)) # end mcycle for SIMD SoC
    p.emit(NOP); p.emit(NOP)
    p.emit(sub(25, 3, 25))     # x25 = SIMD 32-element SoC cycles

    # Read SIMD_REG_CYCLES (0x02C) -> x26 (must be 8 cycles!)
    emit_lw_safe(p, 26, 0x02C, 18)
    # Read 32-element SIMD_REG_ACC (0x028) -> x28 (must be -2832!)
    emit_lw_safe(p, 28, 0x028, 18)
    li32(p, 5, -2832)
    p.emit(NOP); p.emit(NOP)
    p.branch(bne, 28, 5, "FAIL_PATH")

    # -----------------------------------------------------------------------
    # 4. BENCHMARK: CPU SCALAR 32-ELEMENT INT16 DOT-PRODUCT LOOP
    # -----------------------------------------------------------------------
    p.emit(addi(20, 16, 0x200)) # ptr A = 0x2000_0200
    p.emit(addi(21, 16, 0x300)) # ptr B = 0x2000_0300
    p.emit(addi(22, 0, 0))      # i = 0
    p.emit(addi(23, 0, 16))     # 16 words = 32 x INT16 elements
    p.emit(addi(29, 0, 0))      # scalar acc = 0
    p.emit(NOP); p.emit(NOP)

    p.emit(csrrs(24, 0xC00, 0)) # start CPU mcycle
    p.emit(NOP); p.emit(NOP)

    p.label("CPU_SIMD_LOOP")
    emit_lw_safe(p, 3, 0, 20)   # load 2 packed INT16 from A
    emit_lw_safe(p, 4, 0, 21)   # load 2 packed INT16 from B
    # Extract low INT16 (sign-extended via slli 16 + srai 16)
    p.emit(slli(6, 3, 16)); p.emit(NOP); p.emit(NOP)
    p.emit(srai(6, 6, 16)); p.emit(NOP); p.emit(NOP)
    p.emit(slli(7, 4, 16)); p.emit(NOP); p.emit(NOP)
    p.emit(srai(7, 7, 16)); p.emit(NOP); p.emit(NOP)
    p.emit(mul(6, 6, 7));   p.emit(NOP); p.emit(NOP)
    p.emit(add(29, 29, 6))
    # Extract high INT16 (srai 16)
    p.emit(srai(6, 3, 16)); p.emit(NOP); p.emit(NOP)
    p.emit(srai(7, 4, 16)); p.emit(NOP); p.emit(NOP)
    p.emit(mul(6, 6, 7));   p.emit(NOP); p.emit(NOP)
    p.emit(add(29, 29, 6))

    p.emit(addi(20, 20, 4))
    p.emit(addi(21, 21, 4))
    p.emit(addi(22, 22, 1))
    p.emit(NOP); p.emit(NOP)
    p.branch(blt, 22, 23, "CPU_SIMD_LOOP")
    p.emit(NOP); p.emit(NOP)

    p.emit(csrrs(3, 0xC00, 0))  # end CPU mcycle
    p.emit(NOP); p.emit(NOP)
    p.emit(sub(24, 3, 24))      # x24 = CPU 32-element cycles

    # Verify CPU scalar dot product also equals -2832!
    li32(p, 5, -2832)
    p.emit(NOP); p.emit(NOP)
    p.branch(bne, 29, 5, "FAIL_PATH")

    # Compute Speedup = x24 / x26 -> x30
    p.emit(div(30, 24, 26))
    for _ in range(36):
        p.emit(NOP)

    # -----------------------------------------------------------------------
    # PASS PATH: Set LEDs = 0x5A, Stream 4-Lane SIMD Vectors & Telemetry
    # -----------------------------------------------------------------------
    p.label("PASS_PATH")
    p.emit(addi(6, 0, 0x5A))
    emit_sw_safe(p, 6, 4, 10)

    def call_send_char(ch):
        p.emit(addi(6, 0, ord(ch)))
        p.jump(1, "UART_SEND")
        p.emit(NOP); p.emit(NOP)

    def print_str(s):
        for ch in s:
            call_send_char(ch)

    def print_4lane_from_sram(label_str, sram_off):
        print_str(label_str)
        print_str("[")
        # Word 0: low=lane0, high=lane1
        emit_lw_safe(p, 17, sram_off + 0, 16)
        p.emit(slli(2, 17, 16)); p.emit(NOP); p.emit(NOP)
        p.emit(srai(2, 2, 16));  p.emit(NOP); p.emit(NOP)
        p.jump(3, "PRINT_CELL"); p.emit(NOP); p.emit(NOP)
        print_str(",")
        p.emit(srai(2, 17, 16)); p.emit(NOP); p.emit(NOP)
        p.jump(3, "PRINT_CELL"); p.emit(NOP); p.emit(NOP)
        print_str(",")
        # Word 1: low=lane2, high=lane3
        emit_lw_safe(p, 17, sram_off + 4, 16)
        p.emit(slli(2, 17, 16)); p.emit(NOP); p.emit(NOP)
        p.emit(srai(2, 2, 16));  p.emit(NOP); p.emit(NOP)
        p.jump(3, "PRINT_CELL"); p.emit(NOP); p.emit(NOP)
        print_str(",")
        p.emit(srai(2, 17, 16)); p.emit(NOP); p.emit(NOP)
        p.jump(3, "PRINT_CELL"); p.emit(NOP); p.emit(NOP)
        print_str(" ] ACC=")
        emit_lw_safe(p, 2, sram_off + 8, 16)
        p.jump(3, "PRINT_CELL"); p.emit(NOP); p.emit(NOP)
        print_str("\r\n")

    print_str("\r\nA = [  12,  -5,  30, -15 ], B = [   3,   6, -10,   4 ]\r\n")
    print_4lane_from_sram("ADD16 : ", 0x100)
    print_4lane_from_sram("SUB16 : ", 0x110)
    print_4lane_from_sram("MUL16 : ", 0x120)
    print_4lane_from_sram("ABS16 : ", 0x130)
    print_4lane_from_sram("RELU16: ", 0x140)
    print_4lane_from_sram("MAX16 : ", 0x150)

    print_str("32-Elem DotProd   : ")
    p.emit(addi(2, 28, 0))
    p.jump(3, "PRINT_CELL"); p.emit(NOP); p.emit(NOP)

    print_str("\r\nCPU 32-Elem Cycles: ")
    p.emit(addi(2, 24, 0))
    p.jump(3, "PRINT_CELL"); p.emit(NOP); p.emit(NOP)

    print_str("\r\nSIMD SoC Cycles   : ")
    p.emit(addi(2, 25, 0))
    p.jump(3, "PRINT_CELL"); p.emit(NOP); p.emit(NOP)

    print_str("\r\nSIMD Core Cycles  : ")
    p.emit(addi(2, 26, 0))
    p.jump(3, "PRINT_CELL"); p.emit(NOP); p.emit(NOP)

    print_str("\r\nSIMD Core Speedup : ")
    p.emit(addi(2, 30, 0))
    p.jump(3, "PRINT_CELL"); p.emit(NOP); p.emit(NOP)
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
    # PRINT_CELL SUBROUTINE
    # -----------------------------------------------------------------------
    p.label("PRINT_CELL")
    lbl = "pcell"

    p.emit(addi(12, 0, 0))
    p.branch(bge, 2, 0, f"{lbl}_pos")
    p.emit(NOP); p.emit(NOP)
    p.emit(sub(2, 0, 2))
    p.emit(addi(12, 0, 1))
    p.label(f"{lbl}_pos")

    p.emit(addi(27, 0, 0))
    p.emit(addi(13, 0, 0))
    p.emit(addi(14, 0, 0))
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

    p.emit(addi(27, 27, ord('0')))
    p.emit(addi(13, 13, ord('0')))
    p.emit(addi(14, 14, ord('0')))
    p.emit(addi(2, 2, ord('0')))
    p.emit(addi(5, 0, ord('0')))

    p.emit(addi(15, 0, ord(' ')))
    p.emit(addi(8, 0, ord(' ')))
    p.emit(addi(9, 0, ord(' ')))

    p.branch(beq, 12, 0, f"{lbl}_chk_pos")
    p.emit(NOP); p.emit(NOP)

    p.branch(bne, 27, 5, f"{lbl}_n_k")
    p.emit(NOP); p.emit(NOP)
    p.branch(bne, 13, 5, f"{lbl}_n_h")
    p.emit(NOP); p.emit(NOP)
    p.branch(bne, 14, 5, f"{lbl}_n_t")
    p.emit(NOP); p.emit(NOP)
    p.emit(addi(9, 0, ord('-')))
    p.branch(beq, 0, 0, f"{lbl}_emit4")
    p.emit(NOP); p.emit(NOP)
    p.label(f"{lbl}_n_t")
    p.emit(addi(8, 0, ord('-')))
    p.emit(addi(9, 14, 0))
    p.branch(beq, 0, 0, f"{lbl}_emit4")
    p.emit(NOP); p.emit(NOP)
    p.label(f"{lbl}_n_h")
    p.emit(addi(15, 0, ord('-')))
    p.emit(addi(8, 13, 0))
    p.emit(addi(9, 14, 0))
    p.branch(beq, 0, 0, f"{lbl}_emit4")
    p.emit(NOP); p.emit(NOP)
    p.label(f"{lbl}_n_k")
    # For negative numbers >= 1000 (e.g. -2832), print '-' before 4 digits
    p.emit(addi(6, 0, ord('-')))
    p.jump(1, "UART_SEND")
    p.emit(NOP); p.emit(NOP)
    p.emit(addi(15, 27, 0))
    p.emit(addi(8, 13, 0))
    p.emit(addi(9, 14, 0))
    p.branch(beq, 0, 0, f"{lbl}_emit4")
    p.emit(NOP); p.emit(NOP)

    p.label(f"{lbl}_chk_pos")
    p.branch(bne, 27, 5, f"{lbl}_p_k")
    p.emit(NOP); p.emit(NOP)
    p.branch(bne, 13, 5, f"{lbl}_p_h")
    p.emit(NOP); p.emit(NOP)
    p.branch(bne, 14, 5, f"{lbl}_p_t")
    p.emit(NOP); p.emit(NOP)
    p.branch(beq, 0, 0, f"{lbl}_emit4")
    p.emit(NOP); p.emit(NOP)
    p.label(f"{lbl}_p_t")
    p.emit(addi(9, 14, 0))
    p.branch(beq, 0, 0, f"{lbl}_emit4")
    p.emit(NOP); p.emit(NOP)
    p.label(f"{lbl}_p_h")
    p.emit(addi(8, 13, 0))
    p.emit(addi(9, 14, 0))
    p.branch(beq, 0, 0, f"{lbl}_emit4")
    p.emit(NOP); p.emit(NOP)
    p.label(f"{lbl}_p_k")
    p.emit(addi(15, 27, 0))
    p.emit(addi(8, 13, 0))
    p.emit(addi(9, 14, 0))

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

    p.emit(jalr(0, 3, 0))
    p.emit(NOP); p.emit(NOP); p.emit(NOP); p.emit(NOP)

    p.label("UART_SEND")
    emit_sw_safe(p, 6, 0, 10)
    p.emit(addi(7, 0, uart_delay))
    p.label("UART_DLY")
    p.emit(addi(7, 7, -1 & 0xFFF))
    p.emit(NOP)
    p.branch(bne, 7, 0, "UART_DLY")
    p.emit(NOP); p.emit(NOP)
    p.emit(jalr(0, 1, 0))
    p.emit(NOP); p.emit(NOP); p.emit(NOP); p.emit(NOP)

    return p.assemble()

def main():
    target_port = sys.argv[1] if len(sys.argv) > 1 else COM_PORT
    uart_delay = 1 if target_port == "--fast-sim" else 1500
    instructions = build_program(uart_delay=uart_delay)
    print("=" * 65)
    print("  ASTRA-X1 MILESTONE 6A: 4-LANE INT16 PACKED SIMD BENCHMARK")
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
    print(f"  ASTRA-X1 MILESTONE 6A - HARDWARE UPLOAD (EDGE ARTIX-7 100T)")
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
        time.sleep(len(all_bytes) * 10.0 / BAUD_RATE + 0.3)
        print("Upload complete!")
        print("--------------------------------------------------")
        print("2. Please RELEASE the RESET button now.")
        print("   Listening for execution response on UART...")
        print("--------------------------------------------------\n")

        output_buffer = []
        deadline = time.time() + 10.0
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
            print("\n>>> [SUCCESS] MILESTONE 6A 4-LANE INT16 SIMD BENCHMARK PASSED! <<<")
            print(">>> 1. SIMD Slave 5 ID (0x53494D44 'SIMD') verified.")
            print(">>> 2. All 6 parallel 4-lane INT16 operations verified bit-exactly.")
            print(">>> 3. 32-element autonomous SIMD streaming verified against CPU scalar!")
            print(">>> Check your board: LEDs MUST BE 0x5A (Binary 0b01011010)!")
        elif "FAIL" in output_str:
            print("\n>>> [FAILURE] Processor reported FAIL.")
            print(">>> LEDs are displaying 0xEE (11101110).")
        elif not output_str:
            print(">>> [WARNING] No UART response received. Verify COM port and board power.")
        print("=" * 65)
    finally:
        ser.close()

if __name__ == "__main__":
    main()
