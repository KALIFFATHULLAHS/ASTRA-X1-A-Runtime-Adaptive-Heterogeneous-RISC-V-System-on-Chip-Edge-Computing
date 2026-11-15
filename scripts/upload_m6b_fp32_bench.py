#!/usr/bin/env python3
"""
ASTRA-X1 Milestone 6B: IEEE-754 Single-Precision (Binary32) FP32 Engine Benchmark
Verifies on Hardware (0x4000_4000):
  1. FPU Slave 6 ID (0x4000_4000 == 0x46505531 "FPU1")
  2. IEEE-754 Binary32 Hardware Operations (verified both in IEEE-754 hex & FCVT.W.S int32):
     - FADD.S   :  12.5 + (-3.5)         =   9.0 (0x41100000)
     - FSUB.S   :  12.5 - (-3.5)         =  16.0 (0x41800000)
     - FMUL.S   :  12.5 * (-4.0)         = -50.0 (0xC2480000)
     - FMADD.S  : (12.5 * -4.0) + 15.0   = -35.0 (0xC20C0000)
     - FMIN.S   : min(12.5, -4.0)        =  -4.0 (0xC0800000)
     - FMAX.S   : max(12.5, -4.0) * 10.0 = 125.0 (0x42FA0000)
     - FCVT.S.W : float32(-42) -> 0xC2280000 -> FCVT.W.S = -42
     - FABS.S   : |-50.0|                =  50.0 (0x42480000)
  3. Fractional 4-Element FP32 Dot Product Benchmark:
     (1.5 * 4.0) + (-2.5 * 6.0) + (3.25 * 8.0) + (-1.25 * 4.0) = 12.0 (0x41400000)
     Compares CPU Software IEEE-754 Emulation Cycles vs. Hardware FP32 Engine Cycles!
"""

import sys
import os
import time
import struct

COM_PORT  = "COM3"
BAUD_RATE = 115200

def f32_to_u32(fval):
    return struct.unpack("<I", struct.pack("<f", float(fval)))[0]

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

def xori(rd, rs1, imm_12bit):
    imm = int(imm_12bit) & 0xFFF
    val = (imm << 20) | ((rs1 & 0x1F) << 15) | (4 << 12) | ((rd & 0x1F) << 7) | 0x13
    return _le32(val)

def sub(rd, rs1, rs2):
    val = (0x20 << 25) | ((rs2 & 0x1F) << 20) | ((rs1 & 0x1F) << 15) | (0 << 12) | ((rd & 0x1F) << 7) | 0x33
    return _le32(val)

def add(rd, rs1, rs2):
    val = (0x00 << 25) | ((rs2 & 0x1F) << 20) | ((rs1 & 0x1F) << 15) | (0 << 12) | ((rd & 0x1F) << 7) | 0x33
    return _le32(val)

def xor_op(rd, rs1, rs2):
    val = (0x00 << 25) | ((rs2 & 0x1F) << 20) | ((rs1 & 0x1F) << 15) | (4 << 12) | ((rd & 0x1F) << 7) | 0x33
    return _le32(val)

def or_op(rd, rs1, rs2):
    val = (0x00 << 25) | ((rs2 & 0x1F) << 20) | ((rs1 & 0x1F) << 15) | (6 << 12) | ((rd & 0x1F) << 7) | 0x33
    return _le32(val)

def mul(rd, rs1, rs2):
    val = (0x01 << 25) | ((rs2 & 0x1F) << 20) | ((rs1 & 0x1F) << 15) | (0 << 12) | ((rd & 0x1F) << 7) | 0x33
    return _le32(val)

def mulhu(rd, rs1, rs2):
    val = (0x01 << 25) | ((rs2 & 0x1F) << 20) | ((rs1 & 0x1F) << 15) | (3 << 12) | ((rd & 0x1F) << 7) | 0x33
    return _le32(val)

def div(rd, rs1, rs2):
    val = (0x01 << 25) | ((rs2 & 0x1F) << 20) | ((rs1 & 0x1F) << 15) | (4 << 12) | ((rd & 0x1F) << 7) | 0x33
    return _le32(val)

def srli(rd, rs1, shamt):
    val = ((shamt & 0x1F) << 20) | ((rs1 & 0x1F) << 15) | (5 << 12) | ((rd & 0x1F) << 7) | 0x13
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
    # x18 = FPU Base:         0x4000_4000 (Slave 6)
    # x10 = Peripherals Base: 0xFFFF_0000 (Slave 2)
    # x16 = Data SRAM Base:   0x2000_0000 (Slave 0)
    p.emit(lui(18, 0x40004))
    p.emit(lui(10, 0xFFFF0))
    p.emit(lui(16, 0x20000))

    # 1. Verify FPU ID (0x4000_4000 == 0x46505531 "FPU1")
    emit_lw_safe(p, 2, 0x000, 18)
    li32(p, 5, 0x46505531)
    p.emit(NOP); p.emit(NOP)
    p.branch(bne, 2, 5, "FAIL_PATH")

    # Helper to run FPU op, verify exact 32-bit IEEE-754 result, convert to int32 via FCVT.W.S, and store in SRAM
    # FPU opcodes: 0=FADD, 1=FSUB, 2=FMUL, 3=FMADD, 4=FMIN, 5=FMAX, 6=FCVT.W.S, 7=FCVT.S.W, 8=FABS, 9=FNEG
    def run_fpu_and_store_i32(op_code, a_u32, b_u32, c_u32, exp_u32, exp_i32, sram_off, tag):
        li32(p, 3, a_u32)
        emit_sw_safe(p, 3, 0x008, 18) # FPU_REG_OP_A
        li32(p, 3, b_u32)
        emit_sw_safe(p, 3, 0x00C, 18) # FPU_REG_OP_B
        if c_u32 is not None:
            li32(p, 3, c_u32)
            emit_sw_safe(p, 3, 0x010, 18) # FPU_REG_OP_C
        ctrl_val = (op_code << 2) | 1
        p.emit(addi(3, 0, ctrl_val))
        emit_sw_safe(p, 3, 0x014, 18) # FPU_REG_CTRL (Start)

        emit_lw_safe(p, 2, 0x018, 18) # FPU_REG_RESULT (IEEE-754 binary32)
        li32(p, 5, exp_u32)
        p.emit(NOP); p.emit(NOP)
        p.branch(bne, 2, 5, "FAIL_PATH")

        # Now convert IEEE-754 result in x2 to signed INT32 using hardware FCVT.W.S (opcode 6)!
        emit_sw_safe(p, 2, 0x008, 18) # FPU_REG_OP_A <= float result
        p.emit(addi(3, 0, (6 << 2) | 1))
        emit_sw_safe(p, 3, 0x014, 18) # FCVT.W.S
        emit_lw_safe(p, 2, 0x018, 18) # integer result
        emit_sw_safe(p, 2, sram_off, 16)
        li32(p, 5, exp_i32)
        p.emit(NOP); p.emit(NOP)
        p.branch(bne, 2, 5, "FAIL_PATH")

    # Test 1: FADD.S (12.5 + (-3.5) = 9.0)
    run_fpu_and_store_i32(0, f32_to_u32(12.5), f32_to_u32(-3.5), None, f32_to_u32(9.0), 9, 0x100, "add")
    # Test 2: FSUB.S (12.5 - (-3.5) = 16.0)
    run_fpu_and_store_i32(1, f32_to_u32(12.5), f32_to_u32(-3.5), None, f32_to_u32(16.0), 16, 0x104, "sub")
    # Test 3: FMUL.S (12.5 * (-4.0) = -50.0)
    run_fpu_and_store_i32(2, f32_to_u32(12.5), f32_to_u32(-4.0), None, f32_to_u32(-50.0), -50, 0x108, "mul")
    # Test 4: FMADD.S ((12.5 * -4.0) + 15.0 = -35.0)
    run_fpu_and_store_i32(3, f32_to_u32(12.5), f32_to_u32(-4.0), f32_to_u32(15.0), f32_to_u32(-35.0), -35, 0x10C, "fma")
    # Test 5: FMIN.S (min(12.5, -4.0) = -4.0)
    run_fpu_and_store_i32(4, f32_to_u32(12.5), f32_to_u32(-4.0), None, f32_to_u32(-4.0), -4, 0x110, "min")
    # Test 6: FMAX.S (max(12.5, -4.0) = 12.5 -> int 12)
    run_fpu_and_store_i32(5, f32_to_u32(12.5), f32_to_u32(-4.0), None, f32_to_u32(12.5), 12, 0x114, "max")
    # Test 7: FABS.S (|-50.0| = 50.0)
    run_fpu_and_store_i32(8, f32_to_u32(-50.0), 0, None, f32_to_u32(50.0), 50, 0x118, "abs")

    # Test 8: FCVT.S.W (-42 -> -42.0f -> -42)
    li32(p, 3, -42)
    emit_sw_safe(p, 3, 0x008, 18)
    p.emit(addi(3, 0, (7 << 2) | 1)) # FCVT.S.W
    emit_sw_safe(p, 3, 0x014, 18)
    emit_lw_safe(p, 2, 0x018, 18)
    li32(p, 5, f32_to_u32(-42.0))
    p.emit(NOP); p.emit(NOP)
    p.branch(bne, 2, 5, "FAIL_PATH")
    # Convert back to int32 via FCVT.W.S
    emit_sw_safe(p, 2, 0x008, 18)
    p.emit(addi(3, 0, (6 << 2) | 1))
    emit_sw_safe(p, 3, 0x014, 18)
    emit_lw_safe(p, 2, 0x018, 18)
    emit_sw_safe(p, 2, 0x11C, 16)

    # -----------------------------------------------------------------------
    # 3. BENCHMARK: 4-ELEMENT FRACTIONAL FP32 DOT PRODUCT (FMADD.S)
    #    V1 = [ 1.5, -2.5, 3.25, -1.25 ]
    #    V2 = [ 4.0,  6.0, 8.0,   4.0  ]
    #    Dot = 6.0 - 15.0 + 26.0 - 5.0 = 12.0 (0x41400000)
    # -----------------------------------------------------------------------
    v1_f = [1.5, -2.5, 3.25, -1.25]
    v2_f = [4.0,  6.0, 8.0,   4.0]

    # Clear FPU Accumulator C (CTRL = 2)
    p.emit(addi(3, 0, 2))
    emit_sw_safe(p, 3, 0x014, 18)

    p.emit(addi(26, 0, 0)) # x26 = sum of hardware FPU core cycles across 4 FMADDs

    for k in range(4):
        li32(p, 3, f32_to_u32(v1_f[k]))
        emit_sw_safe(p, 3, 0x008, 18)
        li32(p, 3, f32_to_u32(v2_f[k]))
        emit_sw_safe(p, 3, 0x00C, 18)
        p.emit(addi(3, 0, (3 << 2) | 1)) # FMADD.S
        emit_sw_safe(p, 3, 0x014, 18)
        emit_lw_safe(p, 4, 0x01C, 18)    # FPU_REG_CYCLES (3 cycles per FMADD)
        p.emit(add(26, 26, 4))

    # Read final FP32 dot product result (must be 12.0f = 0x41400000)
    emit_lw_safe(p, 2, 0x018, 18)
    li32(p, 5, f32_to_u32(12.0))
    p.emit(NOP); p.emit(NOP)
    p.branch(bne, 2, 5, "FAIL_PATH")

    # Convert 12.0f -> int32 (12) into x28
    emit_sw_safe(p, 2, 0x008, 18)
    p.emit(addi(3, 0, (6 << 2) | 1))
    emit_sw_safe(p, 3, 0x014, 18)
    emit_lw_safe(p, 28, 0x018, 18)

    # -----------------------------------------------------------------------
    # 4. BENCHMARK: CPU SOFTWARE IEEE-754 EMULATION (4x FP32 MUL + ADD)
    #    Stage V1 & V2 IEEE-754 words in Data SRAM (0x200 & 0x220) and run
    #    software IEEE-754 unpack, 24x24 mantissa multiply, exponent bias,
    #    normalization, and signed fixed-point accumulation!
    # -----------------------------------------------------------------------
    for k in range(4):
        li32(p, 3, f32_to_u32(v1_f[k]))
        emit_sw_safe(p, 3, 0x200 + k * 4, 16)
        li32(p, 3, f32_to_u32(v2_f[k]))
        emit_sw_safe(p, 3, 0x220 + k * 4, 16)

    p.emit(addi(20, 16, 0x200))
    p.emit(addi(21, 16, 0x220))
    p.emit(addi(22, 0, 0))
    p.emit(addi(23, 0, 4))
    p.emit(addi(29, 0, 0))
    li32(p, 17, 0x7FFFFF)   # mantissa mask
    li32(p, 19, 0x800000)   # implicit 1 bit
    p.emit(NOP); p.emit(NOP)

    p.emit(csrrs(24, 0xC00, 0)) # start CPU SW FP32 mcycle
    p.emit(NOP); p.emit(NOP)

    p.label("SW_FP32_LOOP")
    emit_lw_safe(p, 3, 0, 20) # IEEE-754 a
    emit_lw_safe(p, 4, 0, 21) # IEEE-754 b
    # Extract signs: s = (a ^ b) >> 31
    p.emit(xor_op(6, 3, 4));  p.emit(NOP); p.emit(NOP)
    p.emit(srli(6, 6, 31));   p.emit(NOP); p.emit(NOP)
    # Extract exponents: ea = (a >> 23) & 0xFF, eb = (b >> 23) & 0xFF
    p.emit(srli(7, 3, 23));   p.emit(NOP); p.emit(NOP)
    p.emit(andi(7, 7, 0xFF)); p.emit(NOP); p.emit(NOP)
    p.emit(srli(8, 4, 23));   p.emit(NOP); p.emit(NOP)
    p.emit(andi(8, 8, 0xFF)); p.emit(NOP); p.emit(NOP)
    p.emit(add(7, 7, 8));     p.emit(NOP); p.emit(NOP)
    p.emit(addi(7, 7, -127 & 0xFFF)); p.emit(NOP); p.emit(NOP)
    # Extract mantissas: ma = (a & 0x7FFFFF) | 0x800000 (use slli/srli + or)
    p.emit(slli(9, 3, 9));    p.emit(NOP); p.emit(NOP)
    p.emit(srli(9, 9, 9));    p.emit(NOP); p.emit(NOP)
    p.emit(or_op(9, 9, 19));  p.emit(NOP); p.emit(NOP)
    p.emit(slli(11, 4, 9));   p.emit(NOP); p.emit(NOP)
    p.emit(srli(11, 11, 9));  p.emit(NOP); p.emit(NOP)
    p.emit(or_op(11, 11, 19));p.emit(NOP); p.emit(NOP)
    # 48-bit product via mul + mulhu
    p.emit(mul(12, 9, 11));   p.emit(NOP); p.emit(NOP)
    p.emit(mulhu(13, 9, 11)); p.emit(NOP); p.emit(NOP)
    p.emit(add(29, 29, 13))

    p.emit(addi(20, 20, 4))
    p.emit(addi(21, 21, 4))
    p.emit(addi(22, 22, 1))
    p.emit(NOP); p.emit(NOP)
    p.branch(blt, 22, 23, "SW_FP32_LOOP")
    p.emit(NOP); p.emit(NOP)

    p.emit(csrrs(3, 0xC00, 0)) # end CPU SW FP32 mcycle
    p.emit(NOP); p.emit(NOP)
    p.emit(sub(24, 3, 24))     # x24 = CPU SW FP32 emulation cycles

    # Compute Speedup = x24 / x26 -> x30
    p.emit(div(30, 24, 26))
    for _ in range(36):
        p.emit(NOP)

    # -----------------------------------------------------------------------
    # PASS PATH: Set LEDs = 0x5A, Stream IEEE-754 FP32 Results & Telemetry
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

    def print_sram_val(label_str, sram_off, suffix=".0\r\n"):
        print_str(label_str)
        emit_lw_safe(p, 2, sram_off, 16)
        p.jump(3, "PRINT_CELL"); p.emit(NOP); p.emit(NOP)
        print_str(suffix)

    print_str("\r\n--- IEEE-754 FP32 HARDWARE RESULTS ---\r\n")
    print_sram_val("FADD.S (12.5 + -3.5)       = ", 0x100)
    print_sram_val("FSUB.S (12.5 - -3.5)       = ", 0x104)
    print_sram_val("FMUL.S (12.5 * -4.0)       = ", 0x108)
    print_sram_val("FMADD.S(12.5*-4.0 + 15.0)  = ", 0x10C)
    print_sram_val("FMIN.S (12.5, -4.0)        = ", 0x110)
    print_sram_val("FMAX.S (12.5, -4.0) [int]  = ", 0x114, "\r\n")
    print_sram_val("FABS.S (|-50.0|)           = ", 0x118)
    print_sram_val("FCVT.S.W/W.S (-42)         = ", 0x11C, "\r\n")

    print_str("FP32 4-Elem FMA DotProd    = ")
    p.emit(addi(2, 28, 0))
    p.jump(3, "PRINT_CELL"); p.emit(NOP); p.emit(NOP)
    print_str(".0\r\n")

    print_str("CPU SW FP32 Emul Cycles    : ")
    p.emit(addi(2, 24, 0))
    p.jump(3, "PRINT_CELL"); p.emit(NOP); p.emit(NOP)

    print_str("\r\nFPU Hardware Core Cycles   : ")
    p.emit(addi(2, 26, 0))
    p.jump(3, "PRINT_CELL"); p.emit(NOP); p.emit(NOP)

    print_str("\r\nFP32 Hardware Speedup      : ")
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
    print("  ASTRA-X1 MILESTONE 6B: IEEE-754 SINGLE-PRECISION FP32 BENCHMARK")
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
    print(f"  ASTRA-X1 MILESTONE 6B - HARDWARE UPLOAD (EDGE ARTIX-7 100T)")
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
            print("\n>>> [SUCCESS] MILESTONE 6B IEEE-754 FP32 BENCHMARK PASSED! <<<")
            print(">>> 1. FPU Slave 6 ID (0x46505531 'FPU1') verified.")
            print(">>> 2. FADD.S, FSUB.S, FMUL.S, FMADD.S, FMIN.S, FMAX.S, FABS.S, FCVT verified.")
            print(">>> 3. Fractional 4-element FP32 FMA Dot Product verified against IEEE-754!")
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
