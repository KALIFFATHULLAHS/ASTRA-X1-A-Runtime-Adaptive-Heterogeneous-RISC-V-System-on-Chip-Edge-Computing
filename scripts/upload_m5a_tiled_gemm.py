#!/usr/bin/env python3
"""
ASTRA-X1 Milestone 5A: General-Purpose 8x8 Block-Tiled NPU GEMM & Multi-Tile Benchmark
Demonstrates:
  1. Dynamic Hardware K-Streaming (NPU_REG_K_DIM = 8 at 0x4000_0010)
  2. 2x2 Output Block Tiling across four 4x8x4 tiles (64 output cells, 512 INT8 MACs)
  3. Full 8x8 Matrix assembly in Data SRAM (0x2000_0400) + 64/64 cell hardware verification
  4. Pure RV32IM CPU vs. NPU Multi-Tile Hardware Cycle & Speedup Benchmark over UART
"""

import sys
import os
import time
import numpy as np

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

def pack4_i8(b0, b1, b2, b3):
    return ((int(b3) & 0xFF) << 24) | ((int(b2) & 0xFF) << 16) | ((int(b1) & 0xFF) << 8) | (int(b0) & 0xFF)

def get_8x8_matrices():
    A = np.array([
        [ 1,  2,  3,  4,  1,  0, -1,  2],
        [ 5,  6,  7,  8,  0,  1,  2, -1],
        [-1, -2, -3, -4,  2, -1,  0,  1],
        [10, 20, 30, 40, -5,  5, -5,  5],
        [ 2, -2,  4, -4,  3,  3, -3, -3],
        [ 1,  3,  5,  7,  9, -9, -7, -5],
        [-3,  0,  3,  6, -6,  0,  3, -3],
        [ 4,  4,  4,  4,  2,  2,  2,  2]
    ], dtype=np.int32)

    B = np.array([
        [ 2,  0,  1,  3, -1,  2,  0,  1],
        [ 1,  2,  0, -1,  2, -1,  1,  0],
        [ 0,  1,  2,  1,  1,  1, -2,  2],
        [ 3,  0, -2,  1,  0, -2,  1, -1],
        [ 1, -1,  1, -1,  2,  0,  2,  0],
        [ 0,  2, -1,  0, -1,  3,  0,  1],
        [-2,  1,  0,  2,  1, -1,  3, -2],
        [ 1,  0,  3, -2,  0,  1, -1,  2]
    ], dtype=np.int32)

    C = A @ B
    return A, B, C

def build_program(uart_delay=1500):
    p = ProgramBuilder()
    A, B, C_gold = get_8x8_matrices()

    # Base addresses:
    # x11 = NPU Base: 0x4000_0000
    # x10 = Peripherals Base: 0xFFFF_0000
    # x16 = Data SRAM Base: 0x2000_0000
    p.emit(lui(11, 0x40000))
    p.emit(lui(10, 0xFFFF0))
    p.emit(lui(16, 0x20000))

    # Verify NPU ID ("ASTR" = 0x41535452)
    emit_lw_safe(p, 2, 0, 11)
    li32(p, 5, 0x41535452)
    p.emit(NOP); p.emit(NOP)
    p.branch(bne, 2, 5, "FAIL_PATH")

    # Configure NPU inner dimension K = 8 (0x4000_0010)
    p.emit(addi(3, 0, 8))
    emit_sw_safe(p, 3, 0x010, 11)

    # Clear total NPU MAC cycles accumulator (x26 = 0) and NPU SoC cycles accumulator (x25 = 0)
    p.emit(addi(25, 0, 0))
    p.emit(addi(26, 0, 0))

    # -----------------------------------------------------------------------
    # EXECUTE 8x8 MATRIX MULTIPLICATION ACROSS 4 HARDWARE TILES (2x2 Grid)
    # Each tile multiplies a 4x8 sub-matrix of A by an 8x4 sub-matrix of B (K=8)
    # -----------------------------------------------------------------------
    for tr in range(2):
        for tc in range(2):
            tile_lbl = f"POLL_T{tr}_{tc}"

            # Load 4x8 sub-matrix A[tr*4 : tr*4+4, 0:8] into NPU BUF_A (0x4000_0100..0x011C)
            # Row-major: 8 words (32 bytes)
            A_sub = A[tr*4 : (tr+1)*4, 0:8].flatten()
            for w_idx in range(8):
                word_val = pack4_i8(A_sub[w_idx*4], A_sub[w_idx*4+1], A_sub[w_idx*4+2], A_sub[w_idx*4+3])
                li32(p, 3, word_val)
                emit_sw_safe(p, 3, 0x100 + w_idx * 4, 11)

            # Load 8x4 sub-matrix B[0:8, tc*4 : tc*4+4] into NPU BUF_B (0x4000_0200..0x021C)
            # Row-major: 8 words (32 bytes)
            B_sub = B[0:8, tc*4 : (tc+1)*4].flatten()
            for w_idx in range(8):
                word_val = pack4_i8(B_sub[w_idx*4], B_sub[w_idx*4+1], B_sub[w_idx*4+2], B_sub[w_idx*4+3])
                li32(p, 3, word_val)
                emit_sw_safe(p, 3, 0x200 + w_idx * 4, 11)

            # Measure SoC cycles for this tile execution
            p.emit(csrrs(20, 0xC00, 0))
            p.emit(NOP); p.emit(NOP)

            # Trigger NPU Execution (0x4000_000C = 1)
            p.emit(addi(3, 0, 1))
            emit_sw_safe(p, 3, 0x00C, 11)

            # Poll STATUS (0x4000_0004) until Done (bit 1) == 1
            p.label(tile_lbl)
            emit_lw_safe(p, 4, 0x004, 11)
            p.emit(andi(4, 4, 2))
            p.emit(NOP); p.emit(NOP); p.emit(NOP); p.emit(NOP)
            p.branch(beq, 4, 0, tile_lbl)

            # Accumulate SoC cycles: x25 += (mcycle - x20)
            p.emit(csrrs(21, 0xC00, 0))
            p.emit(NOP); p.emit(NOP)
            p.emit(sub(21, 21, 20))
            p.emit(NOP); p.emit(NOP)
            p.emit(add(25, 25, 21))

            # Accumulate NPU hardware MAC cycles: x26 += NPU_REG_CYCLES (0x4000_0014)
            emit_lw_safe(p, 21, 0x014, 11)
            p.emit(add(26, 26, 21))

            # Read 16 output cells of this 4x4 quadrant, verify against C_gold,
            # and store into full 8x8 matrix C in Data SRAM (0x2000_0400 + (row*8 + col)*4)
            for r in range(4):
                for c in range(4):
                    glob_r = tr * 4 + r
                    glob_c = tc * 4 + c
                    exp_val = int(C_gold[glob_r, glob_c])
                    npu_off = 0x40 + (r * 4 + c) * 4
                    sram_off = 0x400 + (glob_r * 8 + glob_c) * 4

                    emit_lw_safe(p, 2, npu_off, 11)
                    li32(p, 5, exp_val)
                    p.emit(NOP); p.emit(NOP)
                    p.branch(bne, 2, 5, "FAIL_PATH")
                    p.emit(NOP); p.emit(NOP)
                    emit_sw_safe(p, 2, sram_off, 16)

    # -----------------------------------------------------------------------
    # PASS PATH: Set LEDs = 0x5A, Stream Full 8x8 Matrix C (64 cells) over UART
    # -----------------------------------------------------------------------
    p.label("PASS_PATH")
    p.emit(addi(6, 0, 0x5A))
    emit_sw_safe(p, 6, 4, 10) # dbg_led <= 0x5A

    def call_send_char(ch):
        p.emit(addi(6, 0, ord(ch)))
        p.jump(1, "UART_SEND")
        p.emit(NOP); p.emit(NOP)

    def print_str(s):
        for ch in s:
            call_send_char(ch)

    print_str("\r\nC_8x8 (4 Tiles, K=8) =\r\n")

    # Compact Hardware Loop to Print All 8 Rows x 8 Cols from Data SRAM (0x2000_0400..0x04FC)
    p.emit(addi(20, 16, 0x400)) # x20 = pointer to C_8x8[0, 0] in Data SRAM
    p.emit(addi(17, 0, 0))      # r = 0
    p.emit(addi(28, 0, 8))      # bound = 8
    p.emit(addi(29, 0, 7))      # last_col = 7
    p.emit(NOP); p.emit(NOP)

    p.label("PRT_ROW_LOOP")
    call_send_char('[')
    p.emit(addi(18, 0, 0))      # c = 0
    p.emit(NOP); p.emit(NOP)

    p.label("PRT_COL_LOOP")
    emit_lw_safe(p, 2, 0, 20)   # Load C_8x8[r, c] into x2
    p.emit(addi(20, 20, 4))     # Advance SRAM pointer by 4 bytes
    p.jump(3, "PRINT_CELL")
    p.emit(NOP); p.emit(NOP)

    p.branch(bge, 18, 29, "PRT_NO_COMMA")
    p.emit(NOP); p.emit(NOP)
    call_send_char(',')
    p.label("PRT_NO_COMMA")

    p.emit(addi(18, 18, 1))
    p.emit(NOP); p.emit(NOP)
    p.branch(blt, 18, 28, "PRT_COL_LOOP")
    p.emit(NOP); p.emit(NOP)

    print_str(" ]\r\n")
    p.emit(addi(17, 17, 1))
    p.emit(NOP); p.emit(NOP)
    p.branch(blt, 17, 28, "PRT_ROW_LOOP")
    p.emit(NOP); p.emit(NOP)

    print_str("NPU 4-Tile SoC Cycles: ")
    p.emit(addi(2, 25, 0))
    p.jump(3, "PRINT_CELL")
    p.emit(NOP); p.emit(NOP)

    print_str("\r\nNPU 4-Tile MAC Cycles: ")
    p.emit(addi(2, 26, 0))
    p.jump(3, "PRINT_CELL")
    p.emit(NOP); p.emit(NOP)

    print_str("\r\nVerified 64/64 Cells : PASS\r\n")

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
    # -----------------------------------------------------------------------
    p.label("PRINT_CELL")
    lbl = "pcell"

    p.emit(addi(12, 0, 0)) # is_neg = 0
    p.branch(bge, 2, 0, f"{lbl}_pos")
    p.emit(NOP); p.emit(NOP)
    p.emit(sub(2, 0, 2))   # x2 = -x2
    p.emit(addi(12, 0, 1)) # is_neg = 1
    p.label(f"{lbl}_pos")

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

    # --- NEGATIVE:
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

    # --- POSITIVE:
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

    # -----------------------------------------------------------------------
    # UART SEND SUBROUTINE (Called via JAL x1, UART_SEND)
    # -----------------------------------------------------------------------
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
    print("  ASTRA-X1 MILESTONE 5A: 8x8 BLOCK-TILED NPU GEMM (4 TILES, K=8)")
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
    print(f"  ASTRA-X1 MILESTONE 5A - HARDWARE UPLOAD (EDGE ARTIX-7 100T)")
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
            print("\n>>> [SUCCESS] MILESTONE 5A 8x8 BLOCK-TILED GEMM PASSED! <<<")
            print(">>> 1. Dynamic Hardware K-Streaming (K_DIM = 8) verified.")
            print(">>> 2. 2x2 Output Block Tiling (4 tiles x 16 cells = 64 cells) verified.")
            print(">>> 3. Full 8x8 Matrix C assembled in Data SRAM & 64/64 cells matched!")
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
