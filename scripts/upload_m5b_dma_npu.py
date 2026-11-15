#!/usr/bin/env python3
"""
ASTRA-X1 Milestone 5B: Autonomous DMA + NPU Systolic Array Pipeline Driver
Verifies:
  1. Slave 3 DMA Controller ID (0x4000_1000 == 0x444D4131 "DMA1") & NPU ID ("ASTR")
  2. Multi-Master Wishbone B4 Arbitration (Master 0 CPU + Master 1 DMA)
  3. DMA Transfer #1: Data SRAM (0x2000_0100) -> NPU Buffer A (0x4000_0100) [4 words]
  4. DMA Transfer #2: Data SRAM (0x2000_0200) -> NPU Buffer B (0x4000_0200) [4 words]
  5. NPU 4x4 INT8 Systolic Array GEMM Execution
  6. DMA Transfer #3: NPU Output C (0x4000_0040) -> Data SRAM (0x2000_0300) [16 words]
  7. CPU Copy vs. Hardware DMA Burst Cycle Comparison (16 words) + Full Matrix Print
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

def build_program(uart_delay=1500):
    p = ProgramBuilder()

    # Base addresses:
    # x11 = NPU Base:         0x4000_0000 (Slave 1)
    # x19 = DMA Base:         0x4000_1000 (Slave 3)
    # x10 = Peripherals Base: 0xFFFF_0000 (Slave 2)
    # x16 = Data SRAM Base:   0x2000_0000 (Slave 0)
    p.emit(lui(11, 0x40000))
    p.emit(lui(19, 0x40001))
    p.emit(lui(10, 0xFFFF0))
    p.emit(lui(16, 0x20000))

    # 1. Verify NPU ID (0x4000_0000 == 0x41535452 "ASTR")
    emit_lw_safe(p, 2, 0, 11)
    li32(p, 5, 0x41535452)
    p.emit(NOP); p.emit(NOP)
    p.branch(bne, 2, 5, "FAIL_PATH")

    # 2. Verify DMA Controller ID (0x4000_1000 == 0x444D4131 "DMA1")
    emit_lw_safe(p, 2, 0, 19)
    li32(p, 5, 0x444D4131)
    p.emit(NOP); p.emit(NOP)
    p.branch(bne, 2, 5, "FAIL_PATH")

    # 3. Stage Matrix A (4 words) at Data SRAM 0x2000_0100..0x010C
    li32(p, 3, 0x04030201)
    emit_sw_safe(p, 3, 0x100, 16)
    li32(p, 3, 0x08070605)
    emit_sw_safe(p, 3, 0x104, 16)
    li32(p, 3, 0xFCFDFEFF)
    emit_sw_safe(p, 3, 0x108, 16)
    li32(p, 3, 0x281E140A)
    emit_sw_safe(p, 3, 0x10C, 16)

    # 4. Stage Matrix B (4 words) at Data SRAM 0x2000_0200..0x020C
    li32(p, 3, 0x03010002)
    emit_sw_safe(p, 3, 0x200, 16)
    li32(p, 3, 0xFF000201)
    emit_sw_safe(p, 3, 0x204, 16)
    li32(p, 3, 0x01020100)
    emit_sw_safe(p, 3, 0x208, 16)
    li32(p, 3, 0x01FE0003)
    emit_sw_safe(p, 3, 0x20C, 16)

    # -----------------------------------------------------------------------
    # 5. DMA TRANSFER #1: Data SRAM (0x2000_0100) -> NPU BUF_A (0x4000_0100) [4 words]
    # -----------------------------------------------------------------------
    li32(p, 3, 0x20000100)
    emit_sw_safe(p, 3, 0x008, 19) # DMA_REG_SRC_ADDR
    li32(p, 3, 0x40000100)
    emit_sw_safe(p, 3, 0x00C, 19) # DMA_REG_DST_ADDR
    p.emit(addi(3, 0, 4))
    emit_sw_safe(p, 3, 0x010, 19) # DMA_REG_LEN_WORDS = 4
    p.emit(addi(3, 0, 1))
    emit_sw_safe(p, 3, 0x014, 19) # DMA_REG_CTRL = 1 (Start)

    p.label("POLL_DMA_A")
    emit_lw_safe(p, 4, 0x004, 19) # DMA_REG_STATUS
    p.emit(andi(4, 4, 2))
    p.emit(NOP); p.emit(NOP); p.emit(NOP); p.emit(NOP)
    p.branch(beq, 4, 0, "POLL_DMA_A")

    # -----------------------------------------------------------------------
    # 6. DMA TRANSFER #2: Data SRAM (0x2000_0200) -> NPU BUF_B (0x4000_0200) [4 words]
    # -----------------------------------------------------------------------
    li32(p, 3, 0x20000200)
    emit_sw_safe(p, 3, 0x008, 19) # DMA_REG_SRC_ADDR
    li32(p, 3, 0x40000200)
    emit_sw_safe(p, 3, 0x00C, 19) # DMA_REG_DST_ADDR
    p.emit(addi(3, 0, 4))
    emit_sw_safe(p, 3, 0x010, 19) # DMA_REG_LEN_WORDS = 4
    p.emit(addi(3, 0, 1))
    emit_sw_safe(p, 3, 0x014, 19) # DMA_REG_CTRL = 1 (Start)

    p.label("POLL_DMA_B")
    emit_lw_safe(p, 4, 0x004, 19) # DMA_REG_STATUS
    p.emit(andi(4, 4, 2))
    p.emit(NOP); p.emit(NOP); p.emit(NOP); p.emit(NOP)
    p.branch(beq, 4, 0, "POLL_DMA_B")

    # -----------------------------------------------------------------------
    # 7. NPU SYSTOLIC ARRAY EXECUTION (K = 4)
    # -----------------------------------------------------------------------
    p.emit(addi(3, 0, 4))
    emit_sw_safe(p, 3, 0x010, 11) # NPU_REG_K_DIM = 4
    p.emit(addi(3, 0, 1))
    emit_sw_safe(p, 3, 0x00C, 11) # NPU_REG_CTRL = 1 (Start)

    p.label("POLL_NPU")
    emit_lw_safe(p, 4, 0x004, 11) # NPU_REG_STATUS
    p.emit(andi(4, 4, 2))
    p.emit(NOP); p.emit(NOP); p.emit(NOP); p.emit(NOP)
    p.branch(beq, 4, 0, "POLL_NPU")

    # Read NPU MAC cycles into x26
    emit_lw_safe(p, 26, 0x014, 11)

    # -----------------------------------------------------------------------
    # 8. DMA TRANSFER #3: NPU Output C (0x4000_0040) -> Data SRAM (0x2000_0300) [16 words]
    # -----------------------------------------------------------------------
    li32(p, 3, 0x40000040)
    emit_sw_safe(p, 3, 0x008, 19) # DMA_REG_SRC_ADDR = NPU Output Matrix C
    li32(p, 3, 0x20000300)
    emit_sw_safe(p, 3, 0x00C, 19) # DMA_REG_DST_ADDR = Data SRAM 0x2000_0300
    p.emit(addi(3, 0, 16))
    emit_sw_safe(p, 3, 0x010, 19) # DMA_REG_LEN_WORDS = 16
    p.emit(addi(3, 0, 1))
    emit_sw_safe(p, 3, 0x014, 19) # DMA_REG_CTRL = 1 (Start)

    p.label("POLL_DMA_C")
    emit_lw_safe(p, 4, 0x004, 19)
    p.emit(andi(4, 4, 2))
    p.emit(NOP); p.emit(NOP); p.emit(NOP); p.emit(NOP)
    p.branch(beq, 4, 0, "POLL_DMA_C")

    # Read DMA hardware cycle counter (0x4000_1018) -> x25 (16-word burst cycles)
    emit_lw_safe(p, 25, 0x018, 19)
    # Read DMA words completed (0x4000_101C) -> x28 (must be 16)
    emit_lw_safe(p, 28, 0x01C, 19)
    p.emit(addi(5, 0, 16))
    p.emit(NOP); p.emit(NOP)
    p.branch(bne, 28, 5, "FAIL_PATH")

    # -----------------------------------------------------------------------
    # 9. BENCHMARK: CPU SOFTWARE 16-WORD COPY LOOP (NPU 0x4000_0040 -> SRAM 0x2000_0500)
    # -----------------------------------------------------------------------
    p.emit(addi(20, 11, 0x040)) # src = 0x4000_0040
    p.emit(addi(21, 16, 0x500)) # dst = 0x2000_0500
    p.emit(addi(22, 0, 0))      # i = 0
    p.emit(addi(23, 0, 16))     # bound = 16
    p.emit(NOP); p.emit(NOP)

    p.emit(csrrs(24, 0xC00, 0)) # start mcycle
    p.emit(NOP); p.emit(NOP)

    p.label("CPU_CPY_LOOP")
    emit_lw_safe(p, 3, 0, 20)
    emit_sw_safe(p, 3, 0, 21)
    p.emit(addi(20, 20, 4))
    p.emit(addi(21, 21, 4))
    p.emit(addi(22, 22, 1))
    p.emit(NOP); p.emit(NOP)
    p.branch(blt, 22, 23, "CPU_CPY_LOOP")
    p.emit(NOP); p.emit(NOP)

    p.emit(csrrs(3, 0xC00, 0))  # end mcycle
    p.emit(NOP); p.emit(NOP)
    p.emit(sub(24, 3, 24))      # x24 = CPU 16-word copy cycles

    # -----------------------------------------------------------------------
    # 10. VERIFY ALL 16 CELLS IN DATA SRAM (0x2000_0300..0x2000_033C) WRITTEN BY DMA
    # -----------------------------------------------------------------------
    expected_c = [
        16,   7,  -1,   8,
        40,  19,   3,  24,
       -16,  -7,   1,  -8,
       160,  70, -10,  80
    ]
    for idx, exp_val in enumerate(expected_c):
        sram_off = 0x300 + idx * 4
        emit_lw_safe(p, 2, sram_off, 16)
        li32(p, 5, exp_val)
        p.emit(NOP); p.emit(NOP)
        p.branch(bne, 2, 5, "FAIL_PATH")

    # -----------------------------------------------------------------------
    # PASS PATH: Set LEDs = 0x5A, Stream DMA-Transferred Matrix C & Telemetry
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

    print_str("\r\nC (DMA -> SRAM) =\r\n")

    for r in range(4):
        print_str("[")
        for c in range(4):
            idx = r * 4 + c
            sram_off = 0x300 + idx * 4
            emit_lw_safe(p, 2, sram_off, 16)
            p.jump(3, "PRINT_CELL")
            p.emit(NOP); p.emit(NOP)
            if c < 3:
                print_str(",")
        print_str(" ]\r\n")

    print_str("CPU 16W Copy Cycles: ")
    p.emit(addi(2, 24, 0))
    p.jump(3, "PRINT_CELL")
    p.emit(NOP); p.emit(NOP)

    print_str("\r\nDMA 16W Burst Cycle: ")
    p.emit(addi(2, 25, 0))
    p.jump(3, "PRINT_CELL")
    p.emit(NOP); p.emit(NOP)

    print_str("\r\nDMA Words Completed: ")
    p.emit(addi(2, 28, 0))
    p.jump(3, "PRINT_CELL")
    p.emit(NOP); p.emit(NOP)

    print_str("\r\nNPU MAC Core Cycles: ")
    p.emit(addi(2, 26, 0))
    p.jump(3, "PRINT_CELL")
    p.emit(NOP); p.emit(NOP)

    print_str("\r\nPASS\r\n")

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
    print("  ASTRA-X1 MILESTONE 5B: AUTONOMOUS DMA + NPU GEMM PIPELINE")
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
    print(f"  ASTRA-X1 MILESTONE 5B - HARDWARE UPLOAD (EDGE ARTIX-7 100T)")
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
            print("\n>>> [SUCCESS] MILESTONE 5B AUTONOMOUS DMA + NPU PIPELINE PASSED! <<<")
            print(">>> 1. DMA Controller Slave 3 ID (0x444D4131 'DMA1') verified.")
            print(">>> 2. Multi-Master Wishbone Arbitration (CPU + DMA) verified.")
            print(">>> 3. Autonomous DMA Transfers (SRAM -> NPU & NPU -> SRAM) verified.")
            print(">>> 4. All 16/16 DMA-transferred output cells matched golden reference!")
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
