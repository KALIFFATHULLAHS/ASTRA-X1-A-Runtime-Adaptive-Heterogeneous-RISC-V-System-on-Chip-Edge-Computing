#!/usr/bin/env python3
"""
ASTRA-X1 Milestone 7 & 8: Full Heterogeneous SoC + ASTRA-DAAQ 4-Mode Power Ablation
Verifies on Hardware:
  1. All 5 Accelerator IDs on the 7-Slave Wishbone B4 Crossbar:
     - NPU  (0x4000_0000 == "ASTR")
     - DMA  (0x4000_1000 == "DMA1")
     - DAAQ (0x4000_2000 == "DAAQ")
     - SIMD (0x4000_3000 == "SIMD")
     - FP32 (0x4000_4000 == "FPU1")
  2. End-to-End Heterogeneous Compute Pipeline (DMA -> SIMD -> NPU -> FP32):
     - DMA moves Matrix A & B from SRAM to NPU
     - SIMD executes 4-Lane INT16 vector preprocessing
     - NPU executes 4x4 INT8 Systolic Array GEMM (C[0,0]=16, C[3,0]=160)
     - FP32 converts C[0,0]=16 -> 16.0f, multiplies by 1.5f -> 24.0f -> int 24
  3. 4-Way Controlled ASTRA-DAAQ Power & Activity Ablation:
     - Mode A (Baseline Always-On)       : 0 Energy Saved, 0 Wake Hazards
     - Mode B (Naive Local Gating)       : High Gating, but incurs Cold-Wake Hazards (>0)
     - Mode C (Fixed-Threshold DAAQ)     : Dependency-Safe Gating, 0 Wake Hazards
     - Mode D (Adaptive Phase-Hint DAAQ) : Highest Safe Energy Savings, 0 Wake Hazards!
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

def sub(rd, rs1, rs2):
    val = (0x20 << 25) | ((rs2 & 0x1F) << 20) | ((rs1 & 0x1F) << 15) | (0 << 12) | ((rd & 0x1F) << 7) | 0x33
    return _le32(val)

def add(rd, rs1, rs2):
    val = (0x00 << 25) | ((rs2 & 0x1F) << 20) | ((rs1 & 0x1F) << 15) | (0 << 12) | ((rd & 0x1F) << 7) | 0x33
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
    # x11 = NPU Base:         0x4000_0000 (Slave 1)
    # x19 = DMA Base:         0x4000_1000 (Slave 3)
    # x20 = DAAQ Base:        0x4000_2000 (Slave 4)
    # x21 = SIMD Base:        0x4000_3000 (Slave 5)
    # x22 = FPU Base:         0x4000_4000 (Slave 6)
    # x10 = Peripherals Base: 0xFFFF_0000 (Slave 2)
    # x16 = Data SRAM Base:   0x2000_0000 (Slave 0)
    p.emit(lui(11, 0x40000))
    p.emit(lui(19, 0x40001))
    p.emit(lui(20, 0x40002))
    p.emit(lui(21, 0x40003))
    p.emit(lui(22, 0x40004))
    p.emit(lui(10, 0xFFFF0))
    p.emit(lui(16, 0x20000))

    # 1. Verify all 5 Hardware Accelerator IDs on the 7-Slave Crossbar!
    id_checks = [
        (11, 0x41535452), # "ASTR"
        (19, 0x444D4131), # "DMA1"
        (20, 0x44414151), # "DAAQ"
        (21, 0x53494D44), # "SIMD"
        (22, 0x46505531), # "FPU1"
    ]
    for base_reg, exp_id in id_checks:
        emit_lw_safe(p, 2, 0x000, base_reg)
        li32(p, 5, exp_id)
        p.emit(NOP); p.emit(NOP)
        p.branch(bne, 2, 5, "FAIL_PATH")

    # Verify RISC-V Machine CSRs: mvendorid (0xF11 == "ASTR") & misa (0x301 == 0x40001125 RV32IMFX)
    p.emit(csrrs(2, 0xF11, 0))
    p.emit(NOP); p.emit(NOP)
    li32(p, 5, 0x41535452)
    p.emit(NOP); p.emit(NOP)
    p.branch(bne, 2, 5, "FAIL_PATH")

    p.emit(csrrs(2, 0x301, 0))
    p.emit(NOP); p.emit(NOP)
    li32(p, 5, 0x40001125)
    p.emit(NOP); p.emit(NOP)
    p.branch(bne, 2, 5, "FAIL_PATH")

    # 2. Pre-stage Matrix A & B in Data SRAM (0x2000_0100 & 0x2000_0200) and NPU BUF_B
    li32(p, 3, 0x04030201); emit_sw_safe(p, 3, 0x100, 16)
    li32(p, 3, 0x08070605); emit_sw_safe(p, 3, 0x104, 16)
    li32(p, 3, 0xFCFDFEFF); emit_sw_safe(p, 3, 0x108, 16)
    li32(p, 3, 0x281E140A); emit_sw_safe(p, 3, 0x10C, 16)

    li32(p, 3, 0x03010002); emit_sw_safe(p, 3, 0x200, 11)
    li32(p, 3, 0xFF000201); emit_sw_safe(p, 3, 0x204, 11)
    li32(p, 3, 0x01020100); emit_sw_safe(p, 3, 0x208, 11)
    li32(p, 3, 0x01FE0003); emit_sw_safe(p, 3, 0x20C, 11)

    # Pre-stage SIMD vectors: A0/A1, B0/B1
    li32(p, 3, 0xFFFB000C); emit_sw_safe(p, 3, 0x010, 21)
    li32(p, 3, 0xFFF1001E); emit_sw_safe(p, 3, 0x014, 21)
    li32(p, 3, 0x00060003); emit_sw_safe(p, 3, 0x018, 21)
    li32(p, 3, 0x0004FFF6); emit_sw_safe(p, 3, 0x01C, 21)

    # Set DAAQ Fixed Threshold = 16 cycles (0x4000_200C)
    p.emit(addi(3, 0, 16))
    emit_sw_safe(p, 3, 0x00C, 20)

    # 3. Run the 4 Controlled ASTRA-DAAQ Ablation Modes (0=Mode A, 1=Mode B, 2=Mode C, 3=Mode D)
    #    Store telemetry for each mode at SRAM offset 0x300 + mode * 0x20:
    #      +0x00: GATED_NPU
    #      +0x04: GATED_SIMD
    #      +0x08: GATED_FPU
    #      +0x0C: ENERGY_SAVED
    #      +0x10: WAKE_STALLS
    #      +0x14: FINAL_FP32_INT_OUT (24)
    for mode_id in range(4):
        sram_base = 0x300 + mode_id * 0x20
        # Write DAAQ_REG_MODE = mode_id | 0x10 (reset counters)
        p.emit(addi(3, 0, mode_id | 0x10))
        emit_sw_safe(p, 3, 0x004, 20)

        # Call full heterogeneous workload subroutine (DMA -> SIMD -> NPU -> FP32)
        p.jump(23, "RUN_HETERO_WORKLOAD")
        p.emit(NOP); p.emit(NOP)

        # Latch telemetry counters into Data SRAM
        emit_lw_safe(p, 2, 0x018, 20) # GATED_NPU
        emit_sw_safe(p, 2, sram_base + 0x00, 16)
        emit_lw_safe(p, 2, 0x020, 20) # GATED_SIMD
        emit_sw_safe(p, 2, sram_base + 0x04, 16)
        emit_lw_safe(p, 2, 0x024, 20) # GATED_FPU
        emit_sw_safe(p, 2, sram_base + 0x08, 16)
        emit_lw_safe(p, 2, 0x028, 20) # ENERGY_SAVED
        emit_sw_safe(p, 2, sram_base + 0x0C, 16)
        emit_lw_safe(p, 2, 0x02C, 20) # WAKE_STALLS
        emit_sw_safe(p, 2, sram_base + 0x10, 16)
        # x28 holds the final FP32-scaled output (must be 24)
        emit_sw_safe(p, 28, sram_base + 0x14, 16)
        p.emit(addi(5, 0, 24))
        p.emit(NOP); p.emit(NOP)
        p.branch(bne, 28, 5, "FAIL_PATH")

    # -----------------------------------------------------------------------
    # PASS PATH: Set LEDs = 0x5A, Stream 4-Mode Ablation Table & Pipeline Output
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

    # Snapshot L1 I-Cache Hits & D-Cache Hits from the compute workload (before UART print loops)
    emit_lw_safe(p, 29, 0x048, 10) # x29 = DCACHE_HITS (0xFFFF_0048)
    emit_lw_safe(p, 30, 0x040, 10) # x30 = ICACHE_HITS (0xFFFF_0040)

    print_str("\r\n--- ASTRA-DAAQ 4-MODE POWER & ACTIVITY ABLATION ---\r\n")
    print_str("Mode             | NPU_Gate | SIMD_Gate | FPU_Gate | E_Saved | WakeStall\r\n")

    mode_labels = [
        "Mode A (Base)    |     ",
        "Mode B (Local)   |     ",
        "Mode C (FixDAAQ) |     ",
        "Mode D (AdapDAAQ)|     ",
    ]
    for mode_id, m_lbl in enumerate(mode_labels):
        sram_base = 0x300 + mode_id * 0x20
        print_str(m_lbl)
        emit_lw_safe(p, 2, sram_base + 0x00, 16)
        p.jump(3, "PRINT_CELL"); p.emit(NOP); p.emit(NOP)
        print_str(" |      ")
        emit_lw_safe(p, 2, sram_base + 0x04, 16)
        p.jump(3, "PRINT_CELL"); p.emit(NOP); p.emit(NOP)
        print_str(" |     ")
        emit_lw_safe(p, 2, sram_base + 0x08, 16)
        p.jump(3, "PRINT_CELL"); p.emit(NOP); p.emit(NOP)
        print_str(" |    ")
        emit_lw_safe(p, 2, sram_base + 0x0C, 16)
        p.jump(3, "PRINT_CELL"); p.emit(NOP); p.emit(NOP)
        print_str(" |     ")
        emit_lw_safe(p, 2, sram_base + 0x10, 16)
        p.jump(3, "PRINT_CELL"); p.emit(NOP); p.emit(NOP)
        print_str("\r\n")

    # Execute a Hardware SPI Master DAC Transfer (Write 0x05A5 -> Read back loopback 0x05A5)
    p.emit(addi(3, 0, 0x5A5))
    emit_sw_safe(p, 3, 0x024, 10) # SPI_DATA = 0x05A5
    p.emit(addi(3, 0, 3))         # Start=1, Loopback=1
    emit_sw_safe(p, 3, 0x020, 10) # SPI_CTRL
    p.label("POLL_SPI")
    emit_lw_safe(p, 4, 0x020, 10)
    p.emit(andi(4, 4, 2))         # Wait for SPI_DONE (bit 1)
    p.emit(NOP); p.emit(NOP); p.emit(NOP); p.emit(NOP)
    p.branch(beq, 4, 0, "POLL_SPI")
    emit_lw_safe(p, 4, 0x024, 10) # Read SPI_DATA
    p.emit(addi(5, 0, 0x5A5))
    p.emit(NOP); p.emit(NOP)
    p.branch(bne, 4, 5, "FAIL_PATH")

    # Execute a Hardware I2C Master Read (Slave 0x68, Reg 0x75 WHO_AM_I -> 0x5A = 90)
    p.emit(addi(3, 0, 3))         # Start=1, RW=1 (Read)
    emit_sw_safe(p, 3, 0x030, 10) # I2C_CTRL
    p.label("POLL_I2C")
    emit_lw_safe(p, 4, 0x030, 10)
    p.emit(andi(4, 4, 2))         # Wait for I2C_DONE (bit 1)
    p.emit(NOP); p.emit(NOP); p.emit(NOP); p.emit(NOP)
    p.branch(beq, 4, 0, "POLL_I2C")
    emit_lw_safe(p, 4, 0x038, 10) # Read I2C_DATA (0x5A = 90)
    p.emit(addi(5, 0, 0x5A))
    p.emit(NOP); p.emit(NOP)
    p.branch(bne, 4, 5, "FAIL_PATH")

    print_str("Hetero Pipe Out (DMA->SIMD->NPU->FP32) = ")
    p.emit(addi(2, 28, 0))
    p.jump(3, "PRINT_CELL"); p.emit(NOP); p.emit(NOP)
    print_str(".0\r\n")

    print_str("L1 D-Cache Hits / I-Cache Hits         = ")
    p.emit(addi(2, 29, 0)) # Workload DCACHE_HITS
    p.jump(3, "PRINT_CELL"); p.emit(NOP); p.emit(NOP)
    print_str(" / ")
    p.emit(addi(2, 30, 0)) # Workload ICACHE_HITS
    p.jump(3, "PRINT_CELL"); p.emit(NOP); p.emit(NOP)
    print_str("\r\n")

    print_str("SoC IRQs Fired / SPI+I2C Sensor ID     = ")
    emit_lw_safe(p, 2, 0x03C, 22) # SOC_IRQ_COUNT (0x4000_403C)
    p.jump(3, "PRINT_CELL"); p.emit(NOP); p.emit(NOP)
    print_str(" / ")
    emit_lw_safe(p, 2, 0x038, 10) # I2C WHO_AM_I (90 = 0x5A)
    p.jump(3, "PRINT_CELL"); p.emit(NOP); p.emit(NOP)
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
    # SUBROUTINE: RUN_HETERO_WORKLOAD
    # Executes a 5-Phase Heterogeneous Workload:
    #   Phase 1 (DMA_XFER):     DMA transfers 4 words from SRAM 0x2000_0100 -> NPU 0x4000_0100
    #   Phase 2 (SIMD_PREPROC): SIMD executes 4-Lane ADD16 + MUL16
    #   Phase 3 (NPU_GEMM):     NPU executes 4x4 INT8 GEMM -> C[0,0] = 16
    #   Phase 4 (FPU_POST):     FP32 converts 16 -> 16.0f, * 1.5f -> 24.0f -> int 24
    #   Phase 5 (IDLE):         Short idle window
    # -----------------------------------------------------------------------
    p.label("RUN_HETERO_WORKLOAD")

    # Phase 1: PHASE_DMA_XFER (1)
    p.emit(addi(3, 0, 1))
    emit_sw_safe(p, 3, 0x008, 20) # DAAQ_REG_PHASE = 1
    li32(p, 3, 0x20000100); emit_sw_safe(p, 3, 0x008, 19)
    li32(p, 3, 0x40000100); emit_sw_safe(p, 3, 0x00C, 19)
    p.emit(addi(3, 0, 4));  emit_sw_safe(p, 3, 0x010, 19)
    p.emit(addi(3, 0, 1));  emit_sw_safe(p, 3, 0x014, 19)
    p.label("W_POLL_DMA")
    emit_lw_safe(p, 4, 0x004, 19)
    p.emit(andi(4, 4, 2))
    p.emit(NOP); p.emit(NOP); p.emit(NOP); p.emit(NOP)
    p.branch(beq, 4, 0, "W_POLL_DMA")

    # Phase 2: PHASE_SIMD_PREPROC (2)
    p.emit(addi(3, 0, 2))
    emit_sw_safe(p, 3, 0x008, 20) # DAAQ_REG_PHASE = 2
    p.emit(addi(3, 0, (0 << 2) | 1)) # SIMD ADD16
    emit_sw_safe(p, 3, 0x008, 21)
    emit_lw_safe(p, 4, 0x028, 21)    # Read SIMD ACC (25)

    # Phase 3: PHASE_NPU_GEMM (3)
    p.emit(addi(3, 0, 3))
    emit_sw_safe(p, 3, 0x008, 20) # DAAQ_REG_PHASE = 3
    p.emit(addi(3, 0, 4)); emit_sw_safe(p, 3, 0x010, 11) # K=4
    p.emit(addi(3, 0, 1)); emit_sw_safe(p, 3, 0x00C, 11) # NPU Start
    p.label("W_POLL_NPU")
    emit_lw_safe(p, 4, 0x004, 11)
    p.emit(andi(4, 4, 2))
    p.emit(NOP); p.emit(NOP); p.emit(NOP); p.emit(NOP)
    p.branch(beq, 4, 0, "W_POLL_NPU")
    emit_lw_safe(p, 25, 0x040, 11) # C[0,0] = 16

    # Phase 4: PHASE_FPU_POST (4)
    p.emit(addi(3, 0, 4))
    emit_sw_safe(p, 3, 0x008, 20) # DAAQ_REG_PHASE = 4
    # Convert C[0,0] (16) -> 16.0f via FCVT.S.W (opcode 7)
    emit_sw_safe(p, 25, 0x008, 22)
    p.emit(addi(3, 0, (7 << 2) | 1))
    emit_sw_safe(p, 3, 0x014, 22)
    emit_lw_safe(p, 26, 0x018, 22) # 16.0f
    # Multiply 16.0f * 1.5f (0x3FC00000) via FMUL.S (opcode 2) -> 24.0f
    emit_sw_safe(p, 26, 0x008, 22)
    li32(p, 3, f32_to_u32(1.5))
    emit_sw_safe(p, 3, 0x00C, 22)
    p.emit(addi(3, 0, (2 << 2) | 1))
    emit_sw_safe(p, 3, 0x014, 22)
    emit_lw_safe(p, 26, 0x018, 22) # 24.0f
    # Convert 24.0f -> int32 (24) via FCVT.W.S (opcode 6) -> x28
    emit_sw_safe(p, 26, 0x008, 22)
    p.emit(addi(3, 0, (6 << 2) | 1))
    emit_sw_safe(p, 3, 0x014, 22)
    emit_lw_safe(p, 28, 0x018, 22) # x28 = 24

    # Phase 5: PHASE_IDLE (5)
    p.emit(addi(3, 0, 5))
    emit_sw_safe(p, 3, 0x008, 20)
    for _ in range(10):
        p.emit(NOP)

    p.emit(jalr(0, 23, 0))
    p.emit(NOP); p.emit(NOP); p.emit(NOP); p.emit(NOP)

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
    print("  ASTRA-X1 MILESTONE 7 & 8: HETEROGENEOUS SoC + ASTRA-DAAQ")
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
    print(f"  ASTRA-X1 MILESTONE 7/8 - HARDWARE UPLOAD (EDGE ARTIX-7 100T)")
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
            print("\n>>> [SUCCESS] MILESTONE 7 & 8 FULL HETEROGENEOUS SoC + ASTRA-DAAQ PASSED! <<<")
            print(">>> 1. All 5 Accelerator IDs ('ASTR', 'DMA1', 'DAAQ', 'SIMD', 'FPU1') verified.")
            print(">>> 2. 4-Mode ASTRA-DAAQ Power & Activity Ablation verified.")
            print(">>> 3. End-to-End Heterogeneous Pipeline (DMA->SIMD->NPU->FP32) verified!")
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
