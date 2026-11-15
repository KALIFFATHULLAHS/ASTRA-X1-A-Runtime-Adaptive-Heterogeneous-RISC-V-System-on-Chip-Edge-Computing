#!/usr/bin/env python3
"""
ASTRA-X1 Developer SDK & High-Level Hardware Compiler (astra_sdk.py)
====================================================================
Allows any developer (just like Arduino IDE or ESP-IDF) to write short,
clean, high-level Python/C-style programs targeting the ASTRA-X1 Heterogeneous
RISC-V SoC, compile them to hazard-safe RV32IMFX machine code, and upload +
run them on the physical EDGE Artix-7 100T FPGA board over USB-UART (COM3).
"""

import os
import sys
import time
import struct

COM_PORT  = "COM3"
BAUD_RATE = 115200

def u32(val):
    return val & 0xFFFFFFFF

def f32_to_u32(f):
    return struct.unpack("<I", struct.pack("<f", float(f)))[0]

def pack_i8x4(b0, b1, b2, b3):
    return ((b3 & 0xFF) << 24) | ((b2 & 0xFF) << 16) | ((b1 & 0xFF) << 8) | (b0 & 0xFF)

def pack_i16x2(lo16, hi16):
    return ((hi16 & 0xFFFF) << 16) | (lo16 & 0xFFFF)

def encode_r(opcode, funct3, funct7, rd, rs1, rs2):
    return u32((funct7 << 25) | (rs2 << 20) | (rs1 << 15) | (funct3 << 12) | (rd << 7) | opcode)

def encode_i(opcode, funct3, rd, rs1, imm12):
    return u32(((imm12 & 0xFFF) << 20) | (rs1 << 15) | (funct3 << 12) | (rd << 7) | opcode)

def encode_s(opcode, funct3, rs1, rs2, imm12):
    imm12 &= 0xFFF
    imm_hi = (imm12 >> 5) & 0x7F
    imm_lo = imm12 & 0x1F
    return u32((imm_hi << 25) | (rs2 << 20) | (rs1 << 15) | (funct3 << 12) | (imm_lo << 7) | opcode)

def encode_b(opcode, funct3, rs1, rs2, offset_bytes):
    imm = offset_bytes & 0x1FFF
    b12   = (imm >> 12) & 1
    b11   = (imm >> 11) & 1
    b10_5 = (imm >> 5)  & 0x3F
    b4_1  = (imm >> 1)  & 0xF
    return u32((b12 << 31) | (b10_5 << 25) | (rs2 << 20) | (rs1 << 15) | (funct3 << 12) | (b4_1 << 8) | (b11 << 7) | opcode)

def encode_u(opcode, rd, imm20):
    return u32(((imm20 & 0xFFFFF) << 12) | (rd << 7) | opcode)

def encode_j(opcode, rd, offset_bytes):
    imm = offset_bytes & 0x1FFFFF
    b20    = (imm >> 20) & 1
    b19_12 = (imm >> 12) & 0xFF
    b11    = (imm >> 11) & 1
    b10_1  = (imm >> 1)  & 0x3FF
    return u32((b20 << 31) | (b10_1 << 21) | (b11 << 20) | (b19_12 << 12) | (rd << 7) | opcode)

def lui(rd, imm20):          return encode_u(0x37, rd, imm20)
def addi(rd, rs1, imm12):    return encode_i(0x13, 0x0, rd, rs1, imm12)
def andi(rd, rs1, imm12):    return encode_i(0x13, 0x7, rd, rs1, imm12)
def add(rd, rs1, rs2):       return encode_r(0x33, 0x0, 0x00, rd, rs1, rs2)
def sub(rd, rs1, rs2):       return encode_r(0x33, 0x0, 0x20, rd, rs1, rs2)
def srai(rd, rs1, shamt):    return encode_i(0x13, 0x5, rd, rs1, 0x400 | (shamt & 0x1F))
def slli(rd, rs1, shamt):    return encode_i(0x13, 0x1, rd, rs1, shamt & 0x1F)
def lw(rd, imm12, rs1):      return encode_i(0x03, 0x2, rd, rs1, imm12)
def sw(rs2, imm12, rs1):     return encode_s(0x23, 0x2, rs1, rs2, imm12)
def beq(rs1, rs2, off):      return encode_b(0x63, 0x0, rs1, rs2, off)
def bne(rs1, rs2, off):      return encode_b(0x63, 0x1, rs1, rs2, off)
def blt(rs1, rs2, off):      return encode_b(0x63, 0x4, rs1, rs2, off)
def bge(rs1, rs2, off):      return encode_b(0x63, 0x5, rs1, rs2, off)
def jal(rd, off):            return encode_j(0x6F, rd, off)
def jalr(rd, rs1, imm12=0):  return encode_i(0x67, 0x0, rd, rs1, imm12)
def jal_self():              return jal(0, 0)

NOP = addi(0, 0, 0)
WFI = 0x10500073


class AstraApp:
    """
    High-Level Developer SDK for the ASTRA-X1 Heterogeneous RISC-V SoC.
    Works like Arduino / ESP-IDF: write high-level hardware calls, then call
    app.build_and_run() to compile, upload over USB-UART, and view live output!
    """
    SIMD_OPS = {
        "ADD16": 0, "SUB16": 1, "MUL16": 2, "MAC16": 3,
        "ABS16": 4, "RELU16": 5, "MAX16": 6, "MIN16": 7
    }
    FPU_OPS = {
        "FADD.S": 0, "FSUB.S": 1, "FMUL.S": 2, "FMADD.S": 3,
        "FMIN.S": 4, "FMAX.S": 5, "FCVT.W.S": 6, "FCVT.S.W": 7,
        "FABS.S": 8, "FNEG.S": 9
    }
    DAAQ_MODES = {
        "BASELINE": 0, "LOCAL": 1, "FIXED_DAAQ": 2, "ADAPTIVE": 3
    }

    def __init__(self, name="ASTRA-X1 User Application", uart_delay=1500):
        self.name = name
        self.uart_delay = uart_delay
        self.items = []
        self.labels = {}
        self._lbl_cnt = 0

        # Initialize pipeline & MMIO base pointers
        for _ in range(8):
            self.emit(NOP)
        # x10=Periph(0xFFFF0000), x11=NPU(0x40000000), x16=SRAM(0x20000000),
        # x19=DMA(0x40001000), x20=DAAQ(0x40002000), x21=SIMD(0x40003000), x22=FPU(0x40004000)
        self.emit(lui(10, 0xFFFF0))
        self.emit(lui(11, 0x40000))
        self.emit(lui(16, 0x20000))
        self.emit(lui(19, 0x40001))
        self.emit(lui(20, 0x40002))
        self.emit(lui(21, 0x40003))
        self.emit(lui(22, 0x40004))
        self.emit(NOP); self.emit(NOP); self.emit(NOP); self.emit(NOP)

    def _new_label(self, prefix="L"):
        self._lbl_cnt += 1
        return f"{prefix}_{self._lbl_cnt}"

    def emit(self, inst):
        self.items.append(inst)

    def label(self, name):
        self.labels[name] = len(self.items) * 4

    def branch(self, fn, rs1, rs2, target_label):
        self.items.append(("B", fn, rs1, rs2, target_label))

    def jump(self, rd, target_label):
        self.items.append(("J", rd, target_label))

    def _sw_safe(self, rs2, offset, rs1):
        for _ in range(5): self.emit(NOP)
        self.emit(sw(rs2, offset, rs1))
        for _ in range(5): self.emit(NOP)

    def _lw_safe(self, rd, offset, rs1):
        for _ in range(5): self.emit(NOP)
        self.emit(lw(rd, offset, rs1))
        for _ in range(5): self.emit(NOP)

    def _li32(self, rd, val):
        val = u32(val)
        upper = (val + 0x800) >> 12
        lower = val & 0xFFF
        if lower >= 0x800:
            lower -= 0x1000
        self.emit(lui(rd, upper & 0xFFFFF))
        self.emit(NOP); self.emit(NOP)
        if lower != 0:
            self.emit(addi(rd, rd, lower & 0xFFF))
            self.emit(NOP); self.emit(NOP)

    # ------------------------------------------------------------------
    # Developer-Facing High-Level SDK Methods
    # ------------------------------------------------------------------
    def print(self, text):
        """Print a string to the USB-UART Serial Monitor (COM3 @ 115200 baud)."""
        for ch in text:
            self.emit(addi(6, 0, ord(ch)))
            self.jump(1, "UART_SEND")
            self.emit(NOP); self.emit(NOP)

    def println(self, text=""):
        """Print a line with CRLF to the USB-UART Serial Monitor."""
        self.print(text + "\r\n")

    def led_write(self, val):
        """Write an 8-bit or 16-bit pattern to the physical onboard LEDs."""
        self._li32(3, val & 0xFFFF)
        self._sw_safe(3, 0x004, 10)

    def seven_seg_hex(self, hex16):
        """Display a 4-digit hexadecimal value on the physical 7-Segment Display."""
        self._li32(3, hex16 & 0xFFFF)
        self._sw_safe(3, 0x018, 10)

    def daaq_set_mode(self, mode="ADAPTIVE"):
        """Set ASTRA-DAAQ Runtime Power Controller Mode ('BASELINE','LOCAL','FIXED_DAAQ','ADAPTIVE')."""
        m = self.DAAQ_MODES.get(mode.upper(), 3)
        self.emit(addi(3, 0, m))
        self._sw_safe(3, 0x004, 20)

    def npu_matmul_4x4(self, mat_a, mat_b, use_dma=True, print_label="NPU 4x4 Output C ="):
        """
        Offload a 4x4 Signed INT8 Matrix Multiplication (C = A * B) to the
        ASTRA-X1 16-PE Systolic Array NPU (optionally using Autonomous DMA).
        """
        for r in range(4):
            w_a = pack_i8x4(mat_a[r][0], mat_a[r][1], mat_a[r][2], mat_a[r][3])
            self._li32(3, w_a)
            if use_dma:
                self._sw_safe(3, 0x100 + r * 4, 16)
            else:
                self._sw_safe(3, 0x100 + r * 4, 11)

        for k in range(4):
            w_b = pack_i8x4(mat_b[k][0], mat_b[k][1], mat_b[k][2], mat_b[k][3])
            self._li32(3, w_b)
            if use_dma:
                self._sw_safe(3, 0x120 + k * 4, 16)
            else:
                self._sw_safe(3, 0x200 + k * 4, 11)

        if use_dma:
            # DMA Burst A: SRAM 0x2000_0100 -> NPU 0x4000_0100
            self._li32(3, 0x20000100); self._sw_safe(3, 0x008, 19)
            self._li32(3, 0x40000100); self._sw_safe(3, 0x00C, 19)
            self.emit(addi(3, 0, 4));  self._sw_safe(3, 0x010, 19)
            self.emit(addi(3, 0, 1));  self._sw_safe(3, 0x014, 19)
            lbl_dma_a = self._new_label("DMA_A")
            self.label(lbl_dma_a)
            self._lw_safe(4, 0x004, 19)
            self.emit(andi(4, 4, 2))
            self.emit(NOP); self.emit(NOP); self.emit(NOP); self.emit(NOP)
            self.branch(beq, 4, 0, lbl_dma_a)

            # DMA Burst B: SRAM 0x2000_0120 -> NPU 0x4000_0200
            self._li32(3, 0x20000120); self._sw_safe(3, 0x008, 19)
            self._li32(3, 0x40000200); self._sw_safe(3, 0x00C, 19)
            self.emit(addi(3, 0, 4));  self._sw_safe(3, 0x010, 19)
            self.emit(addi(3, 0, 1));  self._sw_safe(3, 0x014, 19)
            lbl_dma_b = self._new_label("DMA_B")
            self.label(lbl_dma_b)
            self._lw_safe(4, 0x004, 19)
            self.emit(andi(4, 4, 2))
            self.emit(NOP); self.emit(NOP); self.emit(NOP); self.emit(NOP)
            self.branch(beq, 4, 0, lbl_dma_b)

        # Trigger Systolic NPU (K=4)
        self.emit(addi(3, 0, 4)); self._sw_safe(3, 0x010, 11)
        self.emit(addi(3, 0, 1)); self._sw_safe(3, 0x00C, 11)
        lbl_npu = self._new_label("NPU_WAIT")
        self.label(lbl_npu)
        self._lw_safe(4, 0x004, 11)
        self.emit(andi(4, 4, 2))
        self.emit(NOP); self.emit(NOP); self.emit(NOP); self.emit(NOP)
        self.branch(beq, 4, 0, lbl_npu)

        if print_label:
            self.println(print_label)
            for r in range(4):
                self.print("[ ")
                for c in range(4):
                    self._lw_safe(2, 0x040 + (r * 4 + c) * 4, 11)
                    self.jump(3, "PRINT_CELL"); self.emit(NOP); self.emit(NOP)
                    if c < 3:
                        self.print(", ")
                self.println(" ]")

    def simd_vector_op(self, op="ADD16", vec_a=(10, 20, 30, 40), vec_b=(1, 2, 3, 4), print_label=None):
        """Execute a 4-Lane 16-bit Signed SIMD operation on Slave 5 (0x4000_3000)."""
        opcode = self.SIMD_OPS.get(op.upper(), 0)
        self._li32(3, pack_i16x2(vec_a[0], vec_a[1])); self._sw_safe(3, 0x010, 21)
        self._li32(3, pack_i16x2(vec_a[2], vec_a[3])); self._sw_safe(3, 0x014, 21)
        self._li32(3, pack_i16x2(vec_b[0], vec_b[1])); self._sw_safe(3, 0x018, 21)
        self._li32(3, pack_i16x2(vec_b[2], vec_b[3])); self._sw_safe(3, 0x01C, 21)
        self.emit(addi(3, 0, (opcode << 2) | 3));      self._sw_safe(3, 0x008, 21)

        if print_label:
            self.print(f"{print_label} [ ")
            self._lw_safe(24, 0x020, 21)
            self._lw_safe(25, 0x024, 21)
            # Lane 0
            self.emit(slli(2, 24, 16)); self.emit(NOP); self.emit(NOP)
            self.emit(srai(2, 2, 16));  self.emit(NOP); self.emit(NOP)
            self.jump(3, "PRINT_CELL"); self.emit(NOP); self.emit(NOP); self.print(", ")
            # Lane 1
            self.emit(srai(2, 24, 16)); self.emit(NOP); self.emit(NOP)
            self.jump(3, "PRINT_CELL"); self.emit(NOP); self.emit(NOP); self.print(", ")
            # Lane 2
            self.emit(slli(2, 25, 16)); self.emit(NOP); self.emit(NOP)
            self.emit(srai(2, 2, 16));  self.emit(NOP); self.emit(NOP)
            self.jump(3, "PRINT_CELL"); self.emit(NOP); self.emit(NOP); self.print(", ")
            # Lane 3
            self.emit(srai(2, 25, 16)); self.emit(NOP); self.emit(NOP)
            self.jump(3, "PRINT_CELL"); self.emit(NOP); self.emit(NOP)
            self.print(" ] ACC=")
            self._lw_safe(2, 0x028, 21)
            self.jump(3, "PRINT_CELL"); self.emit(NOP); self.emit(NOP)
            self.println()

    def fpu_compute(self, op="FMADD.S", a=12.5, b=4.0, c=0.0, print_label=None):
        """Execute an IEEE-754 Single-Precision FP32 operation on Slave 6 (0x4000_4000)."""
        # Reset DAAQ window counter (bit 4 = 0x10 clears counters, mode = 3)
        self.emit(addi(3, 0, 3 | 0x10)); self._sw_safe(3, 0x004, 20)
        opcode = self.FPU_OPS.get(op.upper(), 0)
        self._li32(3, f32_to_u32(a)); self._sw_safe(3, 0x008, 22)
        self._li32(3, f32_to_u32(b)); self._sw_safe(3, 0x00C, 22)
        self._li32(3, f32_to_u32(c)); self._sw_safe(3, 0x010, 22)
        self.emit(addi(3, 0, (opcode << 2) | 1)); self._sw_safe(3, 0x014, 22)
        self._lw_safe(26, 0x018, 22) # IEEE-754 result
        # Convert result to integer via FCVT.W.S (opcode 6) for printing
        self._sw_safe(26, 0x008, 22)
        self.emit(addi(3, 0, (6 << 2) | 1)); self._sw_safe(3, 0x014, 22)
        self._lw_safe(2, 0x018, 22)
        self._lw_safe(28, 0x028, 20) # Latch compute-window DAAQ energy saved
        if print_label:
            self.print(print_label)
            self.jump(3, "PRINT_CELL"); self.emit(NOP); self.emit(NOP)
            self.println(".0")

    def print_board_telemetry(self):
        """Read and print live physical Slide Switches sw[7:0], L1 Cache Hits, and DAAQ Energy Saved."""
        for off in (0x100, 0x104, 0x108, 0x10C, 0x100, 0x104, 0x108, 0x10C):
            self._lw_safe(4, off, 16)
        self.print("Live Switches sw[7:0] = ")
        self._lw_safe(2, 0x010, 10)
        self.emit(andi(2, 2, 0xFF))
        self.jump(3, "PRINT_CELL"); self.emit(NOP); self.emit(NOP)
        self.print(" | L1 D$ Hits = ")
        self._lw_safe(2, 0x048, 10)
        self.jump(3, "PRINT_CELL"); self.emit(NOP); self.emit(NOP)
        self.print(" | DAAQ Energy Saved = ")
        self.emit(addi(2, 28, 0))
        self.jump(3, "PRINT_CELL"); self.emit(NOP); self.emit(NOP)
        self.println()

    def compile(self):
        """Finalize program with PASS status, WFI halt, and runtime print/UART libraries."""
        self.led_write(0x5A)
        self.println("PASS")
        self.emit(WFI)
        self.emit(jal_self())

        # Emit PRINT_CELL & UART_SEND runtime routines
        self.label("PRINT_CELL")
        lbl = "pcell"
        self.emit(addi(12, 0, 0))
        self.branch(bge, 2, 0, f"{lbl}_pos")
        self.emit(NOP); self.emit(NOP)
        self.emit(sub(2, 0, 2))
        self.emit(addi(12, 0, 1))
        self.label(f"{lbl}_pos")

        self.emit(addi(27, 0, 0))
        self.emit(addi(13, 0, 0))
        self.emit(addi(14, 0, 0))
        self.emit(addi(5, 0, 1000))
        self.emit(NOP); self.emit(NOP)

        self.label(f"{lbl}_k_loop")
        self.branch(blt, 2, 5, f"{lbl}_k_done")
        self.emit(NOP); self.emit(NOP)
        self.emit(sub(2, 2, 5))
        self.emit(addi(27, 27, 1))
        self.branch(beq, 0, 0, f"{lbl}_k_loop")
        self.emit(NOP); self.emit(NOP)
        self.label(f"{lbl}_k_done")

        self.emit(addi(5, 0, 100))
        self.emit(NOP); self.emit(NOP)
        self.label(f"{lbl}_h_loop")
        self.branch(blt, 2, 5, f"{lbl}_h_done")
        self.emit(NOP); self.emit(NOP)
        self.emit(sub(2, 2, 5))
        self.emit(addi(13, 13, 1))
        self.branch(beq, 0, 0, f"{lbl}_h_loop")
        self.emit(NOP); self.emit(NOP)
        self.label(f"{lbl}_h_done")

        self.emit(addi(5, 0, 10))
        self.emit(NOP); self.emit(NOP)
        self.label(f"{lbl}_t_loop")
        self.branch(blt, 2, 5, f"{lbl}_t_done")
        self.emit(NOP); self.emit(NOP)
        self.emit(sub(2, 2, 5))
        self.emit(addi(14, 14, 1))
        self.branch(beq, 0, 0, f"{lbl}_t_loop")
        self.emit(NOP); self.emit(NOP)
        self.label(f"{lbl}_t_done")

        self.emit(addi(27, 27, ord('0')))
        self.emit(addi(13, 13, ord('0')))
        self.emit(addi(14, 14, ord('0')))
        self.emit(addi(2, 2, ord('0')))
        self.emit(addi(5, 0, ord('0')))

        self.emit(addi(15, 0, ord(' ')))
        self.emit(addi(8, 0, ord(' ')))
        self.emit(addi(9, 0, ord(' ')))

        self.branch(beq, 12, 0, f"{lbl}_chk_pos")
        self.emit(NOP); self.emit(NOP)

        self.branch(bne, 13, 5, f"{lbl}_n_h")
        self.emit(NOP); self.emit(NOP)
        self.branch(bne, 14, 5, f"{lbl}_n_t")
        self.emit(NOP); self.emit(NOP)
        self.emit(addi(9, 0, ord('-')))
        self.branch(beq, 0, 0, f"{lbl}_emit4")
        self.emit(NOP); self.emit(NOP)
        self.label(f"{lbl}_n_t")
        self.emit(addi(8, 0, ord('-')))
        self.emit(addi(9, 14, 0))
        self.branch(beq, 0, 0, f"{lbl}_emit4")
        self.emit(NOP); self.emit(NOP)
        self.label(f"{lbl}_n_h")
        self.emit(addi(15, 0, ord('-')))
        self.emit(addi(8, 13, 0))
        self.emit(addi(9, 14, 0))
        self.branch(beq, 0, 0, f"{lbl}_emit4")
        self.emit(NOP); self.emit(NOP)

        self.label(f"{lbl}_chk_pos")
        self.branch(bne, 27, 5, f"{lbl}_p_k")
        self.emit(NOP); self.emit(NOP)
        self.branch(bne, 13, 5, f"{lbl}_p_h")
        self.emit(NOP); self.emit(NOP)
        self.branch(bne, 14, 5, f"{lbl}_p_t")
        self.emit(NOP); self.emit(NOP)
        self.branch(beq, 0, 0, f"{lbl}_emit4")
        self.emit(NOP); self.emit(NOP)
        self.label(f"{lbl}_p_t")
        self.emit(addi(9, 14, 0))
        self.branch(beq, 0, 0, f"{lbl}_emit4")
        self.emit(NOP); self.emit(NOP)
        self.label(f"{lbl}_p_h")
        self.emit(addi(8, 13, 0))
        self.emit(addi(9, 14, 0))
        self.branch(beq, 0, 0, f"{lbl}_emit4")
        self.emit(NOP); self.emit(NOP)
        self.label(f"{lbl}_p_k")
        self.emit(addi(15, 27, 0))
        self.emit(addi(8, 13, 0))
        self.emit(addi(9, 14, 0))

        self.label(f"{lbl}_emit4")
        self.emit(addi(6, 15, 0))
        self.jump(1, "UART_SEND"); self.emit(NOP); self.emit(NOP)
        self.emit(addi(6, 8, 0))
        self.jump(1, "UART_SEND"); self.emit(NOP); self.emit(NOP)
        self.emit(addi(6, 9, 0))
        self.jump(1, "UART_SEND"); self.emit(NOP); self.emit(NOP)
        self.emit(addi(6, 2, 0))
        self.jump(1, "UART_SEND"); self.emit(NOP); self.emit(NOP)

        self.emit(jalr(0, 3, 0))
        for _ in range(4): self.emit(NOP)

        self.label("UART_SEND")
        self._sw_safe(6, 0, 10)
        self.emit(addi(7, 0, self.uart_delay))
        self.label("UART_DLY")
        self.emit(addi(7, 7, -1 & 0xFFF))
        self.emit(NOP)
        self.branch(bne, 7, 0, "UART_DLY")
        self.emit(NOP); self.emit(NOP)
        self.emit(jalr(0, 1, 0))
        for _ in range(4): self.emit(NOP)

        words = []
        for i, item in enumerate(self.items):
            pc = i * 4
            if isinstance(item, int):
                words.append(item)
            elif item[0] == "B":
                _, fn, rs1, rs2, target = item
                off = self.labels[target] - pc
                words.append(fn(rs1, rs2, off))
            elif item[0] == "J":
                _, rd, target = item
                off = self.labels[target] - pc
                words.append(jal(rd, off))
        return [w.to_bytes(4, byteorder="little") for w in words]

    def build_and_run(self, port=None):
        """Compile user app, write program.mem, upload over USB-UART, and open Serial Monitor."""
        arg1 = sys.argv[1] if len(sys.argv) > 1 else ""
        target_port = port or (arg1 if (arg1.upper().startswith("COM") or arg1.startswith("--")) else COM_PORT)
        if target_port == "--fast-sim":
            self.uart_delay = 1
        instructions = self.compile()
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

        print("=" * 65)
        print(f"  ASTRA-SDK COMPILER: {self.name}")
        print("=" * 65)
        print(f"Compiled Size : {len(instructions)} words ({len(all_bytes)} bytes)")
        print(f"Output Binary : {mem_path}")
        print("=" * 65)

        if target_port in ("--sim-only", "--fast-sim"):
            return

        import serial
        print(f"1. Hold down RESET (Pin M14) on your EDGE Artix-7 board and press Enter...")
        input()
        ser = serial.Serial(target_port, BAUD_RATE, timeout=1.0)
        try:
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            print(f"Uploading {len(all_bytes)} bytes to ASTRA-X1 SoC over {target_port}...")
            ser.write(bytes(all_bytes))
            ser.flush()
            time.sleep(len(all_bytes) * 10.0 / BAUD_RATE + 0.3)
            print("2. Upload complete! Release RESET now — Serial Monitor listening:\n")
            print("-" * 65)
            deadline = time.time() + 10.0
            buf = []
            while time.time() < deadline:
                if ser.in_waiting:
                    t = ser.read(ser.in_waiting).decode('ascii', errors='replace')
                    sys.stdout.write(t)
                    sys.stdout.flush()
                    buf.append(t)
                    if "PASS" in "".join(buf):
                        deadline = time.time() + 0.3
                    else:
                        deadline = time.time() + 1.5
                time.sleep(0.01)
            print("-" * 65)
        finally:
            ser.close()
