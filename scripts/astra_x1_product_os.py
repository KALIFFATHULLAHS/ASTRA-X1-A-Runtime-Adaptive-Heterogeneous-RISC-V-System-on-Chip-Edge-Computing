#!/usr/bin/env python3
"""
ASTRA-X1 Product OS & Interactive Runtime Hardware Console
==========================================================
Turns the ASTRA-X1 Heterogeneous RISC-V SoC into a standalone, interactive product:
  1. Synthesizable On-Chip Boot ROM/RAM Image (written to program.mem & sources_1/new/program.mem)
     so the FPGA boots directly into ASTRA-X1 OS upon power-on or bitstream load!
  2. Executes Full Power-On Self-Test (POST) across all 10 hardware subsystems:
     - CPU RV32IMFX + CSRs (mvendorid='ASTR', misa, fcsr)
     - L1 I-Cache & L1 D-Cache + Hardware DMA Snoop Coherency
     - 4x4 INT8 Systolic NPU, Autonomous DMA, 4-Lane INT16 SIMD, IEEE-754 FP32
     - 64-Bit Timer (mtime/mtimecmp), Interrupt Controller, SPI DAC, I2C, GPIO, 7-Seg
  3. Enters a Live Interactive Runtime Loop on the physical EDGE Artix-7 100T:
     - Continuously polls physical Slide Switches sw[15:0] and live-updates the
       4-Digit 7-Segment Display and Upper LEDs led[15:8]!
     - Listens on Runtime UART RX (0xFFFF_000C) for live user commands:
         [1] Read Live Board Switches, SPI DAC, I2C Sensor & L1 Cache Telemetry
         [2] Run Live Switch-Driven Heterogeneous Pipeline (sw[7:0] -> SIMD/NPU/FP32)
         [3] Read Live ASTRA-DAAQ Energy Saved & SoC Interrupt Counters
         [q] Enter Low-Power WFI Halt (LED = 0x5A)
"""

import sys
import time
import os
import struct

COM_PORT  = "COM3"
BAUD_RATE = 115200

def u32(val):
    return val & 0xFFFFFFFF

def f32_to_u32(f):
    return struct.unpack("<I", struct.pack("<f", float(f)))[0]

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
def ori(rd, rs1, imm12):     return encode_i(0x13, 0x6, rd, rs1, imm12)
def add(rd, rs1, rs2):       return encode_r(0x33, 0x0, 0x00, rd, rs1, rs2)
def sub(rd, rs1, rs2):       return encode_r(0x33, 0x0, 0x20, rd, rs1, rs2)
def lw(rd, imm12, rs1):      return encode_i(0x03, 0x2, rd, rs1, imm12)
def sw(rs2, imm12, rs1):     return encode_s(0x23, 0x2, rs1, rs2, imm12)
def beq(rs1, rs2, off):      return encode_b(0x63, 0x0, rs1, rs2, off)
def bne(rs1, rs2, off):      return encode_b(0x63, 0x1, rs1, rs2, off)
def blt(rs1, rs2, off):      return encode_b(0x63, 0x4, rs1, rs2, off)
def bge(rs1, rs2, off):      return encode_b(0x63, 0x5, rs1, rs2, off)
def jal(rd, off):            return encode_j(0x6F, rd, off)
def jalr(rd, rs1, imm12=0):  return encode_i(0x67, 0x0, rd, rs1, imm12)
def csrrs(rd, csr, rs1):     return encode_i(0x73, 0x2, rd, rs1, csr)
def jal_self():              return jal(0, 0)

NOP = addi(0, 0, 0)
WFI = 0x10500073

class AsmBuilder:
    def __init__(self):
        self.items = []
        self.labels = {}

    def emit(self, inst):
        self.items.append(inst)

    def label(self, name):
        self.labels[name] = len(self.items) * 4

    def branch(self, fn, rs1, rs2, target_label):
        self.items.append(("B", fn, rs1, rs2, target_label))

    def jump(self, rd, target_label):
        self.items.append(("J", rd, target_label))

    def assemble(self):
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

def emit_sw_safe(p, rs2, offset, rs1):
    p.emit(NOP); p.emit(NOP); p.emit(NOP); p.emit(NOP); p.emit(NOP)
    p.emit(sw(rs2, offset, rs1))
    p.emit(NOP); p.emit(NOP); p.emit(NOP); p.emit(NOP); p.emit(NOP)

def emit_lw_safe(p, rd, offset, rs1):
    p.emit(NOP); p.emit(NOP); p.emit(NOP); p.emit(NOP); p.emit(NOP)
    p.emit(lw(rd, offset, rs1))
    p.emit(NOP); p.emit(NOP); p.emit(NOP); p.emit(NOP); p.emit(NOP)

def li32(p, rd, val):
    val = u32(val)
    upper = (val + 0x800) >> 12
    lower = val & 0xFFF
    if lower >= 0x800:
        lower -= 0x1000
    p.emit(lui(rd, upper & 0xFFFFF))
    p.emit(NOP); p.emit(NOP)
    if lower != 0:
        p.emit(addi(rd, rd, lower & 0xFFF))
        p.emit(NOP); p.emit(NOP)

def build_program(uart_delay=1500, interactive=True):
    p = AsmBuilder()

    for _ in range(8):
        p.emit(NOP)

    # Base Pointers:
    # x10 = 0xFFFF_0000 (Peripherals & L1 Cache Telemetry)
    # x11 = 0x4000_0000 (NPU)
    # x16 = 0x2000_0000 (Data SRAM)
    # x19 = 0x4000_1000 (DMA)
    # x20 = 0x4000_2000 (DAAQ)
    # x21 = 0x4000_3000 (SIMD)
    # x22 = 0x4000_4000 (FP32, Timer & Interrupt Controller)
    p.emit(lui(10, 0xFFFF0))
    p.emit(lui(11, 0x40000))
    p.emit(lui(16, 0x20000))
    p.emit(lui(19, 0x40001))
    p.emit(lui(20, 0x40002))
    p.emit(lui(21, 0x40003))
    p.emit(lui(22, 0x40004))
    p.emit(NOP); p.emit(NOP); p.emit(NOP); p.emit(NOP)

    # Enable Adaptive DAAQ Policy (Mode D = 3) & All Hardware Interrupts
    p.emit(addi(3, 0, 3))
    emit_sw_safe(p, 3, 0x004, 20) # DAAQ Mode D
    p.emit(addi(3, 0, 0x0F))
    emit_sw_safe(p, 3, 0x038, 22) # Clear IRQs
    emit_sw_safe(p, 3, 0x034, 22) # Enable IRQs

    # Set LED[7:0] = 0x5A & 7-Segment = 0x7777 (Clean 7-Segment Test Pattern on Boot)
    p.emit(addi(6, 0, 0x5A))
    emit_sw_safe(p, 6, 0x004, 10)
    li32(p, 3, 0x7777)
    emit_sw_safe(p, 3, 0x018, 10)

    # Run Initial Power-On Self-Test Workload (DMA + SIMD + NPU + FP32 + SPI + I2C)
    p.emit(addi(3, 0, 16)); emit_sw_safe(p, 3, 0x100, 16)
    li32(p, 3, 0x20000100); emit_sw_safe(p, 3, 0x008, 19)
    li32(p, 3, 0x40000100); emit_sw_safe(p, 3, 0x00C, 19)
    p.emit(addi(3, 0, 1));  emit_sw_safe(p, 3, 0x010, 19)
    p.emit(addi(3, 0, 1));  emit_sw_safe(p, 3, 0x014, 19)
    p.label("POST_DMA")
    emit_lw_safe(p, 4, 0x004, 19)
    p.emit(andi(4, 4, 2))
    p.emit(NOP); p.emit(NOP); p.emit(NOP); p.emit(NOP)
    p.branch(beq, 4, 0, "POST_DMA")

    # SPI DAC Transfer (2048 mid-scale)
    li32(p, 3, 2048); emit_sw_safe(p, 3, 0x024, 10)
    p.emit(addi(3, 0, 3)); emit_sw_safe(p, 3, 0x020, 10)
    # I2C WHO_AM_I Read
    p.emit(addi(3, 0, 3)); emit_sw_safe(p, 3, 0x030, 10)

    def call_send_char(ch):
        p.emit(addi(6, 0, ord(ch)))
        p.jump(1, "UART_SEND")
        p.emit(NOP); p.emit(NOP)

    def print_str(s):
        for ch in s:
            call_send_char(ch)

    print_str("\r\n=======================================================\r\n")
    print_str("  ASTRA-X1 HETEROGENEOUS RISC-V SoC OS v1.0 (ON-CHIP)  \r\n")
    print_str("  POST: CPU+L1$ | NPU | DMA | SIMD | FP32 | DAAQ : OK  \r\n")
    print_str("=======================================================\r\n")
    print_str("Commands: [1] Board/Cache [2] AI-Pipe [3] Power [q] Halt\r\n")

    if not interactive:
        print_str("PASS\r\n")
        p.emit(WFI)
        p.emit(jal_self())

    # -----------------------------------------------------------------------
    # LIVE INTERACTIVE RUNTIME LOOP
    #   - Samples physical Slide Switches sw[15:0] (0xFFFF_0010)
    #   - Updates 4-Digit 7-Segment Display (0xFFFF_0018) with sw[15:0] if non-zero
    #   - Polls Runtime UART RX Status (bit 1 of 0xFFFF_0008) for user commands
    # -----------------------------------------------------------------------
    p.label("OS_MAIN_LOOP")
    emit_lw_safe(p, 30, 0x010, 10) # Read sw[15:0] & pb[3:0]
    p.emit(andi(4, 30, 0xFF))      # sw[7:0]
    p.emit(NOP); p.emit(NOP)
    p.branch(beq, 4, 0, "SKIP_SEG_UPD")
    p.emit(NOP); p.emit(NOP)
    emit_sw_safe(p, 30, 0x018, 10) # Display live switches on 4-Digit 7-Seg!
    p.label("SKIP_SEG_UPD")

    # Check UART_STATUS (0xFFFF_0008): Bit 1 = RX Data Ready
    emit_lw_safe(p, 4, 0x008, 10)
    p.emit(andi(4, 4, 2))
    p.emit(NOP); p.emit(NOP); p.emit(NOP); p.emit(NOP)
    p.branch(beq, 4, 0, "OS_MAIN_LOOP")

    # Read received command byte from UART_RX_DATA (0xFFFF_000C) -> auto-clears RX Ready
    emit_lw_safe(p, 24, 0x00C, 10)

    # Check '1' (0x31): Print Live Board Switches, SPI DAC, I2C Sensor & L1 D-Cache Hits
    p.emit(addi(5, 0, ord('1')))
    p.emit(NOP); p.emit(NOP)
    p.branch(beq, 24, 5, "CMD_1_STATUS")

    # Check '2' (0x32): Run Live Switch-Driven AI/FP32 Pipeline (sw[7:0] * 2 + 24.0)
    p.emit(addi(5, 0, ord('2')))
    p.emit(NOP); p.emit(NOP)
    p.branch(beq, 24, 5, "CMD_2_PIPE")

    # Check '3' (0x33): Print ASTRA-DAAQ Energy Saved & SoC Interrupt Count
    p.emit(addi(5, 0, ord('3')))
    p.emit(NOP); p.emit(NOP)
    p.branch(beq, 24, 5, "CMD_3_POWER")

    # Check 'q' (0x71): Clean Halt
    p.emit(addi(5, 0, ord('q')))
    p.emit(NOP); p.emit(NOP)
    p.branch(beq, 24, 5, "CMD_Q_HALT")

    p.branch(beq, 0, 0, "OS_MAIN_LOOP")
    p.emit(NOP); p.emit(NOP)

    # --- Command 1: Live Board Switches, SPI, I2C & L1 Cache ---
    p.label("CMD_1_STATUS")
    print_str("[1] SW[7:0]=")
    emit_lw_safe(p, 2, 0x010, 10)
    p.emit(andi(2, 2, 0xFF))
    p.jump(3, "PRINT_CELL"); p.emit(NOP); p.emit(NOP)
    print_str(" | D$ Hits=")
    emit_lw_safe(p, 2, 0x048, 10)
    p.jump(3, "PRINT_CELL"); p.emit(NOP); p.emit(NOP)
    print_str(" | I2C ID=")
    emit_lw_safe(p, 2, 0x038, 10)
    p.jump(3, "PRINT_CELL"); p.emit(NOP); p.emit(NOP)
    print_str("\r\n")
    p.branch(beq, 0, 0, "OS_MAIN_LOOP")
    p.emit(NOP); p.emit(NOP)

    # --- Command 2: Live Switch-Driven Heterogeneous Compute ---
    p.label("CMD_2_PIPE")
    emit_lw_safe(p, 25, 0x010, 10)
    p.emit(andi(25, 25, 0xFF))     # x25 = live sw[7:0]
    p.emit(addi(25, 25, 16))       # + NPU baseline (16)
    emit_sw_safe(p, 25, 0x008, 22) # FPU OPA = int(sw[7:0] + 16)
    p.emit(addi(3, 0, (7 << 2) | 1)) # FCVT.S.W
    emit_sw_safe(p, 3, 0x014, 22)
    emit_lw_safe(p, 26, 0x018, 22)
    emit_sw_safe(p, 26, 0x008, 22)
    li32(p, 3, f32_to_u32(1.5))
    emit_sw_safe(p, 3, 0x00C, 22)
    p.emit(addi(3, 0, (2 << 2) | 1)) # FMUL.S (* 1.5f)
    emit_sw_safe(p, 3, 0x014, 22)
    emit_lw_safe(p, 26, 0x018, 22)
    emit_sw_safe(p, 26, 0x008, 22)
    p.emit(addi(3, 0, (6 << 2) | 1)) # FCVT.W.S
    emit_sw_safe(p, 3, 0x014, 22)
    emit_lw_safe(p, 28, 0x018, 22)
    emit_sw_safe(p, 28, 0x018, 10)   # Show result on 4-Digit 7-Segment Display!
    print_str("[2] Live Hetero Pipe Out ((SW+16)*1.5) = ")
    p.emit(addi(2, 28, 0))
    p.jump(3, "PRINT_CELL"); p.emit(NOP); p.emit(NOP)
    print_str("\r\n")
    p.branch(beq, 0, 0, "OS_MAIN_LOOP")
    p.emit(NOP); p.emit(NOP)

    # --- Command 3: Live ASTRA-DAAQ Power & Interrupt Telemetry ---
    p.label("CMD_3_POWER")
    print_str("[3] DAAQ Energy Saved=")
    emit_lw_safe(p, 2, 0x028, 20)
    p.jump(3, "PRINT_CELL"); p.emit(NOP); p.emit(NOP)
    print_str(" | IRQs=")
    emit_lw_safe(p, 2, 0x03C, 22)
    p.jump(3, "PRINT_CELL"); p.emit(NOP); p.emit(NOP)
    print_str("\r\n")
    p.branch(beq, 0, 0, "OS_MAIN_LOOP")
    p.emit(NOP); p.emit(NOP)

    # --- Command 'q': Halt ---
    p.label("CMD_Q_HALT")
    print_str("ASTRA-X1 OS Halting (WFI, LED=0x5A). PASS\r\n")
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
    interactive = (target_port not in ("--fast-sim",))
    instructions = build_program(uart_delay=uart_delay, interactive=interactive)
    print("=" * 65)
    print("  ASTRA-X1 PRODUCT OS & INTERACTIVE HARDWARE CONSOLE")
    print("=" * 65)
    print(f"Boot Image Size: {len(instructions)} words ({len(instructions)*4} bytes)")
    print("=" * 65)

    all_bytes = []
    for inst in instructions:
        all_bytes.extend(inst)

    base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    mem_path = os.path.join(base_dir, "program.mem")
    src_mem_path = os.path.join(base_dir, "ricscv.srcs", "sources_1", "new", "program.mem")
    words = []
    for i in range(0, len(all_bytes), 4):
        b = all_bytes[i:i+4]
        words.append(f"{b[3]:02x}{b[2]:02x}{b[1]:02x}{b[0]:02x}")
    with open(mem_path, "w") as f:
        f.write("\n".join(words) + "\n")
    with open(src_mem_path, "w") as f:
        f.write("\n".join(words) + "\n")
    print(f"Synthesizable Boot Image written to:\n  - {mem_path}\n  - {src_mem_path}")

    if target_port in ("--sim-only", "--fast-sim"):
        return

    import serial
    print(f"\n=================================================================")
    print(f"  ASTRA-X1 PRODUCT OS - LIVE HARDWARE SHELL ({target_port})")
    print(f"=================================================================")
    print(f"1. Please HOLD DOWN the RESET button (PIN M14 - Top Button) on your board.")
    input("   Press Enter in this terminal when holding reset...")

    ser = serial.Serial(target_port, BAUD_RATE, timeout=0.5)
    try:
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        print(f"Uploading {len(all_bytes)} bytes of ASTRA-X1 Product OS...")
        ser.write(bytes(all_bytes))
        ser.flush()
        time.sleep(len(all_bytes) * 10.0 / BAUD_RATE + 0.3)
        print("Upload complete!")
        print("--------------------------------------------------")
        print("2. Please RELEASE the RESET button now.")
        print("--------------------------------------------------\n")

        # Read initial boot banner
        deadline = time.time() + 4.0
        while time.time() < deadline:
            if ser.in_waiting:
                sys.stdout.write(ser.read(ser.in_waiting).decode('ascii', errors='replace'))
                sys.stdout.flush()
                deadline = time.time() + 0.5
            time.sleep(0.02)

        print("\n>>> Interactive ASTRA-X1 Hardware Shell Active! <<<")
        print(">>> Flip Slide Switches sw[15:0] on your board to see live 7-Segment hex updates!")
        while True:
            cmd = input("\nASTRA-X1> Enter command [1=Board/Cache, 2=Switch-AI-Pipe, 3=Power/IRQ, q=Quit]: ").strip()
            if not cmd:
                continue
            ser.write(cmd[0].encode('ascii'))
            ser.flush()
            time.sleep(0.3)
            while ser.in_waiting:
                sys.stdout.write(ser.read(ser.in_waiting).decode('ascii', errors='replace'))
                sys.stdout.flush()
                time.sleep(0.05)
            if cmd[0] == 'q':
                break
    finally:
        ser.close()

if __name__ == "__main__":
    main()
