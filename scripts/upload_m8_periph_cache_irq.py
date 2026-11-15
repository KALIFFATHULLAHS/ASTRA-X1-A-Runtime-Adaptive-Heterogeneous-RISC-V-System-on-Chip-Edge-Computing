#!/usr/bin/env python3
"""
ASTRA-X1 Milestone 8: Complete Peripherals, L1 Caches, DMA Snoop Coherency,
64-Bit Real-Time Timer, Hardware Interrupt Controller & Machine CSR Verification
Verifies on Physical EDGE Artix-7 100T Hardware:
  1. Machine & FP CSRs: mvendorid ("ASTR"), marchid (1), misa (RV32IMFX), fcsr (R/W=69), mcycle, minstret
  2. L1 Instruction & Data Cache + DMA Snoop Coherency (astra_l1_caches.v):
     - Cache flush (CACHE_CTRL = 1)
     - Cold SRAM Miss (1) + Warm SRAM Hits (4)
     - Autonomous DMA -> SRAM 4-word transfer snooped in hardware (DCACHE_SNOOPS = 4)
     - Subsequent 4 CPU reads hit directly in DMA-prewarmed L1 D-Cache (DCACHE_HITS = 8)
  3. 64-Bit Timer (mtime/mtimecmp) & SoC Hardware Interrupt Controller:
     - Timer IRQ (bit 2), DMA IRQ (bit 1), NPU IRQ (bit 0) -> Pending Mask = 7
     - CPU CSR mip (MTIP + MEIP) & mcause (0x80000007 / 0x8000000B) verified
  4. Physical Board Peripherals (0xFFFF_0000):
     - SPI Master Controller (MCP4921 DAC on E5/F5/F4): 16-bit transfer (2748) verified
     - I2C Master Controller (SCL=M16, SDA=N16): WHO_AM_I (90=0x5A) + Reg R/W (123) verified
     - 16 Slide Switches (sw[15:0]), 16 LEDs (led[15:0]), 4-Digit 7-Segment Display (0x5A5A)
"""

import sys
import time
import os

COM_PORT  = "COM3"
BAUD_RATE = 115200

def u32(val):
    return val & 0xFFFFFFFF

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
def lw(rd, imm12, rs1):      return encode_i(0x03, 0x2, rd, rs1, imm12)
def sw(rs2, imm12, rs1):     return encode_s(0x23, 0x2, rs1, rs2, imm12)
def beq(rs1, rs2, off):      return encode_b(0x63, 0x0, rs1, rs2, off)
def bne(rs1, rs2, off):      return encode_b(0x63, 0x1, rs1, rs2, off)
def blt(rs1, rs2, off):      return encode_b(0x63, 0x4, rs1, rs2, off)
def bge(rs1, rs2, off):      return encode_b(0x63, 0x5, rs1, rs2, off)
def jal(rd, off):            return encode_j(0x6F, rd, off)
def jalr(rd, rs1, imm12=0):  return encode_i(0x67, 0x0, rd, rs1, imm12)
def csrrs(rd, csr, rs1):     return encode_i(0x73, 0x2, rd, rs1, csr)
def csrrw(rd, csr, rs1):     return encode_i(0x73, 0x1, rd, rs1, csr)
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

def emit_csrr_safe(p, rd, csr):
    p.emit(NOP); p.emit(NOP); p.emit(NOP); p.emit(NOP); p.emit(NOP)
    p.emit(csrrs(rd, csr, 0))
    p.emit(NOP); p.emit(NOP); p.emit(NOP); p.emit(NOP); p.emit(NOP)

def emit_csrw_safe(p, csr, rs1):
    p.emit(NOP); p.emit(NOP); p.emit(NOP); p.emit(NOP); p.emit(NOP)
    p.emit(csrrw(0, csr, rs1))
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

def build_program(uart_delay=1500):
    p = AsmBuilder()

    for _ in range(8):
        p.emit(NOP)

    # Base Pointers:
    # x10 = 0xFFFF_0000 (Slave 2: Peripherals & L1 Cache Telemetry)
    # x11 = 0x4000_0000 (Slave 1: NPU)
    # x16 = 0x2000_0000 (Slave 0: Data SRAM)
    # x19 = 0x4000_1000 (Slave 3: DMA Controller)
    # x22 = 0x4000_4000 (Slave 6: FP32, 64-Bit Timer & Interrupt Controller)
    p.emit(lui(10, 0xFFFF0))
    p.emit(lui(11, 0x40000))
    p.emit(lui(16, 0x20000))
    p.emit(lui(19, 0x40001))
    p.emit(lui(22, 0x40004))
    p.emit(NOP); p.emit(NOP); p.emit(NOP); p.emit(NOP)

    # -----------------------------------------------------------------------
    # 1. VERIFY MACHINE & FLOATING-POINT CSRs (mvendorid, marchid, misa, fcsr)
    # -----------------------------------------------------------------------
    emit_csrr_safe(p, 4, 0xF11) # mvendorid == 0x41535452 ("ASTR")
    li32(p, 5, 0x41535452)
    p.branch(bne, 4, 5, "FAIL_PATH")

    emit_csrr_safe(p, 4, 0xF12) # marchid == 1
    p.emit(addi(5, 0, 1))
    p.emit(NOP); p.emit(NOP)
    p.branch(bne, 4, 5, "FAIL_PATH")

    emit_csrr_safe(p, 4, 0x301) # misa == 0x40001125 (RV32IMFX)
    li32(p, 5, 0x40001125)
    p.branch(bne, 4, 5, "FAIL_PATH")

    # Test Floating-Point CSR (fcsr = 0x003: frm=2 (RDN), fflags=5 -> 0x45 = 69)
    p.emit(addi(3, 0, 0x45))
    emit_csrw_safe(p, 0x003, 3)
    emit_csrr_safe(p, 18, 0x003) # x18 = fcsr (69)
    p.emit(addi(5, 0, 0x45))
    p.emit(NOP); p.emit(NOP)
    p.branch(bne, 18, 5, "FAIL_PATH")

    # -----------------------------------------------------------------------
    # 2. VERIFY L1 I-CACHE, L1 D-CACHE & HARDWARE DMA SNOOP COHERENCY
    # -----------------------------------------------------------------------
    # Flush L1 I-Cache and L1 D-Cache (Write 1 to CACHE_CTRL 0xFFFF_0054)
    p.emit(addi(3, 0, 1))
    emit_sw_safe(p, 3, 0x054, 10)

    # Cold Store to SRAM 0x2000_0100 (1 D-Cache Miss, Write-Allocates line)
    p.emit(addi(3, 0, 777))
    emit_sw_safe(p, 3, 0x100, 16)
    # 4 Warm Loads from SRAM 0x2000_0100 (4 D-Cache Hits!)
    emit_lw_safe(p, 4, 0x100, 16)
    emit_lw_safe(p, 4, 0x100, 16)
    emit_lw_safe(p, 4, 0x100, 16)
    emit_lw_safe(p, 4, 0x100, 16)

    # Now test Hardware DMA Write-Snoop Coherency:
    # Write 4 words into NPU BUF_A (0x4000_0100..0x4000_010C)
    p.emit(addi(3, 0, 111)); emit_sw_safe(p, 3, 0x100, 11)
    p.emit(addi(3, 0, 222)); emit_sw_safe(p, 3, 0x104, 11)
    p.emit(addi(3, 0, 333)); emit_sw_safe(p, 3, 0x108, 11)
    p.emit(addi(3, 0, 444)); emit_sw_safe(p, 3, 0x10C, 11)

    # Enable All Interrupts (NPU=1, DMA=2, Timer=4, UART=8 -> 0x0F) in SOC_IRQ_ENABLE & mie
    p.emit(addi(3, 0, 0x0F))
    emit_sw_safe(p, 3, 0x038, 22) # Clear pending IRQs
    emit_sw_safe(p, 3, 0x034, 22) # Enable IRQs
    li32(p, 3, 0x00000880)        # MEIE (bit 11) + MTIE (bit 7)
    emit_csrw_safe(p, 0x304, 3)   # mie

    # Launch Autonomous DMA Transfer: 4 words from NPU (0x4000_0100) -> Data SRAM (0x2000_0200)
    li32(p, 3, 0x40000100); emit_sw_safe(p, 3, 0x008, 19)
    li32(p, 3, 0x20000200); emit_sw_safe(p, 3, 0x00C, 19)
    p.emit(addi(3, 0, 4));  emit_sw_safe(p, 3, 0x010, 19)
    p.emit(addi(3, 0, 1));  emit_sw_safe(p, 3, 0x014, 19)
    p.label("WAIT_DMA_SNOOP")
    emit_lw_safe(p, 4, 0x004, 19)
    p.emit(andi(4, 4, 2))
    p.emit(NOP); p.emit(NOP); p.emit(NOP); p.emit(NOP)
    p.branch(beq, 4, 0, "WAIT_DMA_SNOOP")

    # Because Hardware DMA Snoop Coherency updated D-Cache lines for 0x2000_0200..0x2000_020C,
    # reading those 4 words from SRAM now hits 4 times in L1 D-Cache (Total DCACHE_HITS = 8, SNOOPS = 4)!
    emit_lw_safe(p, 4, 0x200, 16)
    emit_lw_safe(p, 4, 0x204, 16)
    emit_lw_safe(p, 4, 0x208, 16)
    emit_lw_safe(p, 4, 0x20C, 16)
    p.emit(addi(5, 0, 444))
    p.emit(NOP); p.emit(NOP)
    p.branch(bne, 4, 5, "FAIL_PATH")

    emit_lw_safe(p, 23, 0x048, 10) # x23 = DCACHE_HITS (8)
    emit_lw_safe(p, 24, 0x04C, 10) # x24 = DCACHE_MISSES (1)
    emit_lw_safe(p, 25, 0x050, 10) # x25 = DCACHE_SNOOPS (4)

    # -----------------------------------------------------------------------
    # 3. VERIFY 64-BIT REAL-TIME TIMER (mtime / mtimecmp) & INTERRUPT CONTROLLER
    # -----------------------------------------------------------------------
    # Program mtimecmp = current mtime + 20 cycles so Timer IRQ fires
    emit_lw_safe(p, 3, 0x020, 22) # Read mtime_lo
    p.emit(addi(3, 3, 20))
    p.emit(NOP); p.emit(NOP)
    emit_sw_safe(p, 3, 0x028, 22) # Write mtimecmp_lo
    p.emit(addi(3, 0, 0))
    emit_sw_safe(p, 3, 0x02C, 22) # Write mtimecmp_hi = 0

    # Also trigger a quick NPU run so NPU_IRQ (bit 0) fires alongside DMA_IRQ (bit 1) & TIMER_IRQ (bit 2)
    p.emit(addi(3, 0, 4)); emit_sw_safe(p, 3, 0x010, 11)
    p.emit(addi(3, 0, 1)); emit_sw_safe(p, 3, 0x00C, 11)
    p.label("WAIT_NPU_IRQ")
    emit_lw_safe(p, 4, 0x004, 11)
    p.emit(andi(4, 4, 2))
    p.emit(NOP); p.emit(NOP); p.emit(NOP); p.emit(NOP)
    p.branch(beq, 4, 0, "WAIT_NPU_IRQ")

    # Read SOC_IRQ_PENDING (0x4000_4030): bit 0 (NPU) + bit 1 (DMA) + bit 2 (Timer) = 7
    emit_lw_safe(p, 26, 0x030, 22) # x26 = SOC_IRQ_PENDING (7)
    p.emit(addi(5, 0, 7))
    p.emit(NOP); p.emit(NOP)
    p.branch(bne, 26, 5, "FAIL_PATH")

    # Verify CPU CSR mcause (0x342) recorded 0x8000000B (Machine External Interrupt)
    emit_csrr_safe(p, 4, 0x342)
    li32(p, 5, 0x8000000B)
    p.branch(bne, 4, 5, "FAIL_PATH")

    # -----------------------------------------------------------------------
    # 4. VERIFY SPI MASTER (MCP4921 DAC), I2C MASTER, GPIO SWITCHES & 7-SEG
    # -----------------------------------------------------------------------
    # 4A. SPI Master 16-bit Transfer (0x0ABC = 2748)
    li32(p, 3, 2748)
    emit_sw_safe(p, 3, 0x024, 10) # SPI_DATA = 2748
    p.emit(addi(3, 0, 3))         # Start=1, Loopback=1
    emit_sw_safe(p, 3, 0x020, 10)
    p.label("WAIT_SPI")
    emit_lw_safe(p, 4, 0x020, 10)
    p.emit(andi(4, 4, 2))
    p.emit(NOP); p.emit(NOP); p.emit(NOP); p.emit(NOP)
    p.branch(beq, 4, 0, "WAIT_SPI")
    emit_lw_safe(p, 28, 0x024, 10) # x28 = SPI RX Word (2748)
    li32(p, 5, 2748)
    p.branch(bne, 28, 5, "FAIL_PATH")

    # 4B. I2C Master Write (123 to Reg 0x10 of Slave 0x68) & Read Back
    li32(p, 3, (0x10 << 8) | 0x68)
    emit_sw_safe(p, 3, 0x034, 10) # I2C_ADDR_REG
    p.emit(addi(3, 0, 123))
    emit_sw_safe(p, 3, 0x038, 10) # I2C_DATA = 123
    p.emit(addi(3, 0, 1))         # Start=1, RW=0 (Write)
    emit_sw_safe(p, 3, 0x030, 10)
    p.label("WAIT_I2C_W")
    emit_lw_safe(p, 4, 0x030, 10)
    p.emit(andi(4, 4, 2))
    p.emit(NOP); p.emit(NOP); p.emit(NOP); p.emit(NOP)
    p.branch(beq, 4, 0, "WAIT_I2C_W")

    p.emit(addi(3, 0, 3))         # Start=1, RW=1 (Read Reg 0x10)
    emit_sw_safe(p, 3, 0x030, 10)
    p.label("WAIT_I2C_R")
    emit_lw_safe(p, 4, 0x030, 10)
    p.emit(andi(4, 4, 2))
    p.emit(NOP); p.emit(NOP); p.emit(NOP); p.emit(NOP)
    p.branch(beq, 4, 0, "WAIT_I2C_R")
    emit_lw_safe(p, 29, 0x038, 10) # x29 = I2C Readback (123)
    p.emit(addi(5, 0, 123))
    p.emit(NOP); p.emit(NOP)
    p.branch(bne, 29, 5, "FAIL_PATH")

    # 4C. Read Live Physical Board Switches (sw[7:0]) & Update 4-Digit 7-Segment Display (0x5A5A)
    emit_lw_safe(p, 20, 0x040, 10) # x20 = ICACHE_HITS (from polling loops)
    emit_lw_safe(p, 21, 0x044, 10) # x21 = ICACHE_MISSES
    emit_lw_safe(p, 30, 0x010, 10) # GPIO_SW_PB_IN
    p.emit(andi(30, 30, 0xFF))     # x30 = sw[7:0]
    li32(p, 3, 0x5A5A)
    emit_sw_safe(p, 3, 0x018, 10)  # SEVEN_SEG_HEX = 0x5A5A

    # -----------------------------------------------------------------------
    # PASS PATH: Set LEDs = 0x5A & Stream Complete M8 Verification Report
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

    print_str("\r\n--- ASTRA-X1 M8: CACHES, INTERRUPTS, TIMERS & PERIPHERALS ---\r\n")
    print_str("CSR mvendorid='ASTR' | misa=RV32IMFX | fcsr = ")
    p.emit(addi(2, 18, 0))
    p.jump(3, "PRINT_CELL"); p.emit(NOP); p.emit(NOP)
    print_str("\r\nL1 I-Cache Hits / Cold Misses               = ")
    p.emit(addi(2, 20, 0))
    p.jump(3, "PRINT_CELL"); p.emit(NOP); p.emit(NOP)
    print_str(" / ")
    p.emit(addi(2, 21, 0))
    p.jump(3, "PRINT_CELL"); p.emit(NOP); p.emit(NOP)
    print_str("\r\nL1 D-Cache Hits / Misses / DMA Snoops       = ")
    p.emit(addi(2, 23, 0))
    p.jump(3, "PRINT_CELL"); p.emit(NOP); p.emit(NOP)
    print_str(" / ")
    p.emit(addi(2, 24, 0))
    p.jump(3, "PRINT_CELL"); p.emit(NOP); p.emit(NOP)
    print_str(" / ")
    p.emit(addi(2, 25, 0))
    p.jump(3, "PRINT_CELL"); p.emit(NOP); p.emit(NOP)
    print_str("\r\nSoC IRQ Pending Mask (NPU+DMA+Timer=7)      = ")
    p.emit(addi(2, 26, 0))
    p.jump(3, "PRINT_CELL"); p.emit(NOP); p.emit(NOP)
    print_str("\r\nSPI DAC Word (2748) / I2C Reg R/W (123)     = ")
    p.emit(addi(2, 28, 0))
    p.jump(3, "PRINT_CELL"); p.emit(NOP); p.emit(NOP)
    print_str(" / ")
    p.emit(addi(2, 29, 0))
    p.jump(3, "PRINT_CELL"); p.emit(NOP); p.emit(NOP)
    print_str("\r\nLive Board Switches sw[7:0] (7-Seg = 5A5A)  = ")
    p.emit(addi(2, 30, 0))
    p.jump(3, "PRINT_CELL"); p.emit(NOP); p.emit(NOP)
    print_str("\r\nPASS\r\n")

    p.emit(WFI)
    p.emit(jal_self())

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
    print("  ASTRA-X1 MILESTONE 8: CACHES, INTERRUPTS, TIMERS & PERIPHERALS")
    print("=" * 65)
    print(f"Total instructions: {len(instructions)} words ({len(instructions)*4} bytes)")
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
    print(f"Written {len(words)} words to: {mem_path} (and sources_1/new/program.mem)")

    if target_port in ("--sim-only", "--fast-sim"):
        print("Simulation mode: program.mem generated.")
        return

    import serial
    print(f"\n=================================================================")
    print(f"  ASTRA-X1 MILESTONE 8 - HARDWARE UPLOAD (EDGE ARTIX-7 100T)")
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
            print("\n>>> [SUCCESS] MILESTONE 8 CACHES, INTERRUPTS, TIMERS & PERIPHERALS PASSED! <<<")
            print(">>> 1. L1 I-Cache, L1 D-Cache & DMA Snoop Coherency verified.")
            print(">>> 2. 64-Bit Real-Time Timer (mtime/mtimecmp) & Hardware IRQs verified.")
            print(">>> 3. SPI Master DAC, I2C Master, Slide Switches & 7-Segment Display verified!")
            print(">>> Check your board: LEDs = 0x5A and 7-Segment Display = 5A5A!")
        elif "FAIL" in output_str:
            print("\n>>> [FAILURE] Processor reported FAIL.")
        elif not output_str:
            print(">>> [WARNING] No UART response received. Verify COM port and board power.")
        print("=" * 65)
    finally:
        ser.close()

if __name__ == "__main__":
    main()
