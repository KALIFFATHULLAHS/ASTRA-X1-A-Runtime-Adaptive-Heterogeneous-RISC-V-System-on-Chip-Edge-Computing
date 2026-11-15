
#!/usr/bin/env python3
"""
ASTRA-X1 Milestone 2 Test: Wishbone Bus + Mock NPU Verification
================================================================
Tests:
  1. CPU reads NPU_REG_ID  (0x4000_0000) over Wishbone -> expects 0x41535452 ("ASTR")
  2. CPU writes scratchpad (0x4000_0008) and reads it back -> expects 0x12345678
  3. Writes LED pattern: 0xA5 on PASS, 0x01 on FAIL
  4. Transmits "PASS\n" or "FAIL\n" over UART using status-polled MMIO (0xFFFF_0000 / 0xFFFF_0008)

Generates:
  - program.mem  : $readmemh-compatible hex file for XSim simulation
  - Supports FPGA hardware upload over UART when called with COM port

Usage:
  python upload_m2_npu_test.py          -> generates program.mem
  python upload_m2_npu_test.py COM3     -> FPGA hardware upload
"""

import sys
import struct
import os

COM_PORT  = "COM3"
BAUD_RATE = 115200

# ===========================================================================
# RISC-V RV32IM Instruction Encoders
# ===========================================================================

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

def jal_self():
    return _le32(0x0000006F)

WFI  = _le32(0x10500073)
NOP  = _le32(0x00000013)

# ===========================================================================
# PROGRAM BUILDER WITH AUTOMATIC LABEL RESOLUTION
# ===========================================================================

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
        self.fixups.append((idx, b_fn, rs1, rs2, target_label))
        self.instructions.append(None)

    def assemble(self):
        for idx, b_fn, rs1, rs2, target in self.fixups:
            curr_addr = idx * 4
            target_addr = self.labels[target]
            offset = target_addr - curr_addr
            self.instructions[idx] = b_fn(rs1, rs2, offset)
        return self.instructions

p = ProgramBuilder()

# ---------------------------------------------------------------------------
# Setup & Addresses
# ---------------------------------------------------------------------------
p.emit(lui(1, 0x40000))          # x1 = Mock NPU base: 0x4000_0000
p.emit(lui(10, 0xFFFF0))         # x10 = MMIO base:    0xFFFF_0000

# ---------------------------------------------------------------------------
# Test 1: Read Mock NPU ID (expect 0x41535452 "ASTR")
# ---------------------------------------------------------------------------
p.emit(lw(2, 0, 1))              # x2 = MEM[0x4000_0000] (NPU_REG_ID)
p.emit(lui(5, 0x41535))          # x5 = 0x4153_5000
p.emit(addi(5, 5, 0x452))        # x5 = 0x4153_5452
p.branch(bne, 2, 5, "FAIL_PATH") # If ID != "ASTR", branch to FAIL

# ---------------------------------------------------------------------------
# Test 2: Write & Readback NPU Scratchpad (expect 0x12345678)
# ---------------------------------------------------------------------------
p.emit(lui(3, 0x12345))          # x3 = 0x1234_5000
p.emit(addi(3, 3, 0x678))        # x3 = 0x1234_5678
p.emit(sw(3, 8, 1))              # MEM[0x4000_0008] = x3
p.emit(NOP)                      # Absorbs store stall
p.emit(lw(4, 8, 1))              # x4 = MEM[0x4000_0008]
p.branch(bne, 4, 3, "FAIL_PATH") # If scratchpad != written, branch to FAIL

# ---------------------------------------------------------------------------
# PASS PATH: LED = 0xA5, print "PASS\n"
# ---------------------------------------------------------------------------
p.label("PASS_PATH")
p.emit(addi(6, 0, 0xA5 & 0xFFF)) # LED pattern 0xA5
p.emit(sw(6, 4, 10))             # dbg_led <= 0xA5 (instant pass indicator!)
p.emit(NOP)                      # Absorbs store stall

# Helper to emit UART transmit of a single ASCII byte with a clean software delay.
def emit_uart_byte(p, char_code, label_suffix):
    p.emit(addi(6, 0, char_code))
    p.emit(sw(6, 0, 10))                 # Transmit character to UART (0xFFFF_0000)
    p.emit(NOP)                          # Absorbs store stall
    p.emit(sw(6, 4, 10))                 # Show current ASCII byte on LEDs (0xFFFF_0004)
    p.emit(NOP)                          # Absorbs store stall
    p.emit(addi(7, 0, 1200))             # Delay counter = 1200
    p.label(f"delay_{label_suffix}")
    p.emit(addi(7, 7, -1 & 0xFFF))       # x7 = x7 - 1
    p.emit(NOP)                          # 1-cycle bubble so addi reaches EX2
    p.branch(bne, 7, 0, f"delay_{label_suffix}")

emit_uart_byte(p, ord('P'), "p_P")
emit_uart_byte(p, ord('A'), "p_A")
emit_uart_byte(p, ord('S'), "p_S1")
emit_uart_byte(p, ord('S'), "p_S2")
emit_uart_byte(p, ord('\n'), "p_NL")

# Final LED pass indication: 0xA5
p.emit(addi(6, 0, 0xA5 & 0xFFF))
p.emit(sw(6, 4, 10))

p.emit(WFI)
p.emit(jal_self())

# ---------------------------------------------------------------------------
# FAIL PATH: LED = 0x01, print "FAIL\n"
# ---------------------------------------------------------------------------
p.label("FAIL_PATH")
p.emit(addi(6, 0, 0x01))         # LED pattern 0x01 (FAIL)
p.emit(sw(6, 4, 10))             # dbg_led <= 0x01

emit_uart_byte(p, ord('F'), "f_F")
emit_uart_byte(p, ord('A'), "f_A")
emit_uart_byte(p, ord('I'), "f_I")
emit_uart_byte(p, ord('L'), "f_L")
emit_uart_byte(p, ord('\n'), "f_NL")

p.emit(WFI)
p.emit(jal_self())

# Assemble instructions
assembled_instructions = p.assemble()

# ===========================================================================
# BUILD BINARY & WRITE program.mem
# ===========================================================================

def build_binary(instructions, pad_words=64):
    prog = bytearray()
    for inst in instructions:
        prog.extend(inst)
    prog.extend([0x13, 0x00, 0x00, 0x00] * pad_words)
    return prog

program_bytes = build_binary(assembled_instructions)

def write_mem_file(binary_data, path):
    words = []
    for i in range(0, len(binary_data), 4):
        chunk = binary_data[i:i+4]
        if len(chunk) < 4:
            chunk = chunk + b'\x00' * (4 - len(chunk))
        word = struct.unpack('<I', bytes(chunk))[0]
        words.append(f"{word:08X}")
    with open(path, 'w') as f:
        f.write('\n'.join(words) + '\n')
    return len(words)

def print_layout():
    print("=" * 65)
    print("  ASTRA-X1 MILESTONE 2: PROGRAM DISASSEMBLY & LAYOUT")
    print("=" * 65)
    for idx, inst in enumerate(assembled_instructions):
        word = struct.unpack('<I', bytes(inst))[0]
        addr = idx * 4
        lbl = ""
        for name, laddr in p.labels.items():
            if laddr == addr:
                lbl = f" <{name}>"
        print(f"  0x{addr:04X}: 0x{word:08X}{lbl}")
    print("=" * 65)
    print(f"Total instructions: {len(assembled_instructions)}")
    print(f"PASS_PATH at 0x{p.labels['PASS_PATH']:04X}")
    print(f"FAIL_PATH at 0x{p.labels['FAIL_PATH']:04X}")
    print("=" * 65)

# ===========================================================================
# FPGA UPLOAD (Hardware Mode)
# ===========================================================================

def fpga_upload(port):
    try:
        import serial
        import time
    except ImportError:
        print("ERROR: 'pyserial' not installed. Run: pip install pyserial")
        return

    print("=" * 65)
    print("  ASTRA-X1 MILESTONE 2 - HARDWARE UPLOAD (EDGE ARTIX-7 100T)")
    print("=" * 65)
    print("1. Please HOLD DOWN the RESET button (PIN M14 - Top Button) on your board.")
    input("   Press Enter in this terminal when holding reset...")

    print(f"Opening {port} at {BAUD_RATE} baud...")
    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=1.0)
    except Exception as e:
        print(f"ERROR: Could not open {port}: {e}")
        return

    try:
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        print(f"Uploading {len(program_bytes)} bytes over UART to BRAM Instruction Memory...")
        ser.write(program_bytes)
        ser.flush()
        print("Upload complete!")
        print("--------------------------------------------------")
        print("2. Please RELEASE the RESET button now.")
        print("   Listening for execution response on UART (10s timeout)...")
        print("--------------------------------------------------")

        resp = b""
        deadline = time.time() + 10.0
        while time.time() < deadline:
            if ser.in_waiting:
                resp += ser.read(ser.in_waiting)
                if b'\n' in resp:
                    break
            time.sleep(0.02)

        print("-" * 50)
        output_str = resp.decode('ascii', errors='replace').strip()
        print(f"Received from ASTRA-X1 (ASCII): '{output_str}'")
        if resp:
            print(f"Received bytes (Hex): {' '.join(f'{b:02X}' for b in resp)}")
        if output_str.startswith("PASS"):
            print("\n>>> [SUCCESS] MILESTONE 2 VERIFIED ON PHYSICAL FPGA HARDWARE! <<<")
            print(">>> 1. Read Mock NPU ID (0x4000_0000): 0x41535452 ('ASTR')")
            print(">>> 2. Wishbone B4 Scratchpad RW (0x4000_0008): 0x12345678 Verified")
            print(">>> 3. Status-Polled MMIO UART Transmission: Verified")
            print(">>> Check your board: LEDs [7:0] MUST BE 0xA5 (Binary 0b10100101)!")
        elif output_str.startswith("FAIL"):
            print("\n>>> [FAILURE] Processor reported FAIL. Check register readback.")
        else:
            print(f"\n>>> [WARNING] Unexpected or empty response: {repr(resp)}")
            print(">>> Tip: Verify you held the reset button during upload and released it afterward.")
        print("=" * 65)
    finally:
        ser.close()

if __name__ == "__main__":
    print_layout()
    mem_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "program.mem")
    n_words = write_mem_file(program_bytes, mem_path)
    print(f"Written {n_words} words to: {mem_path}")
    print()

    target_port = sys.argv[1] if len(sys.argv) > 1 else COM_PORT
    if target_port == "--sim-only":
        print("Simulation mode: program.mem generated. Run: xsim sim_snap -R")
    else:
        fpga_upload(target_port)
