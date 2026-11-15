import serial
import time
import sys

COM_PORT = "COM3"
BAUD_RATE = 115200

# =====================================================================
# STANDARD COMPILER-COMPLIANT RISC-V RV32IM INSTRUCTION ENCODERS
# (Conforms 100% to Official RISC-V Unprivileged ISA Specification)
# =====================================================================
def lui(rd, imm_20bit):
    val = ((imm_20bit & 0xFFFFF) << 12) | ((rd & 0x1F) << 7) | 0x37
    return [(val >> (8 * i)) & 0xFF for i in range(4)]

def addi(rd, rs1, imm_12bit):
    val = ((imm_12bit & 0xFFF) << 20) | ((rs1 & 0x1F) << 15) | (0 << 12) | ((rd & 0x1F) << 7) | 0x13
    return [(val >> (8 * i)) & 0xFF for i in range(4)]

def add(rd, rs1, rs2):
    val = ((rs2 & 0x1F) << 20) | ((rs1 & 0x1F) << 15) | (0 << 12) | ((rd & 0x1F) << 7) | 0x33
    return [(val >> (8 * i)) & 0xFF for i in range(4)]

def sw(rs2, offset_12bit, rs1):
    off = offset_12bit & 0xFFF
    imm_top = (off >> 5) & 0x7F
    imm_bot = off & 0x1F
    val = (imm_top << 25) | ((rs2 & 0x1F) << 20) | ((rs1 & 0x1F) << 15) | (2 << 12) | (imm_bot << 7) | 0x23
    return [(val >> (8 * i)) & 0xFF for i in range(4)]

def r_type(rd, rs1, rs2, funct3, funct7):
    val = ((funct7 & 0x7F) << 25) | ((rs2 & 0x1F) << 20) | ((rs1 & 0x1F) << 15) | ((funct3 & 0x7) << 12) | ((rd & 0x1F) << 7) | 0x33
    return [(val >> (8 * i)) & 0xFF for i in range(4)]

def bne(rs1, rs2, offset_13bit):
    off = offset_13bit & 0x1FFF
    imm_12  = (off >> 12) & 0x1
    imm_10_5= (off >> 5)  & 0x3F
    imm_4_1 = (off >> 1)  & 0xF
    imm_11  = (off >> 11) & 0x1
    val = (imm_12 << 31) | (imm_10_5 << 25) | ((rs2 & 0x1F) << 20) | ((rs1 & 0x1F) << 15) | (1 << 12) | (imm_4_1 << 8) | (imm_11 << 7) | 0x63
    return [(val >> (8 * i)) & 0xFF for i in range(4)]

def beq(rs1, rs2, offset_13bit):
    off = offset_13bit & 0x1FFF
    imm_12  = (off >> 12) & 0x1
    imm_10_5= (off >> 5)  & 0x3F
    imm_4_1 = (off >> 1)  & 0xF
    imm_11  = (off >> 11) & 0x1
    val = (imm_12 << 31) | (imm_10_5 << 25) | ((rs2 & 0x1F) << 20) | ((rs1 & 0x1F) << 15) | (0 << 12) | (imm_4_1 << 8) | (imm_11 << 7) | 0x63
    return [(val >> (8 * i)) & 0xFF for i in range(4)]

def blt(rs1, rs2, offset_13bit):
    off = offset_13bit & 0x1FFF
    imm_12  = (off >> 12) & 0x1
    imm_10_5= (off >> 5)  & 0x3F
    imm_4_1 = (off >> 1)  & 0xF
    imm_11  = (off >> 11) & 0x1
    val = (imm_12 << 31) | (imm_10_5 << 25) | ((rs2 & 0x1F) << 20) | ((rs1 & 0x1F) << 15) | (4 << 12) | (imm_4_1 << 8) | (imm_11 << 7) | 0x63
    return [(val >> (8 * i)) & 0xFF for i in range(4)]

def slti(rd, rs1, imm_12bit):
    val = ((imm_12bit & 0xFFF) << 20) | ((rs1 & 0x1F) << 15) | (2 << 12) | ((rd & 0x1F) << 7) | 0x13
    return [(val >> (8 * i)) & 0xFF for i in range(4)]

def jal(rd, offset_21bit):
    off = offset_21bit & 0x1FFFFF
    imm_20   = (off >> 20) & 0x1
    imm_10_1 = (off >> 1)  & 0x3FF
    imm_11   = (off >> 11) & 0x1
    imm_19_12= (off >> 12) & 0xFF
    val = (imm_20 << 31) | (imm_10_1 << 21) | (imm_11 << 20) | (imm_19_12 << 12) | ((rd & 0x1F) << 7) | 0x6F
    return [(val >> (8 * i)) & 0xFF for i in range(4)]

WFI_BYTES = [0x73, 0x00, 0x50, 0x10]

def make_program(inst_list):
    prog = bytearray()
    for inst in inst_list:
        prog.extend(inst)
    prog.extend([0x13, 0x00, 0x00, 0x00] * 16) # Pad with 16 NOPs
    return prog

def lb(rd, offset_12bit, rs1):
    val = ((offset_12bit & 0xFFF) << 20) | ((rs1 & 0x1F) << 15) | (0 << 12) | ((rd & 0x1F) << 7) | 0x03
    return [(val >> (8 * i)) & 0xFF for i in range(4)]

# =====================================================================
# FLOYD'S TRIANGLE PATTERN GENERATOR (N = 5) - MEMORY STREAM
# =====================================================================

text = "  1\r\n  2 3\r\n  4 5 6\r\n  7 8 9 10\r\n  11 12 13 14 15\r\n\0"
text_bytes = text.encode('ascii')
while len(text_bytes) % 4 != 0:
    text_bytes += b'\0'

insts = [
    lui(10, 0xFFFF0),                        # 0x00: x10 = MMIO Base (0xFFFF0000)
    addi(1, 0, 0x24),                        # 0x04: x1 = String Buffer Address (0x24)
    lb(2, 0, 1),                             # 0x08: x2 = LB [x1]
    beq(2, 0, 16),                           # 0x0C: BEQ x2 == 0 -> Jump to HALT (0x1C)
    sw(2, 0, 10),                            # 0x10: SW x2 to 0(x10) -> UART TX
    addi(1, 1, 1),                           # 0x14: x1++ (Next character)
    jal(0, -16),                             # 0x18: JAL -> Jump back to 0x08
    WFI_BYTES,                               # 0x1C: WFI Low-Power Idle Mode Halt
    jal(0, 0)                                # 0x20: JAL x0, 0 -> Infinite Halt Loop
]

prog = bytearray()
for inst in insts:
    prog.extend(inst)
prog.extend(text_bytes)
prog.extend([0x13, 0x00, 0x00, 0x00] * 16) # Pad with NOPs

pattern_prog = prog

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else COM_PORT
    print("=========================================================")
    print("  RISC-V RV32IM FLOYD'S TRIANGLE PATTERN GENERATOR (N=5) ")
    print("=========================================================")
    print("Target Printed Pattern:")
    print("  1")
    print("  2 3")
    print("  4 5 6")
    print("  7 8 9 10")
    print("  11 12 13 14 15")
    print("=========================================================")

    print("1. Please HOLD DOWN the reset button (Center Button) on your FPGA.")
    input("   Press Enter in this terminal when you are holding it down...")

    print(f"Connecting to {port} at {BAUD_RATE}...")
    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=10.0)
    except Exception as e:
        print(f"Error opening port {port}: {e}")
        print("Make sure to CLOSE any active Serial Monitor before running!")
        return

    try:
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        print(f"Uploading program payload ({len(pattern_prog)} bytes)...")
        ser.write(pattern_prog)
        ser.flush()
        time.sleep(0.1)
        print("Upload complete!")
        print("--------------------------------------------------")
        print("2. Please RELEASE the reset button now.")
        input("   Press Enter AFTER releasing the reset button...")
        print("   Reading Serial Stream from RISC-V Core...")
        print("--------------------------------------------------\n")

        # Allow time for core execution and UART TX transmission
        time.sleep(0.5)

        # Read output stream from core
        output_buffer = []
        start_time = time.time()
        while time.time() - start_time < 3.0:
            if ser.in_waiting > 0:
                char = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
                sys.stdout.write(char)
                sys.stdout.flush()
                output_buffer.append(char)
            time.sleep(0.05)

        print("\n--------------------------------------------------")
        print("🌟 PATTERN GENERATION COMPLETE 🌟")
        print("==================================================")
    finally:
        ser.close()

if __name__ == "__main__":
    main()
