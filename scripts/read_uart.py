import serial
import time
import sys

PORT = "COM3"
BAUD = 115200

print(f"Opening {PORT} @ {BAUD} baud...")
try:
    ser = serial.Serial(PORT, BAUD, timeout=0.1)
    print("Listening for FPGA UART output...")
    print(">>> Press the RESET button on your FPGA board now! <<<")
    print("(Press Ctrl+C to stop listening)\n")
    
    start_time = time.time()
    received_any = False
    
    while time.time() - start_time < 20.0:
        if ser.in_waiting > 0:
            byte = ser.read(1)
            received_any = True
            char = chr(byte[0]) if (32 <= byte[0] <= 126 or byte[0] in (10, 13)) else f"\\x{byte[0]:02x}"
            print(f"[UART RECV] Byte: 0x{byte[0]:02X} ({byte[0]}) -> Char: '{char}'")
            # If we see newline or char, keep reading any trailing bytes
            time.sleep(0.05)
            while ser.in_waiting > 0:
                b = ser.read(1)
                c = chr(b[0]) if (32 <= b[0] <= 126 or b[0] in (10, 13)) else f"\\x{b[0]:02x}"
                print(f"[UART RECV] Byte: 0x{b[0]:02X} ({b[0]}) -> Char: '{c}'")
            break
        time.sleep(0.01)
        
    if not received_any:
        print("\nNo bytes received within 20s. Did you press the reset button on the FPGA?")
    else:
        print("\nTest completed successfully!")
    ser.close()

except KeyboardInterrupt:
    print("\nExited by user.")
    try:
        ser.close()
    except:
        pass
except Exception as e:
    print("\nSerial error:", e)
