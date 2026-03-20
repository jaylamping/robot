"""
Test communication with RS03 at CAN ID 127 (detected by MotorStudio).
Uses the AT-framed serial protocol from RobStride CAN-USB source.

CLOSE MOTORSTUDIO BEFORE RUNNING THIS!
"""

import serial
import struct
import time

COM_PORT = "COM5"
BAUD_RATE = 921600
MOTOR_ID = 127
HOST_CAN_ID = 0xAA


def build_at_frame(arb_id, data):
    wire_id = (arb_id << 3) | 0x04
    return b'AT' + struct.pack('>I', wire_id) + bytes([len(data)]) + data + b'\r\n'


def send_and_recv(ser, name, arb_id, data, wait=1.0):
    frame = build_at_frame(arb_id, data)
    print(f"\n  [{name}] arb_id=0x{arb_id:08X}")
    print(f"  TX ({len(frame)} bytes): {frame.hex()}")
    ser.reset_input_buffer()
    ser.write(frame)
    ser.flush()
    time.sleep(wait)

    avail = ser.in_waiting
    if avail > 0:
        raw = ser.read(avail)
        print(f"  RX ({len(raw)} bytes): {raw.hex()}")
        for i, b in enumerate(raw):
            ch = chr(b) if 32 <= b < 127 else '.'
            print(f"    [{i:2d}] 0x{b:02X} ({b:3d}) {ch}")
        return raw
    else:
        print(f"  RX: (nothing)")
        return None


def main():
    print("=" * 60)
    print(f"RS03 Test - Motor CAN ID {MOTOR_ID}")
    print(f"COM: {COM_PORT} @ {BAUD_RATE} baud")
    print("=" * 60)

    ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=2)
    ser.dtr = True
    ser.rts = True
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    time.sleep(0.2)

    # Enable command: msg_type=3
    enable_id = (3 << 24) | (HOST_CAN_ID << 8) | MOTOR_ID
    resp = send_and_recv(ser, "ENABLE", enable_id, bytes(8))

    if resp is None:
        print("\n--- No response. Trying alternate baud rates ---")
        ser.close()
        for baud in [115200, 460800, 500000, 256000, 2000000]:
            ser = serial.Serial(COM_PORT, baud, timeout=1)
            ser.dtr = True
            ser.rts = True
            ser.reset_input_buffer()
            time.sleep(0.1)
            resp = send_and_recv(ser, f"ENABLE@{baud}", enable_id, bytes(8), wait=0.5)
            ser.close()
            if resp:
                print(f"\n>>> GOT RESPONSE AT {baud} BAUD! <<<")
                break
        else:
            print("\nNo response at any baud rate.")
            print("Is MotorStudio closed? Is the motor powered?")
            return

    # If we got a response, also try disable
    if resp:
        if ser.is_open is False:
            ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=2)
        disable_id = (4 << 24) | (HOST_CAN_ID << 8) | MOTOR_ID
        send_and_recv(ser, "DISABLE", disable_id, bytes(8))

    if ser.is_open:
        ser.close()
    print("\nDone.")


if __name__ == "__main__":
    main()
