"""
RS03 CAN2USB Direct Protocol Test
Uses the AT-framed serial protocol discovered from RobStride's
CAN-USB-data-conversion source code.

Frame format (TX and RX):
  'A' 'T' [4-byte shifted CAN ID] [1-byte len] [data...] '\r' '\n'

The 29-bit extended CAN arbitration ID is encoded as:
  wire_id = (arbitration_id << 3) | 0x04
and sent big-endian in bytes 2-5.

Usage: python test_can2usb.py
"""

import serial
import struct
import time

COM_PORT = "COM5"
BAUD_RATE = 921600
MOTOR_ID = 1
HOST_CAN_ID = 0xAA


def build_frame(arbitration_id: int, data: bytes) -> bytes:
    """Build an AT-framed serial packet for the CAN2USB debugger."""
    wire_id = (arbitration_id << 3) | 0x00000004
    frame = b'AT'
    frame += struct.pack('>I', wire_id)
    frame += bytes([len(data)])
    frame += data
    frame += b'\r\n'
    return frame


def parse_frame(raw: bytes):
    """Parse an AT-framed response from the CAN2USB debugger.
    Returns (arbitration_id, data) or None if invalid."""
    if len(raw) < 9:
        return None
    if raw[0:2] != b'AT':
        return None

    wire_id = struct.unpack('>I', raw[2:6])[0]
    arbitration_id = wire_id >> 3
    data_len = raw[6]
    data = raw[7:7 + data_len]
    return arbitration_id, data


def make_enable_arb_id(host_id: int, motor_id: int) -> int:
    """Build the 29-bit arbitration ID for an Enable command.
    Format: [msg_type(5b)] [host_id(8b)] [motor_id(8b)]
    Enable = msg_type 3"""
    return (3 << 24) | (host_id << 8) | motor_id


def decode_arb_id(arb_id: int):
    """Decode the 29-bit arbitration ID fields."""
    msg_type = (arb_id >> 24) & 0x1F
    data_field = (arb_id >> 8) & 0xFFFF
    motor_id = (arb_id >> 8) & 0xFF
    host_id = arb_id & 0xFF
    error_bits = (arb_id >> 16) & 0x3F
    mode_bits = (arb_id >> 22) & 0x03
    return {
        'msg_type': msg_type,
        'motor_id': motor_id,
        'host_id': host_id,
        'error_bits': error_bits,
        'mode_bits': mode_bits,
    }


def main():
    print("=" * 60)
    print("RS03 CAN2USB Direct Protocol Test")
    print(f"COM Port: {COM_PORT} | Baud: {BAUD_RATE}")
    print(f"Motor CAN ID: {MOTOR_ID} | Host ID: 0x{HOST_CAN_ID:02X}")
    print("=" * 60)
    print()

    ser = serial.Serial(
        port=COM_PORT,
        baudrate=BAUD_RATE,
        timeout=1,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
    )
    print(f"Serial port opened: {ser.name}")
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    time.sleep(0.1)

    # Build Enable Motor command
    arb_id = make_enable_arb_id(HOST_CAN_ID, MOTOR_ID)
    data = bytes(8)  # 8 zero bytes for enable
    frame = build_frame(arb_id, data)

    print(f"\nSending ENABLE command to motor {MOTOR_ID}...")
    print(f"  Arbitration ID: 0x{arb_id:08X}")
    print(f"  Frame hex: {frame.hex()}")
    print(f"  Frame raw: {frame}")

    ser.write(frame)
    ser.flush()

    # Wait for response
    print("\nWaiting for response...")
    time.sleep(0.5)

    available = ser.in_waiting
    if available > 0:
        raw = ser.read(available)
        print(f"  Received {len(raw)} bytes")
        print(f"  Hex: {raw.hex()}")
        print(f"  Raw: {raw}")

        # Try to parse as AT frame
        result = parse_frame(raw)
        if result:
            resp_arb_id, resp_data = result
            fields = decode_arb_id(resp_arb_id)
            print(f"\n  Parsed response:")
            print(f"    Arbitration ID: 0x{resp_arb_id:08X}")
            print(f"    Msg type: {fields['msg_type']}")
            print(f"    Motor ID: {fields['motor_id']}")
            print(f"    Host ID: 0x{fields['host_id']:02X}")
            print(f"    Mode: {fields['mode_bits']}")
            print(f"    Errors: {fields['error_bits']}")
            print(f"    Data: {resp_data.hex()}")
            print("\n  >>> MOTOR RESPONDED! Protocol is working! <<<")
        else:
            print("  Could not parse as AT frame - trying alternate parse...")
            # Show byte-by-byte breakdown
            for i, b in enumerate(raw):
                print(f"    [{i:2d}] 0x{b:02X} ({b:3d}) {chr(b) if 32 <= b < 127 else '.'}")
    else:
        print("  No response received.")
        print()
        print("Troubleshooting:")
        print("  1. Is the motor powered on?")
        print("  2. Is MotorStudio fully closed?")
        print("  3. Is the CAN2USB debugger's green LED on?")

    # Clean up - send disable just in case
    print("\nSending DISABLE command...")
    disable_arb_id = (4 << 24) | (HOST_CAN_ID << 8) | MOTOR_ID
    disable_frame = build_frame(disable_arb_id, bytes(8))
    ser.write(disable_frame)
    ser.flush()
    time.sleep(0.3)

    available = ser.in_waiting
    if available > 0:
        raw = ser.read(available)
        print(f"  Disable response: {raw.hex()}")

    ser.close()
    print("\nDone.")


if __name__ == "__main__":
    main()
