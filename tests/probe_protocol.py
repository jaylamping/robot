"""
Comprehensive CAN2USB protocol probe.
Tries multiple interfaces and baud rates to find what the
RobStride CAN2USB debugger speaks.
"""

import serial
import struct
import time
import sys

COM_PORT = "COM5"
MOTOR_ID = 1
HOST_CAN_ID = 0xAA


def try_robotell_interface(baud):
    """Try python-can's robotell interface (0xAA framing)."""
    print(f"\n--- Trying robotell interface at {baud} baud ---")
    try:
        import can
        bus = can.Bus(
            interface='robotell',
            channel=COM_PORT,
            ttyBaudrate=baud,
            bitrate=1000000,
        )
        print(f"  Robotell bus opened! Serial: {bus.channel_info}")

        arb_id = (3 << 24) | (HOST_CAN_ID << 8) | MOTOR_ID
        msg = can.Message(arbitration_id=arb_id, data=bytes(8),
                          is_extended_id=True, dlc=8)
        print(f"  Sending enable command...")
        bus.send(msg)

        resp = bus.recv(timeout=2)
        if resp:
            print(f"  GOT RESPONSE: ID=0x{resp.arbitration_id:08X} "
                  f"Data={resp.data.hex()}")
            bus.shutdown()
            return True
        else:
            print(f"  No response.")
        bus.shutdown()
    except Exception as e:
        print(f"  Failed: {e}")
    return False


def try_at_protocol(baud):
    """Try the AT + \\r\\n framing from RobStride CAN-USB source."""
    print(f"\n--- Trying AT protocol at {baud} baud ---")
    try:
        ser = serial.Serial(port=COM_PORT, baudrate=baud, timeout=1)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        time.sleep(0.05)

        arb_id = (3 << 24) | (HOST_CAN_ID << 8) | MOTOR_ID
        wire_id = (arb_id << 3) | 0x04
        frame = b'AT' + struct.pack('>I', wire_id) + b'\x08' + bytes(8) + b'\r\n'

        print(f"  Sending: {frame.hex()}")
        ser.write(frame)
        ser.flush()
        time.sleep(0.5)

        if ser.in_waiting > 0:
            raw = ser.read(ser.in_waiting)
            print(f"  GOT RESPONSE ({len(raw)} bytes): {raw.hex()}")
            ser.close()
            return True
        else:
            print(f"  No response.")
        ser.close()
    except Exception as e:
        print(f"  Failed: {e}")
    return False


def try_raw_baud_detect(baud):
    """Open port at given baud, send nothing, just listen for any data."""
    print(f"\n--- Listening at {baud} baud for 1s ---")
    try:
        ser = serial.Serial(port=COM_PORT, baudrate=baud, timeout=1)
        ser.reset_input_buffer()
        time.sleep(1)
        if ser.in_waiting > 0:
            raw = ser.read(ser.in_waiting)
            print(f"  Received {len(raw)} bytes: {raw[:32].hex()}")
            ser.close()
            return True
        else:
            print(f"  Silence.")
        ser.close()
    except Exception as e:
        print(f"  Failed: {e}")
    return False


def try_gs_usb():
    """Try gs_usb interface (bypasses COM port entirely)."""
    print(f"\n--- Trying gs_usb interface (no COM port) ---")
    try:
        import can
        bus = can.Bus(interface='gs_usb', channel=0, bitrate=1000000)
        print(f"  gs_usb opened!")
        bus.shutdown()
        return True
    except Exception as e:
        print(f"  Failed: {e}")
    return False


if __name__ == "__main__":
    print("=" * 60)
    print("CAN2USB Protocol Probe")
    print(f"COM Port: {COM_PORT}")
    print("=" * 60)

    bauds = [115200, 921600, 460800, 500000, 256000, 2000000]

    # 1. Try gs_usb first (in case it's a dual-mode device)
    try_gs_usb()

    # 2. Try robotell at common baud rates
    for b in bauds:
        if try_robotell_interface(b):
            print(f"\n>>> SUCCESS with robotell at {b} baud! <<<")
            sys.exit(0)

    # 3. Try AT protocol at common baud rates
    for b in bauds:
        if try_at_protocol(b):
            print(f"\n>>> SUCCESS with AT protocol at {b} baud! <<<")
            sys.exit(0)

    # 4. Just listen at each baud rate
    print("\n\n--- Raw baud rate detection ---")
    for b in bauds:
        try_raw_baud_detect(b)

    print("\n" + "=" * 60)
    print("No protocol matched. The CAN2USB may need a different approach.")
    print("=" * 60)
