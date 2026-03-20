"""
RS03 Connection Test
Attempts to connect to the RobStride CAN2USB debugger
and communicate with the RS03 motor on CAN ID 1.

Usage: python test_connection.py
"""

import can
import time
import sys

COM_PORT = "COM5"
MOTOR_ID = 1

# ──────────────────────────────────────────────
# Attempt 1: Try SLCAN interface
# The RobStride CAN2USB *might* speak SLCAN protocol
# ──────────────────────────────────────────────
def try_slcan():
    print(f"Trying SLCAN interface on {COM_PORT}...")
    try:
        bus = can.Bus(
            interface='slcan',
            channel=COM_PORT,
            bitrate=1000000,
            ttyBaudrate=921600,  # RobStride CAN2USB runs at 921600 baud
        )
        print("  SLCAN bus opened successfully!")
        print("  Listening for CAN frames for 3 seconds...")

        # Try to receive any frames (the motor might be sending heartbeats)
        start = time.time()
        frame_count = 0
        while time.time() - start < 3:
            msg = bus.recv(timeout=0.5)
            if msg:
                frame_count += 1
                print(f"  Received frame: ID=0x{msg.arbitration_id:08X} "
                      f"Data={msg.data.hex()} DLC={msg.dlc}")

        if frame_count == 0:
            print("  No frames received. Motor may not be powered or SLCAN may not be the right protocol.")
        else:
            print(f"  Received {frame_count} frames!")

        bus.shutdown()
        return True
    except Exception as e:
        print(f"  SLCAN failed: {e}")
        return False


# ──────────────────────────────────────────────
# Attempt 2: Try serial interface directly
# ──────────────────────────────────────────────
def try_serial():
    print(f"\nTrying Serial interface on {COM_PORT}...")
    try:
        bus = can.Bus(
            interface='serial',
            channel=COM_PORT,
            bitrate=1000000,
            baudrate=921600,
        )
        print("  Serial bus opened successfully!")
        print("  Listening for CAN frames for 3 seconds...")

        start = time.time()
        frame_count = 0
        while time.time() - start < 3:
            msg = bus.recv(timeout=0.5)
            if msg:
                frame_count += 1
                print(f"  Received frame: ID=0x{msg.arbitration_id:08X} "
                      f"Data={msg.data.hex()} DLC={msg.dlc}")

        if frame_count == 0:
            print("  No frames received.")
        else:
            print(f"  Received {frame_count} frames!")

        bus.shutdown()
        return True
    except Exception as e:
        print(f"  Serial failed: {e}")
        return False


# ──────────────────────────────────────────────
# Attempt 3: Raw serial probe
# Just open the port and see what comes back
# ──────────────────────────────────────────────
def try_raw_serial():
    print(f"\nTrying raw serial probe on {COM_PORT}...")
    try:
        import serial
        ser = serial.Serial(
            port=COM_PORT,
            baudrate=921600,
            timeout=1,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
        )
        print(f"  Serial port opened: {ser.name}")

        # Flush any stale data
        ser.reset_input_buffer()

        # Try sending an AT command to see if debugger responds
        print("  Sending AT test command...")
        ser.write(b'AT\r\n')
        time.sleep(0.5)

        # Read whatever comes back
        available = ser.in_waiting
        if available > 0:
            data = ser.read(available)
            print(f"  Response ({available} bytes): {data}")
            print(f"  Hex: {data.hex()}")
        else:
            print("  No response to AT command.")

        # Now just listen for raw data (motor might be broadcasting)
        print("  Listening for raw data for 3 seconds...")
        ser.reset_input_buffer()
        start = time.time()
        total_bytes = b''
        while time.time() - start < 3:
            if ser.in_waiting > 0:
                chunk = ser.read(ser.in_waiting)
                total_bytes += chunk
                time.sleep(0.05)
            else:
                time.sleep(0.1)

        if total_bytes:
            print(f"  Received {len(total_bytes)} bytes of raw data")
            print(f"  First 64 bytes hex: {total_bytes[:64].hex()}")
            print(f"  First 64 bytes raw: {total_bytes[:64]}")
        else:
            print("  No raw data received.")

        ser.close()
        return True
    except Exception as e:
        print(f"  Raw serial failed: {e}")
        return False


if __name__ == "__main__":
    print("=" * 60)
    print("RS03 Connection Probe")
    print(f"COM Port: {COM_PORT} | Motor CAN ID: {MOTOR_ID}")
    print("Make sure the motor is POWERED ON and CAN2USB is plugged in!")
    print("=" * 60)
    print()

    # Make sure MotorStudio is CLOSED before running this!
    print("[!] IMPORTANT: Close MotorStudio first! Only one app")
    print("   can use the COM port at a time.")
    print()

    input("Press Enter when ready...")
    print()

    try_slcan()
    try_serial()
    try_raw_serial()

    print()
    print("=" * 60)
    print("Done! Share the output above and we'll determine the")
    print("right approach for your CAN2USB debugger.")
    print("=" * 60)
