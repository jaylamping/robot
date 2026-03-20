"""
Integration test: RobStrideBus + robstride.Client
Connects to the RS03 at CAN ID 127 and exercises the full API.

CLOSE MOTORSTUDIO BEFORE RUNNING!
"""

from hw.robstride_bus import RobStrideBus
from robstride import Client, RunMode
import time

COM_PORT = "COM5"
MOTOR_ID = 127


def main():
    print("=" * 60)
    print("robstride.Client Integration Test")
    print(f"COM: {COM_PORT} | Motor ID: {MOTOR_ID}")
    print("=" * 60)

    bus = RobStrideBus(COM_PORT)
    client = Client(bus, host_can_id=0xAA)

    # 1. Enable
    print("\n[1] Enabling motor...")
    fb = client.enable(MOTOR_ID)
    print(f"    Mode: {fb.mode}")
    print(f"    Angle: {fb.angle:.3f} rad ({fb.angle * 57.2958:.1f} deg)")
    print(f"    Velocity: {fb.velocity:.3f} rad/s")
    print(f"    Torque: {fb.torque:.3f} Nm")
    print(f"    Temp: {fb.temp:.1f} C")
    print(f"    Errors: {fb.errors}")

    # 2. Read parameters
    print("\n[2] Reading parameters...")
    run_mode = client.read_param(MOTOR_ID, 'run_mode')
    print(f"    Run mode: {run_mode}")

    vbus = client.read_param(MOTOR_ID, 'vbus')
    print(f"    Bus voltage: {vbus:.1f} V")

    mechpos = client.read_param(MOTOR_ID, 'mechpos')
    print(f"    Mechanical position: {mechpos:.3f} rad")

    # 3. Read current PID gains
    print("\n[3] Current gains...")
    loc_kp = client.read_param(MOTOR_ID, 'loc_kp')
    spd_kp = client.read_param(MOTOR_ID, 'spd_kp')
    spd_ki = client.read_param(MOTOR_ID, 'spd_ki')
    print(f"    loc_kp: {loc_kp}")
    print(f"    spd_kp: {spd_kp}")
    print(f"    spd_ki: {spd_ki}")

    # 4. Disable
    print("\n[4] Disabling motor...")
    fb = client.disable(MOTOR_ID)
    print(f"    Mode: {fb.mode}")
    print(f"    Angle: {fb.angle:.3f} rad")

    bus.shutdown()
    print("\nDone! Full communication stack working.")


if __name__ == "__main__":
    main()
