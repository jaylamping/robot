"""
Quick smoke test of the high-level Motor API.
Enables, reads state, reads voltage, then disables.
"""

from hw import Motor

def main():
    print("Motor API smoke test")
    print("=" * 40)

    with Motor("COM5", can_id=127) as m:
        state = m.enable()
        print(f"Enabled:")
        print(f"  Position: {state.angle:.3f} rad ({state.angle * 57.2958:.1f} deg)")
        print(f"  Velocity: {state.velocity:.3f} rad/s")
        print(f"  Torque:   {state.torque:.3f} Nm")
        print(f"  Temp:     {state.temperature:.1f} C")
        print(f"  Mode:     {state.mode}")
        print(f"  Errors:   {state.errors}")

        voltage = m.read_voltage()
        print(f"\n  Bus voltage: {voltage:.1f} V")

        pos = m.read_position()
        print(f"  Mech position: {pos:.3f} rad")

        vel = m.read_velocity()
        print(f"  Mech velocity: {vel:.4f} rad/s")

        run_mode = m.read_param('run_mode')
        print(f"  Run mode: {run_mode}")

        state = m.disable()
        print(f"\nDisabled. Mode: {state.mode}")

    print("\nAll good!")


if __name__ == "__main__":
    main()
