"""
High-level RS03 motor control API.

Wraps robstride.Client + RobStrideBus into a clean interface
for controlling RS03 actuators on the robot arm.

Usage:
    from hw import Motor

    m = Motor("COM5", can_id=127)
    m.set_zero()
    m.move_to(1.57)       # go to 90 degrees
    m.spin(5.0)           # 5 rad/s
    m.stop()
    m.close()
"""

import time
import math
from dataclasses import dataclass
from typing import Optional

from hw.robstride_bus import RobStrideBus
from robstride import Client, RunMode
from robstride.client import FeedbackResp, MotorMode


# RS03 limits (from RobStride documentation)
RS03_MAX_TORQUE = 60.0    # N*m peak
RS03_MAX_SPEED = 50.0     # rad/s
RS03_MAX_CURRENT = 30.0   # A


@dataclass
class MotorState:
    angle: float       # radians
    velocity: float    # rad/s
    torque: float      # N*m
    temperature: float # Celsius
    mode: str
    errors: list


class Motor:
    """High-level controller for a single RS03 motor."""

    def __init__(
        self,
        port: str,
        can_id: int,
        host_id: int = 0xAA,
        baud: int = 921600,
        bus: Optional[RobStrideBus] = None,
    ):
        self.can_id = can_id
        self._owns_bus = bus is None
        self._bus = bus or RobStrideBus(port, ttyBaudrate=baud)
        self._client = Client(self._bus, host_can_id=host_id)
        self._enabled = False

    # ── Lifecycle ────────────────────────────────────────────

    def enable(self) -> MotorState:
        """Enable the motor. Must be called before any motion commands."""
        fb = self._client.enable(self.can_id)
        self._enabled = True
        return self._to_state(fb)

    def disable(self) -> MotorState:
        """Disable the motor (coast to stop)."""
        fb = self._client.disable(self.can_id)
        self._enabled = False
        return self._to_state(fb)

    def stop(self) -> MotorState:
        """Alias for disable."""
        return self.disable()

    def close(self):
        """Disable motor and release the serial port."""
        if self._enabled:
            try:
                self.disable()
            except Exception:
                pass
        if self._owns_bus:
            self._bus.shutdown()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    # ── Configuration ────────────────────────────────────────

    def set_zero(self):
        """Set the current position as mechanical zero."""
        self._client.bus.send(self._client._rs_msg(
            type('', (), {'value': 6})(),  # ZeroPos = 6
            self._client.host_can_id,
            self.can_id,
            [0, 0, 0, 0, 0, 0, 0, 0],
        ))
        self._client._recv()

    def set_run_mode(self, mode: RunMode):
        """Set the control mode: Position, Speed, Current, or Operation (MIT)."""
        self._client.write_param(self.can_id, 'run_mode', mode)

    def set_position_gain(self, kp: float):
        self._client.write_param(self.can_id, 'loc_kp', kp)

    def set_speed_gain(self, kp: float, ki: float = 0.02):
        self._client.write_param(self.can_id, 'spd_kp', kp)
        self._client.write_param(self.can_id, 'spd_ki', ki)

    def set_speed_limit(self, limit: float):
        """Set max speed in rad/s (applies in position mode)."""
        self._client.write_param(self.can_id, 'limit_spd', min(limit, RS03_MAX_SPEED))

    def set_torque_limit(self, limit: float):
        """Set max torque in N*m."""
        self._client.write_param(self.can_id, 'limit_torque', min(limit, RS03_MAX_TORQUE))

    # ── Motion commands ──────────────────────────────────────

    def move_to(self, position_rad: float, speed_limit: Optional[float] = None):
        """Move to an absolute position in radians (position mode).

        The motor must be enabled first. This sets position mode
        and sends the target. Returns immediately; the motor
        moves asynchronously.
        """
        self._ensure_enabled()
        self.set_run_mode(RunMode.Position)
        if speed_limit is not None:
            self.set_speed_limit(speed_limit)
        self._client.write_param(self.can_id, 'loc_ref', position_rad)

    def move_to_deg(self, degrees: float, speed_limit: Optional[float] = None):
        """Move to an absolute position in degrees."""
        self.move_to(math.radians(degrees), speed_limit)

    def spin(self, velocity_rad_s: float):
        """Spin at a constant velocity in rad/s (speed mode)."""
        self._ensure_enabled()
        self.set_run_mode(RunMode.Speed)
        self._client.write_param(self.can_id, 'spd_ref', velocity_rad_s)

    def set_torque(self, torque_nm: float):
        """Apply a raw torque in N*m (current/torque mode)."""
        self._ensure_enabled()
        self.set_run_mode(RunMode.Current)
        self._client.write_param(self.can_id, 'iq_ref', torque_nm)

    # ── Telemetry ────────────────────────────────────────────

    def read_state(self) -> MotorState:
        """Read current motor state (position, velocity, torque, temp)."""
        fb = self._client.enable(self.can_id)
        return self._to_state(fb)

    def read_param(self, name: str):
        """Read a raw parameter by name."""
        return self._client.read_param(self.can_id, name)

    def read_voltage(self) -> float:
        return self._client.read_param(self.can_id, 'vbus')

    def read_position(self) -> float:
        return self._client.read_param(self.can_id, 'mechpos')

    def read_velocity(self) -> float:
        return self._client.read_param(self.can_id, 'mechvel')

    # ── Helpers ──────────────────────────────────────────────

    def wait_until_at(self, target_rad: float, tolerance: float = 0.05, timeout: float = 10.0):
        """Block until motor is within tolerance of target position."""
        start = time.time()
        while time.time() - start < timeout:
            pos = self.read_position()
            if abs(pos - target_rad) < tolerance:
                return True
            time.sleep(0.05)
        return False

    def _ensure_enabled(self):
        if not self._enabled:
            self.enable()

    @staticmethod
    def _to_state(fb: FeedbackResp) -> MotorState:
        return MotorState(
            angle=fb.angle,
            velocity=fb.velocity,
            torque=fb.torque,
            temperature=fb.temp,
            mode=fb.mode.name,
            errors=[e.name for e in fb.errors],
        )
