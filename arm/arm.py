"""
4-DOF arm controller.

Orchestrates 4 RS03 motors (shoulder pitch, shoulder roll,
upper arm yaw, elbow pitch) as a single coordinated arm.

Future: forward/inverse kinematics, trajectory interpolation,
collision limits, synchronized multi-joint moves.
"""

from dataclasses import dataclass
from typing import Optional

from hw import Motor, RobStrideBus


@dataclass
class ArmConfig:
    port: str
    shoulder_pitch_id: int
    shoulder_roll_id: Optional[int] = None
    upper_arm_yaw_id: Optional[int] = None
    elbow_pitch_id: Optional[int] = None
    host_id: int = 0xAA


class Arm:
    """Controls a 4-DOF coaxial robot arm."""

    def __init__(self, config: ArmConfig):
        self._bus = RobStrideBus(config.port)
        self._motors = {}

        joint_ids = {
            'shoulder_pitch': config.shoulder_pitch_id,
            'shoulder_roll': config.shoulder_roll_id,
            'upper_arm_yaw': config.upper_arm_yaw_id,
            'elbow_pitch': config.elbow_pitch_id,
        }

        for name, can_id in joint_ids.items():
            if can_id is not None:
                self._motors[name] = Motor(
                    port=config.port,
                    can_id=can_id,
                    host_id=config.host_id,
                    bus=self._bus,
                )

    def enable_all(self):
        for name, motor in self._motors.items():
            motor.enable()

    def disable_all(self):
        for name, motor in self._motors.items():
            motor.disable()

    def set_joint(self, joint_name: str, position_rad: float, speed_limit: Optional[float] = None):
        """Move a single joint to a position."""
        if joint_name not in self._motors:
            raise ValueError(f"Joint '{joint_name}' not configured (no CAN ID)")
        self._motors[joint_name].move_to(position_rad, speed_limit)

    def get_joint_positions(self) -> dict:
        """Read all joint positions."""
        positions = {}
        for name, motor in self._motors.items():
            positions[name] = motor.read_position()
        return positions

    def close(self):
        self.disable_all()
        self._bus.shutdown()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
