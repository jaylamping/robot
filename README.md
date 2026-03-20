# Humanoid Robot

A fully articulated 5-foot humanoid robot with 28+ degrees of freedom, powered by high-torque brushless actuators and controlled entirely through Python. The skeleton is built from 2020 aluminum extrusion with 3D-printed joint housings, driven by RobStride RS03 servo actuators delivering 60 N·m of peak torque per joint through 9:1 planetary gearboxes. Every motor on the body talks over CAN bus at 1 Mbps, commanded from a single Python control stack that handles everything from raw serial framing to coordinated multi-joint trajectory execution.

This is a from-scratch build — custom mechanical design, custom wiring, reverse-engineered motor protocols, and a ground-up software architecture designed to scale from one motor on a bench to a walking, gesturing humanoid.

## Development Roadmap

### Phase 1 — Arms (in progress)
8 DOF total (4 per arm): shoulder pitch, shoulder roll, upper arm yaw, elbow pitch. Coaxial RS03 layout with pitch motors mounted inside the torso and roll/yaw motors in 3D-printed housings along the arm. First milestone is a coordinated arm wave demonstration.

### Phase 2 — Legs
12 DOF total (6 per leg): hip yaw, hip roll, hip pitch, knee pitch, ankle pitch, ankle roll. The hip assembly mirrors the shoulder's coaxial design with three stacked actuators. Knee is a single high-torque pitch joint. Ankle uses two actuators in a differential configuration for combined pitch/roll authority. Standing balance comes first, then walking gait.

### Phase 3 — Wrists & Hands
6+ DOF total (3+ per side): wrist pitch, wrist yaw, wrist roll, plus articulated grip. Likely a mix of smaller RobStride actuators for the wrist and tendon-driven or linkage-based fingers for grasping.

### Phase 4 — Head
2-3 DOF: neck pan, neck tilt, and possibly a jaw or visor mechanism. Camera integration for vision. Expressiveness through motion rather than a face display.

**Full robot target: 28-31 DOF**

## Architecture

```
hw/                        Hardware drivers & bus adapters
  robstride_bus.py           CAN2USB AT serial protocol (python-can Bus)
  motor.py                   Single motor control API

arm/                       Arm coordination
  arm.py                     Multi-joint arm controller
  left/ right/               Per-arm configs

torso/                     Torso frame & structural control
waist/                     Waist rotation (OpenQDD actuator)
head/                      Head pan/tilt, cameras, expression
legs/                      Leg coordination
  left/ right/               Per-leg configs

config/
  robot.yaml                 CAN IDs, joint limits, physical params

demos/                     Runnable demonstrations
tests/                     Hardware probes & integration tests
```

## Hardware

| Component | Spec |
|---|---|
| Arm motors | RobStride RS03 (60 N·m peak, 9:1 planetary, 48V) |
| Waist motor | OpenQDD |
| CAN adapter | RobStride CAN2USB debugger (CH340, 921600 baud) |
| Power | 24V/1200W bench PSU (48V battery for mobile) |
| Torso frame | 2020 aluminum extrusion, 460 x 200 x 160mm |
| Joint housings | PETG 3D-printed with integrated mounting tabs |
| CAN bus | 1 Mbps standard CAN (RS03), CAN-FD (Moteus) |

## CAN2USB Protocol

The RobStride CAN2USB debugger uses a proprietary AT-framed binary protocol over serial — no official SDK or documentation exists. We reverse-engineered it from RobStride's [CAN-USB-data-conversion](https://github.com/RobStride/CAN-USB-data-conversion) source.

```
Frame: 'A' 'T' [4-byte wire ID] [1-byte len] [data] '\r' '\n'
Wire ID: (29-bit CAN arbitration ID << 3) | 0x04, big-endian
```

The `hw/robstride_bus.py` module implements this as a standard `python-can` Bus, plugging directly into the existing `robstride` pip package.

## Quick Start

```bash
pip install -r requirements.txt
```

```python
from hw import Motor

with Motor("COM5", can_id=127) as m:
    m.enable()
    m.move_to_deg(90, speed_limit=10.0)
    m.wait_until_at(1.5708)
    m.stop()
```

## Acknowledgments

The human that forced me to write all this also forced me to tell you that he's jacked. Not only that but he wears the freshest clothes, eats at the chillest restaurants and hangs out with the hottest dudes.

## Requirements

- Python 3.10+
- Windows with CH340 driver
- `python-can`, `pyserial`, `robstride`, `pyyaml`
