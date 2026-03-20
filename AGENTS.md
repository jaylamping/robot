# Humanoid Robot Project Context

5-foot humanoid robot, 28-31 DOF target. 2020 aluminum extrusion frame, PETG 3D-printed joint housings, RobStride RS03 brushless actuators (60 N·m peak, 9:1 planetary, 48V nominal).

## Phases
- **Phase 1 (active):** Arms — 4 DOF/arm (shoulder pitch, shoulder roll, upper arm yaw, elbow pitch). First milestone: arm wave demo.
- **Phase 2:** Legs — 6 DOF/leg (hip yaw/roll/pitch, knee pitch, ankle pitch/roll).
- **Phase 3:** Wrists & hands — 3+ DOF/side.
- **Phase 4:** Head — 2-3 DOF (neck pan/tilt).

## Hardware
- **Actuators:** RobStride RS03 (60 N·m peak, 50 rad/s, 9:1, 880g, 48V). OpenQDD for waist.
- **CAN adapter:** RobStride CAN2USB debugger — CH340 chip (VID:PID 1A86:7523), COM5, 921600 baud.
- **Power:** 24V/1200W bench PSU (48V battery later).
- **CAN:** 1 Mbps standard CAN for RS03s. Separate CAN-FD bus for Moteus (mjcanfd-usb).
- **Torso:** 460×200×160mm, 2020 extrusion, Mankk corner brackets.

## CAN2USB Serial Protocol (CRITICAL)
The CAN2USB debugger does NOT speak SLCAN or robotell. It uses a proprietary AT-framed binary protocol we reverse-engineered:
```
Frame: 'A' 'T' [4-byte wire ID] [1-byte len] [data bytes] '\r' '\n'
Wire ID encoding: (29-bit CAN arbitration ID << 3) | 0x04, big-endian
Baud: 921600, 8N1
```
Implemented in `hw/robstride_bus.py` as a standard `python-can` Bus.

## Software Architecture
```
hw/robstride_bus.py    python-can Bus for CAN2USB AT serial protocol
hw/motor.py            High-level single-motor API (position, velocity, torque)
arm/arm.py             Multi-joint arm controller (shares one bus across motors)
config/robot.yaml      CAN IDs, joint limits, physical parameters
demos/                 Runnable demonstrations
tests/                 Hardware probes & integration tests
```
Third-party `robstride` pip package handles low-level CAN message construction.

## Key Facts
- RS03 default CAN ID is **127** (not 1)
- Host CAN ID is **0xAA**
- MotorStudio must be CLOSED before Python can use COM5
- `robstride.Client` motor_model param affects feedback velocity/torque scaling
- Windows console can't print Unicode emoji — use ASCII
- PowerShell mangles inline Python — use script files
- Multiple motors on same CAN bus MUST share one `RobStrideBus` instance
- Always disable motors in finally/cleanup blocks
- Speed limit < 10 rad/s and torque limit < 30 N·m during development

## Config
All hardware params (CAN IDs, joint limits, COM ports) live in `config/robot.yaml`. Joint limits in radians. `null` CAN ID = not yet assigned.

## Python Environment
- Python 3.13, Windows
- Dependencies: python-can, pyserial, robstride, pyyaml

## Context File Maintenance
This project keeps context in three places that MUST stay in sync:
- `.cursor/rules/*.mdc` — Cursor (scoped rules + always-apply)
- `CLAUDE.md` (this file) — Claude Code
- `AGENTS.md` — ChatGPT, Codex, other tools

**Update all three** whenever a hardware decision, software pattern, gotcha, or project scope change happens. When in doubt, append — it's better to over-document than to lose context. After updating `CLAUDE.md`, always copy it to `AGENTS.md`.

**Auto-push policy:** Any changes to `.cursor/rules/`, `CLAUDE.md`, or `AGENTS.md` MUST be committed and pushed to origin automatically. No asking, no hesitation. Only exceptions: extremely large changes that warrant review, or content that diverges significantly from the project's primary goal.

**Git safety — STRICT:** Auto-push is ONLY for context files. All other project files (code, config, tests, demos, etc.) require explicit user permission before committing or pushing to main. No exceptions. Ask first or work on a branch.
