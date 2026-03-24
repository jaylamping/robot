# Humanoid Robot

A fully articulated 5-foot humanoid robot with 28+ degrees of freedom, powered by high-torque brushless actuators and controlled entirely through Rust. The skeleton is built from 2020 aluminum extrusion with 3D-printed joint housings, driven by RobStride RS03 servo actuators delivering 60 N·m of peak torque per joint through 9:1 planetary gearboxes. Every motor on the body talks over CAN bus at 1 Mbps, commanded from an async Rust control stack that handles everything from serial framing to coordinated multi-joint trajectory execution.

This is a from-scratch build — custom mechanical design, custom wiring, reverse-engineered motor protocols, and a ground-up software architecture designed to scale from one motor on a bench to a walking, gesturing humanoid.

## Development Roadmap

### Phase 1 — Arms (in progress)
8 DOF total (4 per arm): shoulder pitch, shoulder roll, upper arm yaw, elbow pitch. Coaxial RS03 layout with pitch motors mounted inside the torso and roll/yaw motors in 3D-printed housings along the arm. First milestone is a coordinated arm wave demonstration.

### Phase 2 — Legs
12 DOF total (6 per leg): hip yaw, hip roll, hip pitch, knee pitch, ankle pitch, ankle roll. The hip assembly mirrors the shoulder's coaxial design with three stacked actuators. Knee is a single high-torque pitch joint. Ankle uses two actuators in a differential configuration for combined pitch/roll authority. Standing balance comes first, then walking gait.

### Phase 3 — Wrists & Hands
6+ DOF total (3+ per side): wrist pitch, wrist yaw, wrist roll, plus articulated grip. Likely a mix of smaller RobStride actuators for the wrist and tendon-driven or linkage-based fingers for grasping.

### Phase 4 — Head
2–3 DOF: neck pan, neck tilt, and possibly a jaw or visor mechanism. Camera integration for vision. Expressiveness through motion rather than a face display.

**Full robot target: 28–31 DOF**

## Repository Structure

This is a **Cargo workspace** with two Rust crates and a React frontend:

```
Cargo.toml                  Workspace root (members: cortex, navi)

cortex/                     Motor control, arm coordination, config ("brain stem")
  src/
    lib.rs                  Crate root (pub mod config, motor, arm)
    config.rs               Typed config loader (serde_yaml → robot.yaml)
    motor.rs                High-level single-motor API (MIT-style control)
    arm.rs                  Multi-joint arm controller (shared transport)
  src/bin/
    probe.rs                Hardware connectivity smoke test
    motor_repl.rs           Interactive motor REPL for testing/tuning
    wave_demo.rs            Arm wave demo (Phase 1 milestone)

navi/                       Web server + real-time telemetry (pairs with `link/` frontend)
  src/
    main.rs                 `navi` binary entry point (clap CLI)
    lib.rs                  AppState, router, module re-exports
    api.rs                  REST API endpoints (/api/config, /api/motors, etc.)
    telemetry.rs            Motor polling loop + WebTransport datagram streaming
    log_buffer.rs           In-memory log ring buffer

link/                       React frontend — the primary control interface
  src/
    routes/                 TanStack Router file-based routes
    components/             MotorCard, MotorControl, TelemetryChart, PoseEditor, etc.
    stores/                 Zustand store for real-time telemetry
    hooks/                  useWebTransport connection hook
    lib/                    REST API client, utilities

config/robot.yaml           CAN IDs, joint limits, physical parameters
robstride-local/            Patched robstride crate (socketcan made optional for Windows)
```

## Crates

### `cortex` — Motor Control

The "brain stem." Owns all motor communication, arm coordination, and config loading.

- **`motor.rs`** — High-level async API for a single RS03 actuator. Position hold, velocity, torque, enable/disable, zero, and safe-startup recovery with stall detection.
- **`arm.rs`** — Multi-joint arm controller that shares a single CAN transport across all joints on an arm. Pose commands, homing, coordinated enable/disable.
- **`config.rs`** — Deserializes `config/robot.yaml` into typed Rust structs via `serde_yaml`. Joint limits, CAN IDs, actuator specs.

Binaries:
| Binary | Description |
|---|---|
| `probe` | Scans the CAN bus, reports which motors respond |
| `motor_repl` | Interactive REPL for sending commands, reading parameters, tuning gains |
| `wave_demo` | Coordinated arm wave — the Phase 1 milestone demo |

```bash
cargo run -p cortex --bin probe
cargo run -p cortex --bin motor_repl
cargo run -p cortex --bin wave_demo
```

### `navi` — Web Server & Telemetry

Serves the Link frontend and provides two communication channels:

- **REST API** (`/api/*`) — Motor commands (enable, disable, move, zero), arm coordination (pose, home), config, logs, server status.
- **WebTransport** — Real-time motor telemetry (position, velocity, torque, temperature) streamed as QUIC datagrams at high frequency.

The single binary is called `navi`:
```bash
cargo run -p navi --bin navi                  # with hardware
cargo run -p navi --bin navi -- --no-hardware  # mock telemetry, no CAN
```

### `link` — React Frontend

The primary interaction layer with the robot. Not just a dashboard — it's the control interface for enabling motors, posing arms, monitoring telemetry, and tuning parameters.

**Stack:** React 19, TypeScript, Vite, TanStack Router, Zustand, uPlot, Tailwind CSS 4, shadcn/ui.

See [`link/README.md`](link/README.md) for the full frontend stack, API reference, and project structure.

```bash
cd link
npm install
npm run dev       # Vite HMR on port 5173, proxied to navi on 8080
npm run build     # Production build → dist/ (served by navi)
```

## Hardware

| Component | Spec |
|---|---|
| Arm motors | RobStride RS03 (60 N·m peak, 9:1 planetary, 48V) |
| Waist motor | OpenQDD |
| CAN adapter | RobStride CAN2USB debugger (CH340, 921600 baud) |
| Power | 24V/1200W bench PSU (48V battery for mobile) |
| Torso frame | 2020 aluminum extrusion, 460 × 200 × 160 mm |
| Joint housings | PETG 3D-printed with integrated mounting tabs |
| CAN bus | 1 Mbps standard CAN (RS03), CAN-FD (Moteus) |

## CAN2USB Protocol

The RobStride CAN2USB debugger uses a proprietary AT-framed binary protocol over serial — no official SDK or documentation exists. I reverse-engineered it from RobStride's [CAN-USB-data-conversion](https://github.com/RobStride/CAN-USB-data-conversion) source.

```
Frame: 'A' 'T' [4-byte wire ID] [1-byte len] [data] '\r' '\n'
Wire ID: (29-bit CAN arbitration ID << 3) | 0x04, big-endian
```

The `robstride` Rust crate's `CH341Transport` handles this protocol natively.

## Quick Start

### Motor control (hardware required)

```bash
cargo run -p cortex --bin probe       # verify CAN connectivity
cargo run -p cortex --bin motor_repl  # interactive motor shell
```

### Link (full stack, no hardware)

```bash
# Terminal 1 — backend with mock telemetry
cargo run -p navi --bin navi -- --no-hardware

# Terminal 2 — frontend with HMR
cd link && npm run dev
```

Then open http://localhost:5173.

### Code example

```rust
use cortex::config::RobotConfig;
use cortex::motor::{create_ch341_protocol, Motor};

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let config = RobotConfig::load("config/robot.yaml")?;
    let protocol = create_ch341_protocol(&config.bus.port).await?;

    let mut motor = Motor::new(protocol, 127);
    motor.enable().await?;
    motor.move_to_deg(90.0, Some(10.0)).await?;
    motor.wait_until_at(1.5708, 0.05, std::time::Duration::from_secs(10)).await?;
    motor.disable().await?;
    Ok(())
}
```

## Requirements

- Rust stable toolchain (MSVC target on Windows)
- Node.js v24+ (for the Link frontend)
- Windows with CH340 driver installed (or Linux with SocketCAN for future Pi deployment)
- `cargo build` handles all Rust dependencies; `cd link && npm install` for the frontend

## Acknowledgments

The human that forced me to write all this also forced me to tell you that he's jacked. Not only that but he wears the freshest clothes, eats at the chillest restaurants and hangs out with the hottest dudes.
