# Humanoid Robot

Personal project: a ~5-foot humanoid with roughly 28–31 actuated joints. The frame is 2020 aluminum extrusion; joints use PETG-printed housings and RobStride RS03 brushless actuators (about 60 N·m peak, 9:1 planetary, 48 V nominal). Motors are on standard CAN at 1 Mbps; control software is Rust (async I/O, shared transport per bus, coordinated arm commands). The repo is a work in progress—mechanical design, wiring, and software are all being built here, including the CAN2USB serial framing used with the RobStride debugger.

## Development Roadmap

### Phase 1 — Arms (current)
8 DOF (4 per arm): shoulder pitch, shoulder roll, upper arm yaw, elbow pitch. RS03 layout: pitch motors in the torso, roll/yaw in printed housings along the arm. First goal: a simple coordinated arm wave.

### Phase 2 — Legs
12 DOF (6 per leg): hip yaw/roll/pitch, knee pitch, ankle pitch/roll. Hip stack is similar in spirit to the shoulder (coaxial/stacked actuators). Plan: balance before gait.

### Phase 3 — Wrists & Hands
6+ DOF (3+ per side): wrist articulation plus a grip. Likely smaller actuators at the wrist and a simpler hand mechanism (tendon or linkage).

### Phase 4 — Head
2–3 DOF: neck pan/tilt, optional jaw or visor. Room for cameras later.

**Target: 28–31 DOF total**

## Repository Structure

Cargo workspace: two Rust crates and a React frontend.

```
Cargo.toml                  Workspace root (members: cortex, navi)

cortex/                     Motor control, arm coordination, config
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

link/                       Web UI (Vite + React)
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

### `cortex` — Motor control

Motor I/O, arm coordination, and config loading.

- **`motor.rs`** — Async API for one RS03: position, velocity, torque, enable/disable, zero, safe startup.
- **`arm.rs`** — Multi-joint arm on a shared CAN transport: poses, homing, enable/disable together.
- **`config.rs`** — Loads `config/robot.yaml` into typed structs (limits, CAN IDs, bus settings).

Binaries:

| Binary | Description |
|---|---|
| `probe` | Scans the CAN bus, reports which motors respond |
| `motor_repl` | Interactive REPL for commands, parameters, gains |
| `wave_demo` | Coordinated arm wave (Phase 1 demo) |

```bash
cargo run -p cortex --bin probe
cargo run -p cortex --bin motor_repl
cargo run -p cortex --bin wave_demo
```

### `navi` — Web server and telemetry

Serves the Link UI and exposes:

- **REST** (`/api/*`) — Motors (enable, move, etc.), arms, config, logs, status.
- **WebTransport** — High-rate telemetry (position, velocity, torque, temperature) over QUIC datagrams.

```bash
cargo run -p navi --bin navi                  # with hardware
cargo run -p navi --bin navi -- --no-hardware  # mock telemetry, no CAN
```

### `link` — Frontend

Browser UI for motor control, arm posing, telemetry, and tuning.

**Stack:** React 19, TypeScript, Vite, TanStack Router, Zustand, uPlot, Tailwind CSS 4, shadcn/ui.

See [`link/README.md`](link/README.md) for details.

```bash
cd link
npm install
npm run dev       # Vite HMR on port 5173, proxied to navi on 8080
npm run build     # Production build → dist/ (served by navi)
```

## Hardware

| Component | Spec |
|---|---|
| Arm motors | RobStride RS03 (60 N·m peak, 9:1 planetary, 48 V) |
| Waist motor | OpenQDD |
| CAN adapter | RobStride CAN2USB debugger (CH340, 921600 baud) |
| Power | 24 V / 1200 W bench supply (48 V battery later for untethered) |
| Torso frame | 2020 extrusion, 460 × 200 × 160 mm |
| Joint housings | PETG, printed |
| CAN | 1 Mbps standard CAN (RS03); CAN-FD reserved for Moteus (future) |

## CAN2USB protocol

The RobStride CAN2USB debugger speaks a proprietary AT-framed binary protocol over serial (no public spec). Framing was derived from RobStride’s [CAN-USB-data-conversion](https://github.com/RobStride/CAN-USB-data-conversion) repo. The `robstride` crate’s `CH341Transport` implements it.

```
Frame: 'A' 'T' [4-byte wire ID] [1-byte len] [data] '\r' '\n'
Wire ID: (29-bit CAN arbitration ID << 3) | 0x04, big-endian
```

## Quick start

### Motor control (hardware required)

```bash
cargo run -p cortex --bin probe       # verify CAN connectivity
cargo run -p cortex --bin motor_repl  # interactive motor shell
```

### Full stack without motors

```bash
# Terminal 1 — backend with mock telemetry
cargo run -p navi --bin navi -- --no-hardware

# Terminal 2 — frontend with HMR
cd link && npm run dev
```

Open http://localhost:5173.

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

- Rust stable (MSVC on Windows)
- Node.js v24+ (for Link)
- CH340 driver on Windows, or Linux with SocketCAN (e.g. Pi deployment)
- `cargo build` for Rust; `cd link && npm install` for the frontend

## HTTPS on the robot (Tailscale)

Navi uses TLS for the Link UI. For a normal browser lock on the Pi, use Tailscale MagicDNS + **`tailscale cert`** so Let’s Encrypt certificates land in `certs/robot.pem` and `certs/robot-key.pem` (defaults `navi` already loads). Step-by-step: [`deploy/tailscale-https.md`](deploy/tailscale-https.md). Re-run the renewal script periodically (certs expire about every 90 days).

## Acknowledgments

RobStride for publishing reference code that made the CAN2USB framing tractable.
