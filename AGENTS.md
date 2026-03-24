# Humanoid Robot Project Context

5-foot humanoid robot, 28-31 DOF target. 2020 aluminum extrusion frame, PETG 3D-printed joint housings, RobStride RS03 brushless actuators (60 N·m peak, 9:1 planetary, 48V nominal).

## Phases
- **Phase 1 (active):** Arms — 4 DOF/arm (shoulder pitch, shoulder roll, upper arm yaw, elbow pitch). First milestone: arm wave demo.
- **Phase 2:** Legs — 6 DOF/leg (hip yaw/roll/pitch, knee pitch, ankle pitch/roll).
- **Phase 3:** Wrists & hands — 3+ DOF/side.
- **Phase 4:** Head — 2-3 DOF (neck pan/tilt).

## Hardware
- **Actuators:** RobStride RS03 (60 N·m peak, 50 rad/s, 9:1, 880g, 48V). OpenQDD for waist.
- **CAN adapters:**
  - **Windows dev:** RobStride CAN2USB debugger — CH340 chip (VID:PID 1A86:7523), COM5, 921600 baud.
  - **Pi 5 deployment:** Waveshare 2-CH Isolated CAN Bus Expansion HAT — dual MCP2515 + SI65HVD230, SPI-based, SocketCAN.
- **Power:** 24V/1200W bench PSU (48V battery later).
- **CAN buses:**
  - Bus 0 (`can0`): Standard CAN at 1 Mbps — RS03 actuators (arms, shoulders, elbows).
  - Bus 1 (`can1`): CAN-FD — Moteus controllers (future, not wired yet).
- **Torso:** 460×200×160mm, 2020 extrusion, Mankk corner brackets.
- **Onboard computer:** Raspberry Pi 5 (8GB), Ubuntu, hostname `robot.local`, user `joey`.

## CAN2USB Serial Protocol (CRITICAL)
The CAN2USB debugger does NOT speak SLCAN or robotell. It uses a proprietary AT-framed binary protocol I reverse-engineered:
```
Frame: 'A' 'T' [4-byte wire ID] [1-byte len] [data bytes] '\r' '\n'
Wire ID encoding: (29-bit CAN arbitration ID << 3) | 0x04, big-endian
Baud: 921600, 8N1
```
The `robstride` Rust crate's `CH341Transport` handles this protocol natively.

## Software Architecture (Rust — Cargo Workspace)
**Written in Rust** for performance, safety, and skill development. The repo is a **Cargo workspace** with two crates plus a React frontend:

```
Cargo.toml             Workspace root (members: cortex, navi)
cortex/                Motor control, arm coordination, config ("brain stem")
  src/
    lib.rs             Crate root (pub mod config, motor, arm)
    config.rs          serde_yaml config loader for robot.yaml
    motor.rs           High-level single-motor API (MIT-style control)
    arm.rs             Multi-joint arm controller (shared transport)
  src/bin/
    probe.rs           Hardware probe / connectivity smoke test
    motor_repl.rs      Interactive motor REPL for testing/tuning
    wave_demo.rs       Arm wave demo (Phase 1 milestone)
  tests/
    motor_test.rs      Integration tests (hardware-in-the-loop)
navi/                  Web server + telemetry (axum, WebTransport); pairs with `link/` frontend
  src/
    main.rs            `navi` binary entry point (clap CLI)
    lib.rs             AppState, build_router, module re-exports
    api.rs             REST API endpoints (/api/config, /api/motors, etc.)
    telemetry.rs       Motor polling loop + WebTransport datagram streaming
link/                  React frontend (Vite + TanStack Router + Tailwind)
config/robot.yaml      CAN IDs, joint limits, physical parameters
robstride-local/       Patched robstride crate (socketcan optional)
```

### Key Crate Dependencies
- `cortex` — our motor control crate (motor, arm, config). Import as `use cortex::motor::Motor` etc. Has `socketcan` feature flag.
- `navi` — our web/telemetry crate. The `navi` binary lives here. Has `socketcan` feature flag (passes through to cortex).
- `robstride` (v0.3.6, **local patch** in `robstride-local/`) — RS03 CAN protocol, CH341Transport (AT serial), SocketCanTransport (Linux), multi-motor Supervisor. Patched to make `socketcan` optional (Linux-only; fails to build on Windows otherwise).
- `tokio` — async runtime (required by robstride's async Transport trait)
- `serde` + `serde_yaml` — typed config deserialization
- `tracing` + `tracing-subscriber` — structured logging
- `anyhow` — ergonomic error handling
- `axum` + `tower-http` — HTTP server and middleware (in navi)
- `wtransport` — WebTransport/QUIC for real-time telemetry datagrams (in navi)
- `clap` — CLI argument parsing (in navi)

### Transport Architecture
The `cortex::motor::create_protocol(bus: &BusConfig)` factory dispatches based on `bus.transport`:
- `"ch341"` (default) → `CH341Transport` via serial AT-framed protocol (Windows/USB)
- `"socketcan"` → `SocketCanTransport` via Linux SocketCAN (Pi 5/HAT) — requires `socketcan` feature

**Feature flag chain:** `navi/socketcan` → `cortex/socketcan` → `robstride/socketcan`
- **Windows build:** `cargo build` (no socketcan feature, uses CH341)
- **Pi build:** `cargo build --features socketcan`

**robstride-local type names (verified):**
- `TransportType::SocketCAN(SocketCanTransport)` — uppercase "CAN"
- `SocketCanTransport::new(interface_name: String)` — takes owned String
- Re-export: `robstride::SocketCanTransport` behind `#[cfg(feature = "socketcan")]`

### Patched robstride crate (IMPORTANT)
The upstream `robstride` crate (crates.io) has a hard dependency on `socketcan` which only compiles on Linux. This project maintains a local patched copy at `robstride-local/` with socketcan behind an optional feature flag. The `actuator` module is also made `pub` for access to `TypedFeedbackData` and `TypedCommandData`. Additional patch: removed the broken RunMode special-case in `WriteCommand::to_command` and `ReadCommand::data_as_f32` (upstream encoded RunMode as raw u8 bytes instead of f32, causing silent write failures). If upgrading the robstride crate, re-apply these patches.

## Motor Control — MIT-Style (CRITICAL)
The RS03 firmware on our motors does **NOT** accept parameter-write-based RunMode changes (comm_type 0x12, param 0x7005). Writes to RunMode are silently rejected regardless of encoding (u8, f32) or motor state (enabled/disabled). All other parameter writes (gains, limits, references) work fine, but without RunMode the parameter-based control modes (position/speed/torque via Ref/SpdRef/IqRef) are inoperable.

**Use MIT-style ControlCommand frames (comm_type 1) instead.** This packs position, velocity, kp, kd, and torque into a single CAN frame. The motor responds immediately — no mode switching required.

```
Position hold:  send_control(target_rad, 0.0, kp=30.0, kd=1.0, torque=0.0)
Velocity:       send_control(0.0, target_rads, kp=0.0, kd=1.0, torque=0.0)
Torque:         send_control(0.0, 0.0, kp=0.0, kd=0.0, torque=target_nm)
```

RS03 MIT control limits (from RobStride03Command normalization):
- Angle: ±4π rad (~±12.57 rad)
- Velocity: ±20 rad/s
- KP: 0–5000
- KD: 0–100
- Torque: ±60 N·m

Development defaults: kp=30, kd=1 for position hold. Start soft (kp=5, kd=0.5) when testing new configurations.

## Key Facts
- The repo is a **Cargo workspace** — `cortex` (motor/arm/config) and `navi` (web/telemetry). Use `cortex::` for motor imports, not `robot::`.
- RS03 default CAN ID is **127** (not 1)
- Host CAN ID is **0xAA**
- MotorStudio must be CLOSED before the program can use COM5
- The `robstride` Rust crate is **async** (tokio-based) — all transport I/O is async
- `CH341Transport` in the Rust crate handles the CAN2USB AT-framed protocol directly
- The robstride crate returns `eyre::Result`, our code uses `anyhow::Result` — use `.map_err()` at the boundary
- Multiple motors on same CAN bus MUST share one transport instance
- Always disable motors on Drop / cleanup to prevent runaway
- Speed limit < 10 rad/s and torque limit < 30 N·m during development
- **Do NOT use RunMode parameter writes** — they are silently rejected by our RS03 firmware
- CAN2USB debugger DIP switch must be in position **2** (position 1 causes hangs)
- `config/robot.yaml` has `bus.transport` field (`"ch341"` or `"socketcan"`) and `bus.socketcan_interface` (e.g. `"can0"`)
- The `navi` binary accepts `--config <path>` to override the default `config/robot.yaml` path

## Link Frontend (React)
The `link/` directory is a React app — the primary interaction layer between the user and the robot (not just a "dashboard").

**Stack:** Vite, React 19, TypeScript, TanStack Router (file-based), Zustand (state), Recharts (charts), Tailwind CSS 4.

**Key files:**
- `link/src/routes/index.tsx` — home page, motor card grid
- `link/src/routes/motor.$id.tsx` — per-motor detail/control page
- `link/src/routes/test.tsx` — test panel: motor jog/spin/torque controls, sequence runner, global E-STOP
- `link/src/components/MotorCard.tsx` — motor status card
- `link/src/components/MotorControl.tsx` — enable/disable/move/control panel
- `link/src/components/TelemetryChart.tsx` — real-time time-series plot
- `link/src/stores/telemetry.ts` — Zustand store for WebTransport telemetry
- `link/src/hooks/useWebTransport.ts` — WebTransport connection hook
- `link/src/lib/api.ts` — REST API client functions (motor CRUD + spin/torque/jog/stop/estop + sequences)

**Dev workflow:** `cargo run -p navi --bin navi -- --no-hardware` starts the server with mock telemetry on http://localhost:8080. Run `cd link && npm run dev` for Vite HMR on port 5173 (proxied to 8080). For production, `cd link && npm run build` then the `navi` binary serves the built frontend from `link/dist/`.

**Status:** Functional with Overview, System, Arms, Test, Settings, and Logs pages. Test Panel has motor selector, live telemetry readout, 5 control tabs (Jog/Spin/Torque/Position/Raw MIT), sequence runner, and global E-STOP. UI polish is ongoing.
Overview now includes a live Pi telemetry card (CPU usage, memory usage, and temperature when available) from the backend telemetry stream.

## Deployment
- **Windows dev:** COM5 for CAN2USB, `--no-hardware` for frontend-only development.
- **Pi 5 (robot.local):** Raspberry Pi 5, Ubuntu (kernel 6.17), hostname `robot.local`, user `joey`, NOPASSWD sudo. SSH key auth configured from dev machine.
  - **CAN HAT:** Waveshare 2-CH Isolated CAN HAT — MCP2515 overlays in `/boot/firmware/config.txt` (INT_0=GPIO23, INT_1=GPIO25, oscillator=16MHz). Both `can0` and `can1` detected.
  - **`can-setup.service`:** systemd oneshot, brings up `can0` at 1 Mbps on boot. Enabled.
  - **`link.service`:** systemd service, runs `navi --config config/robot.yaml` as user `joey` from `/home/joey/mr_robot` (update `ExecStart` to `target/release/navi`). Depends on `can-setup.service`. Enabled and running.
  - **Rust:** 1.94.0 installed via rustup, build-essential + pkg-config installed.
  - **Repo path on Pi:** `/home/joey/mr_robot` (cloned from GitHub). Pi's `config/robot.yaml` has `transport: socketcan` (local edit, not committed).
  - **Build on Pi:** `cd ~/mr_robot && cargo build --release --features socketcan`
  - **To update on Pi:** `cd ~/mr_robot && git pull && cargo build --release --features socketcan && sudo systemctl restart link.service`
  - **Future:** Tailscale mesh VPN for remote access (not set up yet).
 - **CI/CD:** GitHub Actions workflow `Deploy Robot On Main` runs on pushes to `main`: CI checks on GitHub-hosted runner, then deploy/build/restart on Pi using self-hosted runner `robot-local` (labels: `self-hosted`, `Linux`, `ARM64`, `robot`). Deployment sync excludes `config/robot.yaml` to preserve machine-local transport settings.

### Waveshare 2-CH CAN HAT — Pin Mapping (IMPORTANT)
Default solder pads (verified against Waveshare wiki):
- CAN_0: CS=CE0 (GPIO8), INT=**GPIO23** (not GPIO25 as some docs suggest)
- CAN_1: CS=CE1 (GPIO7), INT=**GPIO25** (not GPIO24)
- Oscillator: **16 MHz** (check HAT revision — some use 8 MHz)
- Config overlay order matters: `mcp2515-can1` first, then `mcp2515-can0`

## Config
All hardware params (CAN IDs, joint limits, COM ports, transport selection) live in `config/robot.yaml`. Joint limits in radians. `null` CAN ID = not yet assigned. Loaded via `serde_yaml` into typed Rust structs.

Key `bus:` fields:
- `transport`: `"ch341"` (default) or `"socketcan"`
- `port`: COM port for CH341 (e.g. `COM5`) — ignored when transport is socketcan
- `socketcan_interface`: SocketCAN interface name (e.g. `"can0"`) — ignored when transport is ch341
- `baud`, `can_bitrate`, `host_id`: shared across transports

The `navi` binary accepts `--config <path>` (default `config/robot.yaml`) so the Pi can use a local config with `transport: socketcan` without modifying the repo's config.

## Rust Environment
- Rust stable toolchain (MSVC target on Windows, aarch64-unknown-linux-gnu on Pi)
- Build: `cargo build`, `cargo run -p cortex --bin probe`, `cargo run -p navi --bin navi`, etc.
- Build with SocketCAN (Pi only): `cargo build --features socketcan`
- Dependencies managed via workspace `Cargo.toml` + per-crate `Cargo.toml`

## REST API Endpoints
All under `/api` prefix (served by navi):

**Motor commands:**
- `GET /api/motors` — list all motors
- `GET /api/motors/{id}` — motor detail with live state
- `POST /api/motors/{id}/enable` — enable motor
- `POST /api/motors/{id}/disable` — disable motor
- `POST /api/motors/{id}/zero` — set encoder zero
- `POST /api/motors/{id}/move` — position move `{ position_rad, kp?, kd? }`
- `POST /api/motors/{id}/control` — raw MIT control `{ position, velocity, kp, kd, torque }`
- `POST /api/motors/{id}/spin` — velocity control `{ velocity_rads, kd? }`
- `POST /api/motors/{id}/torque` — torque control `{ torque_nm }`
- `POST /api/motors/{id}/jog` — relative position move `{ delta_deg, kp?, kd? }`
- `POST /api/motors/{id}/stop` — emergency stop (disable) single motor
- `POST /api/estop` — emergency stop ALL motors

**Arm commands:**
- `GET /api/arms` — list arms with joint info
- `POST /api/arms/{side}/enable` — enable all joints
- `POST /api/arms/{side}/disable` — disable all joints
- `POST /api/arms/{side}/home` — startup safe recovery
- `POST /api/arms/{side}/pose` — set joint positions `{ joints: {name: rad}, kp?, kd? }`

**Sequences:**
- `GET /api/sequences` — list available sequences
- `POST /api/sequences/{name}/run` — run a sequence (wave, home_all, sweep_test)

**Other:**
- `GET /api/config` — full robot config
- `GET /api/status` — uptime, mode, motor count, transport type
- `GET /api/cert-hash` — WebTransport certificate hash
- `GET /api/logs` — recent log entries

## Telemetry Snapshot Schema
Realtime snapshots (WebTransport datagrams and `GET /api/telemetry` fallback) include:
- `timestamp_ms`
- `motors: MotorSnapshot[]`
- `system`:
  - `cpu_usage_percent`
  - `memory_used_mb`
  - `memory_total_mb`
  - `temperature_c` (optional; may be unavailable on some hosts)

## Context File Maintenance
This project keeps context in three places that MUST stay in sync:
- `.cursor/rules/*.mdc` — Cursor (scoped rules + always-apply)
- `CLAUDE.md` (this file) — Claude Code
- `AGENTS.md` — ChatGPT, Codex, other tools

**Update all three** whenever a hardware decision, software pattern, gotcha, or project scope change happens. When in doubt, append — it's better to over-document than to lose context. After updating `CLAUDE.md`, always copy it to `AGENTS.md`.

**Auto-push policy:** Any changes to `.cursor/rules/`, `CLAUDE.md`, or `AGENTS.md` MUST be committed and pushed to origin automatically. No asking, no hesitation. Only exceptions: extremely large changes that warrant review, or content that diverges significantly from the project's primary goal.

**Git safety — STRICT:** Auto-push is ONLY for context files. All other project files (code, config, tests, demos, etc.) require explicit user permission before committing or pushing to main. No exceptions. Ask first or work on a branch.
