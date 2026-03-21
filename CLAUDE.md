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
Cargo.toml             Workspace root (members: cortex, link-server)
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
link-server/           Web server + telemetry (axum, WebTransport)
  src/
    main.rs            `link` binary entry point (clap CLI)
    lib.rs             AppState, build_router, module re-exports
    api.rs             REST API endpoints (/api/config, /api/motors, etc.)
    telemetry.rs       Motor polling loop + WebTransport datagram streaming
link/                  React frontend (Vite + TanStack Router + Tailwind)
config/robot.yaml      CAN IDs, joint limits, physical parameters
robstride-local/       Patched robstride crate (socketcan optional)
```

### Key Crate Dependencies
- `cortex` — our motor control crate (motor, arm, config). Import as `use cortex::motor::Motor` etc.
- `link-server` — our web/telemetry crate. The `link` binary lives here.
- `robstride` (v0.3.6, **local patch** in `robstride-local/`) — RS03 CAN protocol, CH341Transport (AT serial), multi-motor Supervisor. Patched to make `socketcan` optional (Linux-only; fails to build on Windows otherwise).
- `tokio` — async runtime (required by robstride's async Transport trait)
- `serde` + `serde_yaml` — typed config deserialization
- `tracing` + `tracing-subscriber` — structured logging
- `anyhow` — ergonomic error handling
- `axum` + `tower-http` — HTTP server and middleware (in link-server)
- `wtransport` — WebTransport/QUIC for real-time telemetry datagrams (in link-server)
- `clap` — CLI argument parsing (in link-server)

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
- The repo is a **Cargo workspace** — `cortex` (motor/arm/config) and `link-server` (web/telemetry). Use `cortex::` for motor imports, not `robot::`.
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

## Link Frontend (React)
The `link/` directory is a React app — the primary interaction layer between the user and the robot (not just a "dashboard").

**Stack:** Vite, React 19, TypeScript, TanStack Router (file-based), Zustand (state), Recharts (charts), Tailwind CSS 4.

**Key files:**
- `link/src/routes/index.tsx` — home page, motor card grid
- `link/src/routes/motor.$id.tsx` — per-motor detail/control page
- `link/src/components/MotorCard.tsx` — motor status card
- `link/src/components/MotorControl.tsx` — enable/disable/move/control panel
- `link/src/components/TelemetryChart.tsx` — real-time time-series plot
- `link/src/stores/telemetry.ts` — Zustand store for WebTransport telemetry
- `link/src/hooks/useWebTransport.ts` — WebTransport connection hook
- `link/src/lib/api.ts` — REST API client functions

**Dev workflow:** `cargo run -p link-server --bin link -- --no-hardware` starts the server with mock telemetry on http://localhost:8080. Run `cd link && npm run dev` for Vite HMR on port 5173 (proxied to 8080). For production, `cd link && npm run build` then the `link` binary serves the built frontend from `link/dist/`.

**Status:** Scaffolded and functional but UI is bare — motor cards, telemetry chart, and control panel exist but need polish. This is the current active work area.

## Deployment
- **Current:** Everything runs on the Windows dev machine (COM5 for CAN2USB). Use `--no-hardware` for frontend-only development.
- **Future:** Raspberry Pi 5 on the robot, running Ubuntu with SocketCAN. The `robstride-local` crate already has the `socketcan` feature flag ready. Mesh VPN (Tailscale) planned for remote access. Pi is NOT set up yet — not a blocker for current work.

## Config
All hardware params (CAN IDs, joint limits, COM ports) live in `config/robot.yaml`. Joint limits in radians. `null` CAN ID = not yet assigned. Loaded via `serde_yaml` into typed Rust structs.

## Rust Environment
- Rust stable toolchain (MSVC target on Windows, will also target aarch64-linux for Pi)
- Build: `cargo build`, `cargo run -p cortex --bin probe`, `cargo run -p link-server --bin link`, etc.
- Dependencies managed via workspace `Cargo.toml` + per-crate `Cargo.toml`

## Context File Maintenance
This project keeps context in three places that MUST stay in sync:
- `.cursor/rules/*.mdc` — Cursor (scoped rules + always-apply)
- `CLAUDE.md` (this file) — Claude Code
- `AGENTS.md` — ChatGPT, Codex, other tools

**Update all three** whenever a hardware decision, software pattern, gotcha, or project scope change happens. When in doubt, append — it's better to over-document than to lose context. After updating `CLAUDE.md`, always copy it to `AGENTS.md`.

**Auto-push policy:** Any changes to `.cursor/rules/`, `CLAUDE.md`, or `AGENTS.md` MUST be committed and pushed to origin automatically. No asking, no hesitation. Only exceptions: extremely large changes that warrant review, or content that diverges significantly from the project's primary goal.

**Git safety — STRICT:** Auto-push is ONLY for context files. All other project files (code, config, tests, demos, etc.) require explicit user permission before committing or pushing to main. No exceptions. Ask first or work on a branch.
