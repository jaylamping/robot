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
    motor.rs           High-level single-motor API (MIT-style control, joint limits, gravity-catch, faults)
    arm.rs             Multi-joint arm controller (ordered joints, preflight, homing results, home status)
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
    api.rs             REST API endpoints (/api/config, /api/motors, preflight, homing, limits, etc.)
    telemetry.rs       Motor polling loop + WebTransport datagram streaming (home/limit status)
link/                  React frontend (Vite + TanStack Router + Tailwind)
deploy/link.service    Pi systemd unit (Navi binary); copied by deploy workflow
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

## Homing & Joint Safety System

### Three-Layer Joint Limit Enforcement
Every motor command passes through three layers of protection:
1. **Link UI** — sliders constrain visible range, amber/red visual warnings near/at limits
2. **Navi API** — validates and rejects with a clear error message before passing to Motor
3. **Cortex Motor** — hard clamp on every `send_control()`, `move_to()`, `step_toward()` call; no exceptions

### Motor-Level Limits (`cortex/src/motor.rs`)
- `Motor` struct stores `joint_limits: Option<(f32, f32)>` (min_rad, max_rad)
- `send_control()` clamps `position_rad` to limits before building the CAN frame
- **Soft boundary zone** for velocity/torque: configurable `soft_limit_margin_rad` (~0.175 rad / ~10°). When motor position is within this margin of a limit and velocity/torque pushes toward it, command is linearly scaled down to zero at the boundary.
- `last_known_position` tracked from `read_position()`, `read_state()`, `enable()`, and `send_control()` feedback
- `set_joint_limits(min, max)`, `clear_joint_limits()`, `joint_limits()` — getter/setter API
- Limits are set by `Arm::new()` from config and also updatable at runtime via API

### Gravity-Catch Enable (`cortex/src/motor.rs`)
- `enable_with_hold(kp, kd)` — enables motor then immediately sends soft position-hold at current encoder position
- Prevents gravity-loaded joints from dropping in the ~25ms gap between enable and first recovery command
- Used by `startup_safe_recovery()` before homing each joint

### Pre-Flight Check (`cortex/src/arm.rs`)
- `preflight_check()` reads every joint's encoder position (without enabling) and compares to configured limits
- Returns `PreflightResult { pass: bool, joints: Vec<PreflightJoint> }` with per-joint violation details
- `PreflightViolation` includes: exceeded_by_rad/deg, which_limit ("min"/"max"), suggested_fix (human-readable direction)
- `startup_safe_recovery(force)` calls preflight first; blocks if violations exist unless `force: true`

### Deterministic Homing Order (`cortex/src/arm.rs`)
- `Arm` uses `Vec<OrderedJoint>` instead of `HashMap` — joints iterate in YAML field order
- Order: shoulder_pitch → shoulder_roll → upper_arm_yaw → elbow_pitch (proximal to distal)
- Shoulder homes first (supports entire arm weight), elbow homes last (lightest load)

### Per-Joint Homing Results (`cortex/src/arm.rs`)
- `JointHomingStatus` enum: `AlreadyHome`, `Homed`, `StalledButHomed`, `TimedOut`, `Error(String)`, `Skipped`
- `JointHomingResult`: joint_name, status, start/end positions, home target, error_rad, stall_backoffs, duration_ms
- `StartupRecoverySummary` includes `Vec<JointHomingResult>` + aggregate `stall_backoffs`

### Homing Status Query (`cortex/src/arm.rs`)
- `get_homing_status()` — read-only per-joint check returning `Vec<JointHomeStatus>` (home_rad, current_rad, error_rad, at_home, limits)

### Fault Detection (`cortex/src/motor.rs`)
- `read_fault_code()` — reads raw fault register at 0x3022 (bit14=stall, bit7=uncalibrated, bit3=overvoltage, bit2=undervoltage, bit1=driver, bit0=overtemp)
- `clear_faults()` — sends StopCommand with `clear_fault: true`

### Runtime Limit/Home Editing (API)
- `PUT /api/joints/{section}/{joint}/limits` — update limits in config + persist to `robot.yaml` + update Motor instance
- `PUT /api/joints/{section}/{joint}/home` — set home_rad explicitly or `{ set_current: true }` to use current position

### Important Homing Notes
- `startup_safe_recovery(force: bool)` — the `force` parameter is **required**; pass `false` for normal use
- `Arm::get_joint_positions()` returns `Vec<(String, f32)>` (ordered), not `HashMap`
- `Arm::update_joint_limits()` and `Arm::update_joint_home()` update both internal params and Motor instances

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
- **Joint limits are enforced at three layers** — Motor (hard clamp), API (reject), UI (visual). Every code path respects limits.
- **Homing order is deterministic** — YAML field order (shoulder first, elbow last) with gravity-catch hold
- **Pre-flight check blocks homing** if any joint starts outside its limits (unless force override)

## Link Frontend (React)
The `link/` directory is a React app — the primary interaction layer between the user and the robot (not just a "dashboard").

**Stack:** Vite, React 19, TypeScript, TanStack Router (file-based), Zustand (state), Recharts (charts), Tailwind CSS 4.

**Key files:**
- `link/src/routes/index.tsx` — Overview page: motor card grid, **HomingStatusCard**, preflight alerts
- `link/src/routes/motor.$id.tsx` — per-motor detail/control page
- `link/src/routes/test.tsx` — test panel: motor jog/spin/torque controls, sequence runner, global E-STOP
- `link/src/routes/arms.tsx` — arm control: per-joint sliders with **limit proximity indicators**, collapsible **limit/home editor**, **homing result display**, preflight alerts
- `link/src/components/MotorCard.tsx` — motor status card
- `link/src/components/MotorControl.tsx` — enable/disable/move/control panel
- `link/src/components/TelemetryChart.tsx` — real-time time-series plot
- `link/src/components/HomingStatusCard.tsx` — per-arm homing status card with colored dot indicators, inline home buttons, last homing result display
- `link/src/components/PreflightAlert.tsx` — red alert banner for joint limit violations with per-joint details, suggested fixes, re-check and override buttons
- `link/src/stores/telemetry.ts` — Zustand store for WebTransport telemetry (includes `home_rad`, `home_error_rad`, `at_home`, `limits` per motor)
- `link/src/hooks/useWebTransport.ts` — WebTransport connection hook
- `link/src/lib/api.ts` — REST API client functions (motor CRUD + spin/torque/jog/stop/estop + sequences + homing + preflight + limit/home editing)

**Dev workflow:** `cargo run -p navi --bin navi -- --no-hardware` starts the server with mock telemetry on http://localhost:8080. Run `cd link && npm run dev` for Vite HMR on port 5173 (proxied to 8080). For production, `cd link && npm run build` then the `navi` binary serves the built frontend from `link/dist/`.

**Status:** Functional with Overview, System, Arms, Test, Settings, and Logs pages. Test Panel has motor selector, live telemetry readout, 5 control tabs (Jog/Spin/Torque/Position/Raw MIT), sequence runner, and global E-STOP. UI polish is ongoing.
Overview includes live Pi telemetry card, **HomingStatusCard** (per-arm homing status with colored indicators and home buttons), and **PreflightAlert** banners for limit violations.
Arms page includes per-joint sliders with **limit proximity indicators** (amber near, red at limit), collapsible **joint config editor** (edit limits and home position, "Set Current as Home" button), and **homing result feedback** after home commands.

## Deployment
- **Windows dev:** COM5 for CAN2USB, `--no-hardware` for frontend-only development.
- **Pi 5 (robot.local):** Raspberry Pi 5, Ubuntu (kernel 6.17), hostname `robot.local`, user `joey`, NOPASSWD sudo. SSH key auth configured from dev machine.
  - **CAN HAT:** Waveshare 2-CH Isolated CAN HAT — MCP2515 overlays in `/boot/firmware/config.txt` (INT_0=GPIO23, INT_1=GPIO25, oscillator=16MHz). Both `can0` and `can1` detected.
  - **`can-setup.service`:** systemd oneshot, brings up `can0` at 1 Mbps on boot. Enabled.
  - **`link.service`:** systemd unit is versioned in-repo at `deploy/link.service` (`ExecStart` → `target/release/navi`). Deploy workflow copies it to `/etc/systemd/system/link.service` and reloads systemd before each restart. Depends on `can-setup.service`. Enabled and running.
  - **Rust:** 1.94.0 installed via rustup, build-essential + pkg-config installed.
  - **Repo path on Pi:** `/home/joey/mr_robot` (cloned from GitHub). Pi's `config/robot.yaml` has `transport: socketcan` (local edit, not committed).
  - **Build on Pi:** `cd ~/mr_robot && cargo build --release --features socketcan`
  - **To update on Pi:** `cd ~/mr_robot && git pull && cargo build --release --features socketcan && sudo systemctl restart link.service`
  - **HTTPS (Tailscale):** In the [Tailscale admin DNS](https://login.tailscale.com/admin/dns), enable MagicDNS and **HTTPS Certificates**. On the Pi, run `deploy/renew-tailscale-cert.sh` to write Let's Encrypt certs to `certs/robot.pem` / `certs/robot-key.pem` (navi loads them; default paths). Open Link at `https://<machine>.<tailnet>.ts.net:8080` from another tailnet device. Re-run the script periodically (~90 day cert lifetime). Details: `deploy/tailscale-https.md`. WebTransport continues to use the built-in identity plus `/api/cert-hash` pinning.
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

Joint limits and home positions can be updated at runtime via the Link UI (Arms page → joint config editor) or API (`PUT /api/joints/{section}/{joint}/limits` and `/home`). Changes persist to `robot.yaml`.

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
- `POST /api/motors/{id}/move` — position move `{ position_rad, kp?, kd? }` (API validates limits)
- `POST /api/motors/{id}/control` — raw MIT control `{ position, velocity, kp, kd, torque }` (API validates limits)
- `POST /api/motors/{id}/spin` — velocity control `{ velocity_rads, kd? }`
- `POST /api/motors/{id}/torque` — torque control `{ torque_nm }`
- `POST /api/motors/{id}/jog` — relative position move `{ delta_deg, kp?, kd? }` (validates target vs limits)
- `POST /api/motors/{id}/stop` — emergency stop (disable) single motor
- `POST /api/estop` — emergency stop ALL motors

**Arm commands:**
- `GET /api/arms` — list arms with joint info
- `POST /api/arms/{side}/enable` — enable all joints
- `POST /api/arms/{side}/disable` — disable all joints
- `POST /api/arms/{side}/home` — startup safe recovery with pre-flight; accepts `{ override_preflight?: bool }`; returns `HomeResponse` with per-joint results and optional preflight data
- `POST /api/arms/{side}/pose` — set joint positions `{ joints: {name: rad}, kp?, kd? }`
- `GET /api/arms/{side}/preflight` — read-only pre-flight limit check (no motor movement)
- `GET /api/arms/{side}/home-status` — read-only per-joint distance-from-home

**Joint configuration:**
- `PUT /api/joints/{section}/{joint}/limits` — update joint limits `{ min_rad, max_rad }`, persists to `robot.yaml`
- `PUT /api/joints/{section}/{joint}/home` — update home position `{ home_rad }` or `{ set_current: true }`, persists to `robot.yaml`

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
- `motors: MotorSnapshot[]` — each motor includes:
  - `can_id`, `joint_name`, `angle_rad`, `velocity_rads`, `torque_nm`, `temperature_c`, `mode`, `faults`, `online`
  - `home_rad` (configured home position)
  - `home_error_rad` (|current − home|)
  - `at_home` (within settle tolerance)
  - `limits` (configured [min_rad, max_rad])
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
