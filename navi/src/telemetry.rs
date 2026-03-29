use std::collections::HashMap;
use std::sync::Arc;
use std::time::Duration;

use serde::Serialize;
use sysinfo::{Components, CpuRefreshKind, MemoryRefreshKind, RefreshKind, System};
use tracing::{debug, info, warn};

use crate::AppState;

/// Single-motor `read_state` budget per telemetry tick. USB/CAN round-trips often exceed 100ms
/// under load; marking motors offline spuriously breaks the Link Overview.
const MOTOR_READ_TIMEOUT: Duration = Duration::from_millis(400);

/// After this many consecutive read failures a motor is temporarily skipped to avoid blocking
/// the bus for motors that are actually online.
const CONSECUTIVE_FAIL_SKIP_THRESHOLD: u32 = 5;

/// When a motor has been skipped due to consecutive failures, re-probe it every N ticks.
const REPROBE_INTERVAL_TICKS: u32 = 40;

#[derive(Debug, Clone, Serialize)]
pub struct MotorSnapshot {
    pub can_id: u8,
    pub joint_name: String,
    pub angle_rad: f32,
    pub velocity_rads: f32,
    pub torque_nm: f32,
    pub temperature_c: f32,
    pub mode: String,
    pub faults: Vec<String>,
    pub online: bool,
    pub home_rad: Option<f32>,
    pub home_error_rad: Option<f32>,
    pub at_home: bool,
    pub limits: Option<(f32, f32)>,
}

#[derive(Debug, Clone, Serialize)]
pub struct TelemetrySnapshot {
    pub timestamp_ms: u64,
    pub motors: Vec<MotorSnapshot>,
    pub system: SystemSnapshot,
}

#[derive(Debug, Clone, Serialize)]
pub struct SystemSnapshot {
    pub cpu_usage_percent: f32,
    pub memory_used_mb: u64,
    pub memory_total_mb: u64,
    pub temperature_c: Option<f32>,
}

/// Polls all motors at `rate_hz` and broadcasts snapshots.
/// When `mock` is true (no hardware), generates zeroed telemetry.
pub async fn telemetry_loop(state: Arc<AppState>, rate_hz: u32, mock: bool) {
    let period = Duration::from_micros(1_000_000 / rate_hz as u64);
    let start = std::time::Instant::now();
    let mut system_telemetry = SystemTelemetryCollector::new();
    let mut health = MotorHealthTracker::new();

    loop {
        let system = system_telemetry.sample();
        let snapshot = if mock {
            build_mock_snapshot(&state, start.elapsed().as_millis() as u64, system).await
        } else {
            health.tick();
            build_live_snapshot(&state, start.elapsed().as_millis() as u64, system, &mut health).await
        };

        *state.latest_telemetry.write().await = Some(snapshot.clone());

        if state.telemetry_tx.send(snapshot).is_err() {
            debug!("no telemetry subscribers");
        }

        tokio::time::sleep(period).await;
    }
}

/// Accept WebTransport sessions and stream telemetry datagrams.
pub async fn webtransport_server(state: Arc<AppState>, port: u16, identity: wtransport::Identity) {
    let config = wtransport::ServerConfig::builder()
        .with_bind_default(port)
        .with_identity(identity)
        .build();

    let server = match wtransport::Endpoint::server(config) {
        Ok(s) => s,
        Err(e) => {
            tracing::error!(error = %e, "failed to start WebTransport server");
            return;
        }
    };

    info!(port, "WebTransport server listening");

    loop {
        let incoming = server.accept().await;
        let session_request = match incoming.await {
            Ok(req) => req,
            Err(e) => {
                warn!(error = %e, "WebTransport incoming connection failed");
                continue;
            }
        };

        let session = match session_request.accept().await {
            Ok(s) => s,
            Err(e) => {
                warn!(error = %e, "WebTransport session accept failed");
                continue;
            }
        };

        info!("WebTransport session established");

        let mut rx = state.telemetry_tx.subscribe();
        tokio::spawn(async move {
            loop {
                match rx.recv().await {
                    Ok(snapshot) => {
                        let json = match serde_json::to_vec(&snapshot) {
                            Ok(j) => j,
                            Err(e) => {
                                warn!(error = %e, "failed to serialize telemetry");
                                continue;
                            }
                        };
                        if let Err(e) = session.send_datagram(json) {
                            debug!(error = %e, "datagram send failed, client likely disconnected");
                            break;
                        }
                    }
                    Err(tokio::sync::broadcast::error::RecvError::Lagged(n)) => {
                        debug!(skipped = n, "telemetry subscriber lagged");
                    }
                    Err(tokio::sync::broadcast::error::RecvError::Closed) => {
                        break;
                    }
                }
            }
            info!("WebTransport session ended");
        });
    }
}

/// Build a CAN ID -> (home_rad, limits, settle_tolerance) lookup from config.
async fn build_home_info(state: &AppState) -> HashMap<u8, (f32, (f32, f32), f32)> {
    let config = state.config.read().await;
    let mut map = HashMap::new();

    let arm_configs: Vec<&cortex::config::ArmConfig> = [
        config.arm_left.as_ref(),
        config.arm_right.as_ref(),
    ]
    .into_iter()
    .flatten()
    .collect();

    for arm in arm_configs {
        for (_, joint) in arm.joints() {
            if let Some(id) = joint.can_id {
                let settle = joint.startup_recovery.settle_tolerance_rad as f32;
                map.insert(id, (
                    joint.home_rad as f32,
                    (joint.limits.0 as f32, joint.limits.1 as f32),
                    settle,
                ));
            }
        }
    }
    map
}

async fn build_mock_snapshot(
    state: &AppState,
    timestamp_ms: u64,
    system: SystemSnapshot,
) -> TelemetrySnapshot {
    let joint_map = build_joint_name_map(state).await;
    let home_info = build_home_info(state).await;
    let mut motors = Vec::new();

    for (&can_id, joint_name) in &joint_map {
        let (home_rad, limits, _settle) = home_info.get(&can_id)
            .copied()
            .unwrap_or((0.0, (-12.57, 12.57), 0.03));

        motors.push(MotorSnapshot {
            can_id,
            joint_name: joint_name.clone(),
            angle_rad: 0.0,
            velocity_rads: 0.0,
            torque_nm: 0.0,
            temperature_c: 25.0,
            mode: "Reset".into(),
            faults: vec![],
            online: true,
            home_rad: Some(home_rad),
            home_error_rad: Some(home_rad.abs()),
            at_home: home_rad.abs() <= 0.03,
            limits: Some(limits),
        });
    }

    TelemetrySnapshot {
        timestamp_ms,
        motors,
        system,
    }
}

/// Per-motor failure tracking so one dead motor doesn't block the rest of the bus.
struct MotorHealthTracker {
    consecutive_failures: HashMap<u8, u32>,
    tick_count: u32,
}

impl MotorHealthTracker {
    fn new() -> Self {
        Self {
            consecutive_failures: HashMap::new(),
            tick_count: 0,
        }
    }

    fn tick(&mut self) {
        self.tick_count = self.tick_count.wrapping_add(1);
    }

    fn should_skip(&self, can_id: u8) -> bool {
        let fails = self.consecutive_failures.get(&can_id).copied().unwrap_or(0);
        if fails < CONSECUTIVE_FAIL_SKIP_THRESHOLD {
            return false;
        }
        self.tick_count % REPROBE_INTERVAL_TICKS != 0
    }

    fn record_success(&mut self, can_id: u8) {
        self.consecutive_failures.remove(&can_id);
    }

    fn record_failure(&mut self, can_id: u8) {
        let entry = self.consecutive_failures.entry(can_id).or_insert(0);
        *entry = entry.saturating_add(1);
    }
}

async fn build_live_snapshot(
    state: &AppState,
    timestamp_ms: u64,
    system: SystemSnapshot,
    health: &mut MotorHealthTracker,
) -> TelemetrySnapshot {
    let joint_map = build_joint_name_map(state).await;
    let home_info = build_home_info(state).await;

    let motor_ids: Vec<u8> = {
        let motors_guard = state.motors.lock().await;
        motors_guard.keys().copied().collect()
    };

    let mut motors = Vec::new();

    for can_id in motor_ids {
        let joint_name = joint_map
            .get(&can_id)
            .cloned()
            .unwrap_or_else(|| format!("motor_{}", can_id));

        let (home_rad, limits, settle) = home_info.get(&can_id)
            .copied()
            .unwrap_or((0.0, (-12.57, 12.57), 0.03));

        if health.should_skip(can_id) {
            motors.push(MotorSnapshot {
                can_id,
                joint_name,
                angle_rad: 0.0,
                velocity_rads: 0.0,
                torque_nm: 0.0,
                temperature_c: 0.0,
                mode: "Unknown".into(),
                faults: vec!["skipped (not responding)".into()],
                online: false,
                home_rad: Some(home_rad),
                home_error_rad: None,
                at_home: false,
                limits: Some(limits),
            });
            continue;
        }

        let result = {
            let mut motors_guard = state.motors.lock().await;
            if let Some(motor) = motors_guard.get_mut(&can_id) {
                Some(tokio::time::timeout(MOTOR_READ_TIMEOUT, motor.read_state_validated()).await)
            } else {
                None
            }
        };

        match result {
            Some(Ok(Ok(ms))) => {
                health.record_success(can_id);
                let home_err = (ms.angle_rad - home_rad).abs();
                motors.push(MotorSnapshot {
                    can_id,
                    joint_name,
                    angle_rad: ms.angle_rad,
                    velocity_rads: ms.velocity_rads,
                    torque_nm: ms.torque_nm,
                    temperature_c: ms.temperature_c,
                    mode: format!("{:?}", ms.mode),
                    faults: ms.faults.iter().map(|s| s.to_string()).collect(),
                    online: true,
                    home_rad: Some(home_rad),
                    home_error_rad: Some(home_err),
                    at_home: home_err <= settle,
                    limits: Some(limits),
                });
            }
            Some(Ok(Err(e))) => {
                health.record_failure(can_id);
                let fails = health.consecutive_failures.get(&can_id).copied().unwrap_or(0);
                warn!(can_id, error = %e, consecutive_failures = fails, "failed to read motor state");
                motors.push(MotorSnapshot {
                    can_id,
                    joint_name,
                    angle_rad: 0.0,
                    velocity_rads: 0.0,
                    torque_nm: 0.0,
                    temperature_c: 0.0,
                    mode: "Unknown".into(),
                    faults: vec![format!("read error: {}", e)],
                    online: false,
                    home_rad: Some(home_rad),
                    home_error_rad: None,
                    at_home: false,
                    limits: Some(limits),
                });
            }
            Some(Err(_)) => {
                health.record_failure(can_id);
                let fails = health.consecutive_failures.get(&can_id).copied().unwrap_or(0);
                warn!(can_id, consecutive_failures = fails, "motor read timed out");
                motors.push(MotorSnapshot {
                    can_id,
                    joint_name,
                    angle_rad: 0.0,
                    velocity_rads: 0.0,
                    torque_nm: 0.0,
                    temperature_c: 0.0,
                    mode: "Unknown".into(),
                    faults: vec!["timeout".into()],
                    online: false,
                    home_rad: Some(home_rad),
                    home_error_rad: None,
                    at_home: false,
                    limits: Some(limits),
                });
            }
            None => {
                motors.push(MotorSnapshot {
                    can_id,
                    joint_name,
                    angle_rad: 0.0,
                    velocity_rads: 0.0,
                    torque_nm: 0.0,
                    temperature_c: 0.0,
                    mode: "Unknown".into(),
                    faults: vec!["motor removed".into()],
                    online: false,
                    home_rad: Some(home_rad),
                    home_error_rad: None,
                    at_home: false,
                    limits: Some(limits),
                });
            }
        }
    }

    TelemetrySnapshot {
        timestamp_ms,
        motors,
        system,
    }
}

struct SystemTelemetryCollector {
    sys: System,
    components: Components,
}

impl SystemTelemetryCollector {
    fn new() -> Self {
        let mut sys = System::new_with_specifics(
            RefreshKind::nothing()
                .with_cpu(CpuRefreshKind::everything())
                .with_memory(MemoryRefreshKind::everything()),
        );
        sys.refresh_cpu_usage();
        sys.refresh_memory();

        Self {
            sys,
            components: Components::new_with_refreshed_list(),
        }
    }

    fn sample(&mut self) -> SystemSnapshot {
        self.sys.refresh_cpu_usage();
        self.sys.refresh_memory();
        self.components.refresh(false);

        let memory_total_mb = self.sys.total_memory() / (1024 * 1024);
        let memory_used_mb = self.sys.used_memory() / (1024 * 1024);
        let temperature_c = self
            .components
            .iter()
            .filter_map(|component| component.temperature())
            .max_by(f32::total_cmp);

        SystemSnapshot {
            cpu_usage_percent: self.sys.global_cpu_usage(),
            memory_used_mb,
            memory_total_mb,
            temperature_c,
        }
    }
}

/// Build a CAN ID -> joint name lookup from the robot config.
pub async fn build_joint_name_map(state: &AppState) -> HashMap<u8, String> {
    let config = state.config.read().await;
    let mut map = HashMap::new();

    let arms: Vec<(&str, &cortex::config::ArmConfig)> = [
        ("left", config.arm_left.as_ref()),
        ("right", config.arm_right.as_ref()),
    ]
    .into_iter()
    .filter_map(|(side, arm)| arm.map(|a| (side, a)))
    .collect();

    for (side, arm) in arms {
        for (name, joint) in arm.joints() {
            if let Some(id) = joint.can_id {
                map.insert(id, format!("{}_{}", side, name));
            }
        }
    }

    if let Some(ref waist) = config.waist {
        for (name, joint) in waist {
            if let Some(id) = joint.can_id {
                map.insert(id, format!("waist_{}", name));
            }
        }
    }

    map
}
