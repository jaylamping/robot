use std::collections::HashMap;
use std::sync::Arc;
use std::time::Duration;

use serde::Serialize;
use tracing::{debug, info, warn};

use crate::AppState;

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
}

#[derive(Debug, Clone, Serialize)]
pub struct TelemetrySnapshot {
    pub timestamp_ms: u64,
    pub motors: Vec<MotorSnapshot>,
}

/// Polls all motors at `rate_hz` and broadcasts snapshots.
/// When `mock` is true (no hardware), generates zeroed telemetry.
pub async fn telemetry_loop(state: Arc<AppState>, rate_hz: u32, mock: bool) {
    let period = Duration::from_micros(1_000_000 / rate_hz as u64);
    let start = std::time::Instant::now();

    loop {
        let snapshot = if mock {
            build_mock_snapshot(&state, start.elapsed().as_millis() as u64).await
        } else {
            build_live_snapshot(&state, start.elapsed().as_millis() as u64).await
        };

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

async fn build_mock_snapshot(state: &AppState, timestamp_ms: u64) -> TelemetrySnapshot {
    let joint_map = build_joint_name_map(state).await;
    let mut motors = Vec::new();

    for (&can_id, joint_name) in &joint_map {
        motors.push(MotorSnapshot {
            can_id,
            joint_name: joint_name.clone(),
            angle_rad: 0.0,
            velocity_rads: 0.0,
            torque_nm: 0.0,
            temperature_c: 25.0,
            mode: "Reset".into(),
            faults: vec![],
            online: false,
        });
    }

    TelemetrySnapshot {
        timestamp_ms,
        motors,
    }
}

async fn build_live_snapshot(state: &AppState, timestamp_ms: u64) -> TelemetrySnapshot {
    let mut motors_guard = state.motors.lock().await;
    let joint_map = build_joint_name_map(state).await;
    let mut motors = Vec::new();

    for (&can_id, motor) in motors_guard.iter_mut() {
        let joint_name = joint_map
            .get(&can_id)
            .cloned()
            .unwrap_or_else(|| format!("motor_{}", can_id));

        let result = tokio::time::timeout(
            Duration::from_millis(100),
            motor.read_state(),
        ).await;

        match result {
            Ok(Ok(ms)) => {
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
                });
            }
            Ok(Err(e)) => {
                warn!(can_id, error = %e, "failed to read motor state");
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
                });
            }
            Err(_) => {
                warn!(can_id, "motor read timed out");
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
                });
            }
        }
    }

    TelemetrySnapshot {
        timestamp_ms,
        motors,
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
