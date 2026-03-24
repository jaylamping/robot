use std::sync::Arc;

use axum::extract::{Path, Query, State};
use axum::http::StatusCode;
use axum::response::IntoResponse;
use axum::routing::{get, post};
use axum::{Json, Router};
use serde::{Deserialize, Serialize};
use tracing::info;

use cortex::motor::Motor;

use crate::AppState;
use crate::telemetry::build_joint_name_map;

#[derive(Serialize)]
struct MotorInfo {
    can_id: u8,
    joint_name: String,
    actuator_type: String,
    limits: (f64, f64),
    online: bool,
}

#[derive(Serialize)]
struct MotorDetail {
    can_id: u8,
    joint_name: String,
    actuator_type: String,
    limits: (f64, f64),
    online: bool,
    angle_rad: f32,
    velocity_rads: f32,
    torque_nm: f32,
    temperature_c: f32,
    mode: String,
    faults: Vec<String>,
}

#[derive(Deserialize)]
struct MoveRequest {
    position_rad: f32,
    kp: Option<f32>,
    kd: Option<f32>,
}

#[derive(Deserialize)]
struct ControlRequest {
    position: f32,
    velocity: f32,
    kp: f32,
    kd: f32,
    torque: f32,
}

#[derive(Serialize)]
struct CommandResponse {
    success: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    error: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    angle_rad: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    velocity_rads: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    torque_nm: Option<f32>,
}

#[derive(Serialize)]
struct StatusResponse {
    uptime_secs: u64,
    mode: String,
    motor_count: usize,
    transport_type: String,
}

#[derive(Serialize)]
struct ArmInfo {
    side: String,
    joints: Vec<ArmJointInfo>,
}

#[derive(Serialize)]
struct ArmJointInfo {
    name: String,
    can_id: Option<u8>,
    actuator: String,
    limits: (f64, f64),
    home_rad: f64,
    online: bool,
}

#[derive(Deserialize)]
struct PoseRequest {
    joints: std::collections::HashMap<String, f32>,
    kp: Option<f32>,
    kd: Option<f32>,
}

#[derive(Deserialize)]
struct SpinRequest {
    velocity_rads: f32,
    kd: Option<f32>,
}

#[derive(Deserialize)]
struct TorqueRequest {
    torque_nm: f32,
}

#[derive(Deserialize)]
struct JogRequest {
    delta_deg: f32,
    kp: Option<f32>,
    kd: Option<f32>,
}

#[derive(Serialize)]
struct SequenceInfo {
    name: String,
    description: String,
}

pub fn routes() -> Router<Arc<AppState>> {
    Router::new()
        .route("/config", get(get_config))
        .route("/cert-hash", get(get_cert_hash))
        .route("/status", get(get_status))
        .route("/motors", get(get_motors))
        .route("/motors/{id}", get(get_motor))
        .route("/motors/{id}/enable", post(enable_motor))
        .route("/motors/{id}/disable", post(disable_motor))
        .route("/motors/{id}/zero", post(zero_motor))
        .route("/motors/{id}/move", post(move_motor))
        .route("/motors/{id}/control", post(control_motor))
        .route("/arms", get(get_arms))
        .route("/arms/{side}/enable", post(enable_arm))
        .route("/arms/{side}/disable", post(disable_arm))
        .route("/arms/{side}/home", post(home_arm))
        .route("/arms/{side}/pose", post(set_arm_pose))
        .route("/motors/{id}/spin", post(spin_motor))
        .route("/motors/{id}/torque", post(torque_motor))
        .route("/motors/{id}/jog", post(jog_motor))
        .route("/motors/{id}/stop", post(stop_motor))
        .route("/estop", post(estop_all))
        .route("/sequences", get(list_sequences))
        .route("/sequences/{name}/run", post(run_sequence))
        .route("/discover", post(discover_motors))
        .route("/logs", get(get_logs))
}

async fn get_config(State(state): State<Arc<AppState>>) -> impl IntoResponse {
    Json(serde_json::to_value(&state.config).unwrap())
}

async fn get_status(State(state): State<Arc<AppState>>) -> impl IntoResponse {
    let motors = state.motors.lock().await;
    Json(StatusResponse {
        uptime_secs: state.start_time.elapsed().as_secs(),
        mode: state.mode.clone(),
        motor_count: motors.len(),
        transport_type: state.transport_type.clone(),
    })
}

#[derive(Deserialize)]
struct LogsQuery {
    limit: Option<usize>,
}

async fn get_logs(
    State(state): State<Arc<AppState>>,
    Query(q): Query<LogsQuery>,
) -> impl IntoResponse {
    let limit = q.limit.unwrap_or(200).min(500);
    Json(state.log_buffer.recent(limit))
}

async fn get_cert_hash(State(state): State<Arc<AppState>>) -> impl IntoResponse {
    #[derive(Serialize)]
    struct CertHash {
        hash_b64: String,
        port: u16,
    }
    Json(CertHash {
        hash_b64: state.cert_hash_b64.clone(),
        port: state.wt_port,
    })
}

async fn get_motors(State(state): State<Arc<AppState>>) -> impl IntoResponse {
    let joint_map = build_joint_name_map(&state);
    let motors = state.motors.lock().await;

    let mut infos: Vec<MotorInfo> = Vec::new();

    for (&can_id, _motor) in motors.iter() {
        let joint_name = joint_map
            .get(&can_id)
            .cloned()
            .unwrap_or_else(|| format!("motor_{}", can_id));

        let (actuator_type, limits) = find_joint_config(&state, can_id);

        infos.push(MotorInfo {
            can_id,
            joint_name,
            actuator_type,
            limits,
            online: true,
        });
    }

    collect_configured_motors(&state, &motors, &joint_map, &mut infos);

    infos.sort_by_key(|m| m.can_id);
    Json(infos)
}

async fn get_motor(
    State(state): State<Arc<AppState>>,
    Path(id): Path<u8>,
) -> Result<impl IntoResponse, StatusCode> {
    let mut motors = state.motors.lock().await;
    let joint_map = build_joint_name_map(&state);

    let motor = motors.get_mut(&id).ok_or(StatusCode::NOT_FOUND)?;
    let joint_name = joint_map
        .get(&id)
        .cloned()
        .unwrap_or_else(|| format!("motor_{}", id));
    let (actuator_type, limits) = find_joint_config(&state, id);

    match motor.read_state().await {
        Ok(ms) => Ok(Json(MotorDetail {
            can_id: id,
            joint_name,
            actuator_type,
            limits,
            online: true,
            angle_rad: ms.angle_rad,
            velocity_rads: ms.velocity_rads,
            torque_nm: ms.torque_nm,
            temperature_c: ms.temperature_c,
            mode: format!("{:?}", ms.mode),
            faults: ms.faults.iter().map(|s| s.to_string()).collect(),
        })),
        Err(_) => Ok(Json(MotorDetail {
            can_id: id,
            joint_name,
            actuator_type,
            limits,
            online: false,
            angle_rad: 0.0,
            velocity_rads: 0.0,
            torque_nm: 0.0,
            temperature_c: 0.0,
            mode: "Unknown".into(),
            faults: vec!["communication error".into()],
        })),
    }
}

async fn enable_motor(
    State(state): State<Arc<AppState>>,
    Path(id): Path<u8>,
) -> Result<impl IntoResponse, StatusCode> {
    let mut motors = state.motors.lock().await;
    let motor = motors.get_mut(&id).ok_or(StatusCode::NOT_FOUND)?;
    match motor.enable().await {
        Ok(ms) => Ok(Json(CommandResponse {
            success: true,
            error: None,
            angle_rad: Some(ms.angle_rad),
            velocity_rads: Some(ms.velocity_rads),
            torque_nm: Some(ms.torque_nm),
        })),
        Err(e) => Ok(Json(CommandResponse {
            success: false,
            error: Some(format!("{:#}", e)),
            angle_rad: None,
            velocity_rads: None,
            torque_nm: None,
        })),
    }
}

async fn disable_motor(
    State(state): State<Arc<AppState>>,
    Path(id): Path<u8>,
) -> Result<impl IntoResponse, StatusCode> {
    let mut motors = state.motors.lock().await;
    let motor = motors.get_mut(&id).ok_or(StatusCode::NOT_FOUND)?;
    match motor.disable().await {
        Ok(ms) => Ok(Json(CommandResponse {
            success: true,
            error: None,
            angle_rad: Some(ms.angle_rad),
            velocity_rads: Some(ms.velocity_rads),
            torque_nm: Some(ms.torque_nm),
        })),
        Err(e) => Ok(Json(CommandResponse {
            success: false,
            error: Some(format!("{:#}", e)),
            angle_rad: None,
            velocity_rads: None,
            torque_nm: None,
        })),
    }
}

async fn zero_motor(
    State(state): State<Arc<AppState>>,
    Path(id): Path<u8>,
) -> Result<impl IntoResponse, StatusCode> {
    let mut motors = state.motors.lock().await;
    let motor = motors.get_mut(&id).ok_or(StatusCode::NOT_FOUND)?;
    match motor.set_zero().await {
        Ok(()) => Ok(Json(CommandResponse {
            success: true,
            error: None,
            angle_rad: None,
            velocity_rads: None,
            torque_nm: None,
        })),
        Err(e) => Ok(Json(CommandResponse {
            success: false,
            error: Some(format!("{:#}", e)),
            angle_rad: None,
            velocity_rads: None,
            torque_nm: None,
        })),
    }
}

async fn move_motor(
    State(state): State<Arc<AppState>>,
    Path(id): Path<u8>,
    Json(req): Json<MoveRequest>,
) -> Result<impl IntoResponse, StatusCode> {
    let mut motors = state.motors.lock().await;
    let motor = motors.get_mut(&id).ok_or(StatusCode::NOT_FOUND)?;
    match motor.move_to(req.position_rad, req.kp, req.kd).await {
        Ok(ms) => Ok(Json(CommandResponse {
            success: true,
            error: None,
            angle_rad: Some(ms.angle_rad),
            velocity_rads: Some(ms.velocity_rads),
            torque_nm: Some(ms.torque_nm),
        })),
        Err(e) => Ok(Json(CommandResponse {
            success: false,
            error: Some(format!("{:#}", e)),
            angle_rad: None,
            velocity_rads: None,
            torque_nm: None,
        })),
    }
}

async fn control_motor(
    State(state): State<Arc<AppState>>,
    Path(id): Path<u8>,
    Json(req): Json<ControlRequest>,
) -> Result<impl IntoResponse, StatusCode> {
    let mut motors = state.motors.lock().await;
    let motor = motors.get_mut(&id).ok_or(StatusCode::NOT_FOUND)?;
    match motor
        .send_control(req.position, req.velocity, req.kp, req.kd, req.torque)
        .await
    {
        Ok(ms) => Ok(Json(CommandResponse {
            success: true,
            error: None,
            angle_rad: Some(ms.angle_rad),
            velocity_rads: Some(ms.velocity_rads),
            torque_nm: Some(ms.torque_nm),
        })),
        Err(e) => Ok(Json(CommandResponse {
            success: false,
            error: Some(format!("{:#}", e)),
            angle_rad: None,
            velocity_rads: None,
            torque_nm: None,
        })),
    }
}

async fn get_arms(State(state): State<Arc<AppState>>) -> impl IntoResponse {
    let mut result = Vec::new();
    let motors = state.motors.lock().await;

    let arm_configs: Vec<(&str, &cortex::config::ArmConfig)> = [
        ("left", state.config.arm_left.as_ref()),
        ("right", state.config.arm_right.as_ref()),
    ]
    .into_iter()
    .filter_map(|(side, arm)| arm.map(|a| (side, a)))
    .collect();

    for (side, arm_cfg) in arm_configs {
        let mut joints = Vec::new();
        for (name, joint) in arm_cfg.joints() {
            let online = joint.can_id.map_or(false, |id| motors.contains_key(&id));
            joints.push(ArmJointInfo {
                name: name.to_string(),
                can_id: joint.can_id,
                actuator: joint.actuator.clone(),
                limits: joint.limits,
                home_rad: joint.home_rad,
                online,
            });
        }
        result.push(ArmInfo {
            side: side.to_string(),
            joints,
        });
    }

    Json(result)
}

async fn enable_arm(
    State(state): State<Arc<AppState>>,
    Path(side): Path<String>,
) -> Result<impl IntoResponse, StatusCode> {
    let mut arms = state.arms.lock().await;
    let arm = arms.get_mut(&side).ok_or(StatusCode::NOT_FOUND)?;
    match arm.enable_all().await {
        Ok(()) => Ok(Json(CommandResponse {
            success: true,
            error: None,
            angle_rad: None,
            velocity_rads: None,
            torque_nm: None,
        })),
        Err(e) => Ok(Json(CommandResponse {
            success: false,
            error: Some(format!("{:#}", e)),
            angle_rad: None,
            velocity_rads: None,
            torque_nm: None,
        })),
    }
}

async fn disable_arm(
    State(state): State<Arc<AppState>>,
    Path(side): Path<String>,
) -> Result<impl IntoResponse, StatusCode> {
    let mut arms = state.arms.lock().await;
    let arm = arms.get_mut(&side).ok_or(StatusCode::NOT_FOUND)?;
    match arm.disable_all().await {
        Ok(()) => Ok(Json(CommandResponse {
            success: true,
            error: None,
            angle_rad: None,
            velocity_rads: None,
            torque_nm: None,
        })),
        Err(e) => Ok(Json(CommandResponse {
            success: false,
            error: Some(format!("{:#}", e)),
            angle_rad: None,
            velocity_rads: None,
            torque_nm: None,
        })),
    }
}

async fn home_arm(
    State(state): State<Arc<AppState>>,
    Path(side): Path<String>,
) -> Result<impl IntoResponse, StatusCode> {
    let mut arms = state.arms.lock().await;
    let arm = arms.get_mut(&side).ok_or(StatusCode::NOT_FOUND)?;
    match arm.startup_safe_recovery().await {
        Ok(summary) => Ok(Json(CommandResponse {
            success: true,
            error: if summary.stall_backoffs > 0 {
                Some(format!("{} stall backoffs during recovery", summary.stall_backoffs))
            } else {
                None
            },
            angle_rad: None,
            velocity_rads: None,
            torque_nm: None,
        })),
        Err(e) => Ok(Json(CommandResponse {
            success: false,
            error: Some(format!("{:#}", e)),
            angle_rad: None,
            velocity_rads: None,
            torque_nm: None,
        })),
    }
}

async fn set_arm_pose(
    State(state): State<Arc<AppState>>,
    Path(side): Path<String>,
    Json(req): Json<PoseRequest>,
) -> Result<impl IntoResponse, StatusCode> {
    let mut arms = state.arms.lock().await;
    let arm = arms.get_mut(&side).ok_or(StatusCode::NOT_FOUND)?;
    let kp = req.kp;
    let kd = req.kd;
    for (joint_name, position_rad) in &req.joints {
        if let Err(e) = arm.set_joint(joint_name, *position_rad, kp, kd).await {
            return Ok(Json(CommandResponse {
                success: false,
                error: Some(format!("joint '{}': {:#}", joint_name, e)),
                angle_rad: None,
                velocity_rads: None,
                torque_nm: None,
            }));
        }
    }
    Ok(Json(CommandResponse {
        success: true,
        error: None,
        angle_rad: None,
        velocity_rads: None,
        torque_nm: None,
    }))
}

async fn spin_motor(
    State(state): State<Arc<AppState>>,
    Path(id): Path<u8>,
    Json(req): Json<SpinRequest>,
) -> Result<impl IntoResponse, StatusCode> {
    let mut motors = state.motors.lock().await;
    let motor = motors.get_mut(&id).ok_or(StatusCode::NOT_FOUND)?;
    match motor.spin(req.velocity_rads, req.kd).await {
        Ok(ms) => Ok(Json(CommandResponse {
            success: true,
            error: None,
            angle_rad: Some(ms.angle_rad),
            velocity_rads: Some(ms.velocity_rads),
            torque_nm: Some(ms.torque_nm),
        })),
        Err(e) => Ok(Json(CommandResponse {
            success: false,
            error: Some(format!("{:#}", e)),
            angle_rad: None,
            velocity_rads: None,
            torque_nm: None,
        })),
    }
}

async fn torque_motor(
    State(state): State<Arc<AppState>>,
    Path(id): Path<u8>,
    Json(req): Json<TorqueRequest>,
) -> Result<impl IntoResponse, StatusCode> {
    let mut motors = state.motors.lock().await;
    let motor = motors.get_mut(&id).ok_or(StatusCode::NOT_FOUND)?;
    match motor.set_torque(req.torque_nm).await {
        Ok(ms) => Ok(Json(CommandResponse {
            success: true,
            error: None,
            angle_rad: Some(ms.angle_rad),
            velocity_rads: Some(ms.velocity_rads),
            torque_nm: Some(ms.torque_nm),
        })),
        Err(e) => Ok(Json(CommandResponse {
            success: false,
            error: Some(format!("{:#}", e)),
            angle_rad: None,
            velocity_rads: None,
            torque_nm: None,
        })),
    }
}

async fn jog_motor(
    State(state): State<Arc<AppState>>,
    Path(id): Path<u8>,
    Json(req): Json<JogRequest>,
) -> Result<impl IntoResponse, StatusCode> {
    let mut motors = state.motors.lock().await;
    let motor = motors.get_mut(&id).ok_or(StatusCode::NOT_FOUND)?;
    match motor.read_position().await {
        Ok(current_rad) => {
            let target_rad = current_rad + req.delta_deg.to_radians();
            match motor.move_to(target_rad, req.kp, req.kd).await {
                Ok(ms) => Ok(Json(CommandResponse {
                    success: true,
                    error: None,
                    angle_rad: Some(ms.angle_rad),
                    velocity_rads: Some(ms.velocity_rads),
                    torque_nm: Some(ms.torque_nm),
                })),
                Err(e) => Ok(Json(CommandResponse {
                    success: false,
                    error: Some(format!("{:#}", e)),
                    angle_rad: None,
                    velocity_rads: None,
                    torque_nm: None,
                })),
            }
        }
        Err(e) => Ok(Json(CommandResponse {
            success: false,
            error: Some(format!("Failed to read position: {:#}", e)),
            angle_rad: None,
            velocity_rads: None,
            torque_nm: None,
        })),
    }
}

async fn stop_motor(
    State(state): State<Arc<AppState>>,
    Path(id): Path<u8>,
) -> Result<impl IntoResponse, StatusCode> {
    let mut motors = state.motors.lock().await;
    let motor = motors.get_mut(&id).ok_or(StatusCode::NOT_FOUND)?;
    match motor.disable().await {
        Ok(ms) => Ok(Json(CommandResponse {
            success: true,
            error: None,
            angle_rad: Some(ms.angle_rad),
            velocity_rads: Some(ms.velocity_rads),
            torque_nm: Some(ms.torque_nm),
        })),
        Err(e) => Ok(Json(CommandResponse {
            success: false,
            error: Some(format!("{:#}", e)),
            angle_rad: None,
            velocity_rads: None,
            torque_nm: None,
        })),
    }
}

async fn estop_all(
    State(state): State<Arc<AppState>>,
) -> impl IntoResponse {
    let mut motors = state.motors.lock().await;
    let mut errors = Vec::new();
    for (id, motor) in motors.iter_mut() {
        if let Err(e) = motor.disable().await {
            errors.push(format!("motor {}: {:#}", id, e));
        }
    }
    if errors.is_empty() {
        Json(CommandResponse {
            success: true,
            error: None,
            angle_rad: None,
            velocity_rads: None,
            torque_nm: None,
        })
    } else {
        Json(CommandResponse {
            success: false,
            error: Some(errors.join("; ")),
            angle_rad: None,
            velocity_rads: None,
            torque_nm: None,
        })
    }
}

async fn list_sequences() -> impl IntoResponse {
    Json(vec![
        SequenceInfo {
            name: "wave".into(),
            description: "Single arm wave demonstration".into(),
        },
        SequenceInfo {
            name: "home_all".into(),
            description: "Return all joints to home position".into(),
        },
        SequenceInfo {
            name: "sweep_test".into(),
            description: "Sweep each joint through its range slowly".into(),
        },
    ])
}

async fn run_sequence(
    State(state): State<Arc<AppState>>,
    Path(name): Path<String>,
) -> Result<impl IntoResponse, StatusCode> {
    match name.as_str() {
        "home_all" => {
            let mut arms = state.arms.lock().await;
            let mut errors = Vec::new();
            for (side, arm) in arms.iter_mut() {
                if let Err(e) = arm.startup_safe_recovery().await {
                    errors.push(format!("{} arm: {:#}", side, e));
                }
            }
            if errors.is_empty() {
                Ok(Json(CommandResponse {
                    success: true,
                    error: None,
                    angle_rad: None,
                    velocity_rads: None,
                    torque_nm: None,
                }))
            } else {
                Ok(Json(CommandResponse {
                    success: false,
                    error: Some(errors.join("; ")),
                    angle_rad: None,
                    velocity_rads: None,
                    torque_nm: None,
                }))
            }
        }
        "wave" | "sweep_test" => Ok(Json(CommandResponse {
            success: false,
            error: Some(format!("Sequence '{}' not yet implemented", name)),
            angle_rad: None,
            velocity_rads: None,
            torque_nm: None,
        })),
        _ => Err(StatusCode::NOT_FOUND),
    }
}

#[derive(Serialize)]
struct DiscoverResult {
    discovered: Vec<u8>,
    removed: Vec<u8>,
    total: usize,
}

async fn discover_motors(
    State(state): State<Arc<AppState>>,
) -> Result<impl IntoResponse, StatusCode> {
    let protocol = state.protocol.as_ref().ok_or_else(|| {
        StatusCode::SERVICE_UNAVAILABLE
    })?;

    let mut motors = state.motors.lock().await;

    let mut discovered: Vec<u8> = Vec::new();
    let mut removed: Vec<u8> = Vec::new();

    for can_id in 1..=127u8 {
        let mut probe = Motor::new(protocol.clone(), can_id);
        match tokio::time::timeout(
            std::time::Duration::from_millis(50),
            probe.read_state(),
        )
        .await
        {
            Ok(Ok(_)) => {
                if !motors.contains_key(&can_id) {
                    info!(can_id, "discover: new motor found");
                    motors.insert(can_id, Motor::new(protocol.clone(), can_id));
                    discovered.push(can_id);
                }
            }
            _ => {
                if motors.contains_key(&can_id) {
                    info!(can_id, "discover: motor no longer responding, removing");
                    if let Some(mut motor) = motors.remove(&can_id) {
                        let _ = motor.disable().await;
                    }
                    removed.push(can_id);
                }
            }
        }
    }

    let total = motors.len();
    drop(motors);

    info!(
        found = discovered.len(),
        removed = removed.len(),
        total,
        "discovery scan complete"
    );

    Ok(Json(DiscoverResult {
        discovered,
        removed,
        total,
    }))
}

fn find_joint_config(state: &AppState, can_id: u8) -> (String, (f64, f64)) {
    let arms = [
        state.config.arm_left.as_ref(),
        state.config.arm_right.as_ref(),
    ];
    for arm in arms.iter().flatten() {
        for (_name, joint) in arm.joints() {
            if joint.can_id == Some(can_id) {
                return (joint.actuator.clone(), joint.limits);
            }
        }
    }
    if let Some(ref waist) = state.config.waist {
        for (_name, joint) in waist {
            if joint.can_id == Some(can_id) {
                return (joint.actuator.clone(), joint.limits);
            }
        }
    }
    ("unknown".into(), (-3.14, 3.14))
}

fn collect_configured_motors(
    state: &AppState,
    motors: &std::collections::HashMap<u8, cortex::motor::Motor>,
    joint_map: &std::collections::HashMap<u8, String>,
    infos: &mut Vec<MotorInfo>,
) {
    let all_configured = all_configured_can_ids(state);
    for (can_id, actuator_type, limits) in all_configured {
        if motors.contains_key(&can_id) {
            continue;
        }
        let joint_name = joint_map
            .get(&can_id)
            .cloned()
            .unwrap_or_else(|| format!("motor_{}", can_id));
        infos.push(MotorInfo {
            can_id,
            joint_name,
            actuator_type,
            limits,
            online: false,
        });
    }
}

fn all_configured_can_ids(state: &AppState) -> Vec<(u8, String, (f64, f64))> {
    let mut ids = Vec::new();
    let arms = [
        state.config.arm_left.as_ref(),
        state.config.arm_right.as_ref(),
    ];
    for arm in arms.iter().flatten() {
        for (_name, joint) in arm.joints() {
            if let Some(id) = joint.can_id {
                ids.push((id, joint.actuator.clone(), joint.limits));
            }
        }
    }
    if let Some(ref waist) = state.config.waist {
        for (_name, joint) in waist {
            if let Some(id) = joint.can_id {
                ids.push((id, joint.actuator.clone(), joint.limits));
            }
        }
    }
    ids
}
