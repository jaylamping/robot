use std::sync::atomic::Ordering;
use std::sync::Arc;

use axum::extract::{Path, Query, State};
use axum::http::StatusCode;
use axum::response::IntoResponse;
use axum::routing::{get, post, put};
use axum::{Json, Router};
use serde::{Deserialize, Serialize};
use tokio_util::sync::CancellationToken;
use tracing::{info, warn};

use cortex::arm::PreflightResult;
use cortex::motor::Motor;
use cortex::safety;

use crate::AppState;
use crate::telemetry::build_joint_name_map;

use tokio::sync::Mutex;

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
struct CommissioningResponse {
    enabled: bool,
}

#[derive(Deserialize)]
struct CommissioningRequest {
    enabled: bool,
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

// -- Homing response types --

#[derive(Serialize)]
struct HomeResponse {
    success: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    error: Option<String>,
    joints: Vec<JointHomingResultJson>,
    #[serde(skip_serializing_if = "Option::is_none")]
    preflight: Option<PreflightResult>,
}

#[derive(Serialize)]
struct JointHomingResultJson {
    joint_name: String,
    status: String,
    start_position_rad: f32,
    end_position_rad: f32,
    home_target_rad: f32,
    error_rad: f32,
    stall_backoffs: u32,
    duration_ms: u64,
}

// -- Limit/home update types --

#[derive(Deserialize)]
struct UpdateLimitsRequest {
    min_rad: f64,
    max_rad: f64,
}

#[derive(Deserialize)]
struct UpdateHomeRequest {
    #[serde(default)]
    home_rad: Option<f64>,
    #[serde(default)]
    set_current: bool,
}

#[derive(Deserialize)]
struct HomeArmRequest {
    #[serde(default)]
    override_preflight: bool,
}

#[derive(Serialize)]
struct JointHomeStatusJson {
    joint_name: String,
    home_rad: f32,
    current_rad: f32,
    error_rad: f32,
    at_home: bool,
    limits: (f32, f32),
}

pub fn routes() -> Router<Arc<AppState>> {
    Router::new()
        .route("/config", get(get_config))
        .route("/cert-hash", get(get_cert_hash))
        .route("/status", get(get_status))
        .route(
            "/safety/commissioning",
            get(get_commissioning).post(set_commissioning),
        )
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
        .route("/arms/{side}/preflight", get(preflight_arm))
        .route("/arms/{side}/home-status", get(home_status_arm))
        .route("/motors/{id}/spin", post(spin_motor))
        .route("/motors/{id}/torque", post(torque_motor))
        .route("/motors/{id}/jog", post(jog_motor))
        .route("/motors/{id}/stop", post(stop_motor))
        .route("/estop", post(estop_all))
        .route("/sequences", get(list_sequences))
        .route("/sequences/{name}/run", post(run_sequence))
        .route("/discover", post(discover_motors))
        .route("/telemetry", get(get_telemetry))
        .route("/logs", get(get_logs))
        .route("/motors/{id}/assign", post(assign_motor))
        .route("/motors/{id}/unassign", post(unassign_motor))
        .route("/joint-slots", get(get_joint_slots))
        .route("/joints/{section}/{joint}/limits", put(update_joint_limits))
        .route("/joints/{section}/{joint}/home", put(update_joint_home))
        .route(
            "/arms/{side}/joints/{joint}/sweep/start",
            post(start_sweep),
        )
        .route("/arms/{side}/joints/{joint}/sweep/stop", post(stop_sweep))
}

async fn get_config(State(state): State<Arc<AppState>>) -> impl IntoResponse {
    let config = state.config.read().await;
    Json(serde_json::to_value(&*config).unwrap())
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

/// LAN-trusted toggle: when enabled, `/motors/{id}/spin` and `/torque` skip strict limit-direction checks.
async fn get_commissioning(State(state): State<Arc<AppState>>) -> impl IntoResponse {
    Json(CommissioningResponse {
        enabled: state.commissioning_enabled.load(Ordering::SeqCst),
    })
}

async fn set_commissioning(
    State(state): State<Arc<AppState>>,
    Json(req): Json<CommissioningRequest>,
) -> impl IntoResponse {
    state
        .commissioning_enabled
        .store(req.enabled, Ordering::SeqCst);
    if req.enabled {
        warn!(
            "commissioning mode ENABLED — spin/torque limit-direction API rejection disabled (LAN-trusted; no auth)"
        );
    }
    Json(CommissioningResponse {
        enabled: state.commissioning_enabled.load(Ordering::SeqCst),
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

async fn get_telemetry(
    State(state): State<Arc<AppState>>,
) -> impl IntoResponse {
    let cached = state.latest_telemetry.read().await;
    match cached.as_ref() {
        Some(snapshot) => Json(serde_json::to_value(snapshot).unwrap()).into_response(),
        None => StatusCode::SERVICE_UNAVAILABLE.into_response(),
    }
}

async fn get_motors(State(state): State<Arc<AppState>>) -> impl IntoResponse {
    let joint_map = build_joint_name_map(&state).await;
    let motors = state.motors.lock().await;
    let config = state.config.read().await;
    let latest = state.latest_telemetry.read().await;

    let mut infos: Vec<MotorInfo> = Vec::new();

    for (&can_id, _motor) in motors.iter() {
        let joint_name = joint_map
            .get(&can_id)
            .cloned()
            .unwrap_or_else(|| format!("motor_{}", can_id));

        let (actuator_type, limits) = find_joint_config(&config, can_id);

        let online = latest.as_ref()
            .and_then(|snap| snap.motors.iter().find(|m| m.can_id == can_id))
            .map(|m| m.online)
            .unwrap_or(true);

        infos.push(MotorInfo {
            can_id,
            joint_name,
            actuator_type,
            limits,
            online,
        });
    }

    collect_configured_motors(&config, &motors, &joint_map, &mut infos);

    infos.sort_by_key(|m| m.can_id);
    Json(infos)
}

async fn get_motor(
    State(state): State<Arc<AppState>>,
    Path(id): Path<u8>,
) -> Result<impl IntoResponse, StatusCode> {
    let motor_arc = {
        let motors = state.motors.lock().await;
        motors.get(&id).cloned().ok_or(StatusCode::NOT_FOUND)?
    };
    let joint_map = build_joint_name_map(&state).await;
    let joint_name = joint_map
        .get(&id)
        .cloned()
        .unwrap_or_else(|| format!("motor_{}", id));
    let config = state.config.read().await;
    let (actuator_type, limits) = find_joint_config(&config, id);
    drop(config);

    let mut motor = motor_arc.lock().await;
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
    let motor_arc = {
        let motors = state.motors.lock().await;
        motors.get(&id).cloned().ok_or(StatusCode::NOT_FOUND)?
    };
    let mut motor = motor_arc.lock().await;
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
    let motor_arc = {
        let motors = state.motors.lock().await;
        motors.get(&id).cloned().ok_or(StatusCode::NOT_FOUND)?
    };
    let mut motor = motor_arc.lock().await;
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
    let motor_arc = {
        let motors = state.motors.lock().await;
        motors.get(&id).cloned().ok_or(StatusCode::NOT_FOUND)?
    };
    if let Err(e) = motor_arc.lock().await.set_zero().await {
        return Ok(Json(CommandResponse {
            success: false,
            error: Some(format!("{:#}", e)),
            angle_rad: None,
            velocity_rads: None,
            torque_nm: None,
        }));
    }

    Ok(Json(CommandResponse {
        success: true,
        error: None,
        angle_rad: None,
        velocity_rads: None,
        torque_nm: None,
    }))
}

async fn move_motor(
    State(state): State<Arc<AppState>>,
    Path(id): Path<u8>,
    Json(req): Json<MoveRequest>,
) -> Result<impl IntoResponse, StatusCode> {
    if let Some(err) = check_motor_limits(&state, id, Some(req.position_rad)).await {
        return Ok(Json(err));
    }
    let motor_arc = {
        let motors = state.motors.lock().await;
        motors.get(&id).cloned().ok_or(StatusCode::NOT_FOUND)?
    };
    let mut motor = motor_arc.lock().await;
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
    if let Some(err) = check_motor_limits(&state, id, Some(req.position)).await {
        return Ok(Json(err));
    }
    let motor_arc = {
        let motors = state.motors.lock().await;
        motors.get(&id).cloned().ok_or(StatusCode::NOT_FOUND)?
    };
    let mut motor = motor_arc.lock().await;
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
    let motor_ids: Vec<u8> = {
        let motors = state.motors.lock().await;
        motors.keys().copied().collect()
    };
    let config = state.config.read().await;

    let arm_configs: Vec<(&str, &cortex::config::ArmConfig)> = [
        ("left", config.arm_left.as_ref()),
        ("right", config.arm_right.as_ref()),
    ]
    .into_iter()
    .filter_map(|(side, arm)| arm.map(|a| (side, a)))
    .collect();

    for (side, arm_cfg) in arm_configs {
        let mut joints = Vec::new();
        for (name, joint) in arm_cfg.joints() {
            let online = joint.can_id.map_or(false, |id| motor_ids.contains(&id));
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
    let arms = state.arms.lock().await;
    let arm = arms.get(&side).ok_or(StatusCode::NOT_FOUND)?;
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
    let arms = state.arms.lock().await;
    let arm = arms.get(&side).ok_or(StatusCode::NOT_FOUND)?;
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
    body: Option<Json<HomeArmRequest>>,
) -> Result<impl IntoResponse, StatusCode> {
    let force = body.map_or(false, |b| b.override_preflight);

    let arms = state.arms.lock().await;
    let arm = arms.get(&side).ok_or(StatusCode::NOT_FOUND)?;

    if !force {
        match arm.preflight_check().await {
            Ok(pf) if !pf.pass => {
                let violations: Vec<String> = pf.joints.iter()
                    .filter_map(|j| j.violation.as_ref().map(|v| {
                        format!("{}: {:.1}° past {} limit", j.joint_name, v.exceeded_by_deg, v.which_limit)
                    }))
                    .collect();
                return Ok(Json(HomeResponse {
                    success: false,
                    error: Some(format!("Pre-flight check failed: {}", violations.join("; "))),
                    joints: vec![],
                    preflight: Some(pf),
                }));
            }
            Err(e) => {
                return Ok(Json(HomeResponse {
                    success: false,
                    error: Some(format!("Pre-flight check error: {:#}", e)),
                    joints: vec![],
                    preflight: None,
                }));
            }
            _ => {}
        }
    }

    match arm.startup_safe_recovery(force).await {
        Ok(summary) => {
            let joints: Vec<JointHomingResultJson> = summary.joints.iter().map(|j| {
                JointHomingResultJson {
                    joint_name: j.joint_name.clone(),
                    status: j.status.as_str().to_string(),
                    start_position_rad: j.start_position_rad,
                    end_position_rad: j.end_position_rad,
                    home_target_rad: j.home_target_rad,
                    error_rad: j.error_rad,
                    stall_backoffs: j.stall_backoffs,
                    duration_ms: j.duration_ms,
                }
            }).collect();

            Ok(Json(HomeResponse {
                success: true,
                error: if summary.stall_backoffs > 0 {
                    Some(format!("{} stall backoffs during recovery", summary.stall_backoffs))
                } else {
                    None
                },
                joints,
                preflight: None,
            }))
        }
        Err(e) => Ok(Json(HomeResponse {
            success: false,
            error: Some(format!("{:#}", e)),
            joints: vec![],
            preflight: None,
        })),
    }
}

async fn preflight_arm(
    State(state): State<Arc<AppState>>,
    Path(side): Path<String>,
) -> Result<impl IntoResponse, StatusCode> {
    let arms = state.arms.lock().await;
    let arm = arms.get(&side).ok_or(StatusCode::NOT_FOUND)?;
    match arm.preflight_check().await {
        Ok(result) => Ok(Json(result).into_response()),
        Err(e) => Ok(Json(CommandResponse {
            success: false,
            error: Some(format!("{:#}", e)),
            angle_rad: None,
            velocity_rads: None,
            torque_nm: None,
        }).into_response()),
    }
}

async fn home_status_arm(
    State(state): State<Arc<AppState>>,
    Path(side): Path<String>,
) -> Result<impl IntoResponse, StatusCode> {
    let arms = state.arms.lock().await;
    let arm = arms.get(&side).ok_or(StatusCode::NOT_FOUND)?;
    match arm.get_homing_status().await {
        Ok(statuses) => {
            let json: Vec<JointHomeStatusJson> = statuses.into_iter().map(|s| {
                JointHomeStatusJson {
                    joint_name: s.joint_name,
                    home_rad: s.home_rad,
                    current_rad: s.current_rad,
                    error_rad: s.error_rad,
                    at_home: s.at_home,
                    limits: s.limits,
                }
            }).collect();
            Ok(Json(json).into_response())
        }
        Err(e) => Ok(Json(CommandResponse {
            success: false,
            error: Some(format!("{:#}", e)),
            angle_rad: None,
            velocity_rads: None,
            torque_nm: None,
        }).into_response()),
    }
}

async fn set_arm_pose(
    State(state): State<Arc<AppState>>,
    Path(side): Path<String>,
    Json(req): Json<PoseRequest>,
) -> Result<impl IntoResponse, StatusCode> {
    let arms = state.arms.lock().await;
    let arm = arms.get(&side).ok_or(StatusCode::NOT_FOUND)?;
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
    let motor_arc = {
        let motors = state.motors.lock().await;
        motors.get(&id).cloned().ok_or(StatusCode::NOT_FOUND)?
    };
    let mut motor = motor_arc.lock().await;
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
    let motor_arc = {
        let motors = state.motors.lock().await;
        motors.get(&id).cloned().ok_or(StatusCode::NOT_FOUND)?
    };
    let mut motor = motor_arc.lock().await;
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
    let motor_arc = {
        let motors = state.motors.lock().await;
        motors.get(&id).cloned().ok_or(StatusCode::NOT_FOUND)?
    };
    let mut motor = motor_arc.lock().await;
    match motor.read_position().await {
        Ok(raw_pos) => {
            let canonical = safety::canonical_position_for_limits(
                raw_pos,
                motor.home_rad(),
                motor.joint_limits(),
            );
            let target_rad = canonical + req.delta_deg.to_radians();

            if let Some(limits) = motor.joint_limits() {
                if target_rad < limits.0 || target_rad > limits.1 {
                    return Ok(Json(CommandResponse {
                        success: false,
                        error: Some(format!(
                            "jog target {:.3} rad ({:.1} deg) exceeds limits [{:.3}, {:.3}]",
                            target_rad, target_rad.to_degrees(), limits.0, limits.1
                        )),
                        angle_rad: Some(canonical),
                        velocity_rads: None,
                        torque_nm: None,
                    }));
                }
            }

            // Compute the actual encoder-frame command from the canonical target
            let cmd_pos = if let Some(limits) = motor.joint_limits() {
                safety::motor_cmd_for_joint_target(raw_pos, target_rad, limits)
            } else {
                target_rad
            };

            match motor.move_to(cmd_pos, req.kp, req.kd).await {
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
    let motor_arc = {
        let motors = state.motors.lock().await;
        motors.get(&id).cloned().ok_or(StatusCode::NOT_FOUND)?
    };
    let mut motor = motor_arc.lock().await;
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
    let motor_arcs: Vec<(u8, Arc<Mutex<Motor>>)> = {
        let motors = state.motors.lock().await;
        motors.iter().map(|(&id, m)| (id, m.clone())).collect()
    };
    let mut errors = Vec::new();
    for (id, motor_arc) in &motor_arcs {
        if let Err(e) = motor_arc.lock().await.disable().await {
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
            description: "Per-joint sweep — use POST /api/arms/{side}/joints/{joint}/sweep/start".into(),
        },
    ])
}

async fn run_sequence(
    State(state): State<Arc<AppState>>,
    Path(name): Path<String>,
) -> Result<impl IntoResponse, StatusCode> {
    match name.as_str() {
        "home_all" => {
            let arms = state.arms.lock().await;
            let mut errors = Vec::new();
            for (side, arm) in arms.iter() {
                if let Err(e) = arm.startup_safe_recovery(false).await {
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
        "wave" => Ok(Json(CommandResponse {
            success: false,
            error: Some("Sequence 'wave' not yet implemented".into()),
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
                    let mut motor = Motor::new(protocol.clone(), can_id);
                    let config = state.config.read().await;
                    if let Some((lo, hi)) = safety::limits_for_motor(&config, can_id) {
                        motor.set_joint_limits(lo, hi);
                    }
                    if let Some(home) = safety::home_for_motor(&config, can_id) {
                        motor.set_home_rad(home);
                    }
                    drop(config);
                    motors.insert(can_id, Arc::new(Mutex::new(motor)));
                    discovered.push(can_id);
                }
            }
            _ => {
                if motors.contains_key(&can_id) {
                    info!(can_id, "discover: motor no longer responding, removing");
                    if let Some(motor_arc) = motors.remove(&can_id) {
                        let _ = motor_arc.lock().await.disable().await;
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

#[derive(Deserialize)]
struct AssignRequest {
    section: String,
    joint: String,
}

#[derive(Serialize)]
struct JointSlot {
    section: String,
    joint: String,
    can_id: Option<u8>,
    display_name: String,
}

async fn assign_motor(
    State(state): State<Arc<AppState>>,
    Path(id): Path<u8>,
    Json(req): Json<AssignRequest>,
) -> impl IntoResponse {
    let mut config = state.config.write().await;

    if let Err(e) = config.assign_can_id(&req.section, &req.joint, id) {
        return Json(CommandResponse {
            success: false,
            error: Some(format!("{:#}", e)),
            angle_rad: None,
            velocity_rads: None,
            torque_nm: None,
        });
    }

    if let Err(e) = config.save(&state.config_path) {
        return Json(CommandResponse {
            success: false,
            error: Some(format!("Config updated in memory but failed to save to disk: {:#}", e)),
            angle_rad: None,
            velocity_rads: None,
            torque_nm: None,
        });
    }

    info!(can_id = id, section = %req.section, joint = %req.joint, "motor assigned to joint");

    Json(CommandResponse {
        success: true,
        error: None,
        angle_rad: None,
        velocity_rads: None,
        torque_nm: None,
    })
}

async fn unassign_motor(
    State(state): State<Arc<AppState>>,
    Path(id): Path<u8>,
) -> impl IntoResponse {
    let mut config = state.config.write().await;

    config.clear_can_id(id);

    if let Err(e) = config.save(&state.config_path) {
        return Json(CommandResponse {
            success: false,
            error: Some(format!("Config updated in memory but failed to save to disk: {:#}", e)),
            angle_rad: None,
            velocity_rads: None,
            torque_nm: None,
        });
    }

    info!(can_id = id, "motor unassigned");

    Json(CommandResponse {
        success: true,
        error: None,
        angle_rad: None,
        velocity_rads: None,
        torque_nm: None,
    })
}

async fn get_joint_slots(State(state): State<Arc<AppState>>) -> impl IntoResponse {
    let config = state.config.read().await;
    let slots: Vec<JointSlot> = config.joint_slots().into_iter().map(|(section, joint, can_id)| {
        let display_name = format!("{}_{}", match section.as_str() {
            "arm_left" => "left",
            "arm_right" => "right",
            other => other,
        }, joint)
            .split('_')
            .map(|w| {
                let mut c = w.chars();
                match c.next() {
                    Some(f) => f.to_uppercase().collect::<String>() + c.as_str(),
                    None => String::new(),
                }
            })
            .collect::<Vec<_>>()
            .join(" ");
        JointSlot { section, joint, can_id, display_name }
    }).collect();
    Json(slots)
}

// -- Joint limit/home update endpoints --

async fn update_joint_limits(
    State(state): State<Arc<AppState>>,
    Path((section, joint)): Path<(String, String)>,
    Json(req): Json<UpdateLimitsRequest>,
) -> impl IntoResponse {
    use std::f64::consts::TAU;
    let max_range = TAU; // ±2π (±360°) — no humanoid joint needs more than a full revolution

    if req.min_rad >= req.max_rad {
        return Json(CommandResponse {
            success: false,
            error: Some("min_rad must be less than max_rad".into()),
            angle_rad: None,
            velocity_rads: None,
            torque_nm: None,
        });
    }
    if req.min_rad < -max_range || req.max_rad > max_range {
        return Json(CommandResponse {
            success: false,
            error: Some(format!("limits must be within ±{:.1}° (±2π rad)", max_range.to_degrees())),
            angle_rad: None,
            velocity_rads: None,
            torque_nm: None,
        });
    }

    {
        let mut config = state.config.write().await;
        let updated = match section.as_str() {
            "arm_left" => config.arm_left.as_mut().and_then(|arm| {
                arm.joints_mut().into_iter()
                    .find(|(n, _)| *n == joint)
                    .map(|(_, j)| { j.limits = (req.min_rad, req.max_rad); })
            }).is_some(),
            "arm_right" => config.arm_right.as_mut().and_then(|arm| {
                arm.joints_mut().into_iter()
                    .find(|(n, _)| *n == joint)
                    .map(|(_, j)| { j.limits = (req.min_rad, req.max_rad); })
            }).is_some(),
            "waist" => config.waist.as_mut().and_then(|w| {
                w.get_mut(&joint).map(|j| { j.limits = (req.min_rad, req.max_rad); })
            }).is_some(),
            _ => false,
        };

        if !updated {
            return Json(CommandResponse {
                success: false,
                error: Some(format!("joint '{}' not found in section '{}'", joint, section)),
                angle_rad: None,
                velocity_rads: None,
                torque_nm: None,
            });
        }

        if let Err(e) = config.save(&state.config_path) {
            return Json(CommandResponse {
                success: false,
                error: Some(format!("Config updated but save failed: {:#}", e)),
                angle_rad: None,
                velocity_rads: None,
                torque_nm: None,
            });
        }
    }

    let can_id = {
        let config = state.config.read().await;
        can_id_for_joint(&config, &section, &joint)
    };

    {
        let mut arms = state.arms.lock().await;
        let side = match section.as_str() {
            "arm_left" => "left",
            "arm_right" => "right",
            _ => "",
        };
        if let Some(arm) = arms.get_mut(side) {
            arm.update_joint_limits(&joint, req.min_rad as f32, req.max_rad as f32).await;
        } else if let Some(id) = can_id {
            // Waist or other non-arm joints: update the shared motor directly
            let motor_arc = {
                let motors = state.motors.lock().await;
                motors.get(&id).cloned()
            };
            if let Some(motor_arc) = motor_arc {
                motor_arc.lock().await.set_joint_limits(req.min_rad as f32, req.max_rad as f32);
            }
        }
    }

    info!(section = %section, joint = %joint, min = req.min_rad, max = req.max_rad, "joint limits updated");

    Json(CommandResponse {
        success: true,
        error: None,
        angle_rad: None,
        velocity_rads: None,
        torque_nm: None,
    })
}

async fn update_joint_home(
    State(state): State<Arc<AppState>>,
    Path((section, joint)): Path<(String, String)>,
    Json(req): Json<UpdateHomeRequest>,
) -> impl IntoResponse {
    let new_home = if req.set_current {
        let arms = state.arms.lock().await;
        let side = match section.as_str() {
            "arm_left" => "left",
            "arm_right" => "right",
            _ => "",
        };
        let arm = match arms.get(side) {
            Some(a) => a,
            None => return Json(CommandResponse {
                success: false,
                error: Some(format!("arm '{}' not found", side)),
                angle_rad: None,
                velocity_rads: None,
                torque_nm: None,
            }),
        };

        let positions = match arm.get_joint_positions().await {
            Ok(p) => p,
            Err(e) => return Json(CommandResponse {
                success: false,
                error: Some(format!("failed to read positions: {:#}", e)),
                angle_rad: None,
                velocity_rads: None,
                torque_nm: None,
            }),
        };
        match positions.iter().find(|(n, _)| n == &joint) {
            Some((_, pos)) => *pos as f64,
            None => return Json(CommandResponse {
                success: false,
                error: Some(format!("joint '{}' not found in arm", joint)),
                angle_rad: None,
                velocity_rads: None,
                torque_nm: None,
            }),
        }
    } else {
        match req.home_rad {
            Some(h) => h,
            None => return Json(CommandResponse {
                success: false,
                error: Some("provide either home_rad or set_current: true".into()),
                angle_rad: None,
                velocity_rads: None,
                torque_nm: None,
            }),
        }
    };

    {
        let mut config = state.config.write().await;

        let limits = match section.as_str() {
            "arm_left" => config.arm_left.as_ref().and_then(|arm| {
                arm.joints().into_iter().find(|(n, _)| *n == joint).map(|(_, j)| j.limits)
            }),
            "arm_right" => config.arm_right.as_ref().and_then(|arm| {
                arm.joints().into_iter().find(|(n, _)| *n == joint).map(|(_, j)| j.limits)
            }),
            _ => None,
        };

        if let Some((lo, hi)) = limits {
            if new_home < lo || new_home > hi {
                return Json(CommandResponse {
                    success: false,
                    error: Some(format!("home_rad {:.3} is outside limits [{:.3}, {:.3}]", new_home, lo, hi)),
                    angle_rad: None,
                    velocity_rads: None,
                    torque_nm: None,
                });
            }
        }

        let updated = match section.as_str() {
            "arm_left" => config.arm_left.as_mut().and_then(|arm| {
                arm.joints_mut().into_iter()
                    .find(|(n, _)| *n == joint)
                    .map(|(_, j)| { j.home_rad = new_home; })
            }).is_some(),
            "arm_right" => config.arm_right.as_mut().and_then(|arm| {
                arm.joints_mut().into_iter()
                    .find(|(n, _)| *n == joint)
                    .map(|(_, j)| { j.home_rad = new_home; })
            }).is_some(),
            "waist" => config.waist.as_mut().and_then(|w| {
                w.get_mut(&joint).map(|j| { j.home_rad = new_home; })
            }).is_some(),
            _ => false,
        };

        if !updated {
            return Json(CommandResponse {
                success: false,
                error: Some(format!("joint '{}' not found in section '{}'", joint, section)),
                angle_rad: None,
                velocity_rads: None,
                torque_nm: None,
            });
        }

        if let Err(e) = config.save(&state.config_path) {
            return Json(CommandResponse {
                success: false,
                error: Some(format!("Config updated but save failed: {:#}", e)),
                angle_rad: None,
                velocity_rads: None,
                torque_nm: None,
            });
        }
    }

    let can_id = {
        let config = state.config.read().await;
        can_id_for_joint(&config, &section, &joint)
    };

    {
        let mut arms = state.arms.lock().await;
        let side = match section.as_str() {
            "arm_left" => "left",
            "arm_right" => "right",
            _ => "",
        };
        if let Some(arm) = arms.get_mut(side) {
            arm.update_joint_home(&joint, new_home as f32).await;
        } else if let Some(id) = can_id {
            // Waist or other non-arm joints: update the shared motor directly
            let motor_arc = {
                let motors = state.motors.lock().await;
                motors.get(&id).cloned()
            };
            if let Some(motor_arc) = motor_arc {
                motor_arc.lock().await.set_home_rad(new_home as f32);
            }
        }
    }

    info!(section = %section, joint = %joint, home_rad = new_home, "joint home updated");

    Json(CommandResponse {
        success: true,
        error: None,
        angle_rad: Some(new_home as f32),
        velocity_rads: None,
        torque_nm: None,
    })
}

// -- Helpers --

/// Look up the CAN ID for a section+joint from the config.
fn can_id_for_joint(config: &cortex::config::RobotConfig, section: &str, joint: &str) -> Option<u8> {
    match section {
        "arm_left" => config.arm_left.as_ref().and_then(|arm| {
            arm.joints().into_iter()
                .find(|(n, _)| *n == joint)
                .and_then(|(_, j)| j.can_id)
        }),
        "arm_right" => config.arm_right.as_ref().and_then(|arm| {
            arm.joints().into_iter()
                .find(|(n, _)| *n == joint)
                .and_then(|(_, j)| j.can_id)
        }),
        "waist" => config.waist.as_ref().and_then(|w| {
            w.get(joint).and_then(|j| j.can_id)
        }),
        _ => None,
    }
}

/// API-level limit check. Returns Some(error response) if position violates limits.
async fn check_motor_limits(
    state: &AppState,
    can_id: u8,
    position_rad: Option<f32>,
) -> Option<CommandResponse> {
    let pos = position_rad?;
    if pos.abs() < f32::EPSILON && pos == 0.0 {
        return None;
    }
    let config = state.config.read().await;
    let limits = safety::limits_for_motor(&config, can_id);
    let (lo, hi) = match limits {
        Some(l) => l,
        None => return None,
    };

    if pos < lo || pos > hi {
        return Some(CommandResponse {
            success: false,
            error: Some(format!(
                "position {:.3} rad exceeds limits [{:.3}, {:.3}] for motor {}",
                pos, lo, hi, can_id
            )),
            angle_rad: None,
            velocity_rads: None,
            torque_nm: None,
        });
    }
    None
}

fn find_joint_config(config: &cortex::config::RobotConfig, can_id: u8) -> (String, (f64, f64)) {
    let arms = [
        config.arm_left.as_ref(),
        config.arm_right.as_ref(),
    ];
    for arm in arms.iter().flatten() {
        for (_name, joint) in arm.joints() {
            if joint.can_id == Some(can_id) {
                return (joint.actuator.clone(), joint.limits);
            }
        }
    }
    if let Some(ref waist) = config.waist {
        for (_name, joint) in waist {
            if joint.can_id == Some(can_id) {
                return (joint.actuator.clone(), joint.limits);
            }
        }
    }
    ("rs03".into(), (-12.57, 12.57))
}

fn collect_configured_motors(
    config: &cortex::config::RobotConfig,
    motors: &std::collections::HashMap<u8, Arc<Mutex<cortex::motor::Motor>>>,
    joint_map: &std::collections::HashMap<u8, String>,
    infos: &mut Vec<MotorInfo>,
) {
    let all_configured = all_configured_can_ids(config);
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

fn all_configured_can_ids(config: &cortex::config::RobotConfig) -> Vec<(u8, String, (f64, f64))> {
    let mut ids = Vec::new();
    let arms = [
        config.arm_left.as_ref(),
        config.arm_right.as_ref(),
    ];
    for arm in arms.iter().flatten() {
        for (_name, joint) in arm.joints() {
            if let Some(id) = joint.can_id {
                ids.push((id, joint.actuator.clone(), joint.limits));
            }
        }
    }
    if let Some(ref waist) = config.waist {
        for (_name, joint) in waist {
            if let Some(id) = joint.can_id {
                ids.push((id, joint.actuator.clone(), joint.limits));
            }
        }
    }
    ids
}

#[derive(Deserialize)]
struct StartSweepRequest {
    /// Sweep speed in degrees per second. Defaults to 20. Clamped to 1–80.
    speed_deg_per_sec: Option<f32>,
}

/// `POST /api/arms/{side}/joints/{joint}/sweep/start`
///
/// Begins a continuous sweep of the joint between its configured limits.
/// Optional body: `{ "speed_deg_per_sec": 10.0 }` (default 20, range 1–80).
/// Returns immediately; the sweep runs in a background task until stopped.
/// Starting a sweep while one is already active for the same joint cancels the old one.
async fn start_sweep(
    State(state): State<Arc<AppState>>,
    Path((side, joint)): Path<(String, String)>,
    body: Option<Json<StartSweepRequest>>,
) -> impl IntoResponse {
    let key = format!("{}/{}", side, joint);

    // Cancel any existing sweep for this joint and wait for it to finish
    // so two tasks never fight over the same motor.
    {
        let mut tasks = state.sweep_tasks.lock().await;
        if let Some((old_token, old_handle)) = tasks.remove(&key) {
            old_token.cancel();
            drop(tasks); // release lock while we await the old task
            let _ = old_handle.await;
        }
    }

    let token = CancellationToken::new();

    const STEP_DELAY_MS: u64 = 50;
    let speed = body
        .and_then(|b| b.speed_deg_per_sec)
        .unwrap_or(20.0)
        .clamp(1.0, 80.0);
    let step_rad: f32 = speed.to_radians() * (STEP_DELAY_MS as f32 / 1000.0);

    let sweep_ctx = {
        let arms = state.arms.lock().await;
        match arms.get(&side) {
            None => {
                return Json(CommandResponse {
                    success: false,
                    error: Some(format!("Arm '{}' not found", side)),
                    angle_rad: None,
                    velocity_rads: None,
                    torque_nm: None,
                });
            }
            Some(arm) => match arm.sweep_context(&joint) {
                Ok(ctx) => ctx,
                Err(e) => {
                    return Json(CommandResponse {
                        success: false,
                        error: Some(format!("{:#}", e)),
                        angle_rad: None,
                        velocity_rads: None,
                        torque_nm: None,
                    });
                }
            },
        }
    }; // arms lock released here

    let (motor_arc, min, max, home) = sweep_ctx;
    let token_clone = token.clone();
    let side_clone = side.clone();
    let joint_clone = joint.clone();
    let state_clone = state.clone();

    let handle = tokio::spawn(async move {
        info!("Sweep started: {}/{} at {:.1}°/sec", side_clone, joint_clone, speed);

        let watchdog = tokio::time::sleep(std::time::Duration::from_secs(600));
        tokio::pin!(watchdog);

        loop {
            tokio::select! {
                result = cortex::arm::sweep_pass(&motor_arc, min, max, step_rad, STEP_DELAY_MS, &token_clone) => {
                    match result {
                        Ok(cancelled) => {
                            if cancelled || token_clone.is_cancelled() {
                                break;
                            }
                        }
                        Err(e) => {
                            warn!("Sweep error on {}/{}: {:#}", side_clone, joint_clone, e);
                            break;
                        }
                    }
                }
                _ = &mut watchdog => {
                    warn!("Sweep watchdog fired for {}/{} — auto-stopping", side_clone, joint_clone);
                    break;
                }
            }
        }

        // Return to home — respects cancellation so a new sweep can take over quickly.
        if !token_clone.is_cancelled() {
            if let Err(e) = cortex::arm::sweep_home(&motor_arc, home, min, max, step_rad, STEP_DELAY_MS, &token_clone).await {
                warn!("Sweep home failed for {}/{}: {:#}", side_clone, joint_clone, e);
            }
        }

        let key = format!("{}/{}", side_clone, joint_clone);
        state_clone.sweep_tasks.lock().await.remove(&key);
        info!("Sweep finished: {}/{}", side_clone, joint_clone);
    });

    // Store the token and handle so future start/stop calls can cancel and await.
    state.sweep_tasks.lock().await.insert(key, (token, handle));

    Json(CommandResponse {
        success: true,
        error: None,
        angle_rad: None,
        velocity_rads: None,
        torque_nm: None,
    })
}

/// `POST /api/arms/{side}/joints/{joint}/sweep/stop`
///
/// Signals the active sweep for this joint to stop after finishing its current pass,
/// then return to home. Idempotent — returns success even if no sweep is active.
async fn stop_sweep(
    State(state): State<Arc<AppState>>,
    Path((side, joint)): Path<(String, String)>,
) -> impl IntoResponse {
    let key = format!("{}/{}", side, joint);
    let old = {
        let mut tasks = state.sweep_tasks.lock().await;
        tasks.remove(&key)
    };
    if let Some((token, _handle)) = old {
        token.cancel();
        info!("Sweep stop requested: {}", key);
    }
    Json(CommandResponse {
        success: true,
        error: None,
        angle_rad: None,
        velocity_rads: None,
        torque_nm: None,
    })
}
