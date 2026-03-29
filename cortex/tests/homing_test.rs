//! Homing / startup recovery tests using RS03 loopback simulation (no hardware).

use std::sync::Arc;

use cortex::arm::{Arm, JointHomingStatus};
use cortex::config::ArmConfig;
use cortex::motor::Motor;
use std::f32::consts::PI;

use robstride::actuator::{normalize_value, TypedCommandData};
use robstride::robstride03::RobStride03Command;
use robstride::{CommandData, LoopbackSimTransport, Protocol, TransportType};
use tokio::sync::Mutex;

/// Single-joint arm (shoulder_pitch on CAN 8) with fast recovery timings for CI.
fn test_arm_yaml() -> &'static str {
    r"
shoulder_pitch:
  can_id: 8
  actuator: rs03
  limits: [-1.57, 3.14]
  home_rad: 0.0
  straight_down_home_at_startup: false
  startup_recovery:
    recovery_timeout_secs: 25.0
    step_period_ms: 1
    approach_step_period_ms: 1
    resistance_backoff_ms: 5
    approach_enabled: true
    approach_max_secs: 12.0
shoulder_roll:
  can_id: null
  actuator: rs03
  limits: [0.0, 3.14]
  home_rad: 0.0
upper_arm_yaw:
  can_id: null
  actuator: rs03
  limits: [-1.57, 1.57]
  home_rad: 0.0
elbow_pitch:
  can_id: null
  actuator: rs03
  limits: [0.0, 2.36]
  home_rad: 0.0
"
}

fn arm_with_initial_pitch(pos_rad: f32) -> Arm {
    let cfg: ArmConfig = serde_yaml::from_str(test_arm_yaml()).expect("parse test arm yaml");
    let transport = TransportType::LoopbackSim(LoopbackSimTransport::new(pos_rad));
    let protocol = Protocol::new(transport, Arc::new(|_id: u32, _data: Vec<u8>| {}));
    Arm::new(&cfg, Arc::new(Mutex::new(protocol)))
}

fn shoulder_result(summary: &cortex::arm::StartupRecoverySummary) -> &cortex::arm::JointHomingResult {
    summary
        .joints
        .iter()
        .find(|j| j.joint_name == "shoulder_pitch")
        .expect("shoulder_pitch homing result")
}

#[tokio::test]
async fn homing_from_60_deg_completes_without_stall_backoffs() {
    let mut arm = arm_with_initial_pitch(60f32.to_radians());
    let summary = arm.startup_safe_recovery(false).await.expect("startup_safe_recovery");
    let j = shoulder_result(&summary);

    assert_eq!(
        summary.stall_backoffs, 0,
        "false stall backoffs should not occur on smooth simulated motion"
    );
    assert!(
        matches!(
            j.status,
            JointHomingStatus::Homed
                | JointHomingStatus::AlreadyHome
                | JointHomingStatus::StalledButHomed
        ),
        "unexpected status: {:?}",
        j.status
    );
    assert!(
        j.error_rad < 0.06,
        "shoulder should settle near home, error_rad={}",
        j.error_rad
    );
    assert!(
        j.duration_ms < 10_000,
        "homing should not take tens of seconds on sim (got {} ms)",
        j.duration_ms
    );
}

#[tokio::test]
async fn homing_from_multiple_offsets_no_false_stalls() {
    for deg in [30.0f32, 60.0, 90.0] {
        let mut arm = arm_with_initial_pitch(deg.to_radians());
        let summary = arm.startup_safe_recovery(false).await.expect("home");
        assert_eq!(
            summary.stall_backoffs, 0,
            "stall backoff at {deg}° start"
        );
        let j = shoulder_result(&summary);
        assert!(
            j.error_rad < 0.08,
            "at {deg}°: error_rad={}",
            j.error_rad
        );
    }
}

#[test]
fn mit_control_zero_rad_encodes_decodes_via_raw_angle() {
    let typed = RobStride03Command {
        target_angle_rad: 0.0,
        target_velocity_rads: 0.0,
        kp: 15.0,
        kd: 0.9,
        torque_nm: 0.0,
    };
    let cmd = typed.to_control_command().to_command(8);
    let angle_raw = u16::from_be_bytes(cmd.data[0..2].try_into().unwrap()) as f32;
    let rad = normalize_value(angle_raw, 0.0, 65535.0, -4.0 * PI, 4.0 * PI);
    assert!(rad.abs() < 0.05, "0 rad command should decode near 0, got {rad}");
}

#[tokio::test]
async fn loopback_sim_motor_read_and_control() {
    let sim = LoopbackSimTransport::new(1.047f32);
    let probe = sim.clone();
    let transport = TransportType::LoopbackSim(sim);
    let protocol = Arc::new(Mutex::new(Protocol::new(
        transport,
        Arc::new(|_id: u32, _data: Vec<u8>| {}),
    )));
    let mut m = Motor::new(protocol, 8);
    m.set_joint_limits(-1.57, 3.14);
    let p0 = m.read_position().await.expect("read");
    assert!(
        (p0 - 1.047).abs() < 0.02,
        "expected ~60°, got {} rad",
        p0
    );
    let _ = m.enable().await.expect("enable");
    let _ = m
        .send_control(0.0, 0.0, 15.0, 0.9, 0.0)
        .await
        .expect("control");
    let p1 = m.read_position().await.expect("read2");
    let internal = probe.position_rad();
    assert!(
        (p1 - internal).abs() < 0.02,
        "MechPos read {} vs sim state {}",
        p1,
        internal
    );
    assert!(
        p1 < p0,
        "sim should move toward 0, p0={p0} p1={p1} internal={internal}"
    );
}

#[test]
fn default_startup_recovery_config_invariants() {
    let c = cortex::config::StartupRecoveryConfig::default();
    assert!(
        c.stall_detection_min_linear_error_rad + 1e-9 >= c.approach_handoff_rad,
        "stall floor must be >= approach handoff"
    );
    assert!(c.recovery_direct_command_within_rad >= 0.15);
}
