use cortex::config::{BusConfig, RobotConfig};
use cortex::safety::{
    JOINT_UNWRAP_EPS, canonical_joint_angle, canonical_position_for_limits, is_within_limits,
    limits_for_motor, shortest_angle_err, soft_limit_effort_scale, step_delta_toward_home,
    validate_velocity_command,
};
use std::f32::consts::{PI, TAU};

#[test]
fn shortest_angle_small_delta() {
    assert!((shortest_angle_err(0.5, 1.0) - 0.5).abs() < 1e-5);
}

#[test]
fn shortest_angle_wraps_near_tau() {
    let d = shortest_angle_err(6.1, 0.17);
    assert!(d > 0.0 && d < 1.0, "d={}", d);
}

#[test]
fn canonical_maps_wrong_branch() {
    let lo = -1.57_f32;
    let hi = 3.14_f32;
    let home = 0.0_f32;
    let physical = -0.52_f32;
    let raw = physical + TAU;
    let cj = canonical_joint_angle(raw, home, lo, hi);
    assert!(
        (cj - physical).abs() < 0.06,
        "expected ~{physical}, got {cj}"
    );
}

#[test]
fn canonical_position_multiturn() {
    let limits = Some((-1.309_f32, 1.309_f32));
    let raw = 18.85_f32; // ~3 full turns
    let canon = canonical_position_for_limits(raw, 0.0, limits);
    assert!(
        canon >= -1.309 - JOINT_UNWRAP_EPS && canon <= 1.309 + JOINT_UNWRAP_EPS,
        "expected within limits, got {canon}"
    );
}

#[test]
fn soft_limit_no_limits_full_scale() {
    assert_eq!(
        soft_limit_effort_scale(None, Some(0.5), 0.0, 0.175, 2.0),
        1.0
    );
}

#[test]
fn soft_limit_ramps_near_max() {
    let limits = Some((-1.0, 1.0));
    let m = 0.175_f32;
    let pos = 1.0 - 0.05;
    let s = soft_limit_effort_scale(limits, Some(pos), 0.0, m, 1.0);
    assert!((s - (0.05 / m)).abs() < 1e-5, "s={s}");
}

#[test]
fn soft_limit_zero_past_max() {
    let s = soft_limit_effort_scale(Some((-1.0, 1.0)), Some(1.01), 0.0, 0.175, 1.0);
    assert_eq!(s, 0.0);
}

#[test]
fn soft_limit_multiturn_uses_canonical() {
    let limits = Some((-1.309, 1.309));
    let raw_pos = 18.85_f32;
    let s = soft_limit_effort_scale(limits, Some(raw_pos), 0.0, 0.175, 1.0);
    assert!(
        s > 0.0,
        "multi-turn should map to canonical and not return 0, got {s}"
    );
}

#[test]
fn validate_velocity_rejects_outside_limits_wrong_direction() {
    let result = validate_velocity_command(1.5, 0.0, Some((-1.309, 1.309)), 0.175, 2.0);
    assert!(result.is_err(), "should reject positive velocity above max");
}

#[test]
fn validate_velocity_allows_return_to_range() {
    let result = validate_velocity_command(1.5, 0.0, Some((-1.309, 1.309)), 0.175, -2.0);
    assert!(
        result.is_ok(),
        "should allow negative velocity to return to range"
    );
}

#[test]
fn validate_velocity_scales_near_limit() {
    let result = validate_velocity_command(1.2, 0.0, Some((-1.309, 1.309)), 0.175, 2.0);
    match result {
        Ok(v) => assert!(v < 2.0 && v > 0.0, "velocity should be scaled, got {v}"),
        Err(_) => panic!("should not reject within limits"),
    }
}

#[test]
fn validate_velocity_multiturn_maps_correctly() {
    let result = validate_velocity_command(18.85, 0.0, Some((-1.309, 1.309)), 0.175, 1.0);
    assert!(
        result.is_ok(),
        "multi-turn canonical should be within limits"
    );
}

#[test]
fn is_within_limits_basic() {
    assert!(is_within_limits(0.5, (-1.0, 1.0)));
    assert!(!is_within_limits(1.5, (-1.0, 1.0)));
}

#[test]
fn limits_for_motor_not_found() {
    let config = RobotConfig {
        bus: BusConfig {
            transport: "ch341".into(),
            port: "COM5".into(),
            socketcan_interface: None,
            baud: 921600,
            can_bitrate: 1000000,
            host_id: 0xAA,
        },
        actuators: Default::default(),
        arm_left: None,
        arm_right: None,
        waist: None,
        torso: None,
    };
    assert_eq!(limits_for_motor(&config, 127), None);
}

#[test]
fn step_delta_bounded_always_linear() {
    let d = step_delta_toward_home(6.1, 0.17, true, true);
    assert!((d - (0.17 - 6.1)).abs() < 1e-4);
}

#[test]
fn step_delta_unbounded_short_wrap() {
    let d = step_delta_toward_home(6.1, 0.17, true, false);
    assert!(d.abs() < 1.0, "expected short arc, got {}", d);
}

#[test]
fn half_turn() {
    assert!((shortest_angle_err(0.0, PI).abs() - PI).abs() < 1e-4);
}
