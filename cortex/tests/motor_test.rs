//! Integration test for the Motor API.
//! Requires hardware: RS03 on the CAN ID from `config/robot.yaml`, CAN2USB on COM5.
//! Run with: cargo test --test motor_test -- --nocapture

use std::path::PathBuf;

use cortex::config::RobotConfig;
use cortex::motor::{create_ch341_protocol, Motor};

fn repo_robot_yaml() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../config/robot.yaml")
}

#[tokio::test]
async fn smoke_test_motor_api() {
    let config = RobotConfig::load(repo_robot_yaml()).expect("Failed to load config");
    let protocol = create_ch341_protocol(&config.bus.port)
        .await
        .expect("Failed to open CH341 transport");

    let can_id = config
        .arm_left
        .as_ref()
        .and_then(|a| a.shoulder_pitch.can_id)
        .expect("shoulder_pitch can_id in config");
    let mut motor = Motor::new(protocol, can_id);

    let state = motor.enable().await.expect("Failed to enable");
    println!("Enabled:");
    println!("  Position: {:.3} rad ({:.1} deg)", state.angle_rad, state.angle_rad.to_degrees());
    println!("  Velocity: {:.3} rad/s", state.velocity_rads);
    println!("  Torque:   {:.3} N*m", state.torque_nm);
    println!("  Temp:     {:.1} C", state.temperature_c);
    println!("  Mode:     {:?}", state.mode);
    if !state.faults.is_empty() {
        println!("  Faults:   {:?}", state.faults);
    }

    let voltage = motor.read_voltage().await.expect("Failed to read voltage");
    println!("\n  Bus voltage: {:.1} V", voltage);

    let pos = motor.read_position().await.expect("Failed to read position");
    println!("  Mech position: {:.3} rad", pos);

    let vel = motor.read_velocity().await.expect("Failed to read velocity");
    println!("  Mech velocity: {:.4} rad/s", vel);

    let state = motor.disable().await.expect("Failed to disable");
    println!("\nDisabled. Mode: {:?}", state.mode);
}

#[tokio::test]
async fn config_loads_correctly() {
    let config = RobotConfig::load(repo_robot_yaml()).expect("Failed to load config");

    assert_eq!(config.bus.port, "COM5");
    assert_eq!(config.bus.baud, 921600);
    assert_eq!(config.bus.host_id, 0xAA);

    let arm = config.arm_left.as_ref().expect("No left arm config");
    assert_eq!(arm.shoulder_pitch.can_id, Some(8));
    assert_eq!(arm.shoulder_roll.can_id, None);

    let rs03 = config.actuators.get("rs03").expect("No RS03 actuator spec");
    assert!((rs03.max_torque - 60.0).abs() < 0.01);
    assert!((rs03.gear_ratio - 9.0).abs() < 0.01);
}
