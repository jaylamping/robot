use std::collections::HashMap;
use std::sync::Arc;
use std::time::Duration;

use anyhow::Result;
use cortex::config::RobotConfig;
use cortex::motor::{create_ch341_protocol, Motor};
use cortex::arm::Arm;
use cortex::safety;
use tokio::sync::Mutex;

#[tokio::main]
async fn main() -> Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| "info,cortex=debug".parse().unwrap()),
        )
        .init();

    let config = RobotConfig::load("config/robot.yaml")?;
    let arm_config = config.arm_left
        .as_ref()
        .expect("No left arm config in robot.yaml");

    println!("Arm wave demo");
    println!("Active joints: {:?}", arm_config.active_joints()
        .iter()
        .map(|(name, j)| format!("{} (ID {})", name, j.can_id.unwrap()))
        .collect::<Vec<_>>());

    let protocol = create_ch341_protocol(&config.bus.port).await?;

    let mut motors: HashMap<u8, Arc<Mutex<Motor>>> = HashMap::new();
    for (_, joint) in arm_config.joints() {
        if let Some(can_id) = joint.can_id {
            let mut motor = Motor::new(protocol.clone(), can_id);
            if let Some((lo, hi)) = safety::limits_for_motor(&config, can_id) {
                motor.set_joint_limits(lo, hi);
            }
            if let Some(home) = safety::home_for_motor(&config, can_id) {
                motor.set_home_rad(home);
            }
            motors.insert(can_id, Arc::new(Mutex::new(motor)));
        }
    }

    let arm = Arm::new(arm_config, &motors);

    let zeroed = arm.straight_down_home_before_enable().await?;
    if zeroed > 0 {
        println!(
            "Mechanical zero (straight-down home) set on {} joint(s); encoder 0 = that pose.",
            zeroed
        );
    }

    arm.enable_all().await?;
    println!("All joints enabled");

    let recovery = arm.startup_safe_recovery(false).await?;
    if recovery.stall_backoffs > 0 {
        println!(
            "Startup recovery reported {} stall/backoff event(s) (joint was held or blocked).",
            recovery.stall_backoffs
        );
        println!("Skipping wave demo — re-run when the arm is clear and you intend to move.");
        println!("Disabling all joints...");
        arm.disable_all().await?;
        return Ok(());
    }
    println!("Startup position check complete");

    let result = wave_sequence(&arm, "shoulder_pitch").await;

    println!("Disabling all joints...");
    arm.disable_all().await?;

    result
}

async fn wave_sequence(arm: &Arm, joint: &str) -> Result<()> {
    let home = arm
        .configured_home_rad(joint)
        .ok_or_else(|| anyhow::anyhow!("Joint '{}' not in arm or has no home", joint))?;

    for cycle in 1..=3 {
        println!("Wave cycle {}/3", cycle);

        // Absolute angles in joint space (not offset from home); tune if you want motion relative to home_rad.
        arm.set_joint(joint, 1.0, None, None).await?;
        tokio::time::sleep(Duration::from_secs(1)).await;

        arm.set_joint(joint, 0.5, None, None).await?;
        tokio::time::sleep(Duration::from_millis(800)).await;

        arm.set_joint(joint, 1.0, None, None).await?;
        tokio::time::sleep(Duration::from_millis(800)).await;
    }

    println!("Returning to configured home ({:.3} rad)...", home);
    arm.set_joint(joint, home, None, None).await?;
    tokio::time::sleep(Duration::from_secs(2)).await;

    let positions = arm.get_joint_positions().await?;
    println!("Final positions: {:?}", positions);

    Ok(())
}
