use std::time::Duration;

use anyhow::Result;
use robot::config::RobotConfig;
use robot::motor::create_ch341_protocol;
use robot::arm::Arm;

#[tokio::main]
async fn main() -> Result<()> {
    tracing_subscriber::fmt::init();

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
    let mut arm = Arm::new(arm_config, protocol);

    let zeroed = arm.straight_down_home_before_enable().await?;
    if zeroed > 0 {
        println!(
            "Mechanical zero (straight-down home) set on {} joint(s); encoder 0 = that pose.",
            zeroed
        );
    }

    arm.enable_all().await?;
    println!("All joints enabled");

    let recovery = arm.startup_safe_recovery().await?;
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

    let result = wave_sequence(&mut arm, "shoulder_pitch").await;

    println!("Disabling all joints...");
    arm.disable_all().await?;

    result
}

async fn wave_sequence(arm: &mut Arm, joint: &str) -> Result<()> {
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
