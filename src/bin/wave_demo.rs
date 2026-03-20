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

    arm.enable_all().await?;
    println!("All joints enabled");

    let result = wave_sequence(&mut arm).await;

    println!("Disabling all joints...");
    arm.disable_all().await?;

    result
}

async fn wave_sequence(arm: &mut Arm) -> Result<()> {
    for cycle in 1..=3 {
        println!("Wave cycle {}/3", cycle);

        arm.set_joint("shoulder_pitch", 1.0, None, None).await?;
        tokio::time::sleep(Duration::from_secs(1)).await;

        arm.set_joint("shoulder_pitch", 0.5, None, None).await?;
        tokio::time::sleep(Duration::from_millis(800)).await;

        arm.set_joint("shoulder_pitch", 1.0, None, None).await?;
        tokio::time::sleep(Duration::from_millis(800)).await;
    }

    println!("Returning to zero...");
    arm.set_joint("shoulder_pitch", 0.0, None, None).await?;
    tokio::time::sleep(Duration::from_secs(2)).await;

    let positions = arm.get_joint_positions().await?;
    println!("Final positions: {:?}", positions);

    Ok(())
}
