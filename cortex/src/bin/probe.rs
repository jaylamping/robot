use anyhow::Result;
use cortex::config::RobotConfig;
use cortex::motor::{create_ch341_protocol, Motor};

#[tokio::main]
async fn main() -> Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| "info,cortex=debug".parse().unwrap()),
        )
        .init();

    let config = RobotConfig::load("config/robot.yaml")?;
    println!("Config loaded successfully");
    println!("  Port: {}", config.bus.port);
    println!("  Baud: {}", config.bus.baud);
    println!("  Host ID: 0x{:02X}", config.bus.host_id);

    if let Some(ref arm) = config.arm_left {
        println!("\nLeft arm joints:");
        for (name, joint) in arm.joints() {
            match joint.can_id {
                Some(id) => println!("  {} -> CAN ID {}", name, id),
                None => println!("  {} -> not assigned", name),
            }
        }
    }

    println!("\nOpening CH341 transport on {}...", config.bus.port);
    let protocol = create_ch341_protocol(&config.bus.port).await?;
    println!("Transport opened successfully");

    if let Some(ref arm) = config.arm_left {
        for (name, joint) in arm.active_joints() {
            let can_id = joint.can_id.unwrap();
            println!("\nProbing {} (CAN ID {})...", name, can_id);

            let mut motor = Motor::new(protocol.clone(), can_id);
            match motor.read_state().await {
                Ok(state) => {
                    println!("  Angle:       {:.3} rad ({:.1} deg)", state.angle_rad, state.angle_rad.to_degrees());
                    println!("  Velocity:    {:.3} rad/s", state.velocity_rads);
                    println!("  Torque:      {:.3} N*m", state.torque_nm);
                    println!("  Temperature: {:.1} C", state.temperature_c);
                    println!("  Mode:        {:?}", state.mode);
                    if !state.faults.is_empty() {
                        println!("  Faults:      {:?}", state.faults);
                    }
                }
                Err(e) => {
                    println!("  No response: {}", e);
                }
            }
        }
    }

    println!("\nProbe complete.");
    Ok(())
}
