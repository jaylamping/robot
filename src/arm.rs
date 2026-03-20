use std::collections::HashMap;
use std::sync::Arc;

use anyhow::Result;
use robstride::Protocol;
use tokio::sync::Mutex;
use tracing::info;

use crate::config::ArmConfig;
use crate::motor::Motor;

pub struct Arm {
    motors: HashMap<String, Motor>,
}

impl Arm {
    pub fn new(config: &ArmConfig, protocol: Arc<Mutex<Protocol>>) -> Self {
        let mut motors = HashMap::new();

        for (name, joint) in config.joints() {
            if let Some(can_id) = joint.can_id {
                motors.insert(
                    name.to_string(),
                    Motor::new(protocol.clone(), can_id),
                );
            }
        }

        Self { motors }
    }

    pub async fn enable_all(&mut self) -> Result<()> {
        for (name, motor) in &mut self.motors {
            info!("Enabling {}", name);
            motor.enable().await?;
        }
        Ok(())
    }

    pub async fn disable_all(&mut self) -> Result<()> {
        for (name, motor) in &mut self.motors {
            info!("Disabling {}", name);
            let _ = motor.disable().await;
        }
        Ok(())
    }

    pub async fn set_joint(
        &mut self,
        joint_name: &str,
        position_rad: f32,
        kp: Option<f32>,
        kd: Option<f32>,
    ) -> Result<()> {
        let motor = self.motors.get_mut(joint_name)
            .ok_or_else(|| anyhow::anyhow!("Joint '{}' not configured", joint_name))?;
        motor.move_to(position_rad, kp, kd).await?;
        Ok(())
    }

    pub async fn get_joint_positions(&mut self) -> Result<HashMap<String, f32>> {
        let mut positions = HashMap::new();
        for (name, motor) in &mut self.motors {
            let pos = motor.read_position().await?;
            positions.insert(name.clone(), pos);
        }
        Ok(positions)
    }

    pub fn joint_names(&self) -> Vec<&str> {
        self.motors.keys().map(|s| s.as_str()).collect()
    }
}
