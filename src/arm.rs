use std::collections::HashMap;
use std::sync::Arc;

use anyhow::Result;
use robstride::Protocol;
use tokio::sync::Mutex;
use tracing::{info, warn};

use crate::config::{ArmConfig, StartupRecoveryConfig};
use crate::motor::Motor;

/// Result of [`Arm::startup_safe_recovery`]. Non-zero `stall_backoffs` means at least one joint hit
/// a stall/backoff cycle — higher-level motion should usually wait for human acknowledgment.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct StartupRecoverySummary {
    pub stall_backoffs: u32,
}

struct JointStartupParams {
    home_rad: f32,
    limit_min_rad: f32,
    limit_max_rad: f32,
    recovery: StartupRecoveryConfig,
    straight_down_home_at_startup: bool,
}

pub struct Arm {
    motors: HashMap<String, Motor>,
    joint_startup: HashMap<String, JointStartupParams>,
}

impl Arm {
    pub fn new(config: &ArmConfig, protocol: Arc<Mutex<Protocol>>) -> Self {
        let mut motors = HashMap::new();
        let mut joint_startup = HashMap::new();

        for (name, joint) in config.joints() {
            if let Some(can_id) = joint.can_id {
                let straight_down = joint.straight_down_home_at_startup;
                let home_rad = if straight_down {
                    if joint.home_rad.abs() > 1e-9 {
                        warn!(
                            joint = name,
                            yaml_home_rad = joint.home_rad,
                            "straight_down_home_at_startup: ignoring YAML home_rad; home is 0 after set_zero"
                        );
                    }
                    0.0
                } else {
                    joint.home_rad as f32
                };

                joint_startup.insert(
                    name.to_string(),
                    JointStartupParams {
                        home_rad,
                        limit_min_rad: joint.limits.0 as f32,
                        limit_max_rad: joint.limits.1 as f32,
                        recovery: joint.startup_recovery.clone(),
                        straight_down_home_at_startup: straight_down,
                    },
                );
                motors.insert(
                    name.to_string(),
                    Motor::new(protocol.clone(), can_id),
                );
            }
        }

        Self {
            motors,
            joint_startup,
        }
    }

    /// After enable, any joint farther than `startup_recovery.large_error_rad` from `home_rad`
    /// runs recovery: optional fast approach, then gradual steps; stall detection runs in both
    /// phases. See [`StartupRecoverySummary`] for whether any obstruction was seen.
    ///
    /// If the encoder has accumulated multi-turn offset (e.g. from free spinning in the REPL),
    /// the position is normalized via `set_zero` first so recovery doesn't walk through dozens
    /// of phantom revolutions.
    pub async fn startup_safe_recovery(&mut self) -> Result<StartupRecoverySummary> {
        use std::f32::consts::TAU;
        let mut stall_backoffs = 0u32;

        let joint_names: Vec<String> = self.motors.keys().cloned().collect();
        for name in &joint_names {
            let params = self
                .joint_startup
                .get(name.as_str())
                .ok_or_else(|| anyhow::anyhow!("internal: joint '{}' has motor but no startup params", name))?;
            let r = params.recovery.clone();
            let mut home = params.home_rad;
            let limits = (params.limit_min_rad, params.limit_max_rad);

            let motor = self.motors.get_mut(name.as_str()).unwrap();

            // Collapse accumulated multi-turn encoder offset before recovery.
            // The gearbox output can't physically exceed one revolution within joint limits,
            // so anything beyond 2pi from home is phantom accumulation from REPL spinning.
            if let Some(new_home) = motor.normalize_multiturn(home, TAU).await? {
                info!(
                    joint = %name,
                    new_home_rad = %format_args!("{:.3}", new_home),
                    "encoder had multi-turn accumulation; re-zeroed, recovery target adjusted"
                );
                home = new_home;
            }

            let large = r.large_error_rad as f32;
            let pos = motor.read_position().await?;
            let err_mag = (pos - home).abs();
            if err_mag <= large {
                continue;
            }

            info!(
                joint = %name,
                error_rad = err_mag,
                home_rad = home,
                "joint far from home; running startup recovery (approach + gradual)"
            );

            stall_backoffs += motor
                .recover_position_if_far(home, &r, Some(limits))
                .await?;
        }
        Ok(StartupRecoverySummary { stall_backoffs })
    }

    /// Call **before** [`enable_all`] for any joint with `straight_down_home_at_startup: true` in YAML.
    /// Physically hold that joint **straight down**, then this runs drive [`Motor::set_zero`] so
    /// mechanical angle 0 = that pose. Startup recovery then uses `home_rad == 0`.
    /// Returns how many motors received `set_zero`.
    pub async fn straight_down_home_before_enable(&mut self) -> Result<usize> {
        let mut n = 0usize;
        for (name, motor) in &mut self.motors {
            let params = self
                .joint_startup
                .get(name)
                .ok_or_else(|| anyhow::anyhow!("internal: joint '{}' has no startup params", name))?;
            if !params.straight_down_home_at_startup {
                continue;
            }
            info!(
                joint = %name,
                "SetZero -- joint must be straight down; defining mech position 0 as home"
            );
            motor.set_zero().await?;
            n += 1;
        }
        Ok(n)
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

    /// Startup / recovery home angle for this joint (rad), after `straight_down_home_at_startup` override.
    pub fn configured_home_rad(&self, joint_name: &str) -> Option<f32> {
        self.joint_startup.get(joint_name).map(|p| p.home_rad)
    }
}
