use std::collections::HashMap;
use std::sync::Arc;
use std::time::Instant;

use anyhow::Result;
use serde::Serialize;
use tokio::sync::Mutex;
use tracing::{info, warn};

use crate::config::{ArmConfig, StartupRecoveryConfig};
use crate::motor::Motor;
use crate::safety::{
    canonical_joint_angle, joint_space_error_mag, motor_cmd_for_joint_target,
};

// -- Homing result types --

#[derive(Debug, Clone, Serialize)]
pub enum JointHomingStatus {
    AlreadyHome,
    Homed,
    StalledButHomed,
    TimedOut,
    Error(String),
    Skipped,
}

impl JointHomingStatus {
    pub fn as_str(&self) -> &str {
        match self {
            Self::AlreadyHome => "already_home",
            Self::Homed => "homed",
            Self::StalledButHomed => "stalled_but_homed",
            Self::TimedOut => "timed_out",
            Self::Error(_) => "error",
            Self::Skipped => "skipped",
        }
    }
}

#[derive(Debug, Clone, Serialize)]
pub struct JointHomingResult {
    pub joint_name: String,
    pub status: JointHomingStatus,
    pub start_position_rad: f32,
    pub end_position_rad: f32,
    pub home_target_rad: f32,
    pub error_rad: f32,
    pub stall_backoffs: u32,
    pub duration_ms: u64,
}

#[derive(Debug, Clone, Default, Serialize)]
pub struct StartupRecoverySummary {
    pub stall_backoffs: u32,
    pub joints: Vec<JointHomingResult>,
}

// -- Preflight types --

#[derive(Debug, Clone, Serialize)]
pub struct PreflightViolation {
    pub exceeded_by_rad: f32,
    pub exceeded_by_deg: f32,
    pub which_limit: String,
    pub suggested_fix: String,
    /// True when the position is far beyond ±2π, indicating multi-turn encoder
    /// accumulation rather than a genuine out-of-range joint.
    pub multiturn: bool,
}

#[derive(Debug, Clone, Serialize)]
pub struct PreflightJoint {
    pub joint_name: String,
    pub current_rad: f32,
    pub current_deg: f32,
    pub limit_min_rad: f32,
    pub limit_max_rad: f32,
    pub limit_min_deg: f32,
    pub limit_max_deg: f32,
    pub home_rad: f32,
    pub violation: Option<PreflightViolation>,
    pub online: bool,
}

#[derive(Debug, Clone, Serialize)]
pub struct PreflightResult {
    pub pass: bool,
    pub joints: Vec<PreflightJoint>,
}

// -- Home status types --

#[derive(Debug, Clone, Serialize)]
pub struct JointHomeStatus {
    pub joint_name: String,
    pub home_rad: f32,
    pub current_rad: f32,
    pub error_rad: f32,
    pub at_home: bool,
    pub limits: (f32, f32),
}

// -- Internal --

struct JointStartupParams {
    home_rad: f32,
    limit_min_rad: f32,
    limit_max_rad: f32,
    recovery: StartupRecoveryConfig,
    straight_down_home_at_startup: bool,
}

/// Ordered joint entry preserving YAML field order (shoulder_pitch first, elbow_pitch last).
struct OrderedJoint {
    name: String,
    motor: Arc<Mutex<Motor>>,
}

pub struct Arm {
    joints: Vec<OrderedJoint>,
    joint_startup: Vec<(String, JointStartupParams)>,
}

impl Arm {
    /// Build an Arm from config, looking up shared Motor instances by CAN ID.
    /// Motors must already exist in the provided map (created once at startup).
    pub fn new(config: &ArmConfig, motors: &HashMap<u8, Arc<Mutex<Motor>>>) -> Self {
        let mut joints = Vec::new();
        let mut joint_startup = Vec::new();

        for (name, joint) in config.joints() {
            if let Some(can_id) = joint.can_id {
                let shared_motor = match motors.get(&can_id) {
                    Some(m) => m.clone(),
                    None => {
                        warn!(
                            joint = name,
                            can_id,
                            "skipping joint: no Motor found in shared map for CAN ID"
                        );
                        continue;
                    }
                };

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

                joint_startup.push((
                    name.to_string(),
                    JointStartupParams {
                        home_rad,
                        limit_min_rad: joint.limits.0 as f32,
                        limit_max_rad: joint.limits.1 as f32,
                        recovery: joint.startup_recovery.clone(),
                        straight_down_home_at_startup: straight_down,
                    },
                ));
                joints.push(OrderedJoint {
                    name: name.to_string(),
                    motor: shared_motor,
                });
            }
        }

        Self {
            joints,
            joint_startup,
        }
    }

    fn find_motor(&self, name: &str) -> Option<&Arc<Mutex<Motor>>> {
        self.joints.iter().find(|j| j.name == name).map(|j| &j.motor)
    }

    // -- Pre-flight --

    /// Read every joint's encoder position (without enabling) and check against limits.
    /// Returns `pass: false` if any joint is outside its configured range.
    pub async fn preflight_check(&self) -> Result<PreflightResult> {
        let mut joints_result = Vec::new();
        let mut pass = true;

        for oj in &self.joints {
            let params = self.joint_startup.iter()
                .find(|(n, _)| n == &oj.name)
                .map(|(_, p)| p);
            let params = match params {
                Some(p) => p,
                None => continue,
            };

            let pos = match oj.motor.lock().await.read_position().await {
                Ok(p) => p,
                Err(_) => {
                    joints_result.push(PreflightJoint {
                        joint_name: oj.name.clone(),
                        current_rad: 0.0,
                        current_deg: 0.0,
                        limit_min_rad: params.limit_min_rad,
                        limit_max_rad: params.limit_max_rad,
                        limit_min_deg: params.limit_min_rad.to_degrees(),
                        limit_max_deg: params.limit_max_rad.to_degrees(),
                        home_rad: params.home_rad,
                        violation: None,
                        online: false,
                    });
                    continue;
                }
            };

            let pos_joint = canonical_joint_angle(
                pos,
                params.home_rad,
                params.limit_min_rad,
                params.limit_max_rad,
            );
            let pos_deg = pos_joint.to_degrees();
            let mut violation = None;

            let is_multiturn = pos.abs() > std::f32::consts::TAU;

            if pos_joint < params.limit_min_rad {
                let exceeded = params.limit_min_rad - pos_joint;
                pass = false;
                let suggested_fix = if is_multiturn {
                    format!(
                        "Encoder has multi-turn accumulation ({:.1} turns). \
                         Use \"Zero Encoder\" to reset, then re-check.",
                        pos / std::f32::consts::TAU
                    )
                } else {
                    format!(
                        "Manually rotate {} ~{:.0}° toward the positive direction",
                        oj.name, exceeded.to_degrees()
                    )
                };
                violation = Some(PreflightViolation {
                    exceeded_by_rad: exceeded,
                    exceeded_by_deg: exceeded.to_degrees(),
                    which_limit: "min".to_string(),
                    suggested_fix,
                    multiturn: is_multiturn,
                });
            } else if pos_joint > params.limit_max_rad {
                let exceeded = pos_joint - params.limit_max_rad;
                pass = false;
                let suggested_fix = if is_multiturn {
                    format!(
                        "Encoder has multi-turn accumulation ({:.1} turns). \
                         Use \"Zero Encoder\" to reset, then re-check.",
                        pos / std::f32::consts::TAU
                    )
                } else {
                    format!(
                        "Manually rotate {} ~{:.0}° toward the negative direction",
                        oj.name, exceeded.to_degrees()
                    )
                };
                violation = Some(PreflightViolation {
                    exceeded_by_rad: exceeded,
                    exceeded_by_deg: exceeded.to_degrees(),
                    which_limit: "max".to_string(),
                    suggested_fix,
                    multiturn: is_multiturn,
                });
            }

            joints_result.push(PreflightJoint {
                joint_name: oj.name.clone(),
                current_rad: pos_joint,
                current_deg: pos_deg,
                limit_min_rad: params.limit_min_rad,
                limit_max_rad: params.limit_max_rad,
                limit_min_deg: params.limit_min_rad.to_degrees(),
                limit_max_deg: params.limit_max_rad.to_degrees(),
                home_rad: params.home_rad,
                violation,
                online: true,
            });
        }

        Ok(PreflightResult { pass, joints: joints_result })
    }

    // -- Homing --

    /// Run startup recovery with pre-flight check. If `force` is false and any joint is
    /// outside its limits, returns an error with the preflight result. If `force` is true,
    /// logs a warning and proceeds anyway.
    ///
    /// Joints are homed in YAML field order (shoulder_pitch first, elbow_pitch last).
    /// Each joint is enabled with a gravity-catch hold before recovery begins.
    pub async fn startup_safe_recovery(&self, force: bool) -> Result<StartupRecoverySummary> {
        use std::f32::consts::TAU;

        let preflight = self.preflight_check().await?;
        if !preflight.pass && !force {
            let violations: Vec<String> = preflight.joints.iter()
                .filter_map(|j| j.violation.as_ref().map(|v| {
                    format!("{}: {:.1}° past {} limit", j.joint_name, v.exceeded_by_deg, v.which_limit)
                }))
                .collect();
            anyhow::bail!(
                "Pre-flight check failed: {}. Resolve violations or use force override.",
                violations.join("; ")
            );
        }
        if !preflight.pass && force {
            warn!("Pre-flight check failed but force override is active — proceeding with homing");
        }

        let mut total_stall_backoffs = 0u32;
        let mut joint_results = Vec::new();

        for oj in &self.joints {
            let name = &oj.name;
            let params = self.joint_startup.iter()
                .find(|(n, _)| n == name)
                .map(|(_, p)| p)
                .ok_or_else(|| anyhow::anyhow!("internal: joint '{}' has no startup params", name))?;
            let r = params.recovery.clone();
            let mut home = params.home_rad;
            let limits = (params.limit_min_rad, params.limit_max_rad);

            let mut motor = oj.motor.lock().await;

            let joint_start = Instant::now();

            let start_pos = match motor.read_position().await {
                Ok(p) => p,
                Err(e) => {
                    joint_results.push(JointHomingResult {
                        joint_name: name.clone(),
                        status: JointHomingStatus::Error(format!("{:#}", e)),
                        start_position_rad: 0.0,
                        end_position_rad: 0.0,
                        home_target_rad: home,
                        error_rad: 0.0,
                        stall_backoffs: 0,
                        duration_ms: joint_start.elapsed().as_millis() as u64,
                    });
                    continue;
                }
            };

            if let Some(new_home) = motor.normalize_multiturn(home, TAU).await? {
                info!(
                    joint = %name,
                    new_home_rad = %format_args!("{:.3}", new_home),
                    "encoder had multi-turn accumulation; re-zeroed, recovery target adjusted"
                );
                home = new_home;
            }

            let settle = r.settle_tolerance_rad as f32;
            let large = r.large_error_rad as f32;
            let pos = motor.read_position().await?;
            let err_mag = joint_space_error_mag(pos, home, limits);

            if err_mag <= settle {
                joint_results.push(JointHomingResult {
                    joint_name: name.clone(),
                    status: JointHomingStatus::AlreadyHome,
                    start_position_rad: start_pos,
                    end_position_rad: pos,
                    home_target_rad: home,
                    error_rad: err_mag,
                    stall_backoffs: 0,
                    duration_ms: joint_start.elapsed().as_millis() as u64,
                });
                continue;
            }

            let hold_kp = r.kp_soft;
            let hold_kd = r.kd_soft;

            if err_mag <= large {
                info!(
                    joint = %name,
                    error_rad = err_mag,
                    home_rad = home,
                    "joint near home but not settled; enabling and moving directly"
                );

                if let Err(e) = motor.enable_with_hold(hold_kp, hold_kd).await {
                    joint_results.push(JointHomingResult {
                        joint_name: name.clone(),
                        status: JointHomingStatus::Error(format!("enable failed: {:#}", e)),
                        start_position_rad: start_pos,
                        end_position_rad: pos,
                        home_target_rad: home,
                        error_rad: err_mag,
                        stall_backoffs: 0,
                        duration_ms: joint_start.elapsed().as_millis() as u64,
                    });
                    continue;
                }

                let kp_target = r.kp_settle;
                let kd_target = r.kd_settle;
                let ramp_ticks = r.settle_ramp_ticks.max(1);
                let step_period = std::time::Duration::from_millis(r.step_period_ms);
                let timeout = std::time::Duration::from_secs(5);
                let t_start = Instant::now();

                info!(
                    joint = %name,
                    kp_soft = hold_kp,
                    kp_settle = kp_target,
                    ramp_ticks = ramp_ticks,
                    "near-home ramp: starting gain ramp"
                );

                let mut final_tick = 0u32;
                let mut cur = motor.read_position().await.unwrap_or(pos);
                for tick in 0..ramp_ticks + 40 {
                    final_tick = tick;
                    let t = ((tick + 1) as f32 / ramp_ticks as f32).min(1.0);
                    let kp = hold_kp + (kp_target - hold_kp) * t;
                    let kd = hold_kd + (kd_target - hold_kd) * t;
                    let cmd = motor_cmd_for_joint_target(cur, home, limits);
                    let state = motor.send_control(cmd, 0.0, kp, kd, 0.0).await?;

                    if tick % 10 == 0 || tick == ramp_ticks {
                        info!(
                            joint = %name,
                            tick,
                            kp = format_args!("{:.1}", kp),
                            pos_deg = format_args!("{:.2}", state.angle_rad.to_degrees()),
                            torque_nm = format_args!("{:.2}", state.torque_nm),
                            "near-home ramp progress"
                        );
                    }

                    cur = motor.read_position().await.unwrap_or(cur);
                    let err_j = joint_space_error_mag(cur, home, limits);
                    if err_j <= settle {
                        info!(
                            joint = %name,
                            final_err_deg = format_args!("{:.2}", err_j.to_degrees()),
                            ticks = tick,
                            "near-home ramp: settled"
                        );
                        break;
                    }
                    if t_start.elapsed() >= timeout {
                        info!(
                            joint = %name,
                            err_deg = format_args!("{:.2}", err_j.to_degrees()),
                            "near-home ramp: timeout"
                        );
                        break;
                    }
                    tokio::time::sleep(step_period).await;
                }

                let end_pos = motor.read_position().await.unwrap_or(pos);
                let final_err = joint_space_error_mag(end_pos, home, limits);
                info!(
                    joint = %name,
                    final_err_deg = format_args!("{:.2}", final_err.to_degrees()),
                    end_pos_rad = format_args!("{:.4}", end_pos),
                    ticks = final_tick,
                    elapsed_ms = t_start.elapsed().as_millis() as u64,
                    "near-home ramp complete"
                );
                let status = if final_err <= settle {
                    JointHomingStatus::Homed
                } else {
                    JointHomingStatus::StalledButHomed
                };

                joint_results.push(JointHomingResult {
                    joint_name: name.clone(),
                    status,
                    start_position_rad: start_pos,
                    end_position_rad: end_pos,
                    home_target_rad: home,
                    error_rad: final_err,
                    stall_backoffs: 0,
                    duration_ms: joint_start.elapsed().as_millis() as u64,
                });
                continue;
            }

            info!(
                joint = %name,
                error_rad = err_mag,
                home_rad = home,
                "joint far from home; enabling with gravity-catch and running recovery"
            );

            if let Err(e) = motor.enable_with_hold(hold_kp, hold_kd).await {
                joint_results.push(JointHomingResult {
                    joint_name: name.clone(),
                    status: JointHomingStatus::Error(format!("enable failed: {:#}", e)),
                    start_position_rad: start_pos,
                    end_position_rad: pos,
                    home_target_rad: home,
                    error_rad: err_mag,
                    stall_backoffs: 0,
                    duration_ms: joint_start.elapsed().as_millis() as u64,
                });
                continue;
            }

            match motor.recover_position_if_far(home, &r, Some(limits)).await {
                Ok(stalls) => {
                    total_stall_backoffs += stalls;
                    let end_pos = motor.read_position().await.unwrap_or(pos);
                    let final_err = joint_space_error_mag(end_pos, home, limits);
                    let status = if stalls > 0 {
                        JointHomingStatus::StalledButHomed
                    } else {
                        JointHomingStatus::Homed
                    };
                    joint_results.push(JointHomingResult {
                        joint_name: name.clone(),
                        status,
                        start_position_rad: start_pos,
                        end_position_rad: end_pos,
                        home_target_rad: home,
                        error_rad: final_err,
                        stall_backoffs: stalls,
                        duration_ms: joint_start.elapsed().as_millis() as u64,
                    });
                }
                Err(e) => {
                    let end_pos = motor.read_position().await.unwrap_or(pos);
                    let err_str = format!("{:#}", e);
                    let status = if err_str.contains("timed out") {
                        JointHomingStatus::TimedOut
                    } else {
                        JointHomingStatus::Error(err_str)
                    };
                    joint_results.push(JointHomingResult {
                        joint_name: name.clone(),
                        status,
                        start_position_rad: start_pos,
                        end_position_rad: end_pos,
                        home_target_rad: home,
                        error_rad: joint_space_error_mag(end_pos, home, limits),
                        stall_backoffs: 0,
                        duration_ms: joint_start.elapsed().as_millis() as u64,
                    });
                }
            }
        }

        Ok(StartupRecoverySummary {
            stall_backoffs: total_stall_backoffs,
            joints: joint_results,
        })
    }

    // -- Home status (read-only) --

    pub async fn get_homing_status(&self) -> Result<Vec<JointHomeStatus>> {
        let settle = self.joint_startup.first()
            .map(|(_, p)| p.recovery.settle_tolerance_rad as f32)
            .unwrap_or(0.03);

        let mut statuses = Vec::new();
        for oj in &self.joints {
            let params = self.joint_startup.iter()
                .find(|(n, _)| n == &oj.name)
                .map(|(_, p)| p);
            let params = match params {
                Some(p) => p,
                None => continue,
            };

            let pos = oj.motor.lock().await.read_position().await.unwrap_or(0.0);
            let lim = (params.limit_min_rad, params.limit_max_rad);
            let pos_joint = canonical_joint_angle(pos, params.home_rad, lim.0, lim.1);
            let err = joint_space_error_mag(pos, params.home_rad, lim);
            statuses.push(JointHomeStatus {
                joint_name: oj.name.clone(),
                home_rad: params.home_rad,
                current_rad: pos_joint,
                error_rad: err,
                at_home: err <= settle,
                limits: (params.limit_min_rad, params.limit_max_rad),
            });
        }
        Ok(statuses)
    }

    // -- Existing public API (adapted to ordered storage) --

    /// Call **before** homing for any joint with `straight_down_home_at_startup: true`.
    pub async fn straight_down_home_before_enable(&self) -> Result<usize> {
        let mut n = 0usize;
        for oj in &self.joints {
            let params = self.joint_startup.iter()
                .find(|(nm, _)| nm == &oj.name)
                .map(|(_, p)| p)
                .ok_or_else(|| anyhow::anyhow!("internal: joint '{}' has no startup params", oj.name))?;
            if !params.straight_down_home_at_startup {
                continue;
            }
            info!(
                joint = %oj.name,
                "SetZero -- joint must be straight down; defining mech position 0 as home"
            );
            oj.motor.lock().await.set_zero().await?;
            n += 1;
        }
        Ok(n)
    }

    pub async fn enable_all(&self) -> Result<()> {
        for oj in &self.joints {
            info!("Enabling {}", oj.name);
            oj.motor.lock().await.enable().await?;
        }
        Ok(())
    }

    pub async fn disable_all(&self) -> Result<()> {
        for oj in &self.joints {
            info!("Disabling {}", oj.name);
            let _ = oj.motor.lock().await.disable().await;
        }
        Ok(())
    }

    pub async fn set_joint(
        &self,
        joint_name: &str,
        position_rad: f32,
        kp: Option<f32>,
        kd: Option<f32>,
    ) -> Result<()> {
        let motor = self.find_motor(joint_name)
            .ok_or_else(|| anyhow::anyhow!("Joint '{}' not configured", joint_name))?;
        motor.lock().await.move_to(position_rad, kp, kd).await?;
        Ok(())
    }

    pub async fn get_joint_positions(&self) -> Result<Vec<(String, f32)>> {
        let mut positions = Vec::new();
        for oj in &self.joints {
            let pos = oj.motor.lock().await.read_position().await?;
            positions.push((oj.name.clone(), pos));
        }
        Ok(positions)
    }

    pub fn joint_names(&self) -> Vec<&str> {
        self.joints.iter().map(|j| j.name.as_str()).collect()
    }

    pub fn configured_home_rad(&self, joint_name: &str) -> Option<f32> {
        self.find_startup_params_by_name(joint_name).map(|p| p.home_rad)
    }

    fn find_startup_params_by_name(&self, name: &str) -> Option<&JointStartupParams> {
        self.joint_startup.iter().find(|(n, _)| n == name).map(|(_, p)| p)
    }

    pub async fn update_joint_limits(&mut self, joint_name: &str, min_rad: f32, max_rad: f32) -> bool {
        let mut updated = false;
        for (n, p) in &mut self.joint_startup {
            if n == joint_name {
                p.limit_min_rad = min_rad;
                p.limit_max_rad = max_rad;
                updated = true;
                break;
            }
        }
        if updated {
            if let Some(motor_arc) = self.find_motor(joint_name) {
                motor_arc.lock().await.set_joint_limits(min_rad, max_rad);
            }
        }
        updated
    }

    pub async fn update_joint_home(&mut self, joint_name: &str, home_rad: f32) -> bool {
        let mut updated = false;
        for (n, p) in &mut self.joint_startup {
            if n == joint_name {
                p.home_rad = home_rad;
                updated = true;
                break;
            }
        }
        if updated {
            if let Some(motor_arc) = self.find_motor(joint_name) {
                motor_arc.lock().await.set_home_rad(home_rad);
            }
        }
        updated
    }
}
