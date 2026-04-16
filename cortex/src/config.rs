use std::collections::HashMap;
use std::path::Path;

use anyhow::{Context, Result};
use serde::{Deserialize, Serialize};

#[derive(Debug, Deserialize, Serialize)]
pub struct RobotConfig {
    pub bus: BusConfig,
    pub actuators: HashMap<String, ActuatorSpec>,
    pub arm_left: Option<ArmConfig>,
    pub arm_right: Option<ArmConfig>,
    pub waist: Option<HashMap<String, JointConfig>>,
    pub torso: Option<TorsoConfig>,
}

fn default_transport() -> String {
    "ch341".into()
}

#[derive(Debug, Deserialize, Serialize)]
pub struct BusConfig {
    #[serde(default = "default_transport")]
    pub transport: String,
    pub port: String,
    #[serde(default)]
    pub socketcan_interface: Option<String>,
    pub baud: u32,
    pub can_bitrate: u32,
    pub host_id: u32,
}

#[derive(Debug, Deserialize, Serialize)]
pub struct ActuatorSpec {
    pub max_torque: f64,
    pub max_speed: f64,
    pub max_current: f64,
    pub gear_ratio: f64,
    pub weight_kg: f64,
    pub voltage_nominal: f64,
}

#[derive(Debug, Deserialize, Serialize)]
pub struct ArmConfig {
    pub shoulder_pitch: JointConfig,
    pub shoulder_roll: JointConfig,
    pub upper_arm_yaw: JointConfig,
    pub elbow_pitch: JointConfig,
}

fn default_home_rad() -> f64 {
    0.0
}

fn default_startup_large_error_rad() -> f64 {
    0.35
}

fn default_startup_max_step_rad() -> f64 {
    0.04
}

fn default_startup_settle_tolerance_rad() -> f64 {
    0.03
}

fn default_startup_kp_soft() -> f32 {
    15.0
}

fn default_startup_kd_soft() -> f32 {
    0.9
}

fn default_startup_step_period_ms() -> u64 {
    40
}

fn default_startup_recovery_timeout_secs() -> f64 {
    90.0
}

fn default_true() -> bool {
    true
}

fn default_approach_max_step_rad() -> f64 {
    0.12
}

fn default_approach_kp() -> f32 {
    20.0
}

fn default_approach_kd() -> f32 {
    1.0
}

fn default_approach_step_period_ms() -> u64 {
    25
}

fn default_approach_handoff_rad() -> f64 {
    0.28
}

fn default_approach_max_secs() -> f64 {
    45.0
}

fn default_resistance_torque_nm() -> f32 {
    12.0
}

fn default_resistance_velocity_rads() -> f32 {
    0.15
}

fn default_resistance_confirm_ticks() -> u32 {
    2
}

fn default_resistance_backoff_ms() -> u64 {
    1000
}

fn default_post_stall_motion_scale() -> f64 {
    0.5
}

/// Below this linear distance to target, stall detection is off (final creep is slow and reads like a stall).
/// Must be >= `approach_handoff_rad` so gradual phase after handoff does not false-trigger on gravity-loaded joints.
fn default_stall_detection_min_linear_error_rad() -> f64 {
    0.30
}

fn default_recovery_direct_command_within_rad() -> f64 {
    // ~12.6° — covers common residual (~7°) when homing from negative angles; older 0.12 rad left a gap.
    0.22
}

/// Slack around YAML joint limits for pre-flight / homing gate only (~2°). Does not change Motor clamps.
fn default_preflight_limit_margin_rad() -> f64 {
    (2.0_f64).to_radians()
}

fn default_kp_settle() -> f32 {
    100.0
}

fn default_kd_settle() -> f32 {
    1.0
}

fn default_settle_ramp_ticks() -> u32 {
    20
}

/// When |position − home| exceeds `large_error_rad` at startup: optional **fast approach** toward
/// home, then **gradual** small steps. If torque stays high while velocity is near zero (stall /
/// contact), the joint holds, waits `resistance_backoff_ms`, then **continues** the same routine
/// with gains and step sizes scaled by `post_stall_motion_scale`.
#[derive(Debug, Deserialize, Serialize, Clone)]
pub struct StartupRecoveryConfig {
    #[serde(default = "default_startup_large_error_rad")]
    pub large_error_rad: f64,
    #[serde(default = "default_startup_max_step_rad")]
    pub max_step_rad: f64,
    #[serde(default = "default_startup_settle_tolerance_rad")]
    pub settle_tolerance_rad: f64,
    #[serde(default = "default_startup_kp_soft")]
    pub kp_soft: f32,
    #[serde(default = "default_startup_kd_soft")]
    pub kd_soft: f32,
    #[serde(default = "default_startup_step_period_ms")]
    pub step_period_ms: u64,
    #[serde(default = "default_startup_recovery_timeout_secs")]
    pub recovery_timeout_secs: f64,
    #[serde(default = "default_true")]
    pub approach_enabled: bool,
    #[serde(default = "default_approach_max_step_rad")]
    pub approach_max_step_rad: f64,
    #[serde(default = "default_approach_kp")]
    pub approach_kp: f32,
    #[serde(default = "default_approach_kd")]
    pub approach_kd: f32,
    #[serde(default = "default_approach_step_period_ms")]
    pub approach_step_period_ms: u64,
    /// When |error| falls below this, approach stops and the gradual phase finishes the move.
    #[serde(default = "default_approach_handoff_rad")]
    pub approach_handoff_rad: f64,
    /// Wall-clock cap for the approach phase; after this, gradual motion continues until
    /// `recovery_timeout_secs` overall or success.
    #[serde(default = "default_approach_max_secs")]
    pub approach_max_secs: f64,
    #[serde(default = "default_resistance_torque_nm")]
    pub resistance_torque_nm: f32,
    #[serde(default = "default_resistance_velocity_rads")]
    pub resistance_velocity_rads: f32,
    #[serde(default = "default_resistance_confirm_ticks")]
    pub resistance_confirm_ticks: u32,
    /// Hold at the current angle this long after a stall so the obstacle can clear (e.g. 1 s).
    #[serde(
        default = "default_resistance_backoff_ms",
        alias = "resistance_hold_ms"
    )]
    pub resistance_backoff_ms: u64,
    /// After the first stall in a recovery, multiply approach/gradual step sizes and kp/kd by this
    /// (e.g. 0.5 for “half speed / torque”).
    #[serde(default = "default_post_stall_motion_scale")]
    pub post_stall_motion_scale: f64,
    /// When unbounded and linear `|target−pos| > π`, step direction may use the shortest arc.
    /// Ignored when joint limits are supplied (arm always uses linear steps). Set false to always
    /// use linear delta for unbounded recovery.
    #[serde(default = "default_true")]
    pub prefer_shortest_angle: bool,
    /// Stall/resistance logic runs only while linear `|target−pos| ≥` this. Nearer than that, low
    /// velocity + moderate torque is normal “creeping home” and would false-trigger.
    #[serde(default = "default_stall_detection_min_linear_error_rad")]
    pub stall_detection_min_linear_error_rad: f64,
    /// In gradual recovery, when linear `|target−pos|` is below this but above settle tolerance,
    /// command `target` directly each cycle (soft gains) instead of only `pos±step` — helps stiction.
    #[serde(default = "default_recovery_direct_command_within_rad")]
    pub recovery_direct_command_within_rad: f64,
    /// Position gain for the final settle ramp. Once inside `recovery_direct_command_within_rad`,
    /// kp ramps linearly from `kp_soft` up to this value over `settle_ramp_ticks` cycles so the
    /// motor has enough authority to close the last few degrees against gravity/friction.
    #[serde(default = "default_kp_settle")]
    pub kp_settle: f32,
    /// Damping gain for the final settle ramp (pairs with `kp_settle`).
    #[serde(default = "default_kd_settle")]
    pub kd_settle: f32,
    /// How many control cycles to ramp from soft gains to settle gains once inside the direct
    /// command zone. Higher = gentler ramp. 0 or 1 = jump immediately to settle gains.
    #[serde(default = "default_settle_ramp_ticks")]
    pub settle_ramp_ticks: u32,
    /// Extra slack (rad) outside configured joint limits for **pre-flight only**. Joints in this
    /// band still pass and may home; API/motor hard limits are unchanged. Ignored when the raw
    /// encoder reading indicates multi-turn accumulation (`|pos| > 2π`), which stays a hard fail.
    #[serde(default = "default_preflight_limit_margin_rad")]
    pub preflight_limit_margin_rad: f64,
}

impl Default for StartupRecoveryConfig {
    fn default() -> Self {
        Self {
            large_error_rad: default_startup_large_error_rad(),
            max_step_rad: default_startup_max_step_rad(),
            settle_tolerance_rad: default_startup_settle_tolerance_rad(),
            kp_soft: default_startup_kp_soft(),
            kd_soft: default_startup_kd_soft(),
            step_period_ms: default_startup_step_period_ms(),
            recovery_timeout_secs: default_startup_recovery_timeout_secs(),
            approach_enabled: default_true(),
            approach_max_step_rad: default_approach_max_step_rad(),
            approach_kp: default_approach_kp(),
            approach_kd: default_approach_kd(),
            approach_step_period_ms: default_approach_step_period_ms(),
            approach_handoff_rad: default_approach_handoff_rad(),
            approach_max_secs: default_approach_max_secs(),
            resistance_torque_nm: default_resistance_torque_nm(),
            resistance_velocity_rads: default_resistance_velocity_rads(),
            resistance_confirm_ticks: default_resistance_confirm_ticks(),
            resistance_backoff_ms: default_resistance_backoff_ms(),
            post_stall_motion_scale: default_post_stall_motion_scale(),
            prefer_shortest_angle: default_true(),
            stall_detection_min_linear_error_rad: default_stall_detection_min_linear_error_rad(),
            recovery_direct_command_within_rad: default_recovery_direct_command_within_rad(),
            kp_settle: default_kp_settle(),
            kd_settle: default_kd_settle(),
            settle_ramp_ticks: default_settle_ramp_ticks(),
            preflight_limit_margin_rad: default_preflight_limit_margin_rad(),
        }
    }
}

#[derive(Debug, Deserialize, Serialize)]
pub struct JointConfig {
    pub can_id: Option<u8>,
    pub actuator: String,
    pub limits: (f64, f64),
    /// Joint angle (rad) when this link points **straight down** for this actuator placement — a
    /// fixed geometry constant (from CAD / one-time calibration), not something that drifts per boot.
    /// Startup recovery and “return home” target this value.
    #[serde(default = "default_home_rad")]
    pub home_rad: f64,
    /// If true: before enable, `set_zero` with the joint held at straight down so encoder 0 = down;
    /// `home_rad` is then ignored (forced to 0). Use for teach-at-power-up. If down is already a
    /// known `home_rad` in encoder space, leave this false.
    #[serde(default)]
    pub straight_down_home_at_startup: bool,
    #[serde(default)]
    pub startup_recovery: StartupRecoveryConfig,
}

#[derive(Debug, Deserialize, Serialize)]
pub struct TorsoConfig {
    pub frame: String,
    pub dimensions_mm: (u32, u32, u32),
}

impl RobotConfig {
    pub fn load(path: impl AsRef<Path>) -> Result<Self> {
        let contents = std::fs::read_to_string(path.as_ref())
            .with_context(|| format!("Failed to read config: {}", path.as_ref().display()))?;
        let config: RobotConfig = serde_yaml::from_str(&contents)
            .with_context(|| "Failed to parse robot.yaml")?;
        Ok(config)
    }

    pub fn save(&self, path: impl AsRef<Path>) -> Result<()> {
        let yaml = serde_yaml::to_string(self)
            .with_context(|| "Failed to serialize config to YAML")?;
        let header = "# Robot hardware configuration\n\
                      # Auto-saved by Link — manual comments are not preserved.\n\n";
        std::fs::write(path.as_ref(), format!("{}{}", header, yaml))
            .with_context(|| format!("Failed to write config: {}", path.as_ref().display()))?;
        Ok(())
    }

    /// Clear any existing assignment of `can_id` across all sections.
    pub fn clear_can_id(&mut self, can_id: u8) {
        if let Some(ref mut arm) = self.arm_left {
            arm.clear_can_id(can_id);
        }
        if let Some(ref mut arm) = self.arm_right {
            arm.clear_can_id(can_id);
        }
        if let Some(ref mut waist) = self.waist {
            for joint in waist.values_mut() {
                if joint.can_id == Some(can_id) {
                    joint.can_id = None;
                }
            }
        }
    }

    /// Assign `can_id` to a specific section+joint. Returns Err if the slot doesn't exist.
    pub fn assign_can_id(&mut self, section: &str, joint: &str, can_id: u8) -> Result<()> {
        self.clear_can_id(can_id);

        match section {
            "arm_left" => {
                let arm = self.arm_left.as_mut()
                    .with_context(|| "arm_left section not configured")?;
                arm.set_can_id(joint, Some(can_id))
                    .with_context(|| format!("unknown joint '{}' in arm_left", joint))?;
            }
            "arm_right" => {
                let arm = self.arm_right.as_mut()
                    .with_context(|| "arm_right section not configured")?;
                arm.set_can_id(joint, Some(can_id))
                    .with_context(|| format!("unknown joint '{}' in arm_right", joint))?;
            }
            "waist" => {
                let waist = self.waist.as_mut()
                    .with_context(|| "waist section not configured")?;
                let jc = waist.get_mut(joint)
                    .with_context(|| format!("unknown joint '{}' in waist", joint))?;
                jc.can_id = Some(can_id);
            }
            _ => anyhow::bail!("unknown section '{}'", section),
        }
        Ok(())
    }

    /// CAN IDs assigned to joints in this config (same set used to register `Motor` handles at startup).
    pub fn assigned_can_ids(&self) -> Vec<u8> {
        let mut ids = Vec::new();
        for arm in [self.arm_left.as_ref(), self.arm_right.as_ref()]
            .into_iter()
            .flatten()
        {
            for (_name, joint) in arm.joints() {
                if let Some(id) = joint.can_id {
                    ids.push(id);
                }
            }
        }
        if let Some(ref waist) = self.waist {
            for (_name, joint) in waist {
                if let Some(id) = joint.can_id {
                    ids.push(id);
                }
            }
        }
        ids
    }

    /// List all available joint slots with their current CAN ID assignment.
    pub fn joint_slots(&self) -> Vec<(String, String, Option<u8>)> {
        let mut slots = Vec::new();
        for (section, arm_opt) in [("arm_left", &self.arm_left), ("arm_right", &self.arm_right)] {
            if let Some(arm) = arm_opt {
                for (name, joint) in arm.joints() {
                    slots.push((section.to_string(), name.to_string(), joint.can_id));
                }
            }
        }
        if let Some(ref waist) = self.waist {
            for (name, joint) in waist {
                slots.push(("waist".to_string(), name.clone(), joint.can_id));
            }
        }
        slots
    }
}

impl ArmConfig {
    pub fn joints(&self) -> [(&str, &JointConfig); 4] {
        [
            ("shoulder_pitch", &self.shoulder_pitch),
            ("shoulder_roll", &self.shoulder_roll),
            ("upper_arm_yaw", &self.upper_arm_yaw),
            ("elbow_pitch", &self.elbow_pitch),
        ]
    }

    pub fn joints_mut(&mut self) -> [(&str, &mut JointConfig); 4] {
        [
            ("shoulder_pitch", &mut self.shoulder_pitch),
            ("shoulder_roll", &mut self.shoulder_roll),
            ("upper_arm_yaw", &mut self.upper_arm_yaw),
            ("elbow_pitch", &mut self.elbow_pitch),
        ]
    }

    pub fn active_joints(&self) -> Vec<(&str, &JointConfig)> {
        self.joints()
            .into_iter()
            .filter(|(_, j)| j.can_id.is_some())
            .collect()
    }

    pub fn clear_can_id(&mut self, can_id: u8) {
        for (_, joint) in self.joints_mut() {
            if joint.can_id == Some(can_id) {
                joint.can_id = None;
            }
        }
    }

    /// Set the CAN ID for a named joint. Returns Ok(()) if the joint exists, Err otherwise.
    pub fn set_can_id(&mut self, joint_name: &str, can_id: Option<u8>) -> Result<()> {
        for (name, joint) in self.joints_mut() {
            if name == joint_name {
                joint.can_id = can_id;
                return Ok(());
            }
        }
        anyhow::bail!("unknown joint '{}'", joint_name)
    }
}
