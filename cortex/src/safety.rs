use crate::config::RobotConfig;

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

/// Default soft-limit margin in radians (~10 degrees). Velocity/torque commands ramp
/// down linearly within this zone when pushing toward a limit.
pub const DEFAULT_SOFT_LIMIT_MARGIN_RAD: f32 = 0.175;

/// Epsilon (rad) when testing whether a candidate `pos + n*2pi` lies inside joint limits.
pub const JOINT_UNWRAP_EPS: f32 = 0.12;

// ---------------------------------------------------------------------------
// Config lookup helpers
// ---------------------------------------------------------------------------

/// Look up the joint limits for a motor by its CAN ID across all config sections.
/// Returns `Some((min_rad, max_rad))` if found, `None` if the CAN ID is not assigned.
pub fn limits_for_motor(config: &RobotConfig, can_id: u8) -> Option<(f32, f32)> {
    let arms = [config.arm_left.as_ref(), config.arm_right.as_ref()];
    for arm in arms.iter().flatten() {
        for (_name, joint) in arm.joints() {
            if joint.can_id == Some(can_id) {
                return Some((joint.limits.0 as f32, joint.limits.1 as f32));
            }
        }
    }
    if let Some(ref waist) = config.waist {
        for (_name, joint) in waist {
            if joint.can_id == Some(can_id) {
                return Some((joint.limits.0 as f32, joint.limits.1 as f32));
            }
        }
    }
    None
}

/// Look up the home position for a motor by its CAN ID.
pub fn home_for_motor(config: &RobotConfig, can_id: u8) -> Option<f32> {
    let arms = [config.arm_left.as_ref(), config.arm_right.as_ref()];
    for arm in arms.iter().flatten() {
        for (_name, joint) in arm.joints() {
            if joint.can_id == Some(can_id) {
                return Some(joint.home_rad as f32);
            }
        }
    }
    if let Some(ref waist) = config.waist {
        for (_name, joint) in waist {
            if joint.can_id == Some(can_id) {
                return Some(joint.home_rad as f32);
            }
        }
    }
    None
}

// ---------------------------------------------------------------------------
// Pure angle/position math
// ---------------------------------------------------------------------------

/// RS03 MechPos is nominally within ±4π in the MIT frame. Multi-turn accumulation
/// can exceed one revolution; values beyond this are almost always corrupt floats
/// (wrong CAN frame, parse error, or bus noise) and must be rejected.
pub const MAX_REASONABLE_MECH_POS_RAD: f32 = 80.0;

#[inline]
pub fn is_valid_mech_pos_reading(rad: f32) -> bool {
    rad.is_finite() && rad.abs() <= MAX_REASONABLE_MECH_POS_RAD
}

/// Signed smallest angle from `from_rad` to `to_rad`, in (-pi, pi].
#[inline]
pub fn shortest_angle_err(from_rad: f32, to_rad: f32) -> f32 {
    use std::f32::consts::{PI, TAU};
    let d = to_rad - from_rad;
    (d + PI).rem_euclid(TAU) - PI
}

/// Pick the representative of `pos_rad` modulo 2pi that lies in `[limit_lo, limit_hi]` (with slack).
/// If several candidates fit (range > 2pi), choose the one closest to `home_rad`.
///
/// After power loss the RS03 may report the same physical pose on a different 2pi branch than
/// `home_rad` / limits. This maps the raw reading into the joint configuration frame.
pub fn canonical_joint_angle(
    pos_rad: f32,
    home_rad: f32,
    limit_lo: f32,
    limit_hi: f32,
) -> f32 {
    use std::f32::consts::TAU;
    let mut best: Option<(f32, f32)> = None;
    for k in -4..=4 {
        let p = pos_rad + (k as f32) * TAU;
        if p >= limit_lo - JOINT_UNWRAP_EPS && p <= limit_hi + JOINT_UNWRAP_EPS {
            let d = (p - home_rad).abs();
            match best {
                None => best = Some((p, d)),
                Some((_, bd)) if d < bd => best = Some((p, d)),
                _ => {}
            }
        }
    }
    best.map(|(p, _)| p).unwrap_or_else(|| {
        if is_valid_mech_pos_reading(pos_rad) {
            pos_rad
        } else {
            home_rad.clamp(limit_lo, limit_hi)
        }
    })
}

/// Error vs joint home using the canonical 2pi branch inside limits.
#[inline]
pub fn joint_space_error_mag(pos_raw: f32, target_rad: f32, limits: (f32, f32)) -> f32 {
    let cj = canonical_joint_angle(pos_raw, target_rad, limits.0, limits.1);
    (cj - target_rad).abs()
}

/// Raw MIT position command that corresponds to joint angle `target_rad` when the motor
/// currently reads `pos_raw` (handles branch mismatch between feedback and config frame).
#[inline]
pub fn motor_cmd_for_joint_target(pos_raw: f32, target_rad: f32, limits: (f32, f32)) -> f32 {
    let cj = canonical_joint_angle(pos_raw, target_rad, limits.0, limits.1);
    pos_raw + target_rad - cj
}

/// Linear error in the encoder frame (not wrapped).
#[inline]
pub fn linear_error(pos_rad: f32, target_rad: f32) -> f32 {
    target_rad - pos_rad
}

/// Step direction toward target. With **bounded** joints, always linear --
/// shortest arc is wrong for a limited range. Otherwise, shortest arc only
/// when linear error > pi and `prefer_shortest_angle`.
#[inline]
pub fn step_delta_toward_home(
    pos_rad: f32,
    target_rad: f32,
    prefer_shortest_angle: bool,
    bounded_joint: bool,
) -> f32 {
    use std::f32::consts::PI;
    let linear = linear_error(pos_rad, target_rad);
    if bounded_joint || !prefer_shortest_angle || linear.abs() <= PI {
        linear
    } else {
        shortest_angle_err(pos_rad, target_rad)
    }
}

/// Clamp a commanded position to joint limits when limits are set.
#[inline]
pub fn clamp_cmd_to_limits(cmd: f32, joint_limits_rad: Option<(f32, f32)>) -> f32 {
    joint_limits_rad.map_or(cmd, |(lo, hi)| cmd.clamp(lo, hi))
}

// ---------------------------------------------------------------------------
// Multi-turn aware position for limit math
// ---------------------------------------------------------------------------

/// Map a raw encoder position into joint-space for limit checks.
/// If the motor has accumulated multi-turn offset (e.g. 1080 deg after spinning),
/// this finds the equivalent position within the limit band (if one exists).
/// `home_rad` is needed to disambiguate when multiple 2pi branches fit inside limits.
///
/// When no limits are set, returns the raw position unchanged.
pub fn canonical_position_for_limits(
    raw_pos: f32,
    home_rad: f32,
    limits: Option<(f32, f32)>,
) -> f32 {
    match limits {
        Some((lo, hi)) => canonical_joint_angle(raw_pos, home_rad, lo, hi),
        None => raw_pos,
    }
}

/// Check whether a canonical position is within joint limits.
pub fn is_within_limits(canonical_pos: f32, limits: (f32, f32)) -> bool {
    canonical_pos >= limits.0 - JOINT_UNWRAP_EPS && canonical_pos <= limits.1 + JOINT_UNWRAP_EPS
}

/// Whether a persisted `home_rad` is acceptable relative to joint limits.
/// Uses the same ±[`JOINT_UNWRAP_EPS`] slack as [`canonical_joint_angle`] and [`is_within_limits`].
/// This allows `0.0` when limits start slightly above zero (e.g. `[0.087, 2.793]`), matching
/// the case where the encoder was just zeroed at the home pose (`Zero & Set Home`).
pub fn is_home_within_joint_limits(home_rad: f64, limit_lo: f64, limit_hi: f64) -> bool {
    let eps = JOINT_UNWRAP_EPS as f64;
    home_rad >= limit_lo - eps && home_rad <= limit_hi + eps
}

// ---------------------------------------------------------------------------
// Soft-limit effort scaling
// ---------------------------------------------------------------------------

/// Scale factor (0.0-1.0) for velocity or torque near joint limits.
///
/// Uses the **canonical** position (not raw encoder) so multi-turn accumulation
/// does not break the math. Positive `signed_effort` pushes toward max, negative
/// toward min. Returns 1.0 if no limits, no position, or comfortably inside range.
#[inline]
pub fn soft_limit_effort_scale(
    joint_limits: Option<(f32, f32)>,
    last_known_position: Option<f32>,
    home_rad: f32,
    margin_rad: f32,
    signed_effort: f32,
) -> f32 {
    let (lo, hi) = match joint_limits {
        Some(l) => l,
        None => return 1.0,
    };
    let raw_pos = match last_known_position {
        Some(p) => p,
        None => return 1.0,
    };
    let margin = margin_rad;
    if margin <= 0.0 {
        return 1.0;
    }

    let pos = canonical_joint_angle(raw_pos, home_rad, lo, hi);

    if signed_effort > 0.0 {
        let dist_to_max = hi - pos;
        if dist_to_max <= 0.0 {
            return 0.0;
        }
        if dist_to_max < margin {
            return (dist_to_max / margin).clamp(0.0, 1.0);
        }
    } else if signed_effort < 0.0 {
        let dist_to_min = pos - lo;
        if dist_to_min <= 0.0 {
            return 0.0;
        }
        if dist_to_min < margin {
            return (dist_to_min / margin).clamp(0.0, 1.0);
        }
    }
    1.0
}

// ---------------------------------------------------------------------------
// Unified command validators
// ---------------------------------------------------------------------------

/// Validate a velocity (spin) command against joint limits.
///
/// Returns `Ok(scaled_velocity)` if the command is allowed (possibly scaled down
/// near limits), or `Err(message)` if the command must be rejected.
///
/// Hard-rejects when the canonical position is outside limits entirely.
/// Soft-scales the velocity linearly to zero within the margin zone.
pub fn validate_velocity_command(
    raw_pos: f32,
    home_rad: f32,
    limits: Option<(f32, f32)>,
    margin_rad: f32,
    velocity_rads: f32,
) -> Result<f32, String> {
    let (lo, hi) = match limits {
        Some(l) => l,
        None => return Ok(velocity_rads),
    };

    let pos = canonical_joint_angle(raw_pos, home_rad, lo, hi);

    if pos < lo - JOINT_UNWRAP_EPS {
        if velocity_rads < 0.0 {
            return Err(format!(
                "position {:.3} rad below min limit {:.3} — only positive velocity allowed to return to range",
                pos, lo
            ));
        }
        return Ok(velocity_rads);
    }
    if pos > hi + JOINT_UNWRAP_EPS {
        if velocity_rads > 0.0 {
            return Err(format!(
                "position {:.3} rad above max limit {:.3} — only negative velocity allowed to return to range",
                pos, hi
            ));
        }
        return Ok(velocity_rads);
    }

    let scale = soft_limit_effort_scale(
        limits,
        Some(pos),
        home_rad,
        margin_rad,
        velocity_rads,
    );
    Ok(velocity_rads * scale)
}

/// Validate a torque command against joint limits. Same pattern as velocity.
pub fn validate_torque_command(
    raw_pos: f32,
    home_rad: f32,
    limits: Option<(f32, f32)>,
    margin_rad: f32,
    torque_nm: f32,
) -> Result<f32, String> {
    let (lo, hi) = match limits {
        Some(l) => l,
        None => return Ok(torque_nm),
    };

    let pos = canonical_joint_angle(raw_pos, home_rad, lo, hi);

    if pos < lo - JOINT_UNWRAP_EPS {
        if torque_nm < 0.0 {
            return Err(format!(
                "position {:.3} rad below min limit {:.3} — only positive torque allowed to return to range",
                pos, lo
            ));
        }
        return Ok(torque_nm);
    }
    if pos > hi + JOINT_UNWRAP_EPS {
        if torque_nm > 0.0 {
            return Err(format!(
                "position {:.3} rad above max limit {:.3} — only negative torque allowed to return to range",
                pos, hi
            ));
        }
        return Ok(torque_nm);
    }

    let scale = soft_limit_effort_scale(
        limits,
        Some(pos),
        home_rad,
        margin_rad,
        torque_nm,
    );
    Ok(torque_nm * scale)
}

// ---------------------------------------------------------------------------
// Stall / collision detection
// ---------------------------------------------------------------------------

/// Configurable thresholds for stall (collision) detection.
#[derive(Debug, Clone)]
pub struct StallConfig {
    /// Minimum absolute torque (N-m) to consider the motor loaded.
    pub torque_trip_nm: f32,
    /// Maximum absolute velocity (rad/s) below which the motor is considered stalled.
    pub velocity_trip_rads: f32,
    /// How many consecutive stall ticks before tripping.
    pub confirm_ticks: u32,
}

impl Default for StallConfig {
    fn default() -> Self {
        Self {
            torque_trip_nm: 12.0,
            velocity_trip_rads: 0.15,
            confirm_ticks: 3,
        }
    }
}

/// Tracks consecutive stall ticks and trips when the threshold is reached.
///
/// Embed in `Motor` or use standalone. Call `update()` after each control tick
/// with the feedback torque and velocity. Returns `true` when a stall trip fires.
#[derive(Debug)]
pub struct StallDetector {
    config: StallConfig,
    streak: u32,
    enabled: bool,
}

impl StallDetector {
    pub fn new(config: StallConfig) -> Self {
        Self {
            config,
            streak: 0,
            enabled: true,
        }
    }

    /// Feed one tick of motor feedback. Returns `true` if a stall trip fires.
    pub fn update(&mut self, torque_nm: f32, velocity_rads: f32) -> bool {
        if !self.enabled {
            return false;
        }
        let looks_blocked = torque_nm.abs() >= self.config.torque_trip_nm
            && velocity_rads.abs() <= self.config.velocity_trip_rads;
        if looks_blocked {
            self.streak += 1;
            self.streak >= self.config.confirm_ticks
        } else {
            self.streak = 0;
            false
        }
    }

    /// Reset the consecutive stall counter to zero.
    pub fn reset(&mut self) {
        self.streak = 0;
    }

    pub fn enable(&mut self) {
        self.enabled = true;
        self.streak = 0;
    }

    pub fn disable(&mut self) {
        self.enabled = false;
        self.streak = 0;
    }

    pub fn is_enabled(&self) -> bool {
        self.enabled
    }

    pub fn config(&self) -> &StallConfig {
        &self.config
    }

    pub fn set_config(&mut self, config: StallConfig) {
        self.config = config;
        self.streak = 0;
    }
}
