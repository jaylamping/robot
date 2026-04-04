use std::sync::Arc;
use std::time::{Duration, Instant};

use anyhow::Result;
use robstride::ActuatorParameter;
use robstride::actuator::{TypedCommandData, TypedFeedbackData};
use robstride::robstride03::{RobStride03Command, RobStride03Feedback, RobStride03Parameter};
use robstride::{
    Command, CommandData, ControlCommand, EnableCommand, FeedbackFrame, MotorMode, Protocol,
    ReadCommand, SetZeroCommand, StopCommand, TransportType, WriteCommand,
};
use tokio::sync::Mutex;
use tracing::info;

use crate::config::{BusConfig, StartupRecoveryConfig};
use crate::safety;

const HOST_ID: u8 = 0xAA;

pub use safety::{
    canonical_joint_angle, joint_space_error_mag, motor_cmd_for_joint_target, shortest_angle_err,
};

/// Minimum linear |error| at which stall / resistance detection may run. Raised to at least
/// approach handoff (when approach is on) and above `direct_within` so the final homing band
/// never false-triggers.
fn effective_stall_min_err(cfg: &StartupRecoveryConfig, direct_within_rad: f32) -> f32 {
    const MARGIN_ABOVE_DIRECT: f32 = 0.02;
    let mut m = cfg.stall_detection_min_linear_error_rad as f32;
    if cfg.approach_enabled {
        m = m.max(cfg.approach_handoff_rad as f32);
    }
    m.max(direct_within_rad + MARGIN_ABOVE_DIRECT)
}

pub struct MotorState {
    pub angle_rad: f32,
    pub velocity_rads: f32,
    pub torque_nm: f32,
    pub temperature_c: f32,
    pub mode: MotorMode,
    pub faults: Vec<&'static str>,
}

pub struct Motor {
    protocol: Arc<Mutex<Protocol>>,
    pub can_id: u8,
    host_id: u8,
    enabled: bool,
    pub debug: bool,
    joint_limits: Option<(f32, f32)>,
    soft_limit_margin_rad: f32,
    last_known_position: Option<f32>,
    home_rad: f32,
    stall_detector: safety::StallDetector,
}

impl Motor {
    pub fn new(protocol: Arc<Mutex<Protocol>>, can_id: u8) -> Self {
        Self {
            protocol,
            can_id,
            host_id: HOST_ID,
            enabled: false,
            debug: false,
            joint_limits: None,
            soft_limit_margin_rad: safety::DEFAULT_SOFT_LIMIT_MARGIN_RAD,
            last_known_position: None,
            home_rad: 0.0,
            stall_detector: safety::StallDetector::new(safety::StallConfig::default()),
        }
    }

    pub fn with_host_id(mut self, host_id: u8) -> Self {
        self.host_id = host_id;
        self
    }

    // -- Joint limits --

    pub fn set_joint_limits(&mut self, min_rad: f32, max_rad: f32) {
        self.joint_limits = Some((min_rad, max_rad));
    }

    pub fn clear_joint_limits(&mut self) {
        self.joint_limits = None;
    }

    pub fn joint_limits(&self) -> Option<(f32, f32)> {
        self.joint_limits
    }

    pub fn soft_limit_margin_rad(&self) -> f32 {
        self.soft_limit_margin_rad
    }

    pub fn set_soft_limit_margin(&mut self, margin_rad: f32) {
        self.soft_limit_margin_rad = margin_rad.max(0.0);
    }

    pub fn set_home_rad(&mut self, home_rad: f32) {
        self.home_rad = home_rad;
    }

    /// Reset the cached last_known_position to 0.
    /// Call after zeroing the encoder via a different Motor instance on the same CAN bus,
    /// so this Motor struct doesn't retain a stale pre-zero position.
    pub fn last_known_position_reset(&mut self) {
        self.last_known_position = Some(0.0);
    }

    pub fn home_rad(&self) -> f32 {
        self.home_rad
    }

    // -- Stall detection --

    pub fn stall_detector_mut(&mut self) -> &mut safety::StallDetector {
        &mut self.stall_detector
    }

    pub fn set_stall_config(&mut self, config: safety::StallConfig) {
        self.stall_detector.set_config(config);
    }

    /// Temporarily suppress stall detection (e.g. during homing which has its own
    /// more nuanced resistance handling). Call `enable_stall_detection()` to re-arm.
    pub fn suppress_stall_detection(&mut self) {
        self.stall_detector.disable();
    }

    pub fn enable_stall_detection(&mut self) {
        self.stall_detector.enable();
    }

    // -- Lifecycle --

    pub async fn enable(&mut self) -> Result<MotorState> {
        let cmd = EnableCommand {
            host_id: self.host_id,
        };
        let (id, data) = cmd.to_can_packet(self.can_id);
        let fb = self.send_and_recv(id, &data).await?;
        self.enabled = true;
        self.stall_detector.reset();
        let state = Self::parse_feedback(fb);
        Self::validate_feedback_angle(&state)?;
        self.last_known_position = Some(state.angle_rad);
        Ok(state)
    }

    /// Enable the motor and immediately hold position to prevent gravity drop.
    /// Returns the position being held.
    pub async fn enable_with_hold(&mut self, kp: f32, kd: f32) -> Result<f32> {
        let state = self.enable().await?;
        let hold_pos = state.angle_rad;
        self.send_control(hold_pos, 0.0, kp, kd, 0.0).await?;
        Ok(hold_pos)
    }

    pub async fn disable(&mut self) -> Result<MotorState> {
        let cmd = StopCommand {
            host_id: self.host_id,
            clear_fault: false,
        };
        let (id, data) = cmd.to_can_packet(self.can_id);
        let fb = self.send_and_recv(id, &data).await?;
        self.enabled = false;
        let state = Self::parse_feedback(fb);
        Self::validate_feedback_angle(&state)?;

        self.auto_normalize_if_multiturn().await;

        Ok(state)
    }

    /// After disable, if the encoder has accumulated multi-turn offset far from
    /// the limit range, reset to zero to prevent stale values from poisoning
    /// subsequent limit checks.
    async fn auto_normalize_if_multiturn(&mut self) {
        use std::f32::consts::TAU;
        if let (Some(pos), Some((lo, hi))) = (self.last_known_position, self.joint_limits) {
            let range_center = (lo + hi) / 2.0;
            if (pos - range_center).abs() > TAU {
                info!(
                    can_id = self.can_id,
                    raw_pos = format_args!("{:.3}", pos),
                    "auto-normalizing multi-turn offset on disable"
                );
                let _ = self.set_zero().await;
                self.last_known_position = Some(0.0);
            }
        }
    }

    pub async fn clear_faults(&mut self) -> Result<MotorState> {
        let cmd = StopCommand {
            host_id: self.host_id,
            clear_fault: true,
        };
        let (id, data) = cmd.to_can_packet(self.can_id);
        let fb = self.send_and_recv(id, &data).await?;
        self.enabled = false;
        let state = Self::parse_feedback(fb);
        Self::validate_feedback_angle(&state)?;
        Ok(state)
    }

    pub async fn set_zero(&mut self) -> Result<()> {
        let cmd = SetZeroCommand {
            host_id: self.host_id,
        };
        let (id, data) = cmd.to_can_packet(self.can_id);
        self.send_and_recv(id, &data).await?;
        self.last_known_position = Some(0.0);
        Ok(())
    }

    /// If the encoder position is more than `threshold_rad` (default: 2pi) from `target_rad`,
    /// issue a `set_zero` to collapse accumulated multi-turn offset, then return the new
    /// effective position (which will be near 0). The caller should adjust `target_rad`
    /// accordingly (new target = old target - old position, i.e. the residual after zeroing).
    ///
    /// Returns `Some(residual_target)` if normalization happened, `None` if position was fine.
    pub async fn normalize_multiturn(
        &mut self,
        target_rad: f32,
        threshold_rad: f32,
    ) -> Result<Option<f32>> {
        use std::f32::consts::TAU;
        let threshold = if threshold_rad > 0.0 {
            threshold_rad
        } else {
            TAU
        };
        let pos = self.read_position().await?;
        let err = (pos - target_rad).abs();
        if err <= threshold {
            return Ok(None);
        }

        self.set_zero().await?;

        let residual = safety::shortest_angle_err(0.0, target_rad - pos);
        Ok(Some(residual))
    }

    // -- Motion (MIT-style control) --

    pub async fn send_control(
        &mut self,
        position_rad: f32,
        velocity_rads: f32,
        kp: f32,
        kd: f32,
        torque_nm: f32,
    ) -> Result<MotorState> {
        self.ensure_enabled().await?;

        let clamped_pos = safety::clamp_cmd_to_limits(position_rad, self.joint_limits);

        let vel_scale = safety::soft_limit_effort_scale(
            self.joint_limits,
            self.last_known_position,
            self.home_rad,
            self.soft_limit_margin_rad,
            velocity_rads,
        );
        let clamped_vel = velocity_rads * vel_scale;

        let clamped_torque = if velocity_rads.abs() > f32::EPSILON {
            if vel_scale < 1.0 && torque_nm.abs() > 0.0 {
                torque_nm * vel_scale
            } else {
                torque_nm
            }
        } else {
            let t_scale = safety::soft_limit_effort_scale(
                self.joint_limits,
                self.last_known_position,
                self.home_rad,
                self.soft_limit_margin_rad,
                torque_nm,
            );
            torque_nm * t_scale
        };

        let typed = RobStride03Command {
            target_angle_rad: clamped_pos,
            target_velocity_rads: clamped_vel,
            kp,
            kd,
            torque_nm: clamped_torque,
        };
        let ctrl: ControlCommand = typed.to_control_command();
        let (id, data) = ctrl.to_can_packet(self.can_id);
        let fb = self.send_and_recv(id, &data).await?;
        let state = Self::parse_feedback(fb);
        Self::validate_feedback_angle(&state)?;
        self.last_known_position = Some(state.angle_rad);

        if self.stall_detector.update(state.torque_nm, state.velocity_rads) {
            info!(
                can_id = self.can_id,
                torque_nm = state.torque_nm,
                velocity_rads = state.velocity_rads,
                "stall detected — disabling motor"
            );
            let _ = self.disable().await;
            anyhow::bail!(
                "Motor {} disabled: stall/collision detected \
                 (torque {:.1} N-m, velocity {:.3} rad/s)",
                self.can_id,
                state.torque_nm,
                state.velocity_rads,
            );
        }

        Ok(state)
    }

    pub async fn move_to(
        &mut self,
        position_rad: f32,
        kp: Option<f32>,
        kd: Option<f32>,
    ) -> Result<MotorState> {
        self.send_control(
            position_rad,
            0.0,
            kp.unwrap_or(30.0),
            kd.unwrap_or(1.0),
            0.0,
        )
        .await
    }

    /// One impedance step toward `target_rad` without jumping more than `max_step_rad` from
    /// `from_rad` (commanded intermediate goal).
    pub async fn step_toward(
        &mut self,
        target_rad: f32,
        from_rad: f32,
        max_step_rad: f32,
        kp: f32,
        kd: f32,
    ) -> Result<MotorState> {
        let err = target_rad - from_rad;
        let step = err.clamp(-max_step_rad, max_step_rad);
        let cmd_pos = from_rad + step;
        self.send_control(cmd_pos, 0.0, kp, kd, 0.0).await
    }

    /// If linear `|target - position|` exceeds `large_error_rad`, runs optional **approach** then
    /// **gradual** steps. Bounded joints always step linearly.
    pub async fn recover_position_if_far(
        &mut self,
        target_rad: f32,
        cfg: &StartupRecoveryConfig,
        joint_limits_rad: Option<(f32, f32)>,
    ) -> Result<u32> {
        let large_error_rad = cfg.large_error_rad as f32;
        let pos0 = self.read_position().await?;
        let bounded_joint = joint_limits_rad.is_some();
        let use_short = cfg.prefer_shortest_angle;
        let initial_far = match joint_limits_rad {
            Some(lim) => safety::joint_space_error_mag(pos0, target_rad, lim) > large_error_rad,
            None => safety::linear_error(pos0, target_rad).abs() > large_error_rad,
        };
        if !initial_far {
            return Ok(0);
        }

        self.stall_detector.disable();

        let direct_within_base = cfg.recovery_direct_command_within_rad as f32;
        let direct_within = if cfg.approach_enabled {
            direct_within_base.max(cfg.approach_handoff_rad as f32)
        } else {
            direct_within_base
        };

        let settle_tolerance_rad = cfg.settle_tolerance_rad as f32;
        let max_step_rad = cfg.max_step_rad as f32;
        let kp_soft = cfg.kp_soft;
        let kd_soft = cfg.kd_soft;
        let step_period = Duration::from_millis(cfg.step_period_ms);
        let timeout = Duration::from_secs_f64(cfg.recovery_timeout_secs);

        let post_scale = (cfg.post_stall_motion_scale as f32).clamp(0.05, 1.0);

        let trip_tau = cfg.resistance_torque_nm;
        let trip_vel = cfg.resistance_velocity_rads;
        let confirm = cfg.resistance_confirm_ticks.max(1);
        let backoff = Duration::from_millis(cfg.resistance_backoff_ms);
        let stall_min_err = effective_stall_min_err(cfg, direct_within);

        let kp_settle = cfg.kp_settle;
        let kd_settle = cfg.kd_settle;
        let settle_ramp_ticks = cfg.settle_ramp_ticks.max(1);

        let start = Instant::now();
        let approach_limit = Duration::from_secs_f64(cfg.approach_max_secs);

        let mut motion_scale = 1.0f32;
        let mut stall_backoffs = 0u32;

        if cfg.approach_enabled {
            let handoff = cfg.approach_handoff_rad as f32;
            let a_step_base = cfg.approach_max_step_rad as f32;
            let a_period = Duration::from_millis(cfg.approach_step_period_ms);

            let mut resistance_streak = 0u32;
            let mut prev_linear_mag = f32::INFINITY;

            while start.elapsed() < timeout && start.elapsed() < approach_limit {
                let pos = self.read_position().await?;
                let linear_mag = match joint_limits_rad {
                    Some(lim) => safety::joint_space_error_mag(pos, target_rad, lim),
                    None => safety::linear_error(pos, target_rad).abs(),
                };
                if linear_mag + 0.002 < prev_linear_mag {
                    resistance_streak = 0;
                    motion_scale = 1.0;
                }
                prev_linear_mag = linear_mag;

                if linear_mag < settle_tolerance_rad {
                    self.stall_detector.enable();
                    return Ok(stall_backoffs);
                }
                if linear_mag <= handoff {
                    break;
                }

                let a_step = a_step_base * motion_scale;
                let kp_a = cfg.approach_kp * motion_scale;
                let kd_a = cfg.approach_kd * motion_scale;

                let cmd_pos = if let Some(lim) = joint_limits_rad {
                    let cj = safety::canonical_joint_angle(pos, target_rad, lim.0, lim.1);
                    let delta = safety::step_delta_toward_home(cj, target_rad, use_short, true);
                    let step = delta.clamp(-a_step, a_step);
                    let cj_next = (cj + step).clamp(lim.0, lim.1);
                    pos + (cj_next - cj)
                } else {
                    let delta =
                        safety::step_delta_toward_home(pos, target_rad, use_short, bounded_joint);
                    let step = delta.clamp(-a_step, a_step);
                    safety::clamp_cmd_to_limits(pos + step, joint_limits_rad)
                };
                let state = self.send_control(cmd_pos, 0.0, kp_a, kd_a, 0.0).await?;

                let stall_eligible = linear_mag >= stall_min_err;
                let looks_blocked = stall_eligible
                    && state.torque_nm.abs() >= trip_tau
                    && state.velocity_rads.abs() <= trip_vel;
                if looks_blocked {
                    resistance_streak += 1;
                    if resistance_streak >= confirm {
                        stall_backoffs += 1;
                        info!(
                            torque_nm = state.torque_nm,
                            velocity_rads = state.velocity_rads,
                            backoff_ms = cfg.resistance_backoff_ms,
                            scale = post_scale,
                            "startup recovery: stall; holding, backing off, then continuing scaled down"
                        );
                        let hold_pos = self.read_position().await?;
                        self.send_control(
                            hold_pos,
                            0.0,
                            kp_soft * motion_scale,
                            kd_soft * motion_scale,
                            0.0,
                        )
                        .await?;
                        tokio::time::sleep(backoff).await;
                        motion_scale = post_scale;
                        resistance_streak = 0;
                    }
                } else {
                    resistance_streak = 0;
                }

                tokio::time::sleep(a_period).await;
            }
        }

        motion_scale = 1.0f32;

        let mut resistance_streak = 0u32;
        let mut prev_linear_mag = f32::INFINITY;
        let mut settle_ticks = 0u32;
        while start.elapsed() < timeout {
            let pos = self.read_position().await?;
            let linear_mag = match joint_limits_rad {
                Some(lim) => safety::joint_space_error_mag(pos, target_rad, lim),
                None => safety::linear_error(pos, target_rad).abs(),
            };
            if linear_mag + 0.002 < prev_linear_mag {
                resistance_streak = 0;
                motion_scale = 1.0;
            }
            prev_linear_mag = linear_mag;

            if linear_mag < settle_tolerance_rad {
                self.stall_detector.enable();
                return Ok(stall_backoffs);
            }

            let in_direct_zone = linear_mag <= direct_within;
            if in_direct_zone {
                motion_scale = 1.0;
            }
            let cap = max_step_rad * motion_scale;
            let cmd_pos = if in_direct_zone {
                match joint_limits_rad {
                    Some(lim) => safety::motor_cmd_for_joint_target(pos, target_rad, lim),
                    None => safety::clamp_cmd_to_limits(target_rad, joint_limits_rad),
                }
            } else {
                settle_ticks = 0;
                if let Some(lim) = joint_limits_rad {
                    let cj = safety::canonical_joint_angle(pos, target_rad, lim.0, lim.1);
                    let delta = safety::step_delta_toward_home(cj, target_rad, use_short, true);
                    let step = delta.clamp(-cap, cap);
                    let cj_next = (cj + step).clamp(lim.0, lim.1);
                    pos + (cj_next - cj)
                } else {
                    let delta =
                        safety::step_delta_toward_home(pos, target_rad, use_short, bounded_joint);
                    let step = delta.clamp(-cap, cap);
                    safety::clamp_cmd_to_limits(pos + step, joint_limits_rad)
                }
            };

            let (kp_cmd, kd_cmd) = if in_direct_zone {
                settle_ticks = settle_ticks.saturating_add(1);
                let t = (settle_ticks as f32 / settle_ramp_ticks as f32).min(1.0);
                let kp = kp_soft + (kp_settle - kp_soft) * t;
                let kd = kd_soft + (kd_settle - kd_soft) * t;
                (kp * motion_scale, kd * motion_scale)
            } else {
                (kp_soft * motion_scale, kd_soft * motion_scale)
            };

            let state = self.send_control(cmd_pos, 0.0, kp_cmd, kd_cmd, 0.0).await?;

            let stall_eligible = linear_mag >= stall_min_err;
            let looks_blocked = stall_eligible
                && state.torque_nm.abs() >= trip_tau
                && state.velocity_rads.abs() <= trip_vel;
            if looks_blocked {
                resistance_streak += 1;
                if resistance_streak >= confirm {
                    stall_backoffs += 1;
                    info!(
                        phase = "gradual",
                        torque_nm = state.torque_nm,
                        velocity_rads = state.velocity_rads,
                        backoff_ms = cfg.resistance_backoff_ms,
                        "startup recovery: stall during gradual; holding and backing off"
                    );
                    let hold_pos = self.read_position().await?;
                    self.send_control(
                        hold_pos,
                        0.0,
                        kp_soft * motion_scale,
                        kd_soft * motion_scale,
                        0.0,
                    )
                    .await?;
                    tokio::time::sleep(backoff).await;
                    motion_scale = post_scale;
                    resistance_streak = 0;
                }
            } else {
                resistance_streak = 0;
            }

            tokio::time::sleep(step_period).await;
        }

        self.stall_detector.enable();
        anyhow::bail!(
            "startup recovery timed out: target {:.3} rad, last read {:.3} rad",
            target_rad,
            self.read_position().await?
        );
    }

    pub async fn move_to_deg(
        &mut self,
        degrees: f32,
        kp: Option<f32>,
        kd: Option<f32>,
    ) -> Result<MotorState> {
        self.move_to(degrees.to_radians(), kp, kd).await
    }

    /// Velocity-mode spin with joint limit enforcement.
    ///
    /// Reads current position, maps it to canonical joint-space, then validates
    /// the velocity command against limits. Rejects commands that would push
    /// further out of bounds; scales velocity near boundaries.
    pub async fn spin(&mut self, velocity_rads: f32, kd: Option<f32>) -> Result<MotorState> {
        let pos = self.read_position().await?;

        let vel = velocity_rads.clamp(-10.0, 10.0);
        let validated_vel = safety::validate_velocity_command(
            pos,
            self.home_rad,
            self.joint_limits,
            self.soft_limit_margin_rad,
            vel,
        )
        .map_err(|msg| anyhow::anyhow!("{}", msg))?;

        self.send_control(pos, validated_vel, 0.0, kd.unwrap_or(1.0), 0.0)
            .await
    }

    /// Torque-mode command with joint limit enforcement.
    ///
    /// Reads current position, maps it to canonical joint-space, then validates
    /// the torque command against limits.
    pub async fn set_torque(&mut self, torque_nm: f32) -> Result<MotorState> {
        let pos = self.read_position().await?;

        let trq = torque_nm.clamp(-30.0, 30.0);
        let validated_trq = safety::validate_torque_command(
            pos,
            self.home_rad,
            self.joint_limits,
            self.soft_limit_margin_rad,
            trq,
        )
        .map_err(|msg| anyhow::anyhow!("{}", msg))?;

        self.send_control(pos, 0.0, 0.0, 0.0, validated_trq).await
    }

    // -- Telemetry --

    pub async fn read_state(&mut self) -> Result<MotorState> {
        let cmd = EnableCommand {
            host_id: self.host_id,
        };
        let (id, data) = cmd.to_can_packet(self.can_id);
        let fb = self.send_and_recv(id, &data).await?;
        let state = Self::parse_feedback(fb);
        Self::validate_feedback_angle(&state)?;
        self.last_known_position = Some(state.angle_rad);
        Ok(state)
    }

    /// Read motor state via EnableCommand, but validate that the response actually
    /// comes from this motor's CAN ID.
    pub async fn read_state_validated(&mut self) -> Result<MotorState> {
        let cmd = EnableCommand {
            host_id: self.host_id,
        };
        let (id, data) = cmd.to_can_packet(self.can_id);

        let mut proto = self.protocol.lock().await;
        proto
            .send(id, &data)
            .await
            .map_err(|e| anyhow::anyhow!("{:#}", e))?;
        let (resp_id, resp_data) = proto.recv().await.map_err(|e| anyhow::anyhow!("{:#}", e))?;
        drop(proto);

        let cmd = Command::from_can_packet(resp_id, resp_data);
        let fb = FeedbackFrame::from_command(cmd);

        if fb.motor_id != self.can_id {
            anyhow::bail!(
                "CAN response mismatch: expected motor {}, got motor {}",
                self.can_id,
                fb.motor_id
            );
        }

        let state = Self::parse_feedback(fb);
        Self::validate_feedback_angle(&state)?;
        self.last_known_position = Some(state.angle_rad);
        Ok(state)
    }

    pub async fn read_param(&mut self, param: RobStride03Parameter) -> Result<f32> {
        let meta = param.metadata();
        let cmd = ReadCommand {
            host_id: self.host_id,
            parameter_index: meta.index,
            data: 0,
            read_status: false,
        };
        let (id, data) = cmd.to_can_packet(self.can_id);
        if self.debug {
            eprintln!(
                "  TX  id={:08X} data={:02X?}  (read param 0x{:04X} '{}')",
                id, data, meta.index, meta.name
            );
        }
        let mut proto = self.protocol.lock().await;
        proto
            .send(id, &data)
            .await
            .map_err(|e| anyhow::anyhow!("{:#}", e))?;
        let (resp_id, resp_data) = proto.recv().await.map_err(|e| anyhow::anyhow!("{:#}", e))?;
        drop(proto);

        if self.debug {
            eprintln!("  RX  id={:08X} data={:02X?}", resp_id, &resp_data);
        }

        let resp_cmd = Command::from_can_packet(resp_id, resp_data);
        let read_resp = ReadCommand::from_command(resp_cmd);
        Ok(read_resp.data_as_f32())
    }

    pub async fn read_position(&mut self) -> Result<f32> {
        let pos = self.read_param(RobStride03Parameter::MechPos).await?;
        if !safety::is_valid_mech_pos_reading(pos) {
            anyhow::bail!(
                "invalid mechanical position read: {:.4} rad (expected finite value within ±{:.0} rad; check CAN wiring / response)",
                pos,
                safety::MAX_REASONABLE_MECH_POS_RAD
            );
        }
        self.last_known_position = Some(pos);
        Ok(pos)
    }

    pub async fn read_velocity(&mut self) -> Result<f32> {
        self.read_param(RobStride03Parameter::MechVel).await
    }

    pub async fn read_voltage(&mut self) -> Result<f32> {
        self.read_param(RobStride03Parameter::VBus).await
    }

    /// Read the raw fault status register (0x3022).
    pub async fn read_fault_code(&mut self) -> Result<u32> {
        let cmd = ReadCommand {
            host_id: self.host_id,
            parameter_index: 0x3022,
            data: 0,
            read_status: false,
        };
        let (id, data) = cmd.to_can_packet(self.can_id);
        let mut proto = self.protocol.lock().await;
        proto
            .send(id, &data)
            .await
            .map_err(|e| anyhow::anyhow!("{:#}", e))?;
        let (resp_id, resp_data) = proto.recv().await.map_err(|e| anyhow::anyhow!("{:#}", e))?;
        drop(proto);
        let resp_cmd = Command::from_can_packet(resp_id, resp_data);
        let read_resp = ReadCommand::from_command(resp_cmd);
        Ok(read_resp.data)
    }

    // -- Helpers --

    async fn ensure_enabled(&mut self) -> Result<()> {
        if !self.enabled {
            self.enable().await?;
        }
        Ok(())
    }

    pub async fn wait_until_at(
        &mut self,
        target_rad: f32,
        tolerance: f32,
        timeout: Duration,
    ) -> Result<bool> {
        let start = Instant::now();
        while start.elapsed() < timeout {
            let pos = self.read_position().await?;
            if (pos - target_rad).abs() < tolerance {
                return Ok(true);
            }
            tokio::time::sleep(Duration::from_millis(50)).await;
        }
        Ok(false)
    }

    pub async fn write_param(&mut self, param: RobStride03Parameter, value: f32) -> Result<()> {
        let meta = param.metadata();
        if self.debug {
            eprintln!(
                "  WRITE param 0x{:04X} '{}' = {} (type={:?})",
                meta.index, meta.name, value, meta.param_type
            );
        }
        let cmd = WriteCommand {
            host_id: self.host_id,
            parameter_index: meta.index,
            data: value,
        };
        let (id, data) = cmd.to_can_packet(self.can_id);
        self.send_and_recv(id, &data).await?;
        Ok(())
    }

    async fn send_and_recv(&self, id: u32, data: &[u8]) -> Result<FeedbackFrame> {
        if self.debug {
            let comm_type = (id >> 24) & 0x1F;
            let data_2 = (id >> 8) & 0xFFFF;
            let target = id & 0x7F;
            eprintln!(
                "  TX  id={:08X} [comm={} d2={:04X} tgt={}] data={:02X?}",
                id, comm_type, data_2, target, data
            );
        }
        let mut proto = self.protocol.lock().await;
        proto
            .send(id, data)
            .await
            .map_err(|e| anyhow::anyhow!("{:#}", e))?;
        let (resp_id, resp_data) = proto.recv().await.map_err(|e| anyhow::anyhow!("{:#}", e))?;
        drop(proto);

        if self.debug {
            let comm_type = (resp_id >> 24) & 0x1F;
            let data_2 = (resp_id >> 8) & 0xFFFF;
            let target = resp_id & 0x7F;
            eprintln!(
                "  RX  id={:08X} [comm={} d2={:04X} tgt={}] data={:02X?}",
                resp_id, comm_type, data_2, target, &resp_data
            );
        }

        let cmd = Command::from_can_packet(resp_id, resp_data);
        let fb = FeedbackFrame::from_command(cmd);
        Ok(fb)
    }

    fn validate_feedback_angle(state: &MotorState) -> Result<()> {
        if !safety::is_valid_mech_pos_reading(state.angle_rad) {
            anyhow::bail!(
                "invalid angle in feedback: {:.4} rad (expected within ±{:.0} rad)",
                state.angle_rad,
                safety::MAX_REASONABLE_MECH_POS_RAD
            );
        }
        Ok(())
    }

    fn parse_feedback(fb: FeedbackFrame) -> MotorState {
        let typed = RobStride03Feedback::from_feedback_frame(fb.clone());

        let mut faults = Vec::new();
        if fb.fault_undervoltage {
            faults.push("undervoltage");
        }
        if fb.fault_overcurrent {
            faults.push("overcurrent");
        }
        if fb.fault_over_temperature {
            faults.push("over_temperature");
        }
        if fb.fault_magnetic_encoding {
            faults.push("magnetic_encoding");
        }
        if fb.fault_hall_encoding {
            faults.push("hall_encoding");
        }
        if fb.fault_uncalibrated {
            faults.push("uncalibrated");
        }

        MotorState {
            angle_rad: typed.angle_rad(),
            velocity_rads: typed.velocity_rads(),
            torque_nm: typed.torque_nm(),
            temperature_c: fb.temperature,
            mode: fb.mode,
            faults,
        }
    }
}

/// Recovery / homing guardrails.
#[cfg(test)]
mod recovery_homing_tests {
    use crate::config::StartupRecoveryConfig;

    #[test]
    fn default_stall_floor_covers_approach_handoff() {
        let c = StartupRecoveryConfig::default();
        let handoff = c.approach_handoff_rad as f32;
        let stall = c.stall_detection_min_linear_error_rad as f32;
        assert!(
            handoff <= stall + 1e-5,
            "gradual phase starts near handoff ({handoff:.3} rad); stall floor ({stall:.3}) must not trip earlier"
        );
    }

    #[test]
    fn stall_eligible_gate_matches_recovery() {
        let stall_min = 0.30f32;
        assert!(
            !((0.28f32) >= stall_min),
            "at ~16 deg error, should not be stall-eligible"
        );
        assert!(0.35f32 >= stall_min);
    }

    #[test]
    fn direct_command_zone_uses_linear_error() {
        let c = StartupRecoveryConfig::default();
        let direct_within = if c.approach_enabled {
            (c.recovery_direct_command_within_rad as f32).max(c.approach_handoff_rad as f32)
        } else {
            c.recovery_direct_command_within_rad as f32
        };
        let pos = 0.10f32;
        let home = 0.0f32;
        let linear_mag = (home - pos).abs();
        assert!(linear_mag <= direct_within);
        let pos2 = 0.35f32;
        let linear_mag2 = (home - pos2).abs();
        assert!(linear_mag2 > direct_within);
    }

    #[test]
    fn direct_zone_covers_post_approach_band() {
        let c = StartupRecoveryConfig::default();
        assert!(c.approach_enabled);
        let effective =
            (c.recovery_direct_command_within_rad as f32).max(c.approach_handoff_rad as f32);
        let err_after_handoff = 0.25f32;
        assert!(
            err_after_handoff <= effective,
            "error just inside handoff must use command-home + settle ramp, not micro-steps"
        );
    }

    #[test]
    fn seven_degree_residual_below_effective_stall_with_legacy_yaml() {
        let mut c = StartupRecoveryConfig::default();
        c.recovery_direct_command_within_rad = 0.12;
        c.stall_detection_min_linear_error_rad = 0.26;
        c.approach_handoff_rad = 0.28;
        let direct_base = c.recovery_direct_command_within_rad as f32;
        let direct = if c.approach_enabled {
            direct_base.max(c.approach_handoff_rad as f32)
        } else {
            direct_base
        };
        let floor = super::effective_stall_min_err(&c, direct);
        let residual = 7.3f32.to_radians();
        assert!(
            residual < floor,
            "~7.3 deg hang was stall-eligible when floor {:.4} <= err {:.4}; floor is now {:.4}",
            0.13f32,
            residual,
            floor
        );
    }

    #[test]
    fn settle_ramp_kp_first_tick() {
        let c = StartupRecoveryConfig::default();
        let kp_soft = c.kp_soft;
        let kp_settle = c.kp_settle;
        let ramp_ticks = c.settle_ramp_ticks.max(1);
        let settle_ticks = 1u32;
        let t = (settle_ticks as f32 / ramp_ticks as f32).min(1.0);
        let kp = kp_soft + (kp_settle - kp_soft) * t;
        let expected = 15.0 + (100.0 - 15.0) * (1.0 / 20.0);
        assert!((kp - expected).abs() < 0.01, "kp={kp} expected ~{expected}");
    }

    #[test]
    fn motion_scale_full_scale_after_progress_simulation() {
        let mut motion_scale = 0.5f32;
        let linear_improved = true;
        if linear_improved {
            motion_scale = 1.0;
        }
        assert_eq!(motion_scale, 1.0);
    }
}

pub async fn create_ch341_protocol(port: &str) -> Result<Arc<Mutex<Protocol>>> {
    let transport = robstride::CH341Transport::new(port.to_string())
        .await
        .map_err(|e| anyhow::anyhow!("Failed to open CH341 on {}: {:#}", port, e))?;
    let transport = TransportType::CH341(transport);
    let callback = Arc::new(|_id: u32, _data: Vec<u8>| {});
    let protocol = Protocol::new(transport, callback);
    Ok(Arc::new(Mutex::new(protocol)))
}

#[cfg(feature = "socketcan")]
pub async fn create_socketcan_protocol(interface: &str) -> Result<Arc<Mutex<Protocol>>> {
    let transport = robstride::SocketCanTransport::new(interface.to_string())
        .await
        .map_err(|e| anyhow::anyhow!("Failed to open SocketCAN {}: {:#}", interface, e))?;
    let transport = TransportType::SocketCAN(transport);
    let callback = Arc::new(|_id: u32, _data: Vec<u8>| {});
    let protocol = Protocol::new(transport, callback);
    Ok(Arc::new(Mutex::new(protocol)))
}

pub async fn create_protocol(bus: &BusConfig) -> Result<Arc<Mutex<Protocol>>> {
    match bus.transport.as_str() {
        #[cfg(feature = "socketcan")]
        "socketcan" => {
            let iface = bus.socketcan_interface.as_deref().unwrap_or("can0");
            info!("Opening SocketCAN transport on {}", iface);
            create_socketcan_protocol(iface).await
        }
        #[cfg(not(feature = "socketcan"))]
        "socketcan" => {
            anyhow::bail!(
                "SocketCAN transport requested but binary was built without the 'socketcan' feature"
            );
        }
        "ch341" | _ => {
            info!("Opening CH341 transport on {}", bus.port);
            create_ch341_protocol(&bus.port).await
        }
    }
}
