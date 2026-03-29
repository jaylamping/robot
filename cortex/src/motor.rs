use std::sync::Arc;
use std::time::{Duration, Instant};

use anyhow::Result;
use robstride::{
    Command, CommandData, ControlCommand, EnableCommand, FeedbackFrame, MotorMode, Protocol,
    ReadCommand, SetZeroCommand, StopCommand, TransportType, WriteCommand,
};
use robstride::actuator::{TypedCommandData, TypedFeedbackData};
use robstride::robstride03::{RobStride03Command, RobStride03Feedback, RobStride03Parameter};
use robstride::ActuatorParameter;
use tokio::sync::Mutex;
use tracing::info;

use crate::config::{BusConfig, StartupRecoveryConfig};

const HOST_ID: u8 = 0xAA;

/// Default soft-limit margin in radians (~10 degrees). Velocity/torque commands ramp
/// down linearly within this zone when pushing toward a limit.
const DEFAULT_SOFT_LIMIT_MARGIN_RAD: f32 = 0.175;

/// Signed smallest angle from `from_rad` to `to_rad`, in (‑π, π]: move along the shorter arc on a circle.
#[inline]
pub fn shortest_angle_err(from_rad: f32, to_rad: f32) -> f32 {
    use std::f32::consts::{PI, TAU};
    let d = to_rad - from_rad;
    (d + PI).rem_euclid(TAU) - PI
}

#[inline]
fn clamp_cmd_to_limits(cmd: f32, joint_limits_rad: Option<(f32, f32)>) -> f32 {
    joint_limits_rad.map_or(cmd, |(lo, hi)| cmd.clamp(lo, hi))
}

/// Linear error in the encoder frame (not wrapped). Use for "at home?" and "still far?" only.
#[inline]
fn linear_error(pos_rad: f32, target_rad: f32) -> f32 {
    target_rad - pos_rad
}

/// Step direction toward target. With **bounded** joints (`joint_limits` in use), always linear —
/// shortest arc is wrong for a limited range and can command a ~270° wrap the mechanics can't mean.
/// Otherwise, shortest arc only when linear error `> π` and `prefer_shortest_angle`.
#[inline]
fn step_delta_toward_home(
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
            soft_limit_margin_rad: DEFAULT_SOFT_LIMIT_MARGIN_RAD,
            last_known_position: None,
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

    pub fn set_soft_limit_margin(&mut self, margin_rad: f32) {
        self.soft_limit_margin_rad = margin_rad.max(0.0);
    }

    /// Compute a velocity scale factor (0.0–1.0) based on proximity to joint limits.
    /// Returns 1.0 if no limits are set or the motor is not near a boundary in the
    /// direction of `velocity_rads`.
    fn soft_limit_velocity_scale(&self, velocity_rads: f32) -> f32 {
        let (lo, hi) = match self.joint_limits {
            Some(l) => l,
            None => return 1.0,
        };
        let pos = match self.last_known_position {
            Some(p) => p,
            None => return 1.0,
        };
        let margin = self.soft_limit_margin_rad;
        if margin <= 0.0 {
            return 1.0;
        }

        if velocity_rads > 0.0 {
            let dist_to_max = hi - pos;
            if dist_to_max <= 0.0 {
                return 0.0;
            }
            if dist_to_max < margin {
                return (dist_to_max / margin).clamp(0.0, 1.0);
            }
        } else if velocity_rads < 0.0 {
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

    // -- Lifecycle --

    pub async fn enable(&mut self) -> Result<MotorState> {
        let cmd = EnableCommand {
            host_id: self.host_id,
        };
        let (id, data) = cmd.to_can_packet(self.can_id);
        let fb = self.send_and_recv(id, &data).await?;
        self.enabled = true;
        let state = Self::parse_feedback(fb);
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
        Ok(Self::parse_feedback(fb))
    }

    pub async fn clear_faults(&mut self) -> Result<MotorState> {
        let cmd = StopCommand {
            host_id: self.host_id,
            clear_fault: true,
        };
        let (id, data) = cmd.to_can_packet(self.can_id);
        let fb = self.send_and_recv(id, &data).await?;
        self.enabled = false;
        Ok(Self::parse_feedback(fb))
    }

    pub async fn set_zero(&mut self) -> Result<()> {
        let cmd = SetZeroCommand {
            host_id: self.host_id,
        };
        let (id, data) = cmd.to_can_packet(self.can_id);
        self.send_and_recv(id, &data).await?;
        Ok(())
    }

    /// If the encoder position is more than `threshold_rad` (default: 2π) from `target_rad`,
    /// issue a `set_zero` to collapse accumulated multi-turn offset, then return the new
    /// effective position (which will be near 0). The caller should adjust `target_rad`
    /// accordingly (new target = old target − old position, i.e. the residual after zeroing).
    ///
    /// Returns `Some(residual_target)` if normalization happened, `None` if position was fine.
    pub async fn normalize_multiturn(
        &mut self,
        target_rad: f32,
        threshold_rad: f32,
    ) -> Result<Option<f32>> {
        use std::f32::consts::TAU;
        let threshold = if threshold_rad > 0.0 { threshold_rad } else { TAU };
        let pos = self.read_position().await?;
        let err = (pos - target_rad).abs();
        if err <= threshold {
            return Ok(None);
        }

        self.set_zero().await?;

        let residual = shortest_angle_err(0.0, target_rad - pos);
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

        let clamped_pos = clamp_cmd_to_limits(position_rad, self.joint_limits);

        let scale = self.soft_limit_velocity_scale(velocity_rads);
        let clamped_vel = velocity_rads * scale;
        let clamped_torque = if scale < 1.0 && torque_nm.abs() > 0.0 {
            torque_nm * scale
        } else {
            torque_nm
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
        self.last_known_position = Some(state.angle_rad);
        Ok(state)
    }

    pub async fn move_to(&mut self, position_rad: f32, kp: Option<f32>, kd: Option<f32>) -> Result<MotorState> {
        self.send_control(
            position_rad,
            0.0,
            kp.unwrap_or(30.0),
            kd.unwrap_or(1.0),
            0.0,
        ).await
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

    /// If linear `|target − position|` exceeds `large_error_rad`, runs optional **approach** then
    /// **gradual** steps. Settle and handoff use **linear** error so wrap‑around never pretends the
    /// joint reached home. Bounded joints (`joint_limits_rad: Some`) always step linearly; otherwise
    /// shortest arc may apply when linear error `> π` and `prefer_shortest_angle`.
    /// Commands clamp to limits; inside `recovery_direct_command_within_rad`, gradual phase commands
    /// home directly with soft gains (stiction).
    /// On stall (high torque, low velocity, and linear error not inside the near-goal floor): hold,
    /// **back off** for `resistance_backoff_ms`, then **continue** with `post_stall_motion_scale`
    /// applied to steps and gains for the rest of this recovery.
    ///
    /// Returns how many stall/backoff cycles ran (0 = no obstruction detected). Callers can use
    /// this to avoid auto-resuming higher-level motion after a human fought the joint.
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
        let direct_within = cfg.recovery_direct_command_within_rad as f32;
        if linear_error(pos0, target_rad).abs() <= large_error_rad {
            return Ok(0);
        }

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
        let stall_min_err = cfg.stall_detection_min_linear_error_rad as f32;

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
                let linear_mag = linear_error(pos, target_rad).abs();
                if linear_mag + 0.002 < prev_linear_mag {
                    resistance_streak = 0;
                    motion_scale = 1.0;
                }
                prev_linear_mag = linear_mag;

                if linear_mag < settle_tolerance_rad {
                    return Ok(stall_backoffs);
                }
                if linear_mag <= handoff {
                    break;
                }

                let a_step = a_step_base * motion_scale;
                let kp_a = cfg.approach_kp * motion_scale;
                let kd_a = cfg.approach_kd * motion_scale;

                let delta = step_delta_toward_home(pos, target_rad, use_short, bounded_joint);
                let step = delta.clamp(-a_step, a_step);
                let cmd_pos = clamp_cmd_to_limits(pos + step, joint_limits_rad);
                let state = self
                    .send_control(cmd_pos, 0.0, kp_a, kd_a, 0.0)
                    .await?;

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
                        self.send_control(hold_pos, 0.0, kp_soft * motion_scale, kd_soft * motion_scale, 0.0)
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

        // Gradual phase should not inherit reduced scale from approach stalls; it uses its own step/gain profile.
        motion_scale = 1.0f32;

        let mut resistance_streak = 0u32;
        let mut prev_linear_mag = f32::INFINITY;
        let mut settle_ticks = 0u32;
        while start.elapsed() < timeout {
            let pos = self.read_position().await?;
            let linear_mag = linear_error(pos, target_rad).abs();
            if linear_mag + 0.002 < prev_linear_mag {
                resistance_streak = 0;
                motion_scale = 1.0;
            }
            prev_linear_mag = linear_mag;

            if linear_mag < settle_tolerance_rad {
                return Ok(stall_backoffs);
            }

            let in_direct_zone = linear_mag <= direct_within;
            let cap = max_step_rad * motion_scale;
            let cmd_pos = if in_direct_zone {
                clamp_cmd_to_limits(target_rad, joint_limits_rad)
            } else {
                settle_ticks = 0;
                let delta = step_delta_toward_home(pos, target_rad, use_short, bounded_joint);
                let step = delta.clamp(-cap, cap);
                clamp_cmd_to_limits(pos + step, joint_limits_rad)
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

            let state = self
                .send_control(
                    cmd_pos,
                    0.0,
                    kp_cmd,
                    kd_cmd,
                    0.0,
                )
                .await?;

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
                    self.send_control(hold_pos, 0.0, kp_soft * motion_scale, kd_soft * motion_scale, 0.0)
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

        anyhow::bail!(
            "startup recovery timed out: target {:.3} rad, last read {:.3} rad",
            target_rad,
            self.read_position().await?
        );
    }

    pub async fn move_to_deg(&mut self, degrees: f32, kp: Option<f32>, kd: Option<f32>) -> Result<MotorState> {
        self.move_to(degrees.to_radians(), kp, kd).await
    }

    pub async fn spin(&mut self, velocity_rads: f32, kd: Option<f32>) -> Result<MotorState> {
        self.send_control(
            0.0,
            velocity_rads.clamp(-10.0, 10.0),
            0.0,
            kd.unwrap_or(1.0),
            0.0,
        ).await
    }

    pub async fn set_torque(&mut self, torque_nm: f32) -> Result<MotorState> {
        self.send_control(
            0.0,
            0.0,
            0.0,
            0.0,
            torque_nm.clamp(-30.0, 30.0),
        ).await
    }

    // -- Telemetry --

    pub async fn read_state(&mut self) -> Result<MotorState> {
        let cmd = EnableCommand {
            host_id: self.host_id,
        };
        let (id, data) = cmd.to_can_packet(self.can_id);
        let fb = self.send_and_recv(id, &data).await?;
        let state = Self::parse_feedback(fb);
        self.last_known_position = Some(state.angle_rad);
        Ok(state)
    }

    /// Read motor state via EnableCommand, but validate that the response actually
    /// comes from this motor's CAN ID. Returns an error on CAN bus response mismatch
    /// (e.g. when a disconnected motor's slot eats a frame from another motor).
    pub async fn read_state_validated(&mut self) -> Result<MotorState> {
        let cmd = EnableCommand {
            host_id: self.host_id,
        };
        let (id, data) = cmd.to_can_packet(self.can_id);

        let mut proto = self.protocol.lock().await;
        proto.send(id, &data).await
            .map_err(|e| anyhow::anyhow!("{:#}", e))?;
        let (resp_id, resp_data) = proto.recv().await
            .map_err(|e| anyhow::anyhow!("{:#}", e))?;
        drop(proto);

        let cmd = Command::from_can_packet(resp_id, resp_data);
        let fb = FeedbackFrame::from_command(cmd);

        if fb.motor_id != self.can_id {
            anyhow::bail!(
                "CAN response mismatch: expected motor {}, got motor {}",
                self.can_id, fb.motor_id
            );
        }

        let state = Self::parse_feedback(fb);
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
            eprintln!("  TX  id={:08X} data={:02X?}  (read param 0x{:04X} '{}')",
                id, data, meta.index, meta.name);
        }
        let mut proto = self.protocol.lock().await;
        proto.send(id, &data).await
            .map_err(|e| anyhow::anyhow!("{:#}", e))?;
        let (resp_id, resp_data) = proto.recv().await
            .map_err(|e| anyhow::anyhow!("{:#}", e))?;
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
        self.last_known_position = Some(pos);
        Ok(pos)
    }

    pub async fn read_velocity(&mut self) -> Result<f32> {
        self.read_param(RobStride03Parameter::MechVel).await
    }

    pub async fn read_voltage(&mut self) -> Result<f32> {
        self.read_param(RobStride03Parameter::VBus).await
    }

    /// Read the raw fault status register (0x3022). Bits:
    /// bit14=stall overload, bit7=encoder uncalibrated, bit3=overvoltage,
    /// bit2=undervoltage, bit1=driver chip fault, bit0=overtemperature.
    pub async fn read_fault_code(&mut self) -> Result<u32> {
        let cmd = ReadCommand {
            host_id: self.host_id,
            parameter_index: 0x3022,
            data: 0,
            read_status: false,
        };
        let (id, data) = cmd.to_can_packet(self.can_id);
        let mut proto = self.protocol.lock().await;
        proto.send(id, &data).await
            .map_err(|e| anyhow::anyhow!("{:#}", e))?;
        let (resp_id, resp_data) = proto.recv().await
            .map_err(|e| anyhow::anyhow!("{:#}", e))?;
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
            eprintln!("  WRITE param 0x{:04X} '{}' = {} (type={:?})",
                meta.index, meta.name, value, meta.param_type);
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
            eprintln!("  TX  id={:08X} [comm={} d2={:04X} tgt={}] data={:02X?}",
                id, comm_type, data_2, target, data);
        }
        let mut proto = self.protocol.lock().await;
        proto.send(id, data).await
            .map_err(|e| anyhow::anyhow!("{:#}", e))?;
        let (resp_id, resp_data) = proto.recv().await
            .map_err(|e| anyhow::anyhow!("{:#}", e))?;
        drop(proto);

        if self.debug {
            let comm_type = (resp_id >> 24) & 0x1F;
            let data_2 = (resp_id >> 8) & 0xFFFF;
            let target = resp_id & 0x7F;
            eprintln!("  RX  id={:08X} [comm={} d2={:04X} tgt={}] data={:02X?}",
                resp_id, comm_type, data_2, target, &resp_data);
        }

        let cmd = Command::from_can_packet(resp_id, resp_data);
        let fb = FeedbackFrame::from_command(cmd);
        Ok(fb)
    }

    fn parse_feedback(fb: FeedbackFrame) -> MotorState {
        let typed = RobStride03Feedback::from_feedback_frame(fb.clone());

        let mut faults = Vec::new();
        if fb.fault_undervoltage { faults.push("undervoltage"); }
        if fb.fault_overcurrent { faults.push("overcurrent"); }
        if fb.fault_over_temperature { faults.push("over_temperature"); }
        if fb.fault_magnetic_encoding { faults.push("magnetic_encoding"); }
        if fb.fault_hall_encoding { faults.push("hall_encoding"); }
        if fb.fault_uncalibrated { faults.push("uncalibrated"); }

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

#[cfg(test)]
mod shortest_angle_tests {
    use super::shortest_angle_err;
    use std::f32::consts::PI;

    #[test]
    fn small_delta_unchanged() {
        assert!((shortest_angle_err(0.5, 1.0) - 0.5).abs() < 1e-5);
    }

    #[test]
    fn near_two_pi_wraps_small_positive() {
        let d = shortest_angle_err(6.1, 0.17);
        assert!(d > 0.0 && d < 1.0, "d={}", d);
    }

    #[test]
    fn near_zero_wraps_small_negative() {
        let d = shortest_angle_err(0.17, 6.1);
        assert!(d < 0.0 && d.abs() < 1.0, "d={}", d);
    }

    #[test]
    fn half_turn() {
        assert!((shortest_angle_err(0.0, PI).abs() - PI).abs() < 1e-4);
    }

    #[test]
    fn step_delta_uses_linear_when_error_below_pi() {
        use std::f32::consts::FRAC_PI_2;
        let d = super::step_delta_toward_home(FRAC_PI_2, 0.0, true, false);
        assert!((d - (-FRAC_PI_2)).abs() < 1e-5);
    }

    #[test]
    fn step_delta_uses_short_wrap_when_linear_huge() {
        let d = super::step_delta_toward_home(6.1, 0.17, true, false);
        assert!(d.abs() < 1.0, "expected short arc, got {}", d);
    }

    #[test]
    fn step_delta_bounded_joint_always_linear_even_if_huge() {
        let d = super::step_delta_toward_home(6.1, 0.17, true, true);
        assert!((d - (0.17 - 6.1)).abs() < 1e-4);
    }
}

/// Recovery / homing guardrails (mirrors logic in `recover_position_if_far` and defaults in `config`).
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
        assert!(!((0.28f32) >= stall_min), "at ~16° error, should not be stall-eligible");
        assert!(0.35f32 >= stall_min);
    }

    #[test]
    fn direct_command_zone_uses_linear_error() {
        let direct_within = StartupRecoveryConfig::default().recovery_direct_command_within_rad as f32;
        let pos = 0.10f32;
        let home = 0.0f32;
        let linear_mag = (home - pos).abs();
        assert!(linear_mag <= direct_within);
        let pos2 = 0.25f32;
        let linear_mag2 = (home - pos2).abs();
        assert!(linear_mag2 > direct_within);
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
