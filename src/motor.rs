use std::sync::Arc;
use std::time::{Duration, Instant};

use anyhow::Result;
use robstride::{
    Command, CommandData, EnableCommand, FeedbackFrame, MotorMode, Protocol, ReadCommand,
    SetZeroCommand, StopCommand, TransportType, WriteCommand,
};
use robstride::actuator::TypedFeedbackData;
use robstride::robstride03::{RobStride03Feedback, RobStride03Parameter};
use robstride::ActuatorParameter;
use tokio::sync::Mutex;

const HOST_ID: u8 = 0xAA;

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
}

impl Motor {
    pub fn new(protocol: Arc<Mutex<Protocol>>, can_id: u8) -> Self {
        Self {
            protocol,
            can_id,
            host_id: HOST_ID,
            enabled: false,
            debug: false,
        }
    }

    pub fn with_host_id(mut self, host_id: u8) -> Self {
        self.host_id = host_id;
        self
    }

    // -- Lifecycle --

    pub async fn enable(&mut self) -> Result<MotorState> {
        let cmd = EnableCommand {
            host_id: self.host_id,
        };
        let (id, data) = cmd.to_can_packet(self.can_id);
        let fb = self.send_and_recv(id, &data).await?;
        self.enabled = true;
        Ok(Self::parse_feedback(fb))
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

    pub async fn set_zero(&mut self) -> Result<()> {
        let cmd = SetZeroCommand {
            host_id: self.host_id,
        };
        let (id, data) = cmd.to_can_packet(self.can_id);
        self.send_and_recv(id, &data).await?;
        Ok(())
    }

    // -- Configuration --

    pub async fn set_run_mode(&mut self, mode: u8) -> Result<()> {
        self.write_param(RobStride03Parameter::RunMode, mode as f32).await
    }

    pub async fn set_speed_limit(&mut self, limit_rads: f32) -> Result<()> {
        self.write_param(RobStride03Parameter::LimitSpd, limit_rads.min(20.0)).await
    }

    pub async fn set_torque_limit(&mut self, limit_nm: f32) -> Result<()> {
        self.write_param(RobStride03Parameter::LimitTorque, limit_nm.min(60.0)).await
    }

    pub async fn set_position_gain(&mut self, kp: f32) -> Result<()> {
        self.write_param(RobStride03Parameter::LocKp, kp).await
    }

    pub async fn set_speed_gain(&mut self, kp: f32, ki: f32) -> Result<()> {
        self.write_param(RobStride03Parameter::SpdKp, kp).await?;
        self.write_param(RobStride03Parameter::SpdKi, ki).await
    }

    // -- Motion --

    pub async fn move_to(&mut self, position_rad: f32, speed_limit: Option<f32>) -> Result<()> {
        self.ensure_enabled().await?;
        self.set_run_mode(1).await?;
        if let Some(limit) = speed_limit {
            self.set_speed_limit(limit).await?;
        }
        self.write_param(RobStride03Parameter::Ref, position_rad).await
    }

    pub async fn move_to_deg(&mut self, degrees: f32, speed_limit: Option<f32>) -> Result<()> {
        self.move_to(degrees.to_radians(), speed_limit).await
    }

    pub async fn spin(&mut self, velocity_rads: f32) -> Result<()> {
        self.ensure_enabled().await?;
        self.set_run_mode(2).await?;
        self.write_param(RobStride03Parameter::SpdRef, velocity_rads).await
    }

    pub async fn set_torque(&mut self, torque_nm: f32) -> Result<()> {
        self.ensure_enabled().await?;
        self.set_run_mode(3).await?;
        self.write_param(RobStride03Parameter::IqRef, torque_nm).await
    }

    // -- Telemetry --

    pub async fn read_state(&mut self) -> Result<MotorState> {
        let cmd = EnableCommand {
            host_id: self.host_id,
        };
        let (id, data) = cmd.to_can_packet(self.can_id);
        let fb = self.send_and_recv(id, &data).await?;
        Ok(Self::parse_feedback(fb))
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
        self.read_param(RobStride03Parameter::MechPos).await
    }

    pub async fn read_velocity(&mut self) -> Result<f32> {
        self.read_param(RobStride03Parameter::MechVel).await
    }

    pub async fn read_voltage(&mut self) -> Result<f32> {
        self.read_param(RobStride03Parameter::VBus).await
    }

    // -- Helpers --

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

    async fn ensure_enabled(&mut self) -> Result<()> {
        if !self.enabled {
            self.enable().await?;
        }
        Ok(())
    }

    async fn write_param(&mut self, param: RobStride03Parameter, value: f32) -> Result<()> {
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

pub async fn create_ch341_protocol(port: &str) -> Result<Arc<Mutex<Protocol>>> {
    let transport = robstride::CH341Transport::new(port.to_string())
        .await
        .map_err(|e| anyhow::anyhow!("Failed to open CH341 on {}: {:#}", port, e))?;
    let transport = TransportType::CH341(transport);
    let callback = Arc::new(|_id: u32, _data: Vec<u8>| {});
    let protocol = Protocol::new(transport, callback);
    Ok(Arc::new(Mutex::new(protocol)))
}
