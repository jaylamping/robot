use eyre::Error;
#[cfg(feature = "socketcan")]
use socketcan::async_std::CanSocket;
#[cfg(feature = "socketcan")]
use socketcan::{EmbeddedFrame, ExtendedId};
use std::f32::consts::PI;
use std::sync::Arc;
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::sync::Mutex as TokioMutex;
use tokio_serial::{SerialPortBuilderExt, SerialStream};

use crate::actuator::{normalize_value, Command, CommandData};
use crate::actuator_types::{CommunicationType, FeedbackFrame, MotorMode, ReadCommand};

type SendResult = Result<(), Error>;
type RecvResult = Result<(u32, Vec<u8>), Error>;
type SendFuture<'a> = std::pin::Pin<Box<dyn std::future::Future<Output = SendResult> + Send + 'a>>;
type RecvFuture<'a> = std::pin::Pin<Box<dyn std::future::Future<Output = RecvResult> + Send + 'a>>;

#[derive(Clone)]
pub enum TransportType {
    CH341(CH341Transport),
    #[cfg(feature = "socketcan")]
    SocketCAN(SocketCanTransport),
    /// RS03 loopback for tests: MechPos reads and MIT control converge `pos` toward commanded angle.
    LoopbackSim(LoopbackSimTransport),
    Stub(StubTransport),
}

impl Transport for TransportType {
    fn kind(&self) -> &'static str {
        match self {
            TransportType::CH341(t) => t.kind(),
            #[cfg(feature = "socketcan")]
            TransportType::SocketCAN(t) => t.kind(),
            TransportType::LoopbackSim(t) => t.kind(),
            TransportType::Stub(t) => t.kind(),
        }
    }

    fn port(&self) -> String {
        match self {
            TransportType::CH341(t) => t.port(),
            #[cfg(feature = "socketcan")]
            TransportType::SocketCAN(t) => t.port(),
            TransportType::LoopbackSim(t) => t.port(),
            TransportType::Stub(t) => t.port(),
        }
    }

    fn send<'a>(&'a mut self, id: u32, data: &'a [u8]) -> SendFuture<'a> {
        match self {
            TransportType::CH341(t) => t.send(id, data),
            #[cfg(feature = "socketcan")]
            TransportType::SocketCAN(t) => t.send(id, data),
            TransportType::LoopbackSim(t) => t.send(id, data),
            TransportType::Stub(t) => t.send(id, data),
        }
    }

    fn recv(&mut self) -> RecvFuture<'_> {
        match self {
            TransportType::CH341(t) => t.recv(),
            #[cfg(feature = "socketcan")]
            TransportType::SocketCAN(t) => t.recv(),
            TransportType::LoopbackSim(t) => t.recv(),
            TransportType::Stub(t) => t.recv(),
        }
    }
}

pub trait Transport {
    fn kind(&self) -> &'static str;
    fn port(&self) -> String;
    fn send<'a>(&'a mut self, id: u32, data: &'a [u8]) -> SendFuture<'a>;
    fn recv(&mut self) -> RecvFuture<'_>;
}

pub struct CH341Transport {
    ser: Arc<TokioMutex<SerialStream>>,
    port_name: String,
}

#[cfg(feature = "socketcan")]
pub struct SocketCanTransport {
    socket: Arc<TokioMutex<CanSocket>>,
    interface_name: String,
}

/// Minimal RS03 simulation for automated tests (cortex homing / recovery).
#[derive(Clone)]
pub struct LoopbackSimTransport {
    state: Arc<std::sync::Mutex<LoopbackSimState>>,
    port_name: String,
}

struct LoopbackSimState {
    /// Mechanical angle (rad), same frame as `RobStride03Parameter::MechPos`.
    pos: f32,
    pending: Option<(u32, Vec<u8>)>,
}

const RS03_MECH_POS_INDEX: u16 = 0x7019;
/// RS03 MIT angle limits (matches `robstride03::LIMITS`).
const RS03_ANGLE_MIN: f32 = -4.0 * PI;
const RS03_ANGLE_MAX: f32 = 4.0 * PI;

fn loopback_feedback_packet(pos: f32, can_id: u8) -> (u32, Vec<u8>) {
    let rs_min = -4.0 * PI;
    let rs_max = 4.0 * PI;
    // Keep torque below typical stall trip and velocity above trip threshold so recovery does not false-trigger.
    let fb = FeedbackFrame {
        angle: normalize_value(pos, rs_min, rs_max, -100.0, 100.0),
        velocity: normalize_value(2.0f32, -20.0, 20.0, -100.0, 100.0),
        torque: normalize_value(4.0f32, -60.0, 60.0, -100.0, 100.0),
        temperature: 35.0,
        fault_uncalibrated: false,
        fault_hall_encoding: false,
        fault_magnetic_encoding: false,
        fault_over_temperature: false,
        fault_overcurrent: false,
        fault_undervoltage: false,
        mode: MotorMode::Run,
        motor_id: can_id,
    };
    fb.to_command(can_id).to_can_packet()
}

fn loopback_read_response_packet(can_id: u8, host_id: u8, param_index: u16, value: f32) -> (u32, Vec<u8>) {
    let mut d = [0u8; 8];
    d[0..2].copy_from_slice(&param_index.to_le_bytes());
    d[4..8].copy_from_slice(&value.to_bits().to_le_bytes());
    Command::new(d, can_id, host_id as u16, CommunicationType::Read).to_can_packet()
}

impl LoopbackSimTransport {
    pub fn new(initial_pos_rad: f32) -> Self {
        Self {
            state: Arc::new(std::sync::Mutex::new(LoopbackSimState {
                pos: initial_pos_rad,
                pending: None,
            })),
            port_name: "loopback_sim".into(),
        }
    }

    pub fn position_rad(&self) -> f32 {
        self.state.lock().unwrap().pos
    }
}

impl Transport for LoopbackSimTransport {
    fn port(&self) -> String {
        self.port_name.clone()
    }

    fn kind(&self) -> &'static str {
        "LoopbackSim"
    }

    fn send<'a>(&'a mut self, id: u32, data: &'a [u8]) -> SendFuture<'a> {
        let state = self.state.clone();
        Box::pin(async move {
            let mut s = state.lock().unwrap();
            let cmd = Command::from_can_packet(id, data.to_vec());
            let can_id = cmd.can_id;

            s.pending = Some(match cmd.communication_type {
                CommunicationType::Read => {
                    let r = ReadCommand::from_command(cmd);
                    let v = if r.parameter_index == RS03_MECH_POS_INDEX {
                        s.pos
                    } else {
                        0.0f32
                    };
                    loopback_read_response_packet(can_id, r.host_id, r.parameter_index, v)
                }
                CommunicationType::Control => {
                    // Decode MIT angle: u16 big-endian on the wire, same as `FeedbackFrame`.
                    // `ControlCommand::from_command` mis-maps the angle field vs `to_command`;
                    // decode raw → radians like feedback angle decoding.
                    let angle_raw =
                        u16::from_be_bytes(cmd.data[0..2].try_into().unwrap()) as f32;
                    let target_angle_rad = normalize_value(
                        angle_raw,
                        0.0,
                        65535.0,
                        RS03_ANGLE_MIN,
                        RS03_ANGLE_MAX,
                    );
                    s.pos += 0.45 * (target_angle_rad - s.pos);
                    loopback_feedback_packet(s.pos, can_id)
                }
                CommunicationType::SetZero => {
                    s.pos = 0.0;
                    loopback_feedback_packet(s.pos, can_id)
                }
                CommunicationType::Enable | CommunicationType::Stop => {
                    loopback_feedback_packet(s.pos, can_id)
                }
                _ => loopback_feedback_packet(s.pos, can_id),
            });
            Ok(())
        })
    }

    fn recv(&mut self) -> RecvFuture<'_> {
        let state = self.state.clone();
        Box::pin(async move {
            let mut s = state.lock().unwrap();
            let pkt = s
                .pending
                .take()
                .ok_or_else(|| eyre::eyre!("LoopbackSimTransport: recv without send"))?;
            Ok(pkt)
        })
    }
}

pub struct StubTransport {
    port_name: String,
}

impl CH341Transport {
    pub async fn new(port_name: String) -> Result<Self, Error> {
        let ser = tokio_serial::new(&port_name, 921600).open_native_async()?;
        Ok(Self {
            ser: Arc::new(TokioMutex::new(ser)),
            port_name,
        })
    }
}

#[cfg(feature = "socketcan")]
impl SocketCanTransport {
    pub async fn new(interface_name: String) -> Result<Self, Error> {
        let socket = CanSocket::open(&interface_name)?;
        Ok(Self {
            socket: Arc::new(TokioMutex::new(socket)),
            interface_name,
        })
    }
}

impl StubTransport {
    pub fn new(port_name: String) -> Self {
        Self { port_name }
    }
}

impl Transport for CH341Transport {
    fn send<'a>(&'a mut self, id: u32, data: &'a [u8]) -> SendFuture<'a> {
        let ser = self.ser.clone();
        Box::pin(async move {
            let mut pkt = Vec::new();
            pkt.extend_from_slice(b"AT");
            let addr = (id << 3) | 0x4;
            pkt.extend_from_slice(&addr.to_be_bytes());
            pkt.push(data.len() as u8);
            pkt.extend_from_slice(data);
            pkt.extend_from_slice(b"\r\n");

            {
                let mut ser = ser.lock().await;
                ser.write_all(&pkt).await?;
            }
            tokio::time::sleep(tokio::time::Duration::from_nanos(20)).await;
            Ok(())
        })
    }

    fn recv(&mut self) -> RecvFuture<'_> {
        let ser = self.ser.clone();
        Box::pin(async move {
            let mut buf = vec![0; 1024];
            let mut pos = 0;

            loop {
                let n = {
                    let mut ser = ser.lock().await;
                    ser.read(&mut buf[pos..]).await?
                };

                if n == 0 {
                    return Err(eyre::eyre!("EOF"));
                }
                pos += n;

                for i in 0..pos.saturating_sub(7) {
                    if buf[i] == b'A' && buf[i + 1] == b'T' {
                        if let Ok((id, data, _msg_len)) = parse_message(&buf[i..pos]) {
                            return Ok((id, data));
                        }
                    }
                }

                if pos >= buf.len() - 8 {
                    return Err(eyre::eyre!("Buffer full without finding valid message"));
                }
            }
        })
    }

    fn kind(&self) -> &'static str {
        "CH341"
    }

    fn port(&self) -> String {
        self.port_name.clone()
    }
}

fn parse_message(buf: &[u8]) -> Result<(u32, Vec<u8>, usize), Error> {
    if buf.len() < 8 {
        return Err(eyre::eyre!("Buffer too short"));
    }

    if buf[0] != b'A' || buf[1] != b'T' {
        return Err(eyre::eyre!("Invalid AT prefix"));
    }

    let data_len = buf[6] as usize;
    let total_len = 7 + data_len + 2;

    if buf.len() < total_len {
        return Err(eyre::eyre!("Incomplete message"));
    }

    if buf[total_len - 2] != b'\r' || buf[total_len - 1] != b'\n' {
        return Err(eyre::eyre!("Invalid message termination"));
    }

    let mut id_bytes = [0u8; 4];
    id_bytes.copy_from_slice(&buf[2..6]);
    let raw_id = u32::from_be_bytes(id_bytes);
    let id = (raw_id >> 3) & 0x1FFF_FFFF;

    let data = buf[7..7 + data_len].to_vec();

    Ok((id, data, total_len))
}

#[cfg(feature = "socketcan")]
impl Transport for SocketCanTransport {
    fn send<'a>(&'a mut self, id: u32, data: &'a [u8]) -> SendFuture<'a> {
        let socket = self.socket.clone();
        Box::pin(async move {
            let extended_id =
                ExtendedId::new(id).ok_or_else(|| eyre::eyre!("Invalid CAN ID: {}", id))?;
            let msg = socketcan::CanFrame::new(extended_id, data)
                .ok_or_else(|| eyre::eyre!("Failed to create CAN frame"))?;

            {
                let socket = socket.lock().await;
                socket.write_frame(&msg).await?;
            }
            Ok(())
        })
    }

    fn recv(&mut self) -> RecvFuture<'_> {
        let socket = self.socket.clone();
        Box::pin(async move {
            let frame = {
                let socket = socket.lock().await;
                socket.read_frame().await?
            };

            let id = match frame.id() {
                socketcan::Id::Standard(id) => id.as_raw() as u32,
                socketcan::Id::Extended(id) => id.as_raw(),
            };
            Ok((id, frame.data().to_vec()))
        })
    }

    fn kind(&self) -> &'static str {
        "SocketCAN"
    }

    fn port(&self) -> String {
        self.interface_name.clone()
    }
}

impl Transport for StubTransport {
    fn port(&self) -> String {
        self.port_name.clone()
    }

    fn kind(&self) -> &'static str {
        "Stub"
    }

    fn send<'a>(&'a mut self, id: u32, data: &'a [u8]) -> SendFuture<'a> {
        tracing::debug!("StubTransport::send: id={:04x}, data={:02x?}", id, data);
        Box::pin(async move { Ok(()) })
    }

    fn recv(&mut self) -> RecvFuture<'_> {
        let id = 0x2000100;
        let data = vec![0x7f, 0xfe, 0x80, 0x73, 0x7f, 0xff, 0x01, 0x18];
        Box::pin(async move {
            tokio::time::sleep(tokio::time::Duration::from_millis(100)).await;
            Ok((id, data))
        })
    }
}

impl Clone for CH341Transport {
    fn clone(&self) -> Self {
        Self {
            ser: self.ser.clone(),
            port_name: self.port_name.clone(),
        }
    }
}

#[cfg(feature = "socketcan")]
impl Clone for SocketCanTransport {
    fn clone(&self) -> Self {
        Self {
            socket: self.socket.clone(),
            interface_name: self.interface_name.clone(),
        }
    }
}

impl Clone for StubTransport {
    fn clone(&self) -> Self {
        Self {
            port_name: self.port_name.clone(),
        }
    }
}
