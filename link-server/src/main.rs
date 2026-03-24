use std::collections::HashMap;
use std::sync::Arc;
use std::time::Instant;

use anyhow::Result;
use base64::Engine;
use clap::Parser;
use tokio::sync::{broadcast, Mutex};
use tracing::info;
use tracing_subscriber::layer::SubscriberExt;
use tracing_subscriber::util::SubscriberInitExt;
use wtransport::Identity;

use cortex::arm::Arm;
use cortex::config::RobotConfig;
use cortex::motor::{create_protocol, Motor};
use link_server::log_buffer::LogBuffer;
use link_server::telemetry::{self, TelemetrySnapshot};
use link_server::{self, AppState};

#[derive(Parser)]
#[command(
    name = "link",
    about = "Robot Link — telemetry server and control interface"
)]
struct Cli {
    /// Run without hardware (mock telemetry for frontend development)
    #[arg(long)]
    no_hardware: bool,

    /// HTTP server port
    #[arg(long, default_value = "8080")]
    port: u16,

    /// WebTransport (QUIC) server port
    #[arg(long, default_value = "4433")]
    wt_port: u16,

    /// Path to robot.yaml config file
    #[arg(long, default_value = "config/robot.yaml")]
    config: String,

    /// Telemetry polling rate in Hz
    #[arg(long, default_value = "20")]
    telemetry_hz: u32,
}

#[tokio::main]
async fn main() -> Result<()> {
    let log_buffer = LogBuffer::new();
    tracing_subscriber::registry()
        .with(tracing_subscriber::fmt::layer())
        .with(log_buffer.clone())
        .init();
    let cli = Cli::parse();

    let config = RobotConfig::load(&cli.config)?;
    info!(
        "Config loaded: transport={}, port={}, baud={}",
        config.bus.transport, config.bus.port, config.bus.baud
    );

    let mut motors: HashMap<u8, Motor> = HashMap::new();
    let mut arms: HashMap<String, Arm> = HashMap::new();
    let transport_type;

    if !cli.no_hardware {
        let protocol = create_protocol(&config.bus).await?;
        transport_type = config.bus.transport.clone();

        let all_ids = collect_can_ids(&config);
        for can_id in all_ids {
            info!(can_id, "registering motor");
            motors.insert(can_id, Motor::new(protocol.clone(), can_id));
        }
        info!("{} motor(s) registered", motors.len());

        if let Some(ref arm_cfg) = config.arm_left {
            arms.insert("left".into(), Arm::new(arm_cfg, protocol.clone()));
        }
        if let Some(ref arm_cfg) = config.arm_right {
            arms.insert("right".into(), Arm::new(arm_cfg, protocol.clone()));
        }
    } else {
        transport_type = "Mock".to_string();
        info!("--no-hardware: running with mock telemetry");
    }

    let identity = Identity::self_signed(["localhost", "127.0.0.1"])
        .map_err(|e| anyhow::anyhow!("failed to generate self-signed cert: {}", e))?;

    let cert_hash_b64 = {
        let hash = identity
            .certificate_chain()
            .as_slice()
            .first()
            .expect("identity has no certificate")
            .hash();
        let bytes: &[u8; 32] = hash.as_ref();
        base64::engine::general_purpose::STANDARD.encode(bytes)
    };
    info!("WebTransport cert hash: {}", cert_hash_b64);

    let mode = if cli.no_hardware {
        "mock".to_string()
    } else {
        "hardware".to_string()
    };

    let (telemetry_tx, _) = broadcast::channel::<TelemetrySnapshot>(64);

    let state = Arc::new(AppState {
        config,
        motors: Mutex::new(motors),
        arms: Mutex::new(arms),
        telemetry_tx,
        cert_hash_b64,
        wt_port: cli.wt_port,
        start_time: Instant::now(),
        mode,
        transport_type,
        log_buffer,
    });

    let telem_state = state.clone();
    let mock = cli.no_hardware;
    let hz = cli.telemetry_hz;
    tokio::spawn(async move {
        telemetry::telemetry_loop(telem_state, hz, mock).await;
    });

    let wt_state = state.clone();
    let wt_port = cli.wt_port;
    tokio::spawn(async move {
        telemetry::webtransport_server(wt_state, wt_port, identity).await;
    });

    let app = link_server::build_router(state.clone());
    let addr = format!("0.0.0.0:{}", cli.port);
    info!("Link HTTP server starting on http://{}", addr);
    info!("Link WebTransport server on port {}", cli.wt_port);
    let listener = tokio::net::TcpListener::bind(&addr).await?;

    axum::serve(listener, app)
        .with_graceful_shutdown(shutdown_signal(state))
        .await?;

    Ok(())
}

async fn shutdown_signal(state: Arc<AppState>) {
    tokio::signal::ctrl_c()
        .await
        .expect("failed to listen for ctrl-c");
    info!("Shutting down — disabling all motors...");
    let mut motors = state.motors.lock().await;
    for (id, motor) in motors.iter_mut() {
        if let Err(e) = motor.disable().await {
            tracing::warn!(can_id = id, error = %e, "failed to disable motor during shutdown");
        }
    }
    info!("All motors disabled. Goodbye.");
}

fn collect_can_ids(config: &RobotConfig) -> Vec<u8> {
    let mut ids = Vec::new();
    for arm in [config.arm_left.as_ref(), config.arm_right.as_ref()]
        .into_iter()
        .flatten()
    {
        for (_name, joint) in arm.joints() {
            if let Some(id) = joint.can_id {
                ids.push(id);
            }
        }
    }
    if let Some(ref waist) = config.waist {
        for (_name, joint) in waist {
            if let Some(id) = joint.can_id {
                ids.push(id);
            }
        }
    }
    ids
}
