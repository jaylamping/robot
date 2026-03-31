use std::collections::HashMap;
use std::net::SocketAddr;
use std::path::Path;
use std::sync::atomic::AtomicBool;
use std::sync::Arc;
use std::time::{Duration, Instant};

use anyhow::Result;
use axum_server::tls_rustls::RustlsConfig;
use base64::Engine;
use clap::Parser;
use tokio::sync::{broadcast, Mutex, RwLock};
use tracing::{info, warn};
use tracing_subscriber::layer::SubscriberExt;
use tracing_subscriber::util::SubscriberInitExt;
use wtransport::Identity;

use cortex::arm::Arm;
use cortex::config::RobotConfig;
use cortex::motor::{create_protocol, Motor};
use cortex::safety;
use navi::log_buffer::LogBuffer;
use navi::telemetry::{self, TelemetrySnapshot};
use navi::{self, AppState};

#[derive(Parser)]
#[command(
    name = "navi",
    about = "Navi — telemetry server and control API for the Link frontend"
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

    /// Path to TLS certificate (PEM). Falls back to self-signed if missing.
    #[arg(long, default_value = "certs/robot.pem")]
    tls_cert: String,

    /// Path to TLS private key (PEM). Falls back to self-signed if missing.
    #[arg(long, default_value = "certs/robot-key.pem")]
    tls_key: String,
}

#[tokio::main]
async fn main() -> Result<()> {
    rustls::crypto::ring::default_provider()
        .install_default()
        .expect("failed to install rustls crypto provider");

    let log_buffer = LogBuffer::new();
    let env_filter = tracing_subscriber::EnvFilter::try_from_default_env()
        .unwrap_or_else(|_| "info,navi=debug,cortex=debug".parse().unwrap());
    tracing_subscriber::registry()
        .with(env_filter)
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
    let protocol_arc;

    if !cli.no_hardware {
        let protocol = create_protocol(&config.bus).await?;
        transport_type = config.bus.transport.clone();

        let all_ids = collect_can_ids(&config);
        for can_id in all_ids {
            info!(can_id, "registering motor");
            let mut motor = Motor::new(protocol.clone(), can_id);
            if let Some((lo, hi)) = safety::limits_for_motor(&config, can_id) {
                motor.set_joint_limits(lo, hi);
            }
            if let Some(home) = safety::home_for_motor(&config, can_id) {
                motor.set_home_rad(home);
            }
            motors.insert(can_id, motor);
        }
        info!("{} motor(s) registered", motors.len());

        if let Some(ref arm_cfg) = config.arm_left {
            arms.insert("left".into(), Arm::new(arm_cfg, protocol.clone()));
        }
        if let Some(ref arm_cfg) = config.arm_right {
            arms.insert("right".into(), Arm::new(arm_cfg, protocol.clone()));
        }
        protocol_arc = Some(protocol);
    } else {
        transport_type = "Mock".to_string();
        protocol_arc = None;
        info!("--no-hardware: running with mock telemetry");
    }

    let identity = Identity::self_signed(["localhost", "127.0.0.1", "robot.local"])
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

    let wt_identity = identity.clone_identity();

    let tls_config = if Path::new(&cli.tls_cert).exists() && Path::new(&cli.tls_key).exists() {
        info!("Loading TLS cert from {} and key from {}", cli.tls_cert, cli.tls_key);
        RustlsConfig::from_pem_file(&cli.tls_cert, &cli.tls_key)
            .await
            .map_err(|e| anyhow::anyhow!("failed to load TLS PEM files: {}", e))?
    } else {
        warn!(
            "TLS cert/key not found at {}, {}; using self-signed (browser will show 'Not Secure')",
            cli.tls_cert, cli.tls_key
        );
        let cert_der: Vec<Vec<u8>> = identity
            .certificate_chain()
            .as_slice()
            .iter()
            .map(|c| c.der().to_vec())
            .collect();
        let key_der = identity.private_key().secret_der().to_vec();
        RustlsConfig::from_der(cert_der, key_der)
            .await
            .map_err(|e| anyhow::anyhow!("failed to build TLS config: {}", e))?
    };

    let mode = if cli.no_hardware {
        "mock".to_string()
    } else {
        "hardware".to_string()
    };

    let (telemetry_tx, _) = broadcast::channel::<TelemetrySnapshot>(64);

    let state = Arc::new(AppState {
        config: RwLock::new(config),
        config_path: cli.config.clone(),
        motors: Mutex::new(motors),
        arms: Mutex::new(arms),
        protocol: protocol_arc,
        telemetry_tx,
        latest_telemetry: RwLock::new(None),
        cert_hash_b64,
        wt_port: cli.wt_port,
        start_time: Instant::now(),
        mode,
        transport_type,
        log_buffer,
        commissioning_enabled: Arc::new(AtomicBool::new(false)),
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
        telemetry::webtransport_server(wt_state, wt_port, wt_identity).await;
    });

    let app = navi::build_router(state.clone());
    let addr = SocketAddr::from(([0, 0, 0, 0], cli.port));
    info!("Navi HTTPS server starting on https://{}", addr);
    info!("Navi WebTransport server on port {}", cli.wt_port);

    let handle = axum_server::Handle::new();
    let shutdown_handle = handle.clone();
    let shutdown_state = state.clone();
    tokio::spawn(async move {
        shutdown_signal(shutdown_state).await;
        shutdown_handle.graceful_shutdown(Some(Duration::from_secs(5)));
    });

    axum_server::bind_rustls(addr, tls_config)
        .handle(handle)
        .serve(app.into_make_service())
        .await
        .map_err(|e| anyhow::anyhow!("HTTPS server error: {}", e))?;

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
