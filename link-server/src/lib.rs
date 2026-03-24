pub mod api;
pub mod log_buffer;
pub mod telemetry;

use std::collections::HashMap;
use std::sync::Arc;
use std::time::Instant;

use axum::Router;
use tokio::sync::{broadcast, Mutex, RwLock};
use tower_http::cors::{Any, CorsLayer};
use tower_http::services::{ServeDir, ServeFile};

use cortex::arm::Arm;
use cortex::config::RobotConfig;
use cortex::motor::Motor;
use robstride::Protocol;

use crate::log_buffer::LogBuffer;
use crate::telemetry::TelemetrySnapshot;

pub struct AppState {
    pub config: RwLock<RobotConfig>,
    pub config_path: String,
    pub motors: Mutex<HashMap<u8, Motor>>,
    pub arms: Mutex<HashMap<String, Arm>>,
    pub protocol: Option<Arc<Mutex<Protocol>>>,
    pub telemetry_tx: broadcast::Sender<TelemetrySnapshot>,
    pub cert_hash_b64: String,
    pub wt_port: u16,
    pub start_time: Instant,
    pub mode: String,
    pub transport_type: String,
    pub log_buffer: LogBuffer,
}

pub fn build_router(state: Arc<AppState>) -> Router {
    let cors = CorsLayer::new()
        .allow_origin(Any)
        .allow_methods(Any)
        .allow_headers(Any);

    let api = Router::new()
        .nest("/api", api::routes())
        .layer(cors)
        .with_state(state);

    let spa = ServeDir::new("link/dist").fallback(ServeFile::new("link/dist/index.html"));

    api.fallback_service(spa)
}
