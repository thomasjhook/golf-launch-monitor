use std::net::SocketAddr;
use std::path::PathBuf;
use std::sync::Arc;
use std::time::Duration;

use shot_service::store::{MemoryStore, PgStore, ShotStore};
use sqlx::postgres::PgConnectOptions;
use tokio::signal;
use tracing_subscriber::EnvFilter;

#[tokio::main]
async fn main() {
    tracing_subscriber::fmt()
        .with_env_filter(
            EnvFilter::try_from_default_env().unwrap_or_else(|_| EnvFilter::new("info")),
        )
        .init();

    let addr: SocketAddr = std::env::var("LISTEN_ADDR")
        .unwrap_or_else(|_| "0.0.0.0:8080".to_string())
        .parse()
        .expect("LISTEN_ADDR must be a socket address like 0.0.0.0:8080");
    let data_dir = std::env::var_os("DATA_DIR").map(PathBuf::from);
    if let Some(dir) = &data_dir {
        tracing::info!(data_dir = %dir.display(), "data dir configured");
    }

    let listener = tokio::net::TcpListener::bind(addr)
        .await
        .expect("bind failed");
    tracing::info!(%addr, "shot-service (gRPC) listening");

    let store: Arc<dyn ShotStore> = match database_options() {
        Some(options) => match PgStore::connect(options, Duration::from_secs(90)).await {
            Ok(store) => Arc::new(store),
            Err(e) => {
                tracing::error!(error = %e, "cannot use the configured database");
                std::process::exit(1);
            }
        },
        None => {
            tracing::warn!("no database configured; shot history is in memory only");
            Arc::new(MemoryStore::default())
        }
    };

    let state = shot_service::AppState::with_store(data_dir, store);
    if let Err(e) = shot_service::serve(listener, state, shutdown_signal()).await {
        tracing::error!(error = %e, "server error");
        std::process::exit(1);
    }
    tracing::info!("shutdown complete");
}

/// `DATABASE_URL` wins; otherwise the standard libpq variables (`PGHOST`,
/// `PGUSER`, `PGPASSWORD`, `PGDATABASE`, ...) are used when `PGHOST` is set,
/// which lets Kubernetes inject the password from a Secret without building a
/// URL.
fn database_options() -> Option<PgConnectOptions> {
    if let Ok(url) = std::env::var("DATABASE_URL") {
        return Some(
            url.parse()
                .expect("DATABASE_URL is not a valid Postgres URL"),
        );
    }
    std::env::var_os("PGHOST").map(|_| PgConnectOptions::new())
}

async fn shutdown_signal() {
    let ctrl_c = async { signal::ctrl_c().await.expect("ctrl-c handler") };
    #[cfg(unix)]
    let term = async {
        signal::unix::signal(signal::unix::SignalKind::terminate())
            .expect("SIGTERM handler")
            .recv()
            .await;
    };
    #[cfg(not(unix))]
    let term = std::future::pending::<()>();
    tokio::select! { _ = ctrl_c => {}, _ = term => {} }
    tracing::info!("shutdown signal received");
}
