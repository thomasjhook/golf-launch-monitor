//! Shot history storage.
//!
//! [`PgStore`] is the real thing (Postgres, migrations applied at startup).
//! [`MemoryStore`] keeps the service usable without a database: history then
//! lasts only as long as the process.

use std::collections::VecDeque;
use std::sync::Mutex;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use launch_proto::pb::{ShotResult, StoredShot};
use prost::Message;
use sqlx::postgres::{PgConnectOptions, PgPool, PgPoolOptions};
use sqlx::Row;

#[derive(Debug, thiserror::Error)]
pub enum StoreError {
    #[error("database error: {0}")]
    Db(#[from] sqlx::Error),
    #[error("migration failed: {0}")]
    Migrate(#[from] sqlx::migrate::MigrateError),
    #[error("stored shot {id} is corrupt: {source}")]
    Corrupt { id: i64, source: prost::DecodeError },
}

#[tonic::async_trait]
pub trait ShotStore: Send + Sync {
    /// Persist one processed shot and return it with its storage id.
    async fn save(&self, result: &ShotResult) -> Result<StoredShot, StoreError>;

    /// Up to `limit` shots with `id < before_id` (`None` = from the newest),
    /// newest first.
    async fn list(&self, limit: u32, before_id: Option<i64>)
        -> Result<Vec<StoredShot>, StoreError>;
}

fn now_ms() -> i64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0, |d| d.as_millis() as i64)
}

/// In-process store with a bounded history.
pub struct MemoryStore {
    capacity: usize,
    inner: Mutex<MemoryInner>,
}

#[derive(Default)]
struct MemoryInner {
    next_id: i64,
    shots: VecDeque<StoredShot>, // oldest first
}

impl MemoryStore {
    pub fn new(capacity: usize) -> Self {
        Self {
            capacity,
            inner: Mutex::default(),
        }
    }
}

impl Default for MemoryStore {
    fn default() -> Self {
        Self::new(1000)
    }
}

#[tonic::async_trait]
impl ShotStore for MemoryStore {
    async fn save(&self, result: &ShotResult) -> Result<StoredShot, StoreError> {
        let mut inner = self.inner.lock().expect("memory store poisoned");
        inner.next_id += 1;
        let shot = StoredShot {
            id: inner.next_id,
            recorded_unix_ms: now_ms(),
            result: Some(result.clone()),
        };
        if inner.shots.len() == self.capacity {
            inner.shots.pop_front();
        }
        inner.shots.push_back(shot.clone());
        Ok(shot)
    }

    async fn list(
        &self,
        limit: u32,
        before_id: Option<i64>,
    ) -> Result<Vec<StoredShot>, StoreError> {
        let inner = self.inner.lock().expect("memory store poisoned");
        Ok(inner
            .shots
            .iter()
            .rev()
            .filter(|s| before_id.is_none_or(|b| s.id < b))
            .take(limit as usize)
            .cloned()
            .collect())
    }
}

/// Postgres-backed store.
pub struct PgStore {
    pool: PgPool,
}

impl PgStore {
    /// Connect (retrying while the database starts up) and apply migrations.
    ///
    /// In Kubernetes the database pod usually becomes ready after this one
    /// starts, so a failed first attempt is normal; give up after `patience`.
    pub async fn connect(
        options: PgConnectOptions,
        patience: Duration,
    ) -> Result<Self, StoreError> {
        let deadline = tokio::time::Instant::now() + patience;
        let pool = loop {
            match PgPoolOptions::new()
                .max_connections(5)
                .acquire_timeout(Duration::from_secs(5))
                .connect_with(options.clone())
                .await
            {
                Ok(pool) => break pool,
                Err(e) if tokio::time::Instant::now() < deadline => {
                    tracing::warn!(error = %e, "database not ready; retrying");
                    tokio::time::sleep(Duration::from_secs(2)).await;
                }
                Err(e) => return Err(e.into()),
            }
        };
        sqlx::migrate!("./migrations").run(&pool).await?;
        tracing::info!("database connected, migrations applied");
        Ok(Self { pool })
    }

    pub fn pool(&self) -> &PgPool {
        &self.pool
    }
}

#[tonic::async_trait]
impl ShotStore for PgStore {
    async fn save(&self, result: &ShotResult) -> Result<StoredShot, StoreError> {
        let launch = result.launch.unwrap_or_default();
        let flight = result.flight.clone().unwrap_or_default();
        let row = sqlx::query(
            "INSERT INTO shots (shot_id, ball_speed_mps, launch_deg, azimuth_deg, \
                 backspin_rpm, sidespin_rpm, carry_m, apex_m, total_m, result) \
             VALUES ($1, $2, $3, $4, $5, $6, $7, $8, $9, $10) \
             RETURNING id, (extract(epoch FROM recorded_at) * 1000)::BIGINT AS ms",
        )
        .bind(i64::from(result.shot_id))
        .bind(launch.ball_speed_mps)
        .bind(launch.launch_angle_deg)
        .bind(launch.azimuth_deg)
        .bind(launch.backspin_rpm)
        .bind(launch.sidespin_rpm)
        .bind(flight.carry_m)
        .bind(flight.apex_height_m)
        .bind(flight.total_m)
        .bind(result.encode_to_vec())
        .fetch_one(&self.pool)
        .await?;
        Ok(StoredShot {
            id: row.try_get("id")?,
            recorded_unix_ms: row.try_get("ms")?,
            result: Some(result.clone()),
        })
    }

    async fn list(
        &self,
        limit: u32,
        before_id: Option<i64>,
    ) -> Result<Vec<StoredShot>, StoreError> {
        let rows = sqlx::query(
            "SELECT id, (extract(epoch FROM recorded_at) * 1000)::BIGINT AS ms, result \
             FROM shots WHERE id < $1 ORDER BY id DESC LIMIT $2",
        )
        .bind(before_id.unwrap_or(i64::MAX))
        .bind(i64::from(limit))
        .fetch_all(&self.pool)
        .await?;
        rows.into_iter()
            .map(|row| {
                let id: i64 = row.try_get("id")?;
                let bytes: Vec<u8> = row.try_get("result")?;
                let result = ShotResult::decode(bytes.as_slice())
                    .map_err(|source| StoreError::Corrupt { id, source })?;
                Ok(StoredShot {
                    id,
                    recorded_unix_ms: row.try_get("ms")?,
                    result: Some(result),
                })
            })
            .collect()
    }
}
