//! Postgres integration tests. They need a database, so they are `#[ignore]`d
//! in a plain `cargo test`; run them with `make test-db` (starts the compose
//! Postgres) or with `DATABASE_URL` set and `cargo test -- --ignored`.

use std::time::Duration;

use launch_proto::pb::{FlightResult, LaunchConditions, ShotResult};
use shot_service::store::{PgStore, ShotStore};

fn shot(shot_id: u32) -> ShotResult {
    ShotResult {
        shot_id,
        launch: Some(LaunchConditions {
            ball_speed_mps: 60.0 + f64::from(shot_id),
            launch_angle_deg: 12.0,
            azimuth_deg: -1.5,
            ..Default::default()
        }),
        flight: Some(FlightResult {
            carry_m: 150.0,
            ..Default::default()
        }),
    }
}

async fn connect() -> PgStore {
    let url = std::env::var("DATABASE_URL").expect("DATABASE_URL must be set");
    let store = PgStore::connect(url.parse().unwrap(), Duration::from_secs(30))
        .await
        .unwrap();
    // Tests share one database; start from an empty table.
    sqlx::query("TRUNCATE shots RESTART IDENTITY")
        .execute(store.pool())
        .await
        .unwrap();
    store
}

#[tokio::test]
#[ignore = "needs Postgres (make test-db)"]
async fn save_list_roundtrip_and_paging() {
    let store = connect().await;
    let saved = store.save(&shot(1)).await.unwrap();
    assert_eq!(saved.id, 1);
    assert!(saved.recorded_unix_ms > 1_600_000_000_000);
    for id in 2..=4 {
        store.save(&shot(id)).await.unwrap();
    }

    let newest = store.list(2, None).await.unwrap();
    let ids: Vec<u32> = newest
        .iter()
        .map(|s| s.result.as_ref().unwrap().shot_id)
        .collect();
    assert_eq!(ids, [4, 3]);
    // The full message survives the round trip.
    assert_eq!(newest[0].result.as_ref().unwrap(), &shot(4));

    let older = store.list(10, Some(newest[1].id)).await.unwrap();
    let ids: Vec<u32> = older
        .iter()
        .map(|s| s.result.as_ref().unwrap().shot_id)
        .collect();
    assert_eq!(ids, [2, 1]);
}

#[tokio::test]
#[ignore = "needs Postgres (make test-db)"]
async fn queryable_columns_are_populated() {
    let store = connect().await;
    store.save(&shot(3)).await.unwrap();
    let (speed, launch, carry): (f64, f64, f64) =
        sqlx::query_as("SELECT ball_speed_mps, launch_deg, carry_m FROM shots")
            .fetch_one(store.pool())
            .await
            .unwrap();
    assert_eq!((speed, launch, carry), (63.0, 12.0, 150.0));
}

#[tokio::test]
#[ignore = "needs Postgres (make test-db)"]
async fn connect_is_idempotent_across_restarts() {
    // A second startup against an already-migrated database must succeed.
    let first = connect().await;
    first.save(&shot(1)).await.unwrap();
    let url = std::env::var("DATABASE_URL").unwrap();
    let second = PgStore::connect(url.parse().unwrap(), Duration::from_secs(30))
        .await
        .unwrap();
    assert_eq!(second.list(10, None).await.unwrap().len(), 1);
}
