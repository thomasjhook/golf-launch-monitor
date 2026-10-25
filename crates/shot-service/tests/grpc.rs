//! End-to-end gRPC tests: a real server on an ephemeral port, real clients.

use std::net::SocketAddr;
use std::path::PathBuf;
use std::sync::Arc;
use std::time::Duration;

use launch_core::FlashOrder;
use launch_proto::pb::calibration_service_client::CalibrationServiceClient;
use launch_proto::pb::shot_service_client::ShotServiceClient;
use launch_proto::pb::{
    self, CameraSetup, GetSetupRequest, ListShotsRequest, ProcessShotRequest, StreamShotsRequest,
};
use launch_sim::{bench_calibration, default_noise, render_shot, Truth};
use launch_types::StrobeTiming;
use shot_service::store::{ShotStore, StoreError};
use shot_service::AppState;
use tokio::sync::oneshot;
use tonic::transport::Channel;
use tonic::Code;

const PULSES_US: [u32; 3] = [0, 1000, 2000];

struct TestServer {
    addr: SocketAddr,
    shutdown: Option<oneshot::Sender<()>>,
}

impl TestServer {
    async fn start(state: Arc<AppState>) -> Self {
        let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
        let addr = listener.local_addr().unwrap();
        let (tx, rx) = oneshot::channel::<()>();
        tokio::spawn(async move {
            shot_service::serve(listener, state, async {
                let _ = rx.await;
            })
            .await
            .unwrap();
        });
        Self {
            addr,
            shutdown: Some(tx),
        }
    }

    async fn channel(&self) -> Channel {
        let url = format!("http://{}", self.addr);
        // The server task may not be accepting yet; retry briefly.
        for _ in 0..50 {
            if let Ok(ch) = Channel::from_shared(url.clone()).unwrap().connect().await {
                return ch;
            }
            tokio::time::sleep(Duration::from_millis(20)).await;
        }
        panic!("server did not come up");
    }
}

impl Drop for TestServer {
    fn drop(&mut self) {
        if let Some(tx) = self.shutdown.take() {
            let _ = tx.send(());
        }
    }
}

fn bench_setup() -> CameraSetup {
    CameraSetup {
        calibration: Some((&bench_calibration([0.0; 5])).into()),
        flash_order: pb::FlashOrder::from(FlashOrder::XDescending) as i32,
    }
}

fn shot_request(shot_id: u32, truth: &Truth) -> ProcessShotRequest {
    let cal = bench_calibration([0.0; 5]);
    let frame = render_shot(&cal, truth, &PULSES_US, &default_noise());
    ProcessShotRequest {
        frame: Some((&frame).into()),
        timing: Some(
            (&StrobeTiming {
                shot_id,
                pulse_times_us: PULSES_US.to_vec(),
            })
                .into(),
        ),
    }
}

const TRUTH: Truth = Truth {
    speed_mps: 58.0,
    launch_angle_deg: 14.0,
    azimuth_deg: 2.0,
};

fn temp_dir(name: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("lm-test-{}-{name}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    dir
}

#[tokio::test]
async fn process_shot_requires_setup() {
    let server = TestServer::start(AppState::new(None)).await;
    let mut shots = ShotServiceClient::new(server.channel().await);
    let err = shots
        .process_shot(shot_request(1, &TRUTH))
        .await
        .unwrap_err();
    assert_eq!(err.code(), Code::FailedPrecondition, "{err}");
}

#[tokio::test]
async fn setup_then_process_and_stream() {
    let server = TestServer::start(AppState::new(None)).await;
    let channel = server.channel().await;
    let mut cal = CalibrationServiceClient::new(channel.clone());
    let mut shots = ShotServiceClient::new(channel);

    assert_eq!(
        cal.get_setup(GetSetupRequest {}).await.unwrap_err().code(),
        Code::NotFound
    );
    let stored = cal.set_setup(bench_setup()).await.unwrap().into_inner();
    assert_eq!(stored, bench_setup());
    assert_eq!(
        cal.get_setup(GetSetupRequest {})
            .await
            .unwrap()
            .into_inner(),
        stored
    );

    let mut stream = shots
        .stream_shots(StreamShotsRequest {})
        .await
        .unwrap()
        .into_inner();

    let result = shots
        .process_shot(shot_request(42, &TRUTH))
        .await
        .unwrap()
        .into_inner()
        .result
        .unwrap();
    assert_eq!(result.shot_id, 42);
    let launch = result.launch.unwrap();
    assert!(
        (launch.ball_speed_mps - TRUTH.speed_mps).abs() / TRUTH.speed_mps < 0.02,
        "{launch:?}"
    );
    assert!((launch.launch_angle_deg - TRUTH.launch_angle_deg).abs() < 1.0);
    assert!((launch.azimuth_deg - TRUTH.azimuth_deg).abs() < 1.0);
    let flight = result.flight.unwrap();
    assert!(flight.carry_m > 100.0 && !flight.trajectory.is_empty());

    // The subscriber sees the same shot.
    let streamed = tokio::time::timeout(Duration::from_secs(5), stream.message())
        .await
        .expect("stream timed out")
        .unwrap()
        .expect("stream ended");
    assert_eq!(streamed.shot_id, 42);
}

#[tokio::test]
async fn bad_requests_are_invalid_argument() {
    let server = TestServer::start(AppState::new(None)).await;
    let channel = server.channel().await;
    let mut cal = CalibrationServiceClient::new(channel.clone());
    let mut shots = ShotServiceClient::new(channel);

    // Unspecified flash order must not silently pick a direction.
    let mut setup = bench_setup();
    setup.flash_order = 0;
    assert_eq!(
        cal.set_setup(setup).await.unwrap_err().code(),
        Code::InvalidArgument
    );
    // Malformed calibration.
    let mut setup = bench_setup();
    setup
        .calibration
        .as_mut()
        .unwrap()
        .pose
        .as_mut()
        .unwrap()
        .rotation
        .pop();
    assert_eq!(
        cal.set_setup(setup).await.unwrap_err().code(),
        Code::InvalidArgument
    );
    // Nothing was stored by the failed calls.
    assert_eq!(
        cal.get_setup(GetSetupRequest {}).await.unwrap_err().code(),
        Code::NotFound
    );

    // Frame whose data length disagrees with its dimensions.
    cal.set_setup(bench_setup()).await.unwrap();
    let mut req = shot_request(1, &TRUTH);
    req.frame.as_mut().unwrap().data.truncate(10);
    assert_eq!(
        shots.process_shot(req).await.unwrap_err().code(),
        Code::InvalidArgument
    );
    // Missing frame.
    let mut req = shot_request(1, &TRUTH);
    req.frame = None;
    assert_eq!(
        shots.process_shot(req).await.unwrap_err().code(),
        Code::InvalidArgument
    );
    // Flash count that doesn't match the ball images found.
    let mut req = shot_request(1, &TRUTH);
    req.timing.as_mut().unwrap().pulse_times_us = vec![0, 1000];
    assert_eq!(
        shots.process_shot(req).await.unwrap_err().code(),
        Code::FailedPrecondition
    );
}

#[tokio::test]
async fn setup_persists_across_restarts() {
    let dir = temp_dir("persist");
    {
        let server = TestServer::start(AppState::new(Some(dir.clone()))).await;
        let mut cal = CalibrationServiceClient::new(server.channel().await);
        cal.set_setup(bench_setup()).await.unwrap();
    }
    // A fresh state over the same directory picks the setup back up.
    let server = TestServer::start(AppState::new(Some(dir.clone()))).await;
    let mut cal = CalibrationServiceClient::new(server.channel().await);
    let got = cal
        .get_setup(GetSetupRequest {})
        .await
        .unwrap()
        .into_inner();
    assert_eq!(got, bench_setup());
    let _ = std::fs::remove_dir_all(dir);
}

#[tokio::test]
async fn corrupt_stored_setup_is_ignored() {
    let dir = temp_dir("corrupt");
    std::fs::create_dir_all(&dir).unwrap();
    std::fs::write(dir.join("setup.pb"), b"\xff\xff not protobuf").unwrap();
    let server = TestServer::start(AppState::new(Some(dir.clone()))).await;
    let mut cal = CalibrationServiceClient::new(server.channel().await);
    assert_eq!(
        cal.get_setup(GetSetupRequest {}).await.unwrap_err().code(),
        Code::NotFound
    );
    let _ = std::fs::remove_dir_all(dir);
}

#[tokio::test]
async fn health_reports_serving() {
    use tonic_health::pb::health_check_response::ServingStatus;
    use tonic_health::pb::health_client::HealthClient;
    use tonic_health::pb::HealthCheckRequest;

    let server = TestServer::start(AppState::new(None)).await;
    let mut health = HealthClient::new(server.channel().await);
    for service in [
        "",
        "launchmonitor.v1.ShotService",
        "launchmonitor.v1.CalibrationService",
    ] {
        let status = health
            .check(HealthCheckRequest {
                service: service.into(),
            })
            .await
            .unwrap()
            .into_inner()
            .status;
        assert_eq!(status, ServingStatus::Serving as i32, "service {service:?}");
    }
}

#[tokio::test]
async fn reflection_lists_services() {
    use tonic_reflection::pb::v1::server_reflection_client::ServerReflectionClient;
    use tonic_reflection::pb::v1::server_reflection_request::MessageRequest;
    use tonic_reflection::pb::v1::server_reflection_response::MessageResponse;
    use tonic_reflection::pb::v1::ServerReflectionRequest;

    let server = TestServer::start(AppState::new(None)).await;
    let mut client = ServerReflectionClient::new(server.channel().await);
    let request = ServerReflectionRequest {
        host: String::new(),
        message_request: Some(MessageRequest::ListServices(String::new())),
    };
    let mut responses = client
        .server_reflection_info(tokio_stream::iter(vec![request]))
        .await
        .unwrap()
        .into_inner();
    let Some(MessageResponse::ListServicesResponse(list)) = responses
        .message()
        .await
        .unwrap()
        .and_then(|r| r.message_response)
    else {
        panic!("expected a ListServicesResponse");
    };
    let names: Vec<_> = list.service.iter().map(|s| s.name.as_str()).collect();
    for expected in [
        "launchmonitor.v1.ShotService",
        "launchmonitor.v1.CalibrationService",
        "grpc.health.v1.Health",
    ] {
        assert!(
            names.contains(&expected),
            "{expected} missing from {names:?}"
        );
    }
}

async fn process_n(shots: &mut ShotServiceClient<Channel>, ids: impl IntoIterator<Item = u32>) {
    for id in ids {
        shots.process_shot(shot_request(id, &TRUTH)).await.unwrap();
    }
}

#[tokio::test]
async fn list_shots_pages_newest_first() {
    let server = TestServer::start(AppState::new(None)).await;
    let channel = server.channel().await;
    let mut cal = CalibrationServiceClient::new(channel.clone());
    let mut shots = ShotServiceClient::new(channel);
    cal.set_setup(bench_setup()).await.unwrap();

    let empty = shots.list_shots(ListShotsRequest::default()).await.unwrap();
    assert!(empty.into_inner().shots.is_empty());

    process_n(&mut shots, 1..=5).await;

    let page1 = shots
        .list_shots(ListShotsRequest {
            page_size: 2,
            before_id: 0,
        })
        .await
        .unwrap()
        .into_inner()
        .shots;
    let ids: Vec<u32> = page1
        .iter()
        .map(|s| s.result.as_ref().unwrap().shot_id)
        .collect();
    assert_eq!(ids, [5, 4]);
    assert!(page1[0].recorded_unix_ms > 0);

    let page2 = shots
        .list_shots(ListShotsRequest {
            page_size: 10,
            before_id: page1[1].id,
        })
        .await
        .unwrap()
        .into_inner()
        .shots;
    let ids: Vec<u32> = page2
        .iter()
        .map(|s| s.result.as_ref().unwrap().shot_id)
        .collect();
    assert_eq!(ids, [3, 2, 1]);

    let err = shots
        .list_shots(ListShotsRequest {
            page_size: 1,
            before_id: -1,
        })
        .await
        .unwrap_err();
    assert_eq!(err.code(), Code::InvalidArgument);
}

struct BrokenStore;

#[tonic::async_trait]
impl ShotStore for BrokenStore {
    async fn save(&self, _: &pb::ShotResult) -> Result<pb::StoredShot, StoreError> {
        Err(StoreError::Db(sqlx::Error::PoolTimedOut))
    }
    async fn list(&self, _: u32, _: Option<i64>) -> Result<Vec<pb::StoredShot>, StoreError> {
        Err(StoreError::Db(sqlx::Error::PoolTimedOut))
    }
}

#[tokio::test]
async fn database_outage_does_not_lose_the_shot() {
    let server = TestServer::start(AppState::with_store(None, Arc::new(BrokenStore))).await;
    let channel = server.channel().await;
    let mut cal = CalibrationServiceClient::new(channel.clone());
    let mut shots = ShotServiceClient::new(channel);
    cal.set_setup(bench_setup()).await.unwrap();
    let mut stream = shots
        .stream_shots(StreamShotsRequest {})
        .await
        .unwrap()
        .into_inner();

    // Storing fails, but the shot is still returned and streamed.
    let result = shots.process_shot(shot_request(7, &TRUTH)).await.unwrap();
    assert_eq!(result.into_inner().result.unwrap().shot_id, 7);
    let streamed = tokio::time::timeout(Duration::from_secs(5), stream.message())
        .await
        .expect("stream timed out")
        .unwrap()
        .expect("stream ended");
    assert_eq!(streamed.shot_id, 7);

    // History reports the outage instead of pretending to be empty.
    let err = shots
        .list_shots(ListShotsRequest::default())
        .await
        .unwrap_err();
    assert_eq!(err.code(), Code::Unavailable);
}
