//! Daemon tests: full pipeline against an in-process shot-service, plus
//! failure paths driven by a scripted fake device.

use std::path::PathBuf;
use std::time::{Duration, Instant};

use capture_daemon::camera::{Camera, CapturedFrame};
use capture_daemon::daemon::{run, DaemonConfig, DaemonEvent};
use capture_daemon::link::McuLink;
use capture_daemon::service::{connect_lazy, install_bench_setup, RetryPolicy, ShotUploader};
use capture_daemon::sim::{SimRig, SimRigConfig};
use capture_daemon::Error;
use launch_proto::pb::shot_service_client::ShotServiceClient;
use launch_proto::pb::ProcessShotRequest;
use launch_sim::{bench_calibration, default_noise, render_shot, Truth};
use protocol::frame::{self, FrameDecoder, MAX_FRAME_LEN};
use protocol::{ConfigError, DeviceToHost, HostToDevice, ShotTriggered, Status, StrobeConfig};
use tokio::io::{duplex, AsyncReadExt, AsyncWriteExt, DuplexStream};
use tokio::sync::{mpsc, oneshot};

struct TestServer {
    addr: std::net::SocketAddr,
    shutdown: Option<oneshot::Sender<()>>,
}

impl TestServer {
    async fn start() -> Self {
        let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
        let addr = listener.local_addr().unwrap();
        let (tx, rx) = oneshot::channel::<()>();
        let state = shot_service::AppState::new(None);
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

    fn url(&self) -> String {
        format!("http://{}", self.addr)
    }

    async fn uploader(&self, install_setup: bool) -> ShotUploader {
        let channel = connect_lazy(&self.url()).unwrap();
        if install_setup {
            install_bench_setup(channel.clone(), RetryPolicy::default())
                .await
                .unwrap();
        }
        ShotUploader::new(channel, RetryPolicy::default())
    }
}

impl Drop for TestServer {
    fn drop(&mut self) {
        if let Some(tx) = self.shutdown.take() {
            let _ = tx.send(());
        }
    }
}

fn fast_rig() -> SimRigConfig {
    SimRigConfig {
        lockout_us: 20_000,
        readout_delay: Duration::from_millis(5),
        tick_interval: Duration::from_millis(1),
    }
}

fn temp_dir(name: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("capd-test-{}-{name}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    dir
}

type Events = mpsc::UnboundedReceiver<DaemonEvent>;

async fn next_event(rx: &mut Events) -> DaemonEvent {
    tokio::time::timeout(Duration::from_secs(20), rx.recv())
        .await
        .expect("timed out waiting for daemon event")
        .expect("daemon dropped event channel")
}

/// Wait for the first event matching `f`, skipping the rest.
async fn wait_for<R>(rx: &mut Events, mut f: impl FnMut(&DaemonEvent) -> Option<R>) -> R {
    loop {
        let ev = next_event(rx).await;
        if let Some(r) = f(&ev) {
            return r;
        }
    }
}

fn spawn_daemon<T, C>(
    link: McuLink<T>,
    camera: C,
    uploader: ShotUploader,
    cfg: DaemonCfg,
) -> (
    tokio::task::JoinHandle<capture_daemon::Result<()>>,
    Events,
    oneshot::Sender<()>,
)
where
    T: tokio::io::AsyncRead + tokio::io::AsyncWrite + Unpin + Send + 'static,
    C: Camera + 'static,
{
    let (ev_tx, ev_rx) = mpsc::unbounded_channel();
    let (stop_tx, stop_rx) = oneshot::channel::<()>();
    let handle = tokio::spawn(run(link, camera, uploader, cfg, ev_tx, async move {
        let _ = stop_rx.await;
    }));
    (handle, ev_rx, stop_tx)
}

type DaemonCfg = DaemonConfig;

const TRUTHS: [Truth; 3] = [
    Truth {
        speed_mps: 58.0,
        launch_angle_deg: 14.0,
        azimuth_deg: 2.0,
    },
    Truth {
        speed_mps: 66.0,
        launch_angle_deg: 10.0,
        azimuth_deg: -3.0,
    },
    Truth {
        speed_mps: 52.0,
        launch_angle_deg: 18.0,
        azimuth_deg: 4.0,
    },
];

#[tokio::test]
async fn end_to_end_three_shots_recovered_and_archived() {
    let server = TestServer::start().await;
    let uploader = server.uploader(true).await;
    let dir = temp_dir("e2e");
    let SimRig {
        link,
        camera,
        golfer,
        ..
    } = SimRig::start(fast_rig());
    let cfg = DaemonCfg {
        captures_dir: Some(dir.clone()),
        ..DaemonCfg::default()
    };
    let (handle, mut events, stop) = spawn_daemon(link, camera, uploader, cfg);

    wait_for(&mut events, |e| {
        matches!(e, DaemonEvent::Ready { .. }).then_some(())
    })
    .await;
    golfer
        .hit_series(3, Duration::from_millis(80), |i| TRUTHS[i])
        .await
        .unwrap();

    let mut results = Vec::new();
    while results.len() < 3 {
        match next_event(&mut events).await {
            DaemonEvent::ShotProcessed(r) => results.push(r),
            DaemonEvent::Archived { .. } => {}
            other => panic!("unexpected event {other:?}"),
        }
    }
    results.sort_by_key(|r| r.shot_id);
    for (r, truth) in results.iter().zip(TRUTHS) {
        let l = r.launch.unwrap();
        assert!(
            (l.ball_speed_mps - truth.speed_mps).abs() / truth.speed_mps < 0.03,
            "speed {l:?} vs {truth:?}"
        );
        assert!(
            (l.launch_angle_deg - truth.launch_angle_deg).abs() < 1.5,
            "{l:?}"
        );
        assert!((l.azimuth_deg - truth.azimuth_deg).abs() < 1.5, "{l:?}");
    }

    stop.send(()).unwrap();
    handle.await.unwrap().unwrap();

    // Archive: three captures, equal to exactly what was rendered/uploaded,
    // and reprocessing a loaded capture reproduces the live result.
    let metas = capture_store::list(&dir).unwrap();
    assert_eq!(metas.len(), 3);
    let cal = bench_calibration([0.0; 5]);
    let mut client = ShotServiceClient::new(connect_lazy(&server.url()).unwrap());
    for (i, meta) in metas.iter().enumerate() {
        let cap = capture_store::load(meta).unwrap();
        assert_eq!(cap.timing.shot_id, i as u32 + 1);
        assert_eq!(cap.timing.pulse_times_us, vec![0, 1_000, 2_000]);
        let expected = render_shot(&cal, &TRUTHS[i], &[0, 1_000, 2_000], &default_noise());
        assert_eq!(cap.frame, expected, "archived frame == uploaded frame");
        let again = client
            .process_shot(ProcessShotRequest {
                frame: Some((&cap.frame).into()),
                timing: Some((&cap.timing).into()),
            })
            .await
            .unwrap()
            .into_inner()
            .result
            .unwrap();
        assert_eq!(again, results[i]);
    }
    let _ = std::fs::remove_dir_all(&dir);
}

async fn single_shot_with_exposure(exposure_override_us: Option<u32>) -> (DaemonEvent, PathBuf) {
    let server = TestServer::start().await;
    let uploader = server.uploader(true).await;
    let dir = temp_dir(&format!("exp-{exposure_override_us:?}"));
    let SimRig {
        link,
        camera,
        golfer,
        ..
    } = SimRig::start(fast_rig());
    let cfg = DaemonCfg {
        captures_dir: Some(dir.clone()),
        exposure_override_us,
        ..DaemonCfg::default()
    };
    let (handle, mut events, stop) = spawn_daemon(link, camera, uploader, cfg);
    wait_for(&mut events, |e| {
        matches!(e, DaemonEvent::Ready { .. }).then_some(())
    })
    .await;
    golfer.hit(TRUTHS[0]).unwrap();
    let ev = wait_for(&mut events, |e| {
        matches!(
            e,
            DaemonEvent::ShotProcessed(_) | DaemonEvent::ShotFailed { .. }
        )
        .then(|| e.clone())
    })
    .await;
    stop.send(()).unwrap();
    handle.await.unwrap().unwrap();
    (ev, dir)
}

#[tokio::test]
async fn too_short_exposure_loses_flashes_and_fails() {
    // Flashes end at 50, 1050, 2050 us; 1500 us only catches two.
    let (ev, dir) = single_shot_with_exposure(Some(1_500)).await;
    let DaemonEvent::ShotFailed { shot_id, error } = ev else {
        panic!("expected failure, got {ev:?}");
    };
    assert_eq!(shot_id, 1);
    assert!(
        error.contains("detected 2 ball images but timing reports 3"),
        "{error}"
    );
    assert!(error.contains("FailedPrecondition"), "{error}");
    // Failed shots are still archived for later diagnosis.
    assert_eq!(capture_store::list(&dir).unwrap().len(), 1);
    let _ = std::fs::remove_dir_all(&dir);
}

#[tokio::test]
async fn computed_exposure_covers_the_sequence() {
    let (ev, dir) = single_shot_with_exposure(None).await;
    assert!(matches!(ev, DaemonEvent::ShotProcessed(_)), "{ev:?}");
    let _ = std::fs::remove_dir_all(&dir);
}

// ---- scripted fake device -------------------------------------------------

struct FakeDevice {
    config_reply: DeviceToHost,
    /// Send this shot right after `Armed`.
    shot_after_arm: Option<ShotTriggered>,
}

fn spawn_fake_device(mut io: DuplexStream, d: FakeDevice) {
    tokio::spawn(async move {
        let mut dec = FrameDecoder::new();
        let mut buf = [0u8; 128];
        loop {
            let Ok(n @ 1..) = io.read(&mut buf).await else {
                return;
            };
            let mut msgs = vec![];
            dec.feed::<HostToDevice>(&buf[..n], |m| msgs.extend(m.ok()));
            for m in msgs {
                let mut replies = vec![];
                match m {
                    HostToDevice::GetStatus => replies.push(DeviceToHost::Status(Status {
                        fw_version: 1,
                        armed: false,
                        shots_fired: 0,
                        config: StrobeConfig::default(),
                    })),
                    HostToDevice::Ping(n) => replies.push(DeviceToHost::Pong(n)),
                    HostToDevice::SetConfig(_) => replies.push(d.config_reply),
                    HostToDevice::Arm => {
                        replies.push(DeviceToHost::Armed);
                        replies.extend(d.shot_after_arm.map(DeviceToHost::Shot));
                    }
                    HostToDevice::Disarm => replies.push(DeviceToHost::Disarmed),
                }
                for r in replies {
                    let mut out = [0u8; MAX_FRAME_LEN];
                    let bytes = frame::encode(&r, &mut out).unwrap();
                    io.write_all(bytes).await.unwrap();
                }
            }
        }
    });
}

/// Camera that yields frames pushed by the test.
struct StubCamera(mpsc::UnboundedReceiver<CapturedFrame>);

impl Camera for StubCamera {
    async fn set_exposure_us(&mut self, _us: u32) -> capture_daemon::Result<()> {
        Ok(())
    }

    async fn next_frame(&mut self) -> capture_daemon::Result<CapturedFrame> {
        self.0
            .recv()
            .await
            .ok_or_else(|| Error::Camera("stub closed".into()))
    }
}

fn dead_uploader() -> ShotUploader {
    ShotUploader::new(
        connect_lazy("http://127.0.0.1:1").unwrap(),
        RetryPolicy::default(),
    )
}

fn blank_frame() -> CapturedFrame {
    CapturedFrame {
        image: vision::GrayImage::new(8, 8),
        received_at: Instant::now(),
        unix_ms: 0,
    }
}

fn one_pulse_shot(id: u32) -> ShotTriggered {
    ShotTriggered {
        shot_id: id,
        pulse_count: 1,
        pulse_times_us: [0; protocol::MAX_PULSES],
    }
}

#[tokio::test]
async fn mcu_config_rejection_is_fatal_and_clear() {
    let (host, dev) = duplex(1024);
    spawn_fake_device(
        dev,
        FakeDevice {
            config_reply: DeviceToHost::ConfigRejected(ConfigError::DutyCycle),
            shot_after_arm: None,
        },
    );
    let (_tx, rx) = mpsc::unbounded_channel();
    let (handle, _events, _stop) = spawn_daemon(
        McuLink::new(host),
        StubCamera(rx),
        dead_uploader(),
        DaemonCfg::default(),
    );
    let err = handle.await.unwrap().unwrap_err();
    assert!(
        matches!(err, Error::ConfigRejected(ConfigError::DutyCycle)),
        "{err}"
    );
    assert!(err.to_string().contains("DutyCycle"));
}

#[tokio::test]
async fn invalid_host_config_never_reaches_the_device() {
    let SimRig { link, camera, .. } = SimRig::start(fast_rig());
    let cfg = DaemonCfg {
        strobe: StrobeConfig {
            pulse_count: 0,
            ..StrobeConfig::default()
        },
        ..DaemonCfg::default()
    };
    let (handle, _events, _stop) = spawn_daemon(link, camera, dead_uploader(), cfg);
    assert!(matches!(
        handle.await.unwrap(),
        Err(Error::InvalidConfig(ConfigError::PulseCount))
    ));
}

#[tokio::test]
async fn shot_without_frame_is_reported() {
    let (host, dev) = duplex(1024);
    spawn_fake_device(
        dev,
        FakeDevice {
            config_reply: DeviceToHost::ConfigApplied(StrobeConfig::default()),
            shot_after_arm: Some(one_pulse_shot(9)),
        },
    );
    let (_tx, rx) = mpsc::unbounded_channel();
    let cfg = DaemonCfg {
        pair_max_age: Duration::from_millis(100),
        ..DaemonCfg::default()
    };
    let (handle, mut events, stop) =
        spawn_daemon(McuLink::new(host), StubCamera(rx), dead_uploader(), cfg);
    let id = wait_for(&mut events, |e| match e {
        DaemonEvent::ShotWithoutFrame { shot_id } => Some(*shot_id),
        _ => None,
    })
    .await;
    assert_eq!(id, 9);
    stop.send(()).unwrap();
    handle.await.unwrap().unwrap();
}

#[tokio::test]
async fn frame_without_shot_is_reported() {
    let (host, dev) = duplex(1024);
    spawn_fake_device(
        dev,
        FakeDevice {
            config_reply: DeviceToHost::ConfigApplied(StrobeConfig::default()),
            shot_after_arm: None,
        },
    );
    let (tx, rx) = mpsc::unbounded_channel();
    let cfg = DaemonCfg {
        pair_max_age: Duration::from_millis(100),
        ..DaemonCfg::default()
    };
    let (handle, mut events, stop) =
        spawn_daemon(McuLink::new(host), StubCamera(rx), dead_uploader(), cfg);
    wait_for(&mut events, |e| {
        matches!(e, DaemonEvent::Ready { .. }).then_some(())
    })
    .await;
    tx.send(blank_frame()).unwrap();
    wait_for(&mut events, |e| {
        matches!(e, DaemonEvent::FrameWithoutShot).then_some(())
    })
    .await;
    stop.send(()).unwrap();
    handle.await.unwrap().unwrap();
}

#[tokio::test]
async fn link_loss_is_a_fatal_error() {
    let SimRig {
        link,
        camera,
        device,
        ..
    } = SimRig::start(fast_rig());
    let (handle, mut events, _stop) =
        spawn_daemon(link, camera, dead_uploader(), DaemonCfg::default());
    wait_for(&mut events, |e| {
        matches!(e, DaemonEvent::Ready { .. }).then_some(())
    })
    .await;
    device.abort();
    let err = handle.await.unwrap().unwrap_err();
    assert!(matches!(err, Error::LinkClosed), "{err}");
    wait_for(&mut events, |e| {
        matches!(e, DaemonEvent::LinkError(_)).then_some(())
    })
    .await;
}

#[tokio::test]
async fn archive_survives_service_outage() {
    // No server at all: every upload fails after retries, but the shot is
    // archived first. Fast retry policy keeps the test short.
    let dir = temp_dir("outage");
    let SimRig {
        link,
        camera,
        golfer,
        ..
    } = SimRig::start(fast_rig());
    let uploader = ShotUploader::new(
        connect_lazy("http://127.0.0.1:1").unwrap(),
        RetryPolicy {
            max_attempts: 2,
            initial_backoff: Duration::from_millis(5),
            max_backoff: Duration::from_millis(5),
        },
    );
    let cfg = DaemonCfg {
        captures_dir: Some(dir.clone()),
        ..DaemonCfg::default()
    };
    let (handle, mut events, stop) = spawn_daemon(link, camera, uploader, cfg);
    wait_for(&mut events, |e| {
        matches!(e, DaemonEvent::Ready { .. }).then_some(())
    })
    .await;
    golfer.hit(TRUTHS[0]).unwrap();
    let err = wait_for(&mut events, |e| match e {
        DaemonEvent::ShotFailed { error, .. } => Some(error.clone()),
        _ => None,
    })
    .await;
    assert!(err.contains("Unavailable"), "{err}");
    assert_eq!(capture_store::list(&dir).unwrap().len(), 1);
    stop.send(()).unwrap();
    handle.await.unwrap().unwrap();
    let _ = std::fs::remove_dir_all(&dir);
}
