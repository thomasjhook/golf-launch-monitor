//! Orchestration: configure the MCU and camera, arm, then pair shots with
//! frames, archive each pair, upload it and report events.

use std::future::Future;
use std::path::PathBuf;
use std::time::{Duration, Instant};

use launch_proto::pb;
use launch_types::StrobeTiming;
use protocol::{DeviceToHost, HostToDevice, ShotTriggered, StrobeConfig};
use tokio::io::{AsyncRead, AsyncWrite};
use tokio::sync::mpsc;
use tokio::task::JoinSet;

use crate::camera::{Camera, CapturedFrame};
use crate::error::{fmt_status, Error, Result};
use crate::link::{LinkConnector, McuLink, OneShot};
use crate::pairing::{Pairer, Pairing, DEFAULT_MAX_AGE};
use crate::service::ShotUploader;

/// Exposure = strobe sequence duration + this margin, so the last flash is
/// never clipped by the end of the exposure.
pub const EXPOSURE_MARGIN_US: u32 = 500;

#[derive(Debug, Clone)]
pub struct DaemonConfig {
    pub strobe: StrobeConfig,
    /// Archive every paired shot here (before uploading) if set.
    pub captures_dir: Option<PathBuf>,
    /// Max wait for a shot's partner frame (and vice versa).
    pub pair_max_age: Duration,
    /// Timeout for each startup/shutdown request to the MCU.
    pub mcu_timeout: Duration,
    /// Diagnostic/test override of the computed exposure. Normally `None`.
    pub exposure_override_us: Option<u32>,
}

impl Default for DaemonConfig {
    fn default() -> Self {
        Self {
            strobe: StrobeConfig::default(),
            captures_dir: None,
            pair_max_age: DEFAULT_MAX_AGE,
            mcu_timeout: Duration::from_secs(1),
            exposure_override_us: None,
        }
    }
}

impl DaemonConfig {
    pub fn exposure_us(&self) -> u32 {
        self.exposure_override_us
            .unwrap_or_else(|| self.strobe.sequence_duration_us() + EXPOSURE_MARGIN_US)
    }
}

#[derive(Debug, Clone)]
pub enum DaemonEvent {
    /// Configured and armed; hits are now accepted.
    Ready {
        exposure_us: u32,
    },
    Archived {
        shot_id: u32,
        path: PathBuf,
    },
    ShotProcessed(pb::ShotResult),
    ShotFailed {
        shot_id: u32,
        error: String,
    },
    FrameWithoutShot,
    ShotWithoutFrame {
        shot_id: u32,
    },
    LinkError(String),
    /// About to retry the MCU connection after `delay` (attempt 1 is the
    /// first retry after a loss; it grows while retries keep failing).
    Reconnecting {
        attempt: u32,
        delay: Duration,
    },
}

type Events = mpsc::UnboundedSender<DaemonEvent>;

fn emit(events: &Events, ev: DaemonEvent) {
    // Nobody listening is fine.
    let _ = events.send(ev);
}

async fn startup<T>(link: &mut McuLink<T>, cfg: &DaemonConfig) -> Result<()>
where
    T: AsyncRead + AsyncWrite + Unpin + Send,
{
    let t = cfg.mcu_timeout;

    let status = link
        .request(HostToDevice::GetStatus, t, "GetStatus", |m| {
            matches!(m, DeviceToHost::Status(_))
        })
        .await?;
    tracing::info!(?status, "MCU status");
    link.request(HostToDevice::Ping(0x00C0_FFEE), t, "Ping", |m| {
        matches!(m, DeviceToHost::Pong(_))
    })
    .await?;

    let reply = link
        .request(HostToDevice::SetConfig(cfg.strobe), t, "SetConfig", |m| {
            matches!(
                m,
                DeviceToHost::ConfigApplied(_) | DeviceToHost::ConfigRejected(_)
            )
        })
        .await?;
    match reply {
        DeviceToHost::ConfigApplied(c) if c == cfg.strobe => {}
        DeviceToHost::ConfigRejected(e) => return Err(Error::ConfigRejected(e)),
        other => return Err(Error::ConfigMismatch(other)),
    }

    link.request(HostToDevice::Arm, t, "Arm", |m| {
        matches!(m, DeviceToHost::Armed)
    })
    .await?;
    tracing::info!(exposure_us = cfg.exposure_us(), "armed");
    Ok(())
}

/// Backoff between attempts to (re)establish the MCU link: doubles from
/// `initial` up to `max`, and starts over after a successful startup.
#[derive(Debug, Clone)]
pub struct ReconnectPolicy {
    pub initial: Duration,
    pub max: Duration,
}

impl Default for ReconnectPolicy {
    fn default() -> Self {
        Self {
            initial: Duration::from_millis(250),
            max: Duration::from_secs(5),
        }
    }
}

impl ReconnectPolicy {
    fn delay(&self, attempt: u32) -> Duration {
        self.initial
            .saturating_mul(2u32.saturating_pow(attempt.saturating_sub(1)))
            .min(self.max)
    }
}

/// Run on a single, already-open link until `shutdown` resolves (graceful:
/// disarms) or a fatal error occurs. If the MCU link closes unexpectedly this
/// returns `Error::LinkClosed`; see `run_supervised` for reconnecting.
pub async fn run<T, C>(
    link: McuLink<T>,
    camera: C,
    uploader: ShotUploader,
    cfg: DaemonConfig,
    events: Events,
    shutdown: impl Future<Output = ()>,
) -> Result<()>
where
    T: AsyncRead + AsyncWrite + Unpin + Send,
    C: Camera + 'static,
{
    run_supervised(
        OneShot(Some(link)),
        camera,
        uploader,
        cfg,
        events,
        shutdown,
        None,
    )
    .await
}

/// Like `run`, but obtains the link from `connector` and, with a
/// `reconnect` policy, reconnects and re-arms whenever the link fails (device
/// unplugged, unresponsive, or busy), retrying forever with backoff. Without
/// a policy any link failure is returned. Camera and configuration errors are
/// always fatal. The camera and in-flight uploads outlive individual links.
/// Shots in flight when the link drops are lost (reported as
/// `FrameWithoutShot`).
pub async fn run_supervised<K, C>(
    mut connector: K,
    mut camera: C,
    uploader: ShotUploader,
    cfg: DaemonConfig,
    events: Events,
    shutdown: impl Future<Output = ()>,
    reconnect: Option<ReconnectPolicy>,
) -> Result<()>
where
    K: LinkConnector,
    C: Camera + 'static,
{
    cfg.strobe.validate().map_err(Error::InvalidConfig)?;
    let exposure_us = cfg.exposure_us();
    camera.set_exposure_us(exposure_us).await?;

    // `next_frame` is not cancel-safe in general, so it gets its own task.
    let (frame_tx, mut frames) = mpsc::channel::<Result<CapturedFrame>>(4);
    let pump = tokio::spawn(async move {
        loop {
            let r = camera.next_frame().await;
            let failed = r.is_err();
            if frame_tx.send(r).await.is_err() || failed {
                break;
            }
        }
    });

    let mut uploads = JoinSet::new();
    tokio::pin!(shutdown);
    let mut attempt = 0u32;

    let result: Result<()> = 'outer: loop {
        if attempt > 0 {
            let Some(policy) = &reconnect else {
                unreachable!("retry without a policy")
            };
            let delay = policy.delay(attempt);
            emit(&events, DaemonEvent::Reconnecting { attempt, delay });
            tokio::select! {
                () = &mut shutdown => break Ok(()),
                () = tokio::time::sleep(delay) => {}
            }
        }

        let connected = tokio::select! {
            () = &mut shutdown => break Ok(()),
            r = connector.connect() => r,
        };
        let mut link = match connected {
            Ok(l) => l,
            Err(e) if reconnect.is_some() && e.is_link_failure() => {
                emit(&events, DaemonEvent::LinkError(e.to_string()));
                attempt += 1;
                continue;
            }
            Err(e) => break Err(e),
        };

        // A frame from before the (re)connect has no shot to pair with.
        loop {
            match frames.try_recv() {
                Ok(Ok(_)) => emit(&events, DaemonEvent::FrameWithoutShot),
                Ok(Err(e)) => break 'outer Err(e),
                Err(mpsc::error::TryRecvError::Empty) => break,
                Err(mpsc::error::TryRecvError::Disconnected) => {
                    break 'outer Err(Error::Camera("camera task ended".into()))
                }
            }
        }

        let (end, was_ready) = session(
            &mut link,
            &mut frames,
            &uploader,
            &cfg,
            &events,
            &mut uploads,
            exposure_us,
            shutdown.as_mut(),
        )
        .await;
        match end {
            Ok(()) => break Ok(()),
            Err(e) if reconnect.is_some() && e.is_link_failure() => {
                tracing::warn!(error = %e, "MCU link lost; reconnecting");
                // A link that came up fully restarts the backoff.
                attempt = if was_ready { 1 } else { attempt + 1 };
            }
            Err(e) => break Err(e),
        }
    };

    pump.abort();
    // Let in-flight uploads finish so no result is lost.
    while uploads.join_next().await.is_some() {}
    result
}

/// One connection's lifetime: configure and arm, then pair shots with frames
/// until shutdown or an error. The bool says whether the device got armed.
#[allow(clippy::too_many_arguments)]
async fn session<T, F>(
    link: &mut McuLink<T>,
    frames: &mut mpsc::Receiver<Result<CapturedFrame>>,
    uploader: &ShotUploader,
    cfg: &DaemonConfig,
    events: &Events,
    uploads: &mut JoinSet<()>,
    exposure_us: u32,
    mut shutdown: std::pin::Pin<&mut F>,
) -> (Result<()>, bool)
where
    T: AsyncRead + AsyncWrite + Unpin + Send,
    F: Future<Output = ()>,
{
    if let Err(e) = startup(link, cfg).await {
        return (Err(e), false);
    }
    let mut pairer: Pairer<ShotTriggered, CapturedFrame> = Pairer::new(cfg.pair_max_age);
    let mut expiry = tokio::time::interval(Duration::from_millis(50));
    emit(events, DaemonEvent::Ready { exposure_us });

    let result: Result<()> = loop {
        let pairings = tokio::select! {
            () = shutdown.as_mut() => break Ok(()),
            msg = link.recv() => match msg {
                Ok(DeviceToHost::Shot(shot)) => {
                    tracing::debug!(shot_id = shot.shot_id, pulses = ?shot.pulses(), "shot triggered");
                    pairer.push_shot(shot, Instant::now())
                }
                Ok(other) => {
                    tracing::debug!(?other, "ignoring MCU message");
                    continue;
                }
                Err(e) => {
                    emit(events, DaemonEvent::LinkError(e.to_string()));
                    break Err(e);
                }
            },
            frame = frames.recv() => match frame {
                Some(Ok(f)) => {
                    let at = f.received_at;
                    pairer.push_frame(f, at)
                }
                Some(Err(e)) => break Err(e),
                None => break Err(Error::Camera("camera task ended".into())),
            },
            _ = expiry.tick() => pairer.expire(Instant::now()),
        };
        for p in pairings {
            handle_pairing(p, cfg, uploader, events, uploads);
        }
        // Reap finished uploads so the set doesn't grow.
        while uploads.try_join_next().is_some() {}
    };

    // Whatever is still waiting for a partner never gets one.
    for p in pairer.expire(Instant::now() + cfg.pair_max_age + Duration::from_secs(1)) {
        handle_pairing(p, cfg, uploader, events, uploads);
    }
    // Best effort: leave the device disarmed even after errors (not if the
    // link itself is dead).
    if !matches!(&result, Err(e) if e.is_link_failure()) {
        match link
            .request(HostToDevice::Disarm, cfg.mcu_timeout, "Disarm", |m| {
                matches!(m, DeviceToHost::Disarmed)
            })
            .await
        {
            Ok(_) => tracing::info!("disarmed"),
            Err(e) => tracing::warn!(error = %e, "could not confirm disarm"),
        }
    }
    (result, true)
}

fn handle_pairing(
    p: Pairing<ShotTriggered, CapturedFrame>,
    cfg: &DaemonConfig,
    uploader: &ShotUploader,
    events: &Events,
    uploads: &mut JoinSet<()>,
) {
    match p {
        Pairing::FrameWithoutShot(_) => {
            tracing::warn!("dropping frame with no matching shot");
            emit(events, DaemonEvent::FrameWithoutShot);
        }
        Pairing::ShotWithoutFrame(shot) => {
            tracing::warn!(shot_id = shot.shot_id, "dropping shot with no frame");
            emit(
                events,
                DaemonEvent::ShotWithoutFrame {
                    shot_id: shot.shot_id,
                },
            );
        }
        Pairing::Paired { shot, frame } => {
            let timing = StrobeTiming {
                shot_id: shot.shot_id,
                pulse_times_us: shot.pulses().to_vec(),
            };
            // Archive BEFORE uploading so a service outage loses nothing.
            if let Some(dir) = &cfg.captures_dir {
                let capture = capture_store::Capture {
                    timing: timing.clone(),
                    frame: frame.image.clone(),
                    captured_at_unix_ms: frame.unix_ms,
                };
                match capture_store::save(dir, &capture) {
                    Ok(path) => emit(
                        events,
                        DaemonEvent::Archived {
                            shot_id: timing.shot_id,
                            path,
                        },
                    ),
                    Err(e) => {
                        tracing::error!(error = %e, shot_id = timing.shot_id, "archive failed")
                    }
                }
            }
            let (uploader, events) = (uploader.clone(), events.clone());
            uploads.spawn(async move {
                let shot_id = timing.shot_id;
                match uploader.upload(&frame.image, &timing).await {
                    Ok(result) => emit(&events, DaemonEvent::ShotProcessed(result)),
                    Err(status) => {
                        let error = fmt_status(&status);
                        tracing::warn!(shot_id, %error, "shot failed");
                        emit(&events, DaemonEvent::ShotFailed { shot_id, error });
                    }
                }
            });
        }
    }
}
