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
use crate::link::McuLink;
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
}

type Events = mpsc::UnboundedSender<DaemonEvent>;

fn emit(events: &Events, ev: DaemonEvent) {
    // Nobody listening is fine.
    let _ = events.send(ev);
}

async fn startup<T, C>(link: &mut McuLink<T>, camera: &mut C, cfg: &DaemonConfig) -> Result<u32>
where
    T: AsyncRead + AsyncWrite + Unpin + Send,
    C: Camera,
{
    cfg.strobe.validate().map_err(Error::InvalidConfig)?;
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

    let exposure_us = cfg.exposure_us();
    camera.set_exposure_us(exposure_us).await?;
    link.request(HostToDevice::Arm, t, "Arm", |m| {
        matches!(m, DeviceToHost::Armed)
    })
    .await?;
    tracing::info!(exposure_us, "armed");
    Ok(exposure_us)
}

/// Run until `shutdown` resolves (graceful: disarms) or a fatal error occurs.
/// If the MCU link closes unexpectedly this returns `Error::LinkClosed`;
/// reconnecting is out of scope.
pub async fn run<T, C>(
    mut link: McuLink<T>,
    mut camera: C,
    uploader: ShotUploader,
    cfg: DaemonConfig,
    events: Events,
    shutdown: impl Future<Output = ()>,
) -> Result<()>
where
    T: AsyncRead + AsyncWrite + Unpin + Send,
    C: Camera + 'static,
{
    let exposure_us = startup(&mut link, &mut camera, &cfg).await?;

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

    let mut pairer: Pairer<ShotTriggered, CapturedFrame> = Pairer::new(cfg.pair_max_age);
    let mut uploads = JoinSet::new();
    let mut expiry = tokio::time::interval(Duration::from_millis(50));
    tokio::pin!(shutdown);
    emit(&events, DaemonEvent::Ready { exposure_us });

    let result: Result<()> = loop {
        let pairings = tokio::select! {
            () = &mut shutdown => break Ok(()),
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
                    emit(&events, DaemonEvent::LinkError(e.to_string()));
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
            handle_pairing(p, &cfg, &uploader, &events, &mut uploads);
        }
        // Reap finished uploads so the set doesn't grow.
        while uploads.try_join_next().is_some() {}
    };

    pump.abort();
    // Best effort: leave the device disarmed even after errors (not if the
    // link itself is dead).
    if !matches!(result, Err(Error::LinkClosed | Error::Io(_))) {
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
    // Let in-flight uploads finish so no result is lost.
    while uploads.join_next().await.is_some() {}
    result
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
