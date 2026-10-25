//! Simulated hardware: runs the REAL firmware `Sequencer` behind an in-memory
//! stream speaking the real framed wire protocol, plus a camera that renders
//! synthetic frames from the device's planned pulse times.
//!
//! Timing model: the camera exposure window starts at the trigger. A flash
//! appears in the frame only if it ends before the exposure does.

use std::time::{Duration, Instant};

use calibration::Calibration;
use firmware_core::{Schedule, Sequencer};
use launch_sim::{bench_calibration, default_noise, render_shot, Truth};
use protocol::frame::{self, FrameDecoder, MAX_FRAME_LEN};
use protocol::{DeviceToHost, HostToDevice};
use tokio::io::{duplex, AsyncReadExt, AsyncWriteExt, DuplexStream};
use tokio::sync::mpsc;
use tokio::task::JoinHandle;

use crate::camera::{unix_ms_now, Camera, CapturedFrame};
use crate::error::{Error, Result};
use crate::link::McuLink;

#[derive(Debug, Clone)]
pub struct SimRigConfig {
    /// Sequencer lockout after each shot.
    pub lockout_us: u32,
    /// Delay between the end of the exposure and frame delivery.
    pub readout_delay: Duration,
    /// How often the device calls `Sequencer::tick`.
    pub tick_interval: Duration,
}

impl Default for SimRigConfig {
    fn default() -> Self {
        Self {
            lockout_us: firmware_core::DEFAULT_LOCKOUT_US,
            readout_delay: Duration::from_millis(30),
            tick_interval: Duration::from_millis(2),
        }
    }
}

pub struct SimRig {
    /// Host end of the simulated USB serial link.
    pub link: McuLink<DuplexStream>,
    pub camera: SimCamera,
    pub golfer: SimGolfer,
    /// Device task; ends when the host drops the link.
    pub device: JoinHandle<()>,
}

impl SimRig {
    /// Must be called inside a tokio runtime.
    pub fn start(cfg: SimRigConfig) -> Self {
        let (host_io, device_io) = duplex(4096);
        let (hit_tx, hit_rx) = mpsc::unbounded_channel();
        let (shot_tx, shot_rx) = mpsc::unbounded_channel();
        let device = tokio::spawn(run_device(device_io, cfg.clone(), hit_rx, shot_tx));
        Self {
            link: McuLink::new(host_io),
            camera: SimCamera {
                rx: shot_rx,
                exposure_us: 10_000,
                readout_delay: cfg.readout_delay,
                calibration: bench_calibration([0.0; 5]),
            },
            golfer: SimGolfer { tx: hit_tx },
            device,
        }
    }
}

async fn run_device(
    io: DuplexStream,
    cfg: SimRigConfig,
    mut hits: mpsc::UnboundedReceiver<Truth>,
    camera_tx: mpsc::UnboundedSender<(Schedule, Truth)>,
) {
    let (mut rd, mut wr) = tokio::io::split(io);
    let mut decoder = FrameDecoder::new();
    let mut seq = Sequencer::new().with_lockout_us(cfg.lockout_us);
    let epoch = Instant::now();
    let now_us = move || epoch.elapsed().as_micros() as u64;
    let mut firing: Option<(Instant, Truth)> = None;
    let mut tick = tokio::time::interval(cfg.tick_interval);
    let mut buf = [0u8; 128];

    async fn write<W: AsyncWriteExt + Unpin>(wr: &mut W, msg: &DeviceToHost) -> bool {
        let mut out = [0u8; MAX_FRAME_LEN];
        let Ok(bytes) = frame::encode(msg, &mut out) else {
            return false;
        };
        wr.write_all(bytes).await.is_ok() && wr.flush().await.is_ok()
    }

    loop {
        let sequence_end = firing.map(|(t, _)| tokio::time::Instant::from_std(t));
        tokio::select! {
            n = rd.read(&mut buf) => {
                let Ok(n @ 1..) = n else { break };
                let mut msgs = Vec::new();
                decoder.feed::<HostToDevice>(&buf[..n], |m| {
                    if let Ok(m) = m { msgs.push(m) }
                });
                for m in msgs {
                    if let Some(resp) = seq.handle(m) {
                        if !write(&mut wr, &resp).await { return }
                    }
                }
            }
            Some(truth) = hits.recv() => {
                // The break beam: the Sequencer decides whether it counts.
                if let Some(schedule) = seq.on_trigger(now_us()) {
                    let dur = Duration::from_micros(u64::from(schedule.sequence_duration_us));
                    firing = Some((Instant::now() + dur, truth));
                    let _ = camera_tx.send((schedule, truth));
                }
            }
            _ = tokio::time::sleep_until(sequence_end.unwrap_or_else(tokio::time::Instant::now)),
                if sequence_end.is_some() =>
            {
                firing = None;
                if let Some(shot) = seq.on_sequence_done(now_us(), &[]) {
                    if !write(&mut wr, &shot).await { return }
                }
            }
            _ = tick.tick() => seq.tick(now_us()),
        }
    }
}

/// Simulated golfer / break beam.
#[derive(Clone)]
pub struct SimGolfer {
    tx: mpsc::UnboundedSender<Truth>,
}

impl SimGolfer {
    /// Break the beam with a ball moving per `truth`. Does nothing on the
    /// device if it is disarmed, mid-sequence or locked out.
    pub fn hit(&self, truth: Truth) -> Result<()> {
        self.tx
            .send(truth)
            .map_err(|_| Error::Setup("simulated device is gone".into()))
    }

    /// Hit `n` shots, `interval` apart (first immediately), truth from `truth_for(i)`.
    pub async fn hit_series(
        &self,
        n: usize,
        interval: Duration,
        truth_for: impl Fn(usize) -> Truth,
    ) -> Result<()> {
        for i in 0..n {
            if i > 0 {
                tokio::time::sleep(interval).await;
            }
            self.hit(truth_for(i))?;
        }
        Ok(())
    }
}

/// Varied but physically valid truths: all exceed ball diameter / 1 ms spacing
/// (~43 m/s) and stay in the bench camera's view.
pub fn demo_truth(i: usize) -> Truth {
    const TABLE: [(f64, f64, f64); 6] = [
        (58.0, 14.0, 2.0),
        (66.0, 10.0, -3.0),
        (52.0, 18.0, 4.0),
        (62.0, 12.0, 0.0),
        (55.0, 16.0, -2.0),
        (70.0, 8.0, 3.0),
    ];
    let (speed_mps, launch_angle_deg, azimuth_deg) = TABLE[i % TABLE.len()];
    Truth {
        speed_mps,
        launch_angle_deg,
        azimuth_deg,
    }
}

pub struct SimCamera {
    rx: mpsc::UnboundedReceiver<(Schedule, Truth)>,
    exposure_us: u32,
    readout_delay: Duration,
    calibration: Calibration,
}

/// Flash start times (relative to flash 0) of the pulses that fall inside the
/// exposure window, which opens at the trigger.
pub fn visible_pulses(schedule: &Schedule, exposure_us: u32) -> Vec<u32> {
    let first = schedule.pulse_start_us[0];
    schedule
        .pulses()
        .iter()
        .filter(|&&start| start + u32::from(schedule.pulse_width_us) <= exposure_us)
        .map(|&start| start - first)
        .collect()
}

impl Camera for SimCamera {
    async fn set_exposure_us(&mut self, us: u32) -> Result<()> {
        self.exposure_us = us;
        Ok(())
    }

    async fn next_frame(&mut self) -> Result<CapturedFrame> {
        let (schedule, truth) = self
            .rx
            .recv()
            .await
            .ok_or_else(|| Error::Camera("simulated rig shut down".into()))?;
        let pulses = visible_pulses(&schedule, self.exposure_us);
        let cal = self.calibration.clone();
        let image = tokio::task::spawn_blocking(move || {
            render_shot(&cal, &truth, &pulses, &default_noise())
        })
        .await
        .map_err(|e| Error::Camera(format!("render task failed: {e}")))?;
        tokio::time::sleep(self.readout_delay).await;
        Ok(CapturedFrame {
            image,
            received_at: Instant::now(),
            unix_ms: unix_ms_now(),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use protocol::{ConfigError, StrobeConfig};

    fn fast() -> SimRigConfig {
        SimRigConfig {
            lockout_us: 40_000,
            readout_delay: Duration::from_millis(2),
            tick_interval: Duration::from_millis(1),
        }
    }

    const TRUTH: Truth = Truth {
        speed_mps: 58.0,
        launch_angle_deg: 14.0,
        azimuth_deg: 2.0,
    };

    async fn next_shot(link: &mut McuLink<DuplexStream>) -> protocol::ShotTriggered {
        loop {
            let m = tokio::time::timeout(Duration::from_millis(500), link.recv())
                .await
                .expect("timed out waiting for shot")
                .unwrap();
            if let DeviceToHost::Shot(s) = m {
                return s;
            }
        }
    }

    #[tokio::test]
    async fn arm_hit_yields_exactly_one_shot_with_planned_pulses() {
        let SimRig {
            mut link, golfer, ..
        } = SimRig::start(fast());
        let cfg = StrobeConfig {
            pulse_count: 3,
            pulse_width_us: 50,
            pulse_spacing_us: 1_200,
            trigger_delay_us: 300,
        };
        let r = link
            .request(
                HostToDevice::SetConfig(cfg),
                Duration::from_millis(200),
                "cfg",
                |_| true,
            )
            .await
            .unwrap();
        assert_eq!(r, DeviceToHost::ConfigApplied(cfg));
        link.request(HostToDevice::Arm, Duration::from_millis(200), "arm", |_| {
            true
        })
        .await
        .unwrap();
        golfer.hit(TRUTH).unwrap();
        let shot = next_shot(&mut link).await;
        assert_eq!(shot.shot_id, 1);
        assert_eq!(shot.pulses(), &[0, 1_200, 2_400]);
        // Hits during lockout are ignored: nothing more arrives.
        golfer.hit(TRUTH).unwrap();
        assert!(tokio::time::timeout(Duration::from_millis(30), link.recv())
            .await
            .is_err());
    }

    #[tokio::test]
    async fn hit_while_disarmed_or_locked_out_does_nothing() {
        let SimRig {
            mut link,
            golfer,
            mut camera,
            ..
        } = SimRig::start(fast());
        golfer.hit(TRUTH).unwrap(); // disarmed
        assert!(tokio::time::timeout(Duration::from_millis(50), link.recv())
            .await
            .is_err());
        assert!(
            tokio::time::timeout(Duration::from_millis(20), camera.next_frame())
                .await
                .is_err(),
            "no frame without a real trigger"
        );

        link.request(HostToDevice::Arm, Duration::from_millis(200), "arm", |_| {
            true
        })
        .await
        .unwrap();
        golfer.hit(TRUTH).unwrap();
        next_shot(&mut link).await;
        // Locked out for 40 ms; a hit now must not produce a second shot.
        golfer.hit(TRUTH).unwrap();
        assert!(tokio::time::timeout(Duration::from_millis(25), link.recv())
            .await
            .is_err());
        // After the lockout it re-arms by itself.
        tokio::time::sleep(Duration::from_millis(60)).await;
        golfer.hit(TRUTH).unwrap();
        assert_eq!(next_shot(&mut link).await.shot_id, 2);
    }

    #[tokio::test]
    async fn invalid_config_is_rejected_by_the_real_sequencer() {
        let SimRig { mut link, .. } = SimRig::start(fast());
        let bad = StrobeConfig {
            pulse_count: 0,
            ..StrobeConfig::default()
        };
        let r = link
            .request(
                HostToDevice::SetConfig(bad),
                Duration::from_millis(200),
                "cfg",
                |_| true,
            )
            .await
            .unwrap();
        assert_eq!(r, DeviceToHost::ConfigRejected(ConfigError::PulseCount));
    }

    #[tokio::test]
    async fn device_task_ends_when_host_drops_the_link() {
        let SimRig { link, device, .. } = SimRig::start(fast());
        drop(link);
        tokio::time::timeout(Duration::from_millis(200), device)
            .await
            .expect("device should exit")
            .unwrap();
    }

    #[tokio::test]
    async fn camera_honours_exposure_window() {
        let SimRig {
            mut link,
            golfer,
            mut camera,
            ..
        } = SimRig::start(fast());
        link.request(HostToDevice::Arm, Duration::from_millis(200), "arm", |_| {
            true
        })
        .await
        .unwrap();
        // Default sequence: flashes end at 50, 1050, 2050 us.
        camera.set_exposure_us(1_500).await.unwrap();
        golfer.hit(TRUTH).unwrap();
        let short = camera.next_frame().await.unwrap();
        tokio::time::sleep(Duration::from_millis(60)).await; // lockout
        camera.set_exposure_us(2_550).await.unwrap();
        golfer.hit(TRUTH).unwrap();
        let full = camera.next_frame().await.unwrap();

        let count = |img: &vision::GrayImage| vision::detect_balls(img, &Default::default()).len();
        assert_eq!(count(&short.image), 2);
        assert_eq!(count(&full.image), 3);
    }

    #[test]
    fn visible_pulses_prefix_relative_to_first_flash() {
        let mut starts = [0u32; protocol::MAX_PULSES];
        starts[..3].copy_from_slice(&[300, 1_300, 2_300]);
        let s = Schedule {
            shot_id: 1,
            pulse_count: 3,
            pulse_start_us: starts,
            pulse_width_us: 50,
            sequence_duration_us: 2_350,
        };
        assert_eq!(visible_pulses(&s, 10_000), vec![0, 1_000, 2_000]);
        assert_eq!(visible_pulses(&s, 1_350), vec![0, 1_000]);
        assert_eq!(visible_pulses(&s, 349), Vec::<u32>::new());
    }
}
