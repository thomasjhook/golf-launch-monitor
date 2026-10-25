//! Pipeline facade: one multi-exposure frame plus strobe timing in,
//! [`ShotResult`] out.
//!
//! ```text
//! GrayImage ──detect_balls──▶ BallDetection[] ──locate_ball_world──▶ BallState[]
//!   StrobeTiming ───────────────────────────────────────────────────────┘
//!                                   metrics::launch_conditions ──▶ LaunchConditions
//!                                   flight_model::simulate     ──▶ FlightResult
//! ```
//!
//! Spin is not measured yet (no rotation tracking), so `backspin_rpm` and
//! `sidespin_rpm` are zero until that phase lands.

use calibration::{Calibration, CalibrationError};
use flight_model::Environment;
use launch_types::{BallState, ShotResult, StrobeTiming};
use metrics::MetricsError;
use vision::{DetectParams, GrayImage};

/// Which way the ball travels across the image, i.e. how detections sorted by
/// ascending x map to flash order. Depends on where the camera is mounted.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FlashOrder {
    /// Flash 0 is the left-most image in the frame.
    XAscending,
    /// Flash 0 is the right-most image in the frame.
    XDescending,
}

#[derive(Debug, thiserror::Error)]
pub enum PipelineError {
    #[error("detected {detected} ball images but timing reports {expected} flashes")]
    FlashCountMismatch { detected: usize, expected: usize },
    #[error("calibration: {0}")]
    Calibration(#[from] CalibrationError),
    #[error("metrics: {0}")]
    Metrics(#[from] MetricsError),
}

pub struct Pipeline {
    pub calibration: Calibration,
    pub detect: DetectParams,
    pub env: Environment,
    pub flash_order: FlashOrder,
}

impl Pipeline {
    pub fn new(calibration: Calibration, flash_order: FlashOrder) -> Self {
        Self {
            calibration,
            detect: DetectParams::default(),
            env: Environment::default(),
            flash_order,
        }
    }

    pub fn process(
        &self,
        frame: &GrayImage,
        timing: &StrobeTiming,
    ) -> Result<ShotResult, PipelineError> {
        let mut detections = vision::detect_balls(frame, &self.detect);
        if self.flash_order == FlashOrder::XDescending {
            detections.reverse();
        }
        if detections.len() != timing.pulse_times_us.len() {
            return Err(PipelineError::FlashCountMismatch {
                detected: detections.len(),
                expected: timing.pulse_times_us.len(),
            });
        }

        let states = detections
            .iter()
            .zip(&timing.pulse_times_us)
            .map(|(det, &t_us)| {
                Ok(BallState {
                    t_s: f64::from(t_us) * 1e-6,
                    position_m: self.calibration.locate_ball_world(det)?,
                })
            })
            .collect::<Result<Vec<_>, CalibrationError>>()?;

        let launch = metrics::launch_conditions(&states, &[])?;
        let flight = flight_model::simulate(&launch, &self.env);
        Ok(ShotResult {
            shot_id: timing.shot_id,
            launch,
            flight,
        })
    }
}
