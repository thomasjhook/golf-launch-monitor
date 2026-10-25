//! Render a synthetic multi-exposure shot from known ground truth, run the
//! whole pipeline, and check the recovered launch conditions.

use launch_core::{FlashOrder, Pipeline, PipelineError};
use launch_sim::{bench_calibration, default_noise, render_shot, Truth};
use launch_types::StrobeTiming;

// Spacing must exceed ball diameter / speed (~0.8 ms at 55 m/s) or the ball
// images overlap and can't be separated.
const PULSES_US: [u32; 3] = [0, 1000, 2000];

fn run(distortion: [f64; 5], truth: &Truth) -> launch_types::ShotResult {
    let cal = bench_calibration(distortion);
    let frame = render_shot(&cal, truth, &PULSES_US, &default_noise());
    Pipeline::new(cal, FlashOrder::XDescending)
        .process(
            &frame,
            &StrobeTiming {
                shot_id: 42,
                pulse_times_us: PULSES_US.to_vec(),
            },
        )
        .expect("pipeline succeeds")
}

#[test]
fn recovers_launch_conditions_without_distortion() {
    let truth = Truth {
        speed_mps: 58.0,
        launch_angle_deg: 14.0,
        azimuth_deg: 2.0,
    };
    let r = run([0.0; 5], &truth);
    assert_eq!(r.shot_id, 42);
    assert!(
        (r.launch.ball_speed_mps - truth.speed_mps).abs() / truth.speed_mps < 0.02,
        "{:?}",
        r.launch
    );
    assert!((r.launch.launch_angle_deg - truth.launch_angle_deg).abs() < 1.0);
    assert!((r.launch.azimuth_deg - truth.azimuth_deg).abs() < 1.0);
    assert!(r.flight.carry_m > 100.0, "carry {}", r.flight.carry_m);
}

#[test]
fn recovers_launch_conditions_with_mild_distortion() {
    let truth = Truth {
        speed_mps: 70.0,
        launch_angle_deg: 10.5,
        azimuth_deg: -3.0,
    };
    let r = run([-0.08, 0.01, 0.0005, -0.0003, 0.0], &truth);
    assert!(
        (r.launch.ball_speed_mps - truth.speed_mps).abs() / truth.speed_mps < 0.03,
        "{:?}",
        r.launch
    );
    assert!((r.launch.launch_angle_deg - truth.launch_angle_deg).abs() < 1.5);
    assert!((r.launch.azimuth_deg - truth.azimuth_deg).abs() < 1.5);
}

#[test]
fn flash_count_mismatch_is_an_error() {
    let cal = bench_calibration([0.0; 5]);
    let truth = Truth {
        speed_mps: 50.0,
        launch_angle_deg: 12.0,
        azimuth_deg: 0.0,
    };
    let frame = render_shot(&cal, &truth, &PULSES_US, &default_noise());
    let err = Pipeline::new(cal, FlashOrder::XDescending)
        .process(
            &frame,
            &StrobeTiming {
                shot_id: 1,
                pulse_times_us: vec![0, 1000],
            },
        )
        .unwrap_err();
    assert!(matches!(
        err,
        PipelineError::FlashCountMismatch {
            detected: 3,
            expected: 2
        }
    ));
}
