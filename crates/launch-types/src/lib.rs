//! Shared domain types for the launch monitor pipeline.
//!
//! Units are SI everywhere (metres, seconds, m/s, radians only where a field
//! says so). Angles named `*_deg` are degrees. Spin is in rpm.
//!
//! World frame (right-handed): +X downrange (toward the target), +Y up,
//! +Z to the golfer's right. The origin is the ball's rest position on the
//! tee, at ground level.
//!
//! Camera frame follows the OpenCV convention: +x right in the image,
//! +y down, +z forward out of the lens. Pixel coordinates have the origin at
//! the top-left of the image.

use serde::{Deserialize, Serialize};

/// Regulation golf ball diameter in metres (42.67 mm).
pub const BALL_DIAMETER_M: f64 = 0.04267;
/// Regulation golf ball radius in metres.
pub const BALL_RADIUS_M: f64 = BALL_DIAMETER_M / 2.0;
/// Regulation maximum golf ball mass in kilograms (45.93 g).
pub const BALL_MASS_KG: f64 = 0.04593;

#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
pub struct Vec3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vec3 {
    pub const ZERO: Vec3 = Vec3 {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };
    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }
    pub fn norm(&self) -> f64 {
        (self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }
}

/// Pinhole camera intrinsics with Brown–Conrady distortion
/// `[k1, k2, p1, p2, k3]` (OpenCV ordering).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Intrinsics {
    pub width: u32,
    pub height: u32,
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,
    pub distortion: [f64; 5],
}

/// Rigid transform taking camera-frame points to world-frame points:
/// `p_world = R * p_cam + t`. `rotation` is row-major 3x3.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CameraPose {
    pub rotation: [[f64; 3]; 3],
    pub translation_m: Vec3,
}

/// One ball image found in a captured frame.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct BallDetection {
    /// Centre in (distorted) pixel coordinates.
    pub center_px: [f64; 2],
    pub radius_px: f64,
    /// 0.0..=1.0 detector confidence.
    pub confidence: f64,
}

/// Strobe timing for a single shot as reported by the firmware.
/// `pulse_times_us[i]` is the time of flash `i` relative to flash 0, so
/// `pulse_times_us[0] == 0`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct StrobeTiming {
    pub shot_id: u32,
    pub pulse_times_us: Vec<u32>,
}

/// Ball centre position in the world frame at a given time since flash 0.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct BallState {
    pub t_s: f64,
    pub position_m: Vec3,
}

/// Ball rotation between two flashes, as an axis-angle in the world frame.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct BallRotation {
    pub dt_s: f64,
    pub axis: Vec3,
    pub angle_rad: f64,
}

/// Initial conditions of ball flight, measured at impact.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct LaunchConditions {
    pub ball_speed_mps: f64,
    /// Vertical launch angle above horizontal.
    pub launch_angle_deg: f64,
    /// Horizontal launch direction relative to +X. Positive = to the right.
    pub azimuth_deg: f64,
    /// Backspin about the horizontal axis perpendicular to travel (>= 0 for a
    /// normal shot).
    pub backspin_rpm: f64,
    /// Sidespin about the vertical axis. Positive curves the ball to the right
    /// (a slice for a right-handed golfer).
    pub sidespin_rpm: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct TrajectoryPoint {
    pub t_s: f64,
    pub position_m: Vec3,
}

/// Output of the flight model.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FlightResult {
    pub trajectory: Vec<TrajectoryPoint>,
    /// Horizontal distance to first ground contact.
    pub carry_m: f64,
    pub apex_height_m: f64,
    pub flight_time_s: f64,
    /// Lateral offset (+Z) at landing.
    pub lateral_m: f64,
    /// Carry plus bounce and roll.
    pub total_m: f64,
}

/// A complete processed shot, ready for display and persistence.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ShotResult {
    pub shot_id: u32,
    pub launch: LaunchConditions,
    pub flight: FlightResult,
}
