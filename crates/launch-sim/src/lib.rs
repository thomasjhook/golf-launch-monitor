//! Synthetic shot generation for tests and the replay client.
//!
//! Renders a multi-exposure frame from known launch conditions using a fixed
//! bench camera, so pipeline output can be checked against ground truth.
//!
//! The bench camera sits 0.6 m to the golfer's left of the ball path, looking
//! across it. Image-right is world -X, so the ball travels leftward and flash
//! order is x-descending.

use calibration::{look_at_pose, Calibration};
use launch_types::{Intrinsics, Vec3, BALL_RADIUS_M};
use vision::synth::{render, SynthBall, SynthParams};
use vision::GrayImage;

pub const WIDTH: u32 = 1280;
pub const HEIGHT: u32 = 1024;
pub const FOCAL_PX: f64 = 1600.0;

/// Ground-truth launch used to render a frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Truth {
    pub speed_mps: f64,
    pub launch_angle_deg: f64,
    pub azimuth_deg: f64,
}

/// Calibration of the bench camera, with the given Brown-Conrady distortion.
pub fn bench_calibration(distortion: [f64; 5]) -> Calibration {
    let intrinsics = Intrinsics {
        width: WIDTH,
        height: HEIGHT,
        fx: FOCAL_PX,
        fy: FOCAL_PX,
        cx: f64::from(WIDTH) / 2.0,
        cy: f64::from(HEIGHT) / 2.0,
        distortion,
    };
    let pose = look_at_pose(
        Vec3::new(0.12, 0.08, -0.6),
        Vec3::new(0.12, 0.08, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
    );
    Calibration::new(intrinsics, pose).expect("bench calibration is valid")
}

/// Moderate sensor noise, blur, and a few hot pixels.
pub fn default_noise() -> SynthParams {
    SynthParams {
        noise_sigma: 3.0,
        background: 12,
        seed: 7,
        blur_sigma_px: 0.8,
        hot_pixels: 5,
    }
}

/// Project a world point into the (distorted) image, returning the pixel
/// centre and area-equivalent radius of the ball's image.
///
/// A sphere seen off-axis images as an ellipse whose centre is shifted away
/// from the projected sphere centre and whose area-equivalent radius is larger
/// than the on-axis value. This uses the exact tangent-cone geometry (on the
/// z=1 plane) so the synthetic frame matches real optics.
fn project_ball(cal: &Calibration, p: Vec3) -> ([f64; 2], f64) {
    let r = &cal.pose.rotation;
    let t = cal.pose.translation_m;
    let d = Vec3::new(p.x - t.x, p.y - t.y, p.z - t.z);
    // p_cam = R^T * (p - t)
    let cam = Vec3::new(
        r[0][0] * d.x + r[1][0] * d.y + r[2][0] * d.z,
        r[0][1] * d.x + r[1][1] * d.y + r[2][1] * d.z,
        r[0][2] * d.x + r[1][2] * d.y + r[2][2] * d.z,
    );
    let (nx, ny) = (cam.x / cam.z, cam.y / cam.z);
    let t = nx.hypot(ny);
    let tt = (BALL_RADIUS_M / cam.norm()).asin().tan();
    let dd = 1.0 - t * t * tt * tt;
    let m = t * (1.0 + tt * tt) / dd;
    let a = tt * (1.0 + t * t) / dd;
    let b = tt * (1.0 + t * t).sqrt() / dd.sqrt();
    let s = (a * b).sqrt();
    let k = if t > 0.0 { m / t } else { 1.0 };
    let (cx, cy) = (nx * k, ny * k);
    let proj = |x: f64, y: f64| {
        cal.project(Vec3::new(x, y, 1.0))
            .expect("point in front of camera")
    };
    let px = proj(cx, cy);
    let radius_px = [(s, 0.0), (-s, 0.0), (0.0, s), (0.0, -s)]
        .iter()
        .map(|(dx, dy)| {
            let q = proj(cx + dx, cy + dy);
            (q[0] - px[0]).hypot(q[1] - px[1])
        })
        .sum::<f64>()
        / 4.0;
    (px, radius_px)
}

/// Render one multi-exposure frame: the ball starts at the bench tee position
/// and is imaged at each of `pulses_us` (microseconds since flash 0).
///
/// Flash spacing must exceed ball diameter / ball speed or the images overlap.
pub fn render_shot(
    cal: &Calibration,
    truth: &Truth,
    pulses_us: &[u32],
    params: &SynthParams,
) -> GrayImage {
    let (la, az) = (
        truth.launch_angle_deg.to_radians(),
        truth.azimuth_deg.to_radians(),
    );
    let v = Vec3::new(
        truth.speed_mps * la.cos() * az.cos(),
        truth.speed_mps * la.sin(),
        truth.speed_mps * la.cos() * az.sin(),
    );
    let start = Vec3::new(0.18, 0.06, 0.0);
    let balls: Vec<SynthBall> = pulses_us
        .iter()
        .map(|&t_us| {
            let t = f64::from(t_us) * 1e-6;
            let p = Vec3::new(start.x + v.x * t, start.y + v.y * t, start.z + v.z * t);
            let (center_px, radius_px) = project_ball(cal, p);
            SynthBall {
                center_px,
                radius_px,
                brightness: 230,
            }
        })
        .collect();
    render(WIDTH, HEIGHT, &balls, params)
}
