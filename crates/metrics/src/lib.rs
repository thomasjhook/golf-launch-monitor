//! Launch metrics from strobe-captured ball positions and rotations.
//!
//! Conventions (world frame: +X downrange, +Y up, +Z right, right-handed):
//! * `ball_speed_mps = |v|`, `launch_angle_deg = atan2(vy, hypot(vx, vz))`,
//!   `azimuth_deg = atan2(vz, vx)` (positive = right).
//! * Angular velocity `omega` (rad/s) = mean of `axis_unit * angle / dt`.
//! * Let `h` = horizontal unit travel direction `(vx, 0, vz)/|..|`
//!   (falls back to +X if the ball goes straight up/down).
//!   Backspin axis `b = h x up` (for h = +X this is +Z). Positive rotation
//!   about `b` moves the ball's top backward, so
//!   `backspin_rpm = omega . b` in rpm.
//! * `sidespin_rpm = -omega_y` in rpm: clockwise seen from above (rotation
//!   about -Y) curves the ball right and is positive.

use launch_types::{BallRotation, BallState, LaunchConditions, Vec3};

#[derive(Debug, thiserror::Error)]
pub enum MetricsError {
    #[error("not enough samples (need at least 2 ball states)")]
    NotEnoughSamples,
    #[error("timestamps must be strictly increasing")]
    NonMonotonicTime,
    #[error("ball speed is ~0; direction undefined")]
    DegenerateMotion,
    #[error("non-finite input value")]
    NonFinite,
    #[error("invalid rotation measurement (dt <= 0 or zero axis with nonzero angle)")]
    InvalidRotation,
}

const MIN_SPEED_MPS: f64 = 1e-6;
const RAD_S_TO_RPM: f64 = 60.0 / (2.0 * std::f64::consts::PI);

fn validate(states: &[BallState]) -> Result<(), MetricsError> {
    if states.len() < 2 {
        return Err(MetricsError::NotEnoughSamples);
    }
    for s in states {
        let p = s.position_m;
        if !(s.t_s.is_finite() && p.x.is_finite() && p.y.is_finite() && p.z.is_finite()) {
            return Err(MetricsError::NonFinite);
        }
    }
    if states.windows(2).any(|w| w[1].t_s <= w[0].t_s) {
        return Err(MetricsError::NonMonotonicTime);
    }
    Ok(())
}

/// Least-squares velocity (m/s) from position-vs-time line fits per axis.
/// Does not check for zero speed.
fn fit_velocity(states: &[BallState]) -> Result<Vec3, MetricsError> {
    validate(states)?;
    let n = states.len() as f64;
    let tm = states.iter().map(|s| s.t_s).sum::<f64>() / n;
    let mut stt = 0.0;
    let (mut sx, mut sy, mut sz) = (0.0, 0.0, 0.0);
    let (mut mx, mut my, mut mz) = (0.0, 0.0, 0.0);
    for s in states {
        mx += s.position_m.x / n;
        my += s.position_m.y / n;
        mz += s.position_m.z / n;
    }
    for s in states {
        let dt = s.t_s - tm;
        stt += dt * dt;
        sx += dt * (s.position_m.x - mx);
        sy += dt * (s.position_m.y - my);
        sz += dt * (s.position_m.z - mz);
    }
    if stt <= 0.0 || !stt.is_finite() {
        return Err(MetricsError::NonMonotonicTime);
    }
    let v = Vec3::new(sx / stt, sy / stt, sz / stt);
    if !(v.x.is_finite() && v.y.is_finite() && v.z.is_finite()) {
        return Err(MetricsError::NonFinite);
    }
    Ok(v)
}

/// Ball velocity vector (m/s, world frame): least-squares slope of position vs time.
pub fn ball_velocity(states: &[BallState]) -> Result<Vec3, MetricsError> {
    let v = fit_velocity(states)?;
    if v.norm() < MIN_SPEED_MPS {
        return Err(MetricsError::DegenerateMotion);
    }
    Ok(v)
}

/// Mean angular velocity (rad/s, world frame) of the rotation measurements.
fn mean_omega(rotations: &[BallRotation]) -> Result<Vec3, MetricsError> {
    let mut sum = Vec3::ZERO;
    for r in rotations {
        let a = r.axis;
        if !(r.dt_s.is_finite()
            && r.angle_rad.is_finite()
            && a.x.is_finite()
            && a.y.is_finite()
            && a.z.is_finite())
        {
            return Err(MetricsError::NonFinite);
        }
        if r.dt_s <= 0.0 {
            return Err(MetricsError::InvalidRotation);
        }
        let n = a.norm();
        if n <= 0.0 {
            if r.angle_rad == 0.0 {
                // Zero-angle, zero-axis measurement: contributes zero omega (still counted).
                continue;
            }
            return Err(MetricsError::InvalidRotation);
        }
        let k = r.angle_rad / r.dt_s / n;
        sum = Vec3::new(sum.x + a.x * k, sum.y + a.y * k, sum.z + a.z * k);
    }
    let c = rotations.len() as f64;
    Ok(Vec3::new(sum.x / c, sum.y / c, sum.z / c))
}

/// Compute launch conditions. Spin is zero if `rotations` is empty.
pub fn launch_conditions(
    states: &[BallState],
    rotations: &[BallRotation],
) -> Result<LaunchConditions, MetricsError> {
    let v = ball_velocity(states)?;
    let speed = v.norm();
    let horiz = v.x.hypot(v.z);
    let launch = v.y.atan2(horiz).to_degrees();
    let azimuth = v.z.atan2(v.x).to_degrees();

    let (backspin_rpm, sidespin_rpm) = if rotations.is_empty() {
        (0.0, 0.0)
    } else {
        let w = mean_omega(rotations)?;
        let (hx, hz) = if horiz > 1e-12 {
            (v.x / horiz, v.z / horiz)
        } else {
            (1.0, 0.0)
        };
        // b = h x up with h = (hx,0,hz), up = (0,1,0)  =>  b = (-hz, 0, hx)
        let back = (-hz * w.x + hx * w.z) * RAD_S_TO_RPM;
        let side = -w.y * RAD_S_TO_RPM;
        (back, side)
    };

    Ok(LaunchConditions {
        ball_speed_mps: speed,
        launch_angle_deg: launch,
        azimuth_deg: azimuth,
        backspin_rpm,
        sidespin_rpm,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn states_for(speed: f64, launch_deg: f64, az_deg: f64, n: usize, dt: f64) -> Vec<BallState> {
        let (l, a) = (launch_deg.to_radians(), az_deg.to_radians());
        let v = Vec3::new(
            speed * l.cos() * a.cos(),
            speed * l.sin(),
            speed * l.cos() * a.sin(),
        );
        (0..n)
            .map(|i| {
                let t = i as f64 * dt;
                BallState {
                    t_s: t,
                    position_m: Vec3::new(0.1 + v.x * t, 0.02 + v.y * t, -0.05 + v.z * t),
                }
            })
            .collect()
    }

    /// Build rotation from omega vector over dt.
    fn rot(omega: Vec3, dt: f64) -> BallRotation {
        let m = omega.norm();
        BallRotation {
            dt_s: dt,
            axis: Vec3::new(omega.x / m, omega.y / m, omega.z / m),
            angle_rad: m * dt,
        }
    }

    /// omega from backspin/sidespin rpm for a given azimuth (deg).
    fn omega_for(back_rpm: f64, side_rpm: f64, az_deg: f64) -> Vec3 {
        let a = az_deg.to_radians();
        let (hx, hz) = (a.cos(), a.sin());
        let k = 2.0 * std::f64::consts::PI / 60.0;
        // b = h x up = (-hz, 0, hx)
        Vec3::new(-hz * back_rpm * k, -side_rpm * k, hx * back_rpm * k)
    }

    #[test]
    fn round_trip_kinematics() {
        for &(s, l, a) in &[(70.0, 12.0, 3.0), (30.0, 45.0, -10.0), (80.0, -2.0, 20.0)] {
            let lc = launch_conditions(&states_for(s, l, a, 5, 0.5e-3), &[]).unwrap();
            assert!((lc.ball_speed_mps - s).abs() < 1e-6);
            assert!((lc.launch_angle_deg - l).abs() < 1e-6);
            assert!((lc.azimuth_deg - a).abs() < 1e-6);
            assert_eq!(lc.backspin_rpm, 0.0);
            assert_eq!(lc.sidespin_rpm, 0.0);
        }
        // exact for two points
        let lc = launch_conditions(&states_for(60.0, 10.0, 5.0, 2, 1e-3), &[]).unwrap();
        assert!((lc.ball_speed_mps - 60.0).abs() < 1e-6);
    }

    #[test]
    fn noisy_speed_within_one_percent() {
        let mut seed: u64 = 12345;
        let mut next = || {
            // sum of 4 uniforms in [-1,1], scaled -> ~Gaussian, max ~±0.2mm
            let mut acc = 0.0;
            for _ in 0..4 {
                seed = seed
                    .wrapping_mul(6364136223846793005)
                    .wrapping_add(1442695040888963407);
                acc += ((seed >> 11) as f64 / (1u64 << 53) as f64) * 2.0 - 1.0;
            }
            acc / 4.0 * 0.2e-3
        };
        let mut st = states_for(65.0, 14.0, -4.0, 6, 1e-3);
        for s in st.iter_mut() {
            s.position_m.x += next();
            s.position_m.y += next();
            s.position_m.z += next();
        }
        let lc = launch_conditions(&st, &[]).unwrap();
        assert!((lc.ball_speed_mps - 65.0).abs() / 65.0 < 0.01);
        assert!((lc.launch_angle_deg - 14.0).abs() < 0.5);
    }

    #[test]
    fn spin_round_trip_and_signs() {
        for &az in &[0.0, 7.0, -15.0] {
            let st = states_for(60.0, 15.0, az, 4, 0.5e-3);
            let w = omega_for(3000.0, 500.0, az);
            let lc = launch_conditions(&st, &[rot(w, 1e-3)]).unwrap();
            assert!((lc.backspin_rpm - 3000.0).abs() < 30.0);
            assert!((lc.sidespin_rpm - 500.0).abs() < 5.0);
        }
    }

    #[test]
    fn explicit_sign_conventions() {
        let st = states_for(60.0, 10.0, 0.0, 3, 1e-3);
        // Ball moving +X, omega = +Z: top (+Y) velocity = omega x r = (-w,0,0): backward.
        let w = Vec3::new(0.0, 0.0, 300.0);
        let top = Vec3::new(w.y * 0.0 - w.z * 1.0, 0.0, 0.0);
        assert!(top.x < 0.0);
        let lc = launch_conditions(&st, &[rot(w, 1e-3)]).unwrap();
        assert!(lc.backspin_rpm > 0.0 && lc.sidespin_rpm.abs() < 1e-9);
        // omega = -Y is clockwise from above => curves right => positive sidespin.
        let lc = launch_conditions(&st, &[rot(Vec3::new(0.0, -300.0, 0.0), 1e-3)]).unwrap();
        assert!(lc.sidespin_rpm > 0.0 && lc.backspin_rpm.abs() < 1e-9);
        let lc = launch_conditions(&st, &[rot(Vec3::new(0.0, 300.0, 0.0), 1e-3)]).unwrap();
        assert!(lc.sidespin_rpm < 0.0);
    }

    #[test]
    fn rotations_are_averaged() {
        let st = states_for(60.0, 10.0, 0.0, 3, 1e-3);
        let r1 = rot(omega_for(2800.0, 0.0, 0.0), 1e-3);
        let r2 = rot(omega_for(3200.0, 400.0, 0.0), 2e-3);
        let lc = launch_conditions(&st, &[r1, r2]).unwrap();
        assert!((lc.backspin_rpm - 3000.0).abs() < 1.0);
        assert!((lc.sidespin_rpm - 200.0).abs() < 1.0);
    }

    #[test]
    fn error_cases() {
        let ok = states_for(50.0, 10.0, 0.0, 3, 1e-3);
        assert!(matches!(
            launch_conditions(&[], &[]),
            Err(MetricsError::NotEnoughSamples)
        ));
        assert!(matches!(
            launch_conditions(&ok[..1], &[]),
            Err(MetricsError::NotEnoughSamples)
        ));
        let mut eq = ok.clone();
        eq[1].t_s = eq[0].t_s;
        assert!(matches!(
            ball_velocity(&eq),
            Err(MetricsError::NonMonotonicTime)
        ));
        let mut dec = ok.clone();
        dec.reverse();
        assert!(matches!(
            ball_velocity(&dec),
            Err(MetricsError::NonMonotonicTime)
        ));
        let still: Vec<BallState> = (0..3)
            .map(|i| BallState {
                t_s: i as f64 * 1e-3,
                position_m: Vec3::new(1.0, 2.0, 3.0),
            })
            .collect();
        assert!(matches!(
            ball_velocity(&still),
            Err(MetricsError::DegenerateMotion)
        ));
        let mut nan = ok.clone();
        nan[1].position_m.y = f64::NAN;
        assert!(matches!(ball_velocity(&nan), Err(MetricsError::NonFinite)));
        let mut nant = ok.clone();
        nant[1].t_s = f64::NAN;
        assert!(ball_velocity(&nant).is_err());
        let bad = BallRotation {
            dt_s: 0.0,
            axis: Vec3::new(0.0, 0.0, 1.0),
            angle_rad: 0.1,
        };
        assert!(matches!(
            launch_conditions(&ok, &[bad]),
            Err(MetricsError::InvalidRotation)
        ));
        let nanr = BallRotation {
            dt_s: 1e-3,
            axis: Vec3::new(f64::NAN, 0.0, 1.0),
            angle_rad: 0.1,
        };
        assert!(launch_conditions(&ok, &[nanr]).is_err());
    }
}
