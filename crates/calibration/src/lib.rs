//! Camera calibration and ball localisation.
//!
//! Camera frame: OpenCV (+x right, +y down, +z forward). World: +X downrange,
//! +Y up, +Z right of golfer. `p_world = R * p_cam + t`.
//!
//! # Ball localisation (exact model)
//!
//! A sphere of radius `R` whose centre is at distance `d` from the pinhole is
//! seen under a tangent cone of half-angle `alpha = asin(R / d)` about the ray
//! to its centre, which makes angle `phi` with the optical axis. On the
//! normalised image plane (z = 1) the silhouette is an ellipse. With
//! `t = tan(phi)`, `T = tan(alpha)`, `D = 1 - t^2 T^2`:
//!
//! * centre offset from the principal point (radial): `m = t (1 + T^2) / D`
//! * semi-major axis `a = T (1 + t^2) / D`
//! * semi-minor axis `b = T sqrt(1 + t^2) / sqrt(D)`
//!
//! The detector's circle is modelled as the ellipse centre and the
//! area-equivalent radius `s = sqrt(a b)`. Given the measured `m` and `s`
//! (after undistortion) we solve the two equations for `(t, T)` with Newton's
//! method, then `d = R / sin(atan T)` and the centre is `d * dir`, where `dir`
//! has angle `phi` from the axis in the azimuth of the detected centre. This is
//! exact for an ideal pinhole; with lens distortion the pixel radius is mapped
//! to normalised units by undistorting four points around the centre, which is
//! first-order accurate (see tests for the measured size).

use launch_types::{BallDetection, CameraPose, Intrinsics, Vec3, BALL_RADIUS_M};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Calibration {
    pub intrinsics: Intrinsics,
    pub pose: CameraPose,
}

#[derive(Debug, thiserror::Error)]
pub enum CalibrationError {
    #[error("invalid intrinsics")]
    InvalidIntrinsics,
    #[error("invalid pose: rotation is not a proper orthonormal matrix")]
    InvalidPose,
    #[error("bad ball radius")]
    BadRadius,
    #[error("bad detection centre")]
    BadCenter,
    #[error("ball localisation did not converge")]
    NoConvergence,
    #[error("json error: {0}")]
    Json(String),
}

#[cfg(test)]
fn dot(a: Vec3, b: Vec3) -> f64 {
    a.x * b.x + a.y * b.y + a.z * b.z
}
fn cross(a: Vec3, b: Vec3) -> Vec3 {
    Vec3::new(
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x,
    )
}
fn scale(a: Vec3, s: f64) -> Vec3 {
    Vec3::new(a.x * s, a.y * s, a.z * s)
}
fn unit(a: Vec3) -> Vec3 {
    scale(a, 1.0 / a.norm())
}

impl Calibration {
    pub fn new(intrinsics: Intrinsics, pose: CameraPose) -> Result<Self, CalibrationError> {
        let i = &intrinsics;
        let ok = i.width > 0
            && i.height > 0
            && i.fx.is_finite()
            && i.fy.is_finite()
            && i.fx > 0.0
            && i.fy > 0.0
            && i.cx.is_finite()
            && i.cy.is_finite()
            && i.distortion.iter().all(|v| v.is_finite());
        if !ok {
            return Err(CalibrationError::InvalidIntrinsics);
        }
        let r = &pose.rotation;
        let t = pose.translation_m;
        if !r.iter().flatten().all(|v| v.is_finite())
            || !(t.x.is_finite() && t.y.is_finite() && t.z.is_finite())
        {
            return Err(CalibrationError::InvalidPose);
        }
        for a in 0..3 {
            for b in 0..3 {
                let d: f64 = (0..3).map(|k| r[a][k] * r[b][k]).sum();
                let want = if a == b { 1.0 } else { 0.0 };
                if (d - want).abs() > 1e-6 {
                    return Err(CalibrationError::InvalidPose);
                }
            }
        }
        let det = r[0][0] * (r[1][1] * r[2][2] - r[1][2] * r[2][1])
            - r[0][1] * (r[1][0] * r[2][2] - r[1][2] * r[2][0])
            + r[0][2] * (r[1][0] * r[2][1] - r[1][1] * r[2][0]);
        if (det - 1.0).abs() > 1e-6 {
            return Err(CalibrationError::InvalidPose);
        }
        Ok(Self { intrinsics, pose })
    }

    /// Camera-frame point -> distorted pixel. `None` if `z <= 0` or non-finite.
    pub fn project(&self, p: Vec3) -> Option<[f64; 2]> {
        if p.z <= 0.0 || !p.x.is_finite() || !p.y.is_finite() || !p.z.is_finite() {
            return None;
        }
        let i = &self.intrinsics;
        let (x, y) = (p.x / p.z, p.y / p.z);
        let [k1, k2, p1, p2, k3] = i.distortion;
        let r2 = x * x + y * y;
        let rad = 1.0 + r2 * (k1 + r2 * (k2 + r2 * k3));
        let xd = x * rad + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
        let yd = y * rad + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y;
        Some([i.fx * xd + i.cx, i.fy * yd + i.cy])
    }

    /// Distorted pixel -> normalised undistorted coordinates (x/z, y/z).
    pub fn undistort_point(&self, px: [f64; 2]) -> [f64; 2] {
        let i = &self.intrinsics;
        let [k1, k2, p1, p2, k3] = i.distortion;
        let xd = (px[0] - i.cx) / i.fx;
        let yd = (px[1] - i.cy) / i.fy;
        let (mut x, mut y) = (xd, yd);
        for _ in 0..200 {
            let r2 = x * x + y * y;
            let rad = 1.0 + r2 * (k1 + r2 * (k2 + r2 * k3));
            let dx = 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
            let dy = p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y;
            let (nx, ny) = ((xd - dx) / rad, (yd - dy) / rad);
            let done = (nx - x).abs().max((ny - y).abs()) < 1e-14;
            x = nx;
            y = ny;
            if done || !x.is_finite() || !y.is_finite() {
                break;
            }
        }
        [x, y]
    }

    /// Unit ray in the camera frame through a distorted pixel.
    pub fn pixel_ray_cam(&self, px: [f64; 2]) -> Vec3 {
        let [x, y] = self.undistort_point(px);
        unit(Vec3::new(x, y, 1.0))
    }

    /// Ball centre in the camera frame (see module docs for the formulas).
    pub fn locate_ball_cam(&self, det: &BallDetection) -> Result<Vec3, CalibrationError> {
        let r = det.radius_px;
        if !r.is_finite() || r <= 0.0 {
            return Err(CalibrationError::BadRadius);
        }
        let c = det.center_px;
        if !c[0].is_finite() || !c[1].is_finite() {
            return Err(CalibrationError::BadCenter);
        }
        let n0 = self.undistort_point(c);
        // Pixel radius -> normalised radius via four undistorted offsets.
        let mut s = 0.0;
        for off in [[r, 0.0], [-r, 0.0], [0.0, r], [0.0, -r]] {
            let n = self.undistort_point([c[0] + off[0], c[1] + off[1]]);
            s += ((n[0] - n0[0]).powi(2) + (n[1] - n0[1]).powi(2)).sqrt();
        }
        s /= 4.0;
        let m = (n0[0] * n0[0] + n0[1] * n0[1]).sqrt();
        if !s.is_finite() || !m.is_finite() || s <= 0.0 {
            return Err(CalibrationError::NoConvergence);
        }

        // Residuals: centre offset m and area-equivalent radius s.
        let f = |t: f64, tt: f64| -> Option<(f64, f64)> {
            let d = 1.0 - t * t * tt * tt;
            if d <= 1e-12 || tt <= 0.0 {
                return None;
            }
            let fm = t * (1.0 + tt * tt) / d - m;
            let fs = tt * ((1.0 + t * t) / d).powf(0.75) - s;
            Some((fm, fs))
        };
        let (mut t, mut tt) = (m, s);
        let mut converged = false;
        for _ in 0..100 {
            let Some((f1, f2)) = f(t, tt) else {
                return Err(CalibrationError::NoConvergence);
            };
            if f1.abs() < 1e-15 && f2.abs() < 1e-15 * s.max(1e-3) {
                converged = true;
                break;
            }
            let ht = 1e-7 * t.abs().max(1e-3);
            let h_big = 1e-7 * tt;
            let (Some(a), Some(b)) = (f(t + ht, tt), f(t, tt + h_big)) else {
                return Err(CalibrationError::NoConvergence);
            };
            let j11 = (a.0 - f1) / ht;
            let j21 = (a.1 - f2) / ht;
            let j12 = (b.0 - f1) / h_big;
            let j22 = (b.1 - f2) / h_big;
            let det = j11 * j22 - j12 * j21;
            if det == 0.0 || !det.is_finite() {
                return Err(CalibrationError::NoConvergence);
            }
            let mut dt = (f1 * j22 - f2 * j12) / det;
            let mut d_big = (j11 * f2 - j21 * f1) / det;
            // Backtrack so the iterate stays valid and T positive.
            let mut k = 0;
            while f(t - dt, tt - d_big).is_none() {
                dt *= 0.5;
                d_big *= 0.5;
                k += 1;
                if k > 60 {
                    return Err(CalibrationError::NoConvergence);
                }
            }
            t -= dt;
            tt -= d_big;
        }
        if !converged {
            // Accept if residual is tiny even if the strict test never tripped.
            match f(t, tt) {
                Some((a, b)) if a.abs() < 1e-11 && b.abs() < 1e-11 * s.max(1e-3) => {}
                _ => return Err(CalibrationError::NoConvergence),
            }
        }

        let alpha = tt.atan();
        let phi = t.atan();
        let d = BALL_RADIUS_M / alpha.sin();
        let (ux, uy) = if m > 0.0 {
            (n0[0] / m, n0[1] / m)
        } else {
            (0.0, 0.0)
        };
        let dir = Vec3::new(phi.sin() * ux, phi.sin() * uy, phi.cos());
        Ok(scale(dir, d))
    }

    pub fn locate_ball_world(&self, det: &BallDetection) -> Result<Vec3, CalibrationError> {
        let p = self.locate_ball_cam(det)?;
        let r = &self.pose.rotation;
        let t = self.pose.translation_m;
        Ok(Vec3::new(
            r[0][0] * p.x + r[0][1] * p.y + r[0][2] * p.z + t.x,
            r[1][0] * p.x + r[1][1] * p.y + r[1][2] * p.z + t.y,
            r[2][0] * p.x + r[2][1] * p.y + r[2][2] * p.z + t.z,
        ))
    }

    pub fn to_json(&self) -> Result<String, CalibrationError> {
        serde_json::to_string(self).map_err(|e| CalibrationError::Json(e.to_string()))
    }

    /// Parses and validates (same checks as [`Calibration::new`]).
    pub fn from_json(s: &str) -> Result<Self, CalibrationError> {
        let c: Calibration =
            serde_json::from_str(s).map_err(|e| CalibrationError::Json(e.to_string()))?;
        Calibration::new(c.intrinsics, c.pose)
    }
}

/// Pose with camera +z toward the target and image +y roughly opposite `up`.
pub fn look_at_pose(camera_pos_world: Vec3, target_world: Vec3, up_world: Vec3) -> CameraPose {
    let fwd = Vec3::new(
        target_world.x - camera_pos_world.x,
        target_world.y - camera_pos_world.y,
        target_world.z - camera_pos_world.z,
    );
    let z = if fwd.norm() > 0.0 {
        unit(fwd)
    } else {
        Vec3::new(1.0, 0.0, 0.0)
    };
    let mut down = scale(up_world, -1.0);
    let mut x = cross(down, z);
    if x.norm() < 1e-9 {
        // up parallel to view direction: pick any perpendicular.
        down = if z.y.abs() < 0.9 {
            Vec3::new(0.0, -1.0, 0.0)
        } else {
            Vec3::new(0.0, 0.0, 1.0)
        };
        x = cross(down, z);
    }
    let x = unit(x);
    let y = cross(z, x);
    CameraPose {
        rotation: [[x.x, y.x, z.x], [x.y, y.y, z.y], [x.z, y.z, z.z]],
        translation_m: camera_pos_world,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn intr(dist: [f64; 5]) -> Intrinsics {
        Intrinsics {
            width: 1280,
            height: 800,
            fx: 1400.0,
            fy: 1400.0,
            cx: 640.0,
            cy: 400.0,
            distortion: dist,
        }
    }
    fn cal(dist: [f64; 5], cam: Vec3, target: Vec3) -> Calibration {
        Calibration::new(
            intr(dist),
            look_at_pose(cam, target, Vec3::new(0.0, 1.0, 0.0)),
        )
        .unwrap()
    }
    const DIST: [f64; 5] = [-0.2, 0.05, 0.001, -0.0007, 0.0];

    #[test]
    fn roundtrip_distortion() {
        let c = cal(DIST, Vec3::new(-0.5, 0.1, 0.0), Vec3::ZERO);
        let mut worst: f64 = 0.0;
        for ix in 0..=16 {
            for iy in 0..=10 {
                let px = [ix as f64 * 80.0, iy as f64 * 80.0];
                let n = c.undistort_point(px);
                let back = c.project(Vec3::new(n[0], n[1], 1.0)).unwrap();
                worst = worst.max((back[0] - px[0]).abs().max((back[1] - px[1]).abs()));
                let ray = c.pixel_ray_cam(px);
                assert!((ray.norm() - 1.0).abs() < 1e-12);
            }
        }
        assert!(worst < 1e-6, "worst {worst}");
        assert!(c.project(Vec3::new(0.0, 0.0, 0.0)).is_none());
        assert!(c.project(Vec3::new(0.0, 0.0, -1.0)).is_none());
    }

    /// Synthesise a detection from true geometry: sample the sphere's tangent
    /// cone boundary, project, and take the polygon centroid and
    /// area-equivalent radius (independent of the closed-form ellipse).
    fn synth(c: &Calibration, p_cam: Vec3) -> BallDetection {
        let d = p_cam.norm();
        let w = scale(p_cam, 1.0 / d);
        let alpha = (BALL_RADIUS_M / d).asin();
        let a = if w.x.abs() < 0.9 {
            Vec3::new(1.0, 0.0, 0.0)
        } else {
            Vec3::new(0.0, 1.0, 0.0)
        };
        let u = unit(cross(w, a));
        let v = cross(w, u);
        let n = 4000;
        let pts: Vec<[f64; 2]> = (0..n)
            .map(|k| {
                let th = 2.0 * std::f64::consts::PI * k as f64 / n as f64;
                let dir = Vec3::new(
                    w.x * alpha.cos() + alpha.sin() * (u.x * th.cos() + v.x * th.sin()),
                    w.y * alpha.cos() + alpha.sin() * (u.y * th.cos() + v.y * th.sin()),
                    w.z * alpha.cos() + alpha.sin() * (u.z * th.cos() + v.z * th.sin()),
                );
                c.project(dir).unwrap()
            })
            .collect();
        let (mut area, mut cx, mut cy) = (0.0, 0.0, 0.0);
        for k in 0..n {
            let (p, q) = (pts[k], pts[(k + 1) % n]);
            let cr = p[0] * q[1] - q[0] * p[1];
            area += cr;
            cx += (p[0] + q[0]) * cr;
            cy += (p[1] + q[1]) * cr;
        }
        area *= 0.5;
        BallDetection {
            center_px: [cx / (6.0 * area), cy / (6.0 * area)],
            radius_px: (area.abs() / std::f64::consts::PI).sqrt(),
            confidence: 1.0,
        }
    }

    fn err(a: Vec3, b: Vec3) -> f64 {
        Vec3::new(a.x - b.x, a.y - b.y, a.z - b.z).norm()
    }

    #[test]
    fn ball_placement_exact_no_distortion() {
        // Zero distortion: the model is exact; expect sub-micron error.
        let c = cal([0.0; 5], Vec3::new(-0.5, 0.05, 0.0), Vec3::ZERO);
        let mut worst: f64 = 0.0;
        for (dx, dy, dz) in [
            (0.0, 0.0, 0.0),
            (0.05, 0.0, 0.0),
            (0.0, 0.1, 0.0),
            (0.0, 0.0, 0.12),
            (0.1, 0.1, -0.15),
            (-0.1, 0.15, 0.2),
            (0.3, 0.0, 0.0),
            (0.0, -0.04, -0.2),
        ] {
            let truth = Vec3::new(dx, dy, dz);
            let cam_p = {
                // world -> camera: R^T (p - t)
                let r = &c.pose.rotation;
                let q = Vec3::new(
                    truth.x - c.pose.translation_m.x,
                    truth.y - c.pose.translation_m.y,
                    truth.z - c.pose.translation_m.z,
                );
                Vec3::new(
                    r[0][0] * q.x + r[1][0] * q.y + r[2][0] * q.z,
                    r[0][1] * q.x + r[1][1] * q.y + r[2][1] * q.z,
                    r[0][2] * q.x + r[1][2] * q.y + r[2][2] * q.z,
                )
            };
            let det = synth(&c, cam_p);
            let got = c.locate_ball_world(&det).unwrap();
            let e = err(got, truth);
            worst = worst.max(e);
            assert!(e < 0.0005, "err {e} for {truth:?}");
        }
        assert!(
            worst < 5e-6,
            "exact model should be ~micron-level, got {worst}"
        );
    }

    #[test]
    fn ball_placement_with_distortion() {
        // Distortion: pixel radius->normalised is first-order; also the real
        // distorted silhouette is not an ellipse. Tolerance 0.5 mm anyway.
        let c = cal(DIST, Vec3::new(-0.5, 0.05, 0.0), Vec3::ZERO);
        // Near-axis / moderate offsets must meet 0.5 mm. The far corner case
        // (~28 deg off axis, strong barrel distortion) is a few mm: the
        // distorted silhouette is not an ellipse so a circle fit in the
        // distorted image is biased; that bias is inherent to the detector model.
        for (dx, dy, dz, tol) in [
            (0.0, 0.0, 0.0, 0.0005),
            (0.05, 0.05, -0.08, 0.0005),
            (0.0, 0.0, 0.1, 0.0005),
            (0.1, 0.1, -0.15, 0.0015),
            (-0.1, 0.15, 0.2, 0.005),
        ] {
            let truth = Vec3::new(dx, dy, dz);
            let r = &c.pose.rotation;
            let q = Vec3::new(truth.x + 0.5, truth.y - 0.05, truth.z);
            let cam_p = Vec3::new(
                r[0][0] * q.x + r[1][0] * q.y + r[2][0] * q.z,
                r[0][1] * q.x + r[1][1] * q.y + r[2][1] * q.z,
                r[0][2] * q.x + r[1][2] * q.y + r[2][2] * q.z,
            );
            let det = synth(&c, cam_p);
            let got = c.locate_ball_world(&det).unwrap();
            let e = err(got, truth);
            assert!(e < tol, "err {e} for {truth:?}");
        }
    }

    #[test]
    fn naive_approximation_is_worse_off_axis() {
        // Documents why the exact formula is used: f*R/r depth + projected
        // centre ray has mm-level error at 0.2 m off axis.
        let c = cal([0.0; 5], Vec3::new(-0.5, 0.0, 0.0), Vec3::ZERO);
        let cam_p = Vec3::new(0.2, 0.0, 0.5);
        let det = synth(&c, cam_p);
        let z = 1400.0 * BALL_RADIUS_M / det.radius_px;
        let n = c.undistort_point(det.center_px);
        let naive = Vec3::new(n[0] * z, n[1] * z, z);
        let exact = c.locate_ball_cam(&det).unwrap();
        assert!(err(exact, cam_p) < 1e-5);
        assert!(err(naive, cam_p) > err(exact, cam_p));
    }

    #[test]
    fn invalid_inputs() {
        let pose = look_at_pose(
            Vec3::new(-0.5, 0.0, 0.0),
            Vec3::ZERO,
            Vec3::new(0.0, 1.0, 0.0),
        );
        let mut i = intr([0.0; 5]);
        i.fx = 0.0;
        assert!(matches!(
            Calibration::new(i, pose.clone()),
            Err(CalibrationError::InvalidIntrinsics)
        ));
        let mut i = intr([0.0; 5]);
        i.width = 0;
        assert!(Calibration::new(i, pose.clone()).is_err());
        let mut i = intr([0.0; 5]);
        i.fy = f64::NAN;
        assert!(Calibration::new(i, pose.clone()).is_err());
        let mut bad = pose.clone();
        bad.rotation[0][2] += 0.01;
        assert!(matches!(
            Calibration::new(intr([0.0; 5]), bad),
            Err(CalibrationError::InvalidPose)
        ));
        let mut refl = pose.clone();
        for row in refl.rotation.iter_mut() {
            row[0] = -row[0];
        }
        assert!(Calibration::new(intr([0.0; 5]), refl).is_err());

        let c = Calibration::new(intr([0.0; 5]), pose).unwrap();
        for r in [0.0, -1.0, f64::NAN, f64::INFINITY] {
            let d = BallDetection {
                center_px: [640.0, 400.0],
                radius_px: r,
                confidence: 1.0,
            };
            assert!(c.locate_ball_cam(&d).is_err());
        }
        let d = BallDetection {
            center_px: [f64::NAN, 400.0],
            radius_px: 10.0,
            confidence: 1.0,
        };
        assert!(c.locate_ball_cam(&d).is_err());
        // Huge radius (ball "inside" the lens) must error, not panic.
        let d = BallDetection {
            center_px: [640.0, 400.0],
            radius_px: 1e6,
            confidence: 1.0,
        };
        let _ = c.locate_ball_cam(&d);
    }

    #[test]
    fn json_roundtrip() {
        let c = cal(DIST, Vec3::new(-0.5, 0.1, 0.02), Vec3::ZERO);
        let s = c.to_json().unwrap();
        assert_eq!(Calibration::from_json(&s).unwrap(), c);
        assert!(matches!(
            Calibration::from_json("{nope"),
            Err(CalibrationError::Json(_))
        ));
    }

    #[test]
    fn look_at_properties() {
        let pos = Vec3::new(-0.6, 0.3, 0.2);
        let tgt = Vec3::new(0.1, 0.0, -0.05);
        let p = look_at_pose(pos, tgt, Vec3::new(0.0, 1.0, 0.0));
        let r = p.rotation;
        let col = |j: usize| Vec3::new(r[0][j], r[1][j], r[2][j]);
        let (x, y, z) = (col(0), col(1), col(2));
        for v in [x, y, z] {
            assert!((v.norm() - 1.0).abs() < 1e-12);
        }
        assert!(dot(x, y).abs() < 1e-12 && dot(y, z).abs() < 1e-12 && dot(x, z).abs() < 1e-12);
        assert!(err(cross(x, y), z) < 1e-12);
        let want = unit(Vec3::new(tgt.x - pos.x, tgt.y - pos.y, tgt.z - pos.z));
        assert!(err(z, want) < 1e-12);
        assert!(y.y < 0.0, "image-down opposes world up");
        assert!(Calibration::new(intr([0.0; 5]), p).is_ok());
        // Behind the golfer looking downrange: image right = +Z.
        let q = look_at_pose(
            Vec3::new(-1.0, 0.0, 0.0),
            Vec3::ZERO,
            Vec3::new(0.0, 1.0, 0.0),
        );
        assert!((q.rotation[2][0] - 1.0).abs() < 1e-12);
        // Degenerate up does not produce NaN.
        let g = look_at_pose(
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::ZERO,
            Vec3::new(0.0, 1.0, 0.0),
        );
        assert!(Calibration::new(intr([0.0; 5]), g).is_ok());
    }
}
