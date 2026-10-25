//! Conversions between wire messages ([`crate::pb`]) and domain types.
//!
//! Domain -> wire is infallible. Wire -> domain validates and returns
//! [`ConvertError`], which maps to gRPC `INVALID_ARGUMENT`.

use calibration::Calibration;
use launch_core::FlashOrder;
use launch_types::{
    CameraPose, FlightResult, Intrinsics, LaunchConditions, ShotResult, StrobeTiming,
    TrajectoryPoint, Vec3,
};
use vision::GrayImage;

use crate::pb;

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ConvertError {
    Missing(&'static str),
    BadLength {
        field: &'static str,
        expected: usize,
        actual: usize,
    },
    Invalid(String),
}

impl std::fmt::Display for ConvertError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Missing(field) => write!(f, "missing field `{field}`"),
            Self::BadLength {
                field,
                expected,
                actual,
            } => write!(f, "field `{field}` needs {expected} values, got {actual}"),
            Self::Invalid(msg) => f.write_str(msg),
        }
    }
}

impl std::error::Error for ConvertError {}

impl From<ConvertError> for tonic::Status {
    fn from(e: ConvertError) -> Self {
        tonic::Status::invalid_argument(e.to_string())
    }
}

fn require<T>(field: Option<T>, name: &'static str) -> Result<T, ConvertError> {
    field.ok_or(ConvertError::Missing(name))
}

fn fixed<const N: usize>(v: &[f64], field: &'static str) -> Result<[f64; N], ConvertError> {
    v.try_into().map_err(|_| ConvertError::BadLength {
        field,
        expected: N,
        actual: v.len(),
    })
}

// ---- Vec3

impl From<Vec3> for pb::Vec3 {
    fn from(v: Vec3) -> Self {
        Self {
            x: v.x,
            y: v.y,
            z: v.z,
        }
    }
}

impl From<pb::Vec3> for Vec3 {
    fn from(v: pb::Vec3) -> Self {
        Vec3::new(v.x, v.y, v.z)
    }
}

// ---- Calibration

impl From<&Calibration> for pb::Calibration {
    fn from(c: &Calibration) -> Self {
        let i = &c.intrinsics;
        Self {
            intrinsics: Some(pb::Intrinsics {
                width: i.width,
                height: i.height,
                fx: i.fx,
                fy: i.fy,
                cx: i.cx,
                cy: i.cy,
                distortion: i.distortion.to_vec(),
            }),
            pose: Some(pb::CameraPose {
                rotation: c.pose.rotation.iter().flatten().copied().collect(),
                translation_m: Some(c.pose.translation_m.into()),
            }),
        }
    }
}

impl TryFrom<pb::Calibration> for Calibration {
    type Error = ConvertError;

    fn try_from(c: pb::Calibration) -> Result<Self, ConvertError> {
        let i = require(c.intrinsics, "calibration.intrinsics")?;
        let p = require(c.pose, "calibration.pose")?;
        let flat: [f64; 9] = fixed(&p.rotation, "pose.rotation")?;
        let intrinsics = Intrinsics {
            width: i.width,
            height: i.height,
            fx: i.fx,
            fy: i.fy,
            cx: i.cx,
            cy: i.cy,
            distortion: fixed(&i.distortion, "intrinsics.distortion")?,
        };
        let pose = CameraPose {
            rotation: [
                [flat[0], flat[1], flat[2]],
                [flat[3], flat[4], flat[5]],
                [flat[6], flat[7], flat[8]],
            ],
            translation_m: require(p.translation_m, "pose.translation_m")?.into(),
        };
        Calibration::new(intrinsics, pose).map_err(|e| ConvertError::Invalid(e.to_string()))
    }
}

// ---- FlashOrder

impl From<FlashOrder> for pb::FlashOrder {
    fn from(o: FlashOrder) -> Self {
        match o {
            FlashOrder::XAscending => pb::FlashOrder::XAscending,
            FlashOrder::XDescending => pb::FlashOrder::XDescending,
        }
    }
}

/// Decode the raw `i32` carried on the wire. `UNSPECIFIED` and unknown values
/// are rejected so a forgotten field can't silently pick a direction.
pub fn flash_order_from_i32(v: i32) -> Result<FlashOrder, ConvertError> {
    match pb::FlashOrder::try_from(v) {
        Ok(pb::FlashOrder::XAscending) => Ok(FlashOrder::XAscending),
        Ok(pb::FlashOrder::XDescending) => Ok(FlashOrder::XDescending),
        Ok(pb::FlashOrder::Unspecified) => Err(ConvertError::Missing("flash_order")),
        Err(_) => Err(ConvertError::Invalid(format!("unknown flash_order {v}"))),
    }
}

// ---- Frame and timing

impl TryFrom<pb::GrayFrame> for GrayImage {
    type Error = ConvertError;

    fn try_from(f: pb::GrayFrame) -> Result<Self, ConvertError> {
        GrayImage::from_raw(f.width, f.height, f.data)
            .map_err(|e| ConvertError::Invalid(e.to_string()))
    }
}

impl From<&GrayImage> for pb::GrayFrame {
    fn from(img: &GrayImage) -> Self {
        Self {
            width: img.width,
            height: img.height,
            data: img.data.clone(),
        }
    }
}

impl From<pb::StrobeTiming> for StrobeTiming {
    fn from(t: pb::StrobeTiming) -> Self {
        Self {
            shot_id: t.shot_id,
            pulse_times_us: t.pulse_times_us,
        }
    }
}

impl From<&StrobeTiming> for pb::StrobeTiming {
    fn from(t: &StrobeTiming) -> Self {
        Self {
            shot_id: t.shot_id,
            pulse_times_us: t.pulse_times_us.clone(),
        }
    }
}

// ---- Results (domain -> wire)

impl From<&LaunchConditions> for pb::LaunchConditions {
    fn from(l: &LaunchConditions) -> Self {
        Self {
            ball_speed_mps: l.ball_speed_mps,
            launch_angle_deg: l.launch_angle_deg,
            azimuth_deg: l.azimuth_deg,
            backspin_rpm: l.backspin_rpm,
            sidespin_rpm: l.sidespin_rpm,
        }
    }
}

impl From<&TrajectoryPoint> for pb::TrajectoryPoint {
    fn from(p: &TrajectoryPoint) -> Self {
        Self {
            t_s: p.t_s,
            position_m: Some(p.position_m.into()),
        }
    }
}

impl From<&FlightResult> for pb::FlightResult {
    fn from(f: &FlightResult) -> Self {
        Self {
            trajectory: f.trajectory.iter().map(Into::into).collect(),
            carry_m: f.carry_m,
            apex_height_m: f.apex_height_m,
            flight_time_s: f.flight_time_s,
            lateral_m: f.lateral_m,
            total_m: f.total_m,
        }
    }
}

impl From<&ShotResult> for pb::ShotResult {
    fn from(s: &ShotResult) -> Self {
        Self {
            shot_id: s.shot_id,
            launch: Some((&s.launch).into()),
            flight: Some((&s.flight).into()),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A calibration shaped like a real one: non-trivial rotation, off-centre
    /// principal point, and distortion.
    fn bench() -> Calibration {
        let pose = calibration::look_at_pose(
            Vec3::new(0.12, 0.08, -0.6),
            Vec3::new(0.12, 0.08, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        );
        Calibration::new(
            Intrinsics {
                width: 1280,
                height: 1024,
                fx: 1600.0,
                fy: 1601.5,
                cx: 641.25,
                cy: 510.5,
                distortion: [-0.08, 0.01, 0.0005, -0.0003, 0.002],
            },
            pose,
        )
        .unwrap()
    }

    #[test]
    fn calibration_round_trips_exactly() {
        let cal = bench();
        let wire = pb::Calibration::from(&cal);
        assert_eq!(Calibration::try_from(wire).unwrap(), cal);
    }

    #[test]
    fn calibration_rejects_bad_shapes() {
        let mut wire = pb::Calibration::from(&bench());
        wire.pose.as_mut().unwrap().rotation.pop();
        assert!(matches!(
            Calibration::try_from(wire),
            Err(ConvertError::BadLength {
                field: "pose.rotation",
                expected: 9,
                actual: 8
            })
        ));

        let mut wire = pb::Calibration::from(&bench());
        wire.intrinsics.as_mut().unwrap().distortion.push(0.0);
        assert!(matches!(
            Calibration::try_from(wire),
            Err(ConvertError::BadLength {
                field: "intrinsics.distortion",
                ..
            })
        ));

        let wire = pb::Calibration::default();
        assert_eq!(
            Calibration::try_from(wire),
            Err(ConvertError::Missing("calibration.intrinsics"))
        );
    }

    #[test]
    fn calibration_validation_errors_surface() {
        let mut wire = pb::Calibration::from(&bench());
        wire.intrinsics.as_mut().unwrap().fx = -1.0;
        assert!(matches!(
            Calibration::try_from(wire),
            Err(ConvertError::Invalid(_))
        ));
    }

    #[test]
    fn frame_and_timing_round_trip() {
        let mut img = GrayImage::new(4, 3);
        img.set(2, 1, 200);
        let back = GrayImage::try_from(pb::GrayFrame::from(&img)).unwrap();
        assert_eq!(back, img);

        let timing = StrobeTiming {
            shot_id: 9,
            pulse_times_us: vec![0, 1000, 2000],
        };
        assert_eq!(StrobeTiming::from(pb::StrobeTiming::from(&timing)), timing);

        let bad = pb::GrayFrame {
            width: 4,
            height: 3,
            data: vec![0; 5],
        };
        assert!(GrayImage::try_from(bad).is_err());
    }

    #[test]
    fn flash_order_rejects_unspecified() {
        assert_eq!(
            flash_order_from_i32(pb::FlashOrder::XDescending as i32),
            Ok(FlashOrder::XDescending)
        );
        assert!(flash_order_from_i32(0).is_err());
        assert!(flash_order_from_i32(99).is_err());
    }
}
