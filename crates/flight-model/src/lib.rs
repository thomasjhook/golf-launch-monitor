//! flight-model: golf ball flight simulation (gravity + drag + Magnus lift).
//!
//! # Model
//! State is integrated with classic RK4 at a fixed 1 ms step, trajectory sampled every 10 ms.
//! Forces on the ball (mass 45.93 g, radius 21.335 mm, from `launch-types`):
//! * gravity `g = 9.80665 m/s^2` along -Y;
//! * drag `F = -1/2 rho A Cd |v_rel| v_rel`, where `v_rel = v - wind`;
//! * Magnus lift `F = 1/2 rho A Cl |v_rel|^2 * unit(omega x v_rel)`.
//!
//! Spin factor `S = r |omega| / |v_rel|`. Coefficients are empirical fits in the spirit of the
//! published golf ball wind-tunnel / launch data (Bearman & Harvey 1976; Smits & Smith 1994;
//! Mehta 1985; Trackman/Nathan-style trajectory studies):
//! * `Cd = 0.22 + 0.40 S` (clamped to <= 0.45); about 0.26-0.30 in typical flight.
//! * `Cl = 0.38 * (1 - exp(-S / 0.12))` : roughly linear (slope ~3.2) at small S, saturating at
//!   0.38, matching the known ~0.15-0.25 range at S = 0.05-0.2.
//!
//! Spin: backspin axis is horizontal and perpendicular to the launch direction (lift up); positive
//! sidespin is rotation about -Y, which pushes the ball toward +Z (right). Spin decays as
//! `exp(-t / 50 s)` (2 %/s), a conservative figure (measured values for golf balls are about
//! 2-5 %/s; the low end is used since we do not model it more finely).
//!
//! Landing is the first `y <= 0` after the start, linearly interpolated. `carry_m` is
//! `sqrt(x^2 + z^2)` there.
//!
//! # Bounce and roll (APPROXIMATION)
//! `total_m` adds a crude estimate: up to 3 bounces with normal restitution
//! `e = 0.15 + 0.35 * hardness` (harder ground = bouncier), horizontal speed
//! retention reduced by backspin and by steep landing, then a constant-deceleration roll
//! (`a = g * (0.08 + 0.35 * (1 - hardness))`). Trust carry, not total.
#![forbid(unsafe_code)]

use launch_types::{
    FlightResult, LaunchConditions, TrajectoryPoint, Vec3, BALL_MASS_KG, BALL_RADIUS_M,
};

const G: f64 = 9.80665;
const DT: f64 = 0.001;
const SAMPLE_EVERY: usize = 10;
const MAX_TIME_S: f64 = 60.0;
const SPIN_DECAY_TAU_S: f64 = 50.0;
const CD_BASE: f64 = 0.22;
const CD_SPIN: f64 = 0.40;
const CD_MAX: f64 = 0.45;
const CL_MAX: f64 = 0.38;
const CL_SCALE: f64 = 0.12;

/// Environmental conditions.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Environment {
    pub air_density_kgm3: f64,
    /// Wind velocity (air movement) in the world frame, m/s.
    pub wind_mps: Vec3,
    /// 0 (soft) .. 1 (hard).
    pub ground_hardness: f64,
}

impl Default for Environment {
    fn default() -> Self {
        Self {
            air_density_kgm3: 1.225,
            wind_mps: Vec3::ZERO,
            ground_hardness: 0.5,
        }
    }
}

#[derive(Clone, Copy)]
struct V([f64; 3]);
impl V {
    fn add(self, o: V) -> V {
        V([self.0[0] + o.0[0], self.0[1] + o.0[1], self.0[2] + o.0[2]])
    }
    fn sub(self, o: V) -> V {
        V([self.0[0] - o.0[0], self.0[1] - o.0[1], self.0[2] - o.0[2]])
    }
    fn mul(self, k: f64) -> V {
        V([self.0[0] * k, self.0[1] * k, self.0[2] * k])
    }
    fn dot(self, o: V) -> f64 {
        self.0[0] * o.0[0] + self.0[1] * o.0[1] + self.0[2] * o.0[2]
    }
    fn cross(self, o: V) -> V {
        let (a, b) = (self.0, o.0);
        V([
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        ])
    }
    fn norm(self) -> f64 {
        self.dot(self).sqrt()
    }
}

fn zeroed() -> FlightResult {
    FlightResult {
        trajectory: vec![TrajectoryPoint {
            t_s: 0.0,
            position_m: Vec3::ZERO,
        }],
        carry_m: 0.0,
        apex_height_m: 0.0,
        flight_time_s: 0.0,
        lateral_m: 0.0,
        total_m: 0.0,
    }
}

/// Acceleration given position-independent state: velocity, time, spin vector at t=0.
fn accel(vel: V, t: f64, omega0: V, env: &Environment) -> V {
    let rel = vel.sub(V([env.wind_mps.x, env.wind_mps.y, env.wind_mps.z]));
    let speed = rel.norm();
    let a_ref = std::f64::consts::PI * BALL_RADIUS_M * BALL_RADIUS_M;
    let k = 0.5 * env.air_density_kgm3 * a_ref / BALL_MASS_KG;
    let mut a = V([0.0, -G, 0.0]);
    if speed > 1e-9 {
        let omega = omega0.mul((-t / SPIN_DECAY_TAU_S).exp());
        let s = BALL_RADIUS_M * omega.norm() / speed;
        let cd = (CD_BASE + CD_SPIN * s).min(CD_MAX);
        let cl = CL_MAX * (1.0 - (-s / CL_SCALE).exp());
        a = a.add(rel.mul(-k * cd * speed));
        let lift_dir = omega.cross(rel);
        let ln = lift_dir.norm();
        if ln > 1e-12 {
            a = a.add(lift_dir.mul(k * cl * speed * speed / ln));
        }
    }
    a
}

fn rk4(p: V, v: V, t: f64, omega0: V, env: &Environment) -> (V, V) {
    let a1 = accel(v, t, omega0, env);
    let k1p = v;
    let v2 = v.add(a1.mul(DT / 2.0));
    let a2 = accel(v2, t + DT / 2.0, omega0, env);
    let v3 = v.add(a2.mul(DT / 2.0));
    let a3 = accel(v3, t + DT / 2.0, omega0, env);
    let v4 = v.add(a3.mul(DT));
    let a4 = accel(v4, t + DT, omega0, env);
    let dp = k1p.add(v2.mul(2.0)).add(v3.mul(2.0)).add(v4).mul(DT / 6.0);
    let dv = a1.add(a2.mul(2.0)).add(a3.mul(2.0)).add(a4).mul(DT / 6.0);
    (p.add(dp), v.add(dv))
}

/// Simulate a shot. Never panics; invalid input yields a zeroed result.
pub fn simulate(launch: &LaunchConditions, env: &Environment) -> FlightResult {
    let inputs = [
        launch.ball_speed_mps,
        launch.launch_angle_deg,
        launch.azimuth_deg,
        launch.backspin_rpm,
        launch.sidespin_rpm,
        env.air_density_kgm3,
        env.wind_mps.x,
        env.wind_mps.y,
        env.wind_mps.z,
        env.ground_hardness,
    ];
    if inputs.iter().any(|x| !x.is_finite())
        || launch.ball_speed_mps <= 0.0
        || env.air_density_kgm3 < 0.0
    {
        return zeroed();
    }
    let speed = launch.ball_speed_mps.min(120.0);
    let la = launch.launch_angle_deg.clamp(-89.0, 89.0).to_radians();
    let az = launch.azimuth_deg.clamp(-89.0, 89.0).to_radians();
    let dir = V([la.cos() * az.cos(), la.sin(), la.cos() * az.sin()]);
    let mut vel = dir.mul(speed);
    // Backspin axis: v x Y (horizontal, perpendicular to travel); lift is omega x v = up.
    let mut b_axis = dir.cross(V([0.0, 1.0, 0.0]));
    let bn = b_axis.norm();
    b_axis = if bn > 1e-9 {
        b_axis.mul(1.0 / bn)
    } else {
        V([0.0, 0.0, 1.0])
    };
    let rpm = std::f64::consts::TAU / 60.0;
    let omega0 =
        b_axis
            .mul(launch.backspin_rpm * rpm)
            .add(V([0.0, -launch.sidespin_rpm * rpm, 0.0]));
    let hardness = env.ground_hardness.clamp(0.0, 1.0);

    let mut pos = V([0.0; 3]);
    let mut t = 0.0;
    let mut traj = vec![TrajectoryPoint {
        t_s: 0.0,
        position_m: Vec3::ZERO,
    }];
    let mut apex: f64 = 0.0;
    let mut step = 0usize;
    let (land_p, land_v, land_t);
    loop {
        let (np, nv) = rk4(pos, vel, t, omega0, env);
        step += 1;
        let nt = step as f64 * DT;
        if !(np.0[0].is_finite() && np.0[1].is_finite() && np.0[2].is_finite()) {
            return zeroed();
        }
        if np.0[1] <= 0.0 {
            let denom = pos.0[1] - np.0[1];
            let f = if denom > 1e-12 {
                (pos.0[1] / denom).clamp(0.0, 1.0)
            } else {
                0.0
            };
            land_p = pos.add(np.sub(pos).mul(f));
            land_v = vel.add(nv.sub(vel).mul(f));
            land_t = t + f * DT;
            break;
        }
        apex = apex.max(np.0[1]);
        pos = np;
        vel = nv;
        t = nt;
        if step.is_multiple_of(SAMPLE_EVERY) {
            traj.push(TrajectoryPoint {
                t_s: t,
                position_m: Vec3::new(pos.0[0], pos.0[1], pos.0[2]),
            });
        }
        if t >= MAX_TIME_S {
            land_p = V([pos.0[0], 0.0, pos.0[2]]);
            land_v = vel;
            land_t = t;
            break;
        }
    }
    traj.push(TrajectoryPoint {
        t_s: land_t,
        position_m: Vec3::new(land_p.0[0], 0.0, land_p.0[2]),
    });
    let carry = (land_p.0[0].powi(2) + land_p.0[2].powi(2)).sqrt();
    let total = carry + bounce_and_roll(land_v, launch.backspin_rpm, hardness);
    FlightResult {
        trajectory: traj,
        carry_m: carry,
        apex_height_m: apex,
        flight_time_s: land_t,
        lateral_m: land_p.0[2],
        total_m: total.max(carry),
    }
}

/// Rough extra distance (m) from bounces and roll along the landing heading. Approximation only.
fn bounce_and_roll(land_v: V, backspin_rpm: f64, hardness: f64) -> f64 {
    let mut vh = (land_v.0[0].powi(2) + land_v.0[2].powi(2)).sqrt();
    let mut vy = (-land_v.0[1]).max(0.0);
    if vh < 1e-6 {
        return 0.0;
    }
    let descent = vy.atan2(vh); // radians below horizontal
    let spin_term = (backspin_rpm.max(0.0) / 8000.0).min(1.0);
    let e = 0.15 + 0.35 * hardness; // restitution of the normal component
                                    // Fraction of horizontal speed kept on first impact: soft ground, backspin, steep landing remove it.
    let mut keep =
        (0.30 + 0.45 * hardness - 0.35 * spin_term - 0.25 * (descent / 1.2)).clamp(0.05, 0.85);
    let mut dist = 0.0;
    for _ in 0..3 {
        vh *= keep;
        vy *= e;
        if vy < 0.5 {
            break;
        }
        dist += vh * 2.0 * vy / G;
        keep = 0.8; // later bounces lose less
    }
    let decel = G * (0.08 + 0.35 * (1.0 - hardness));
    dist + vh * vh / (2.0 * decel)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn lc(v: f64, a: f64, az: f64, back: f64, side: f64) -> LaunchConditions {
        LaunchConditions {
            ball_speed_mps: v,
            launch_angle_deg: a,
            azimuth_deg: az,
            backspin_rpm: back,
            sidespin_rpm: side,
        }
    }
    fn sim(l: LaunchConditions) -> FlightResult {
        simulate(&l, &Environment::default())
    }

    #[test]
    fn driver_plausible() {
        let r = sim(lc(67.0, 11.0, 0.0, 2700.0, 0.0));
        assert!(
            (210.0..=265.0).contains(&r.carry_m),
            "carry {} apex {}",
            r.carry_m,
            r.apex_height_m
        );
        assert!(
            (28.0..=40.0).contains(&r.apex_height_m),
            "apex {}",
            r.apex_height_m
        );
        assert!(r.total_m >= r.carry_m);
        assert!(r.flight_time_s > 4.0 && r.flight_time_s < 9.0);
    }

    #[test]
    fn seven_iron_plausible() {
        let r = sim(lc(53.0, 17.0, 0.0, 6500.0, 0.0));
        let yards = r.carry_m / 0.9144;
        assert!(
            (140.0..=175.0).contains(&yards),
            "yards {} apex {}",
            yards,
            r.apex_height_m
        );
        assert!(
            (28.0..=36.0).contains(&r.apex_height_m),
            "apex {}",
            r.apex_height_m
        );
    }

    #[test]
    fn spin_effects() {
        let lo = sim(lc(67.0, 11.0, 0.0, 2000.0, 0.0));
        let hi = sim(lc(67.0, 11.0, 0.0, 4000.0, 0.0));
        assert!(hi.apex_height_m > lo.apex_height_m);
        let none = sim(lc(67.0, 11.0, 0.0, 0.0, 0.0));
        let mid = sim(lc(67.0, 11.0, 0.0, 2700.0, 0.0));
        assert!(none.carry_m < mid.carry_m && none.apex_height_m < mid.apex_height_m);
    }

    #[test]
    fn lateral() {
        let r = sim(lc(67.0, 11.0, 0.0, 2700.0, 0.0));
        assert!(r.lateral_m.abs() < 0.5);
        assert!(sim(lc(67.0, 11.0, 0.0, 2700.0, 1000.0)).lateral_m > 1.0);
        assert!(sim(lc(67.0, 11.0, 0.0, 2700.0, -1000.0)).lateral_m < -1.0);
        assert!(sim(lc(67.0, 11.0, 2.0, 2700.0, 0.0)).lateral_m > 1.0);
        assert!(sim(lc(67.0, 11.0, -2.0, 2700.0, 0.0)).lateral_m < -1.0);
    }

    #[test]
    fn wind_and_density() {
        let l = lc(67.0, 11.0, 0.0, 2700.0, 0.0);
        let base = simulate(&l, &Environment::default());
        let head = simulate(
            &l,
            &Environment {
                wind_mps: Vec3::new(-5.0, 0.0, 0.0),
                ..Default::default()
            },
        );
        let tail = simulate(
            &l,
            &Environment {
                wind_mps: Vec3::new(5.0, 0.0, 0.0),
                ..Default::default()
            },
        );
        let dense = simulate(
            &l,
            &Environment {
                air_density_kgm3: 1.4,
                ..Default::default()
            },
        );
        assert!(head.carry_m < base.carry_m && tail.carry_m > base.carry_m);
        assert!(dense.carry_m < base.carry_m);
    }

    #[test]
    fn deterministic_and_finite() {
        let l = lc(60.0, 14.0, 1.0, 3000.0, 500.0);
        assert_eq!(sim(l), sim(l));
        for v in [1.0, 20.0, 45.0, 70.0, 90.0] {
            for a in [-5.0, 0.0, 5.0, 15.0, 30.0, 60.0, 85.0] {
                for s in [0.0, 1500.0, 3000.0, 9000.0] {
                    let r = sim(lc(v, a, 3.0, s, s * 0.2 - 200.0));
                    assert!(
                        r.carry_m.is_finite()
                            && r.apex_height_m.is_finite()
                            && r.total_m.is_finite()
                    );
                    assert!(r.lateral_m.is_finite() && r.flight_time_s.is_finite());
                    assert!(r.trajectory.iter().all(|p| p.position_m.norm().is_finite()));
                }
            }
        }
    }

    #[test]
    fn edge_cases() {
        for l in [
            lc(0.0, 10.0, 0.0, 0.0, 0.0),
            lc(-5.0, 10.0, 0.0, 0.0, 0.0),
            lc(f64::NAN, 10.0, 0.0, 0.0, 0.0),
            lc(50.0, f64::INFINITY, 0.0, 0.0, 0.0),
        ] {
            let r = sim(l);
            assert_eq!(r.carry_m, 0.0);
            assert_eq!(r.total_m, 0.0);
        }
        let r = sim(lc(50.0, 89.9, 0.0, 3000.0, 0.0));
        assert!(r.carry_m.is_finite());
    }
}
