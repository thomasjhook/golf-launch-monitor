//! Hardware-independent firmware logic: the shot sequencer state machine.
//!
//! The microcontroller glue feeds this three kinds of input and acts on what
//! it returns:
//!
//! * host messages -> [`Sequencer::handle`] (returns the reply to send)
//! * trigger edge  -> [`Sequencer::on_trigger`] (returns the [`Schedule`] of
//!   camera/strobe events to execute with hardware timing)
//! * sequence end  -> [`Sequencer::on_sequence_done`] (returns the `Shot`
//!   message, given the flash times the hardware actually measured)
//!
//! plus [`Sequencer::tick`] to leave the post-shot lockout. All times are
//! microseconds on a monotonic clock supplied by the caller, so the same code
//! runs on the device, in host unit tests, and inside the simulated device
//! used by the capture daemon.
#![no_std]

use protocol::{
    ConfigError, DeviceToHost, HostToDevice, ShotTriggered, Status, StrobeConfig, MAX_PULSES,
};

pub const FW_VERSION: u16 = 1;

/// How long after a shot the trigger is ignored. Covers the follow-through
/// (club passing the gate again) and gives the host time to read the frame.
pub const DEFAULT_LOCKOUT_US: u32 = 250_000;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum State {
    Disarmed,
    /// Waiting for a trigger edge.
    Armed,
    /// A shot sequence is executing.
    Firing,
    /// Shot done; ignoring triggers until `until_us`, then re-arming.
    Lockout {
        until_us: u64,
    },
}

/// What the hardware must do for one shot. All offsets are microseconds after
/// the trigger edge.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Schedule {
    pub shot_id: u32,
    pub pulse_count: u8,
    /// Planned start of each flash (only the first `pulse_count` are used).
    pub pulse_start_us: [u32; MAX_PULSES],
    pub pulse_width_us: u16,
    /// Camera exposure must cover at least this long after the trigger edge.
    pub sequence_duration_us: u32,
}

impl Schedule {
    pub fn pulses(&self) -> &[u32] {
        &self.pulse_start_us[..usize::from(self.pulse_count).min(MAX_PULSES)]
    }
}

pub struct Sequencer {
    state: State,
    config: StrobeConfig,
    lockout_us: u32,
    next_shot_id: u32,
    shots_fired: u32,
    /// Host asked to disarm while a shot was executing.
    pending_disarm: bool,
    schedule: Option<Schedule>,
}

impl Default for Sequencer {
    fn default() -> Self {
        Self::new()
    }
}

impl Sequencer {
    pub const fn new() -> Self {
        Self {
            state: State::Disarmed,
            config: StrobeConfig {
                pulse_count: 3,
                pulse_width_us: 50,
                pulse_spacing_us: 1_000,
                trigger_delay_us: 0,
            },
            lockout_us: DEFAULT_LOCKOUT_US,
            next_shot_id: 1,
            shots_fired: 0,
            pending_disarm: false,
            schedule: None,
        }
    }

    pub fn with_lockout_us(mut self, lockout_us: u32) -> Self {
        self.lockout_us = lockout_us;
        self
    }

    pub fn state(&self) -> State {
        self.state
    }

    pub fn config(&self) -> StrobeConfig {
        self.config
    }

    fn is_armed(&self) -> bool {
        !self.pending_disarm && !matches!(self.state, State::Disarmed)
    }

    /// Handle a message from the host, returning the reply (if any).
    pub fn handle(&mut self, msg: HostToDevice) -> Option<DeviceToHost> {
        match msg {
            HostToDevice::Arm => {
                self.pending_disarm = false;
                if self.state == State::Disarmed {
                    self.state = State::Armed;
                }
                Some(DeviceToHost::Armed)
            }
            HostToDevice::Disarm => {
                if self.state == State::Firing {
                    // Let the shot in flight finish and report; then disarm.
                    self.pending_disarm = true;
                } else {
                    self.state = State::Disarmed;
                }
                Some(DeviceToHost::Disarmed)
            }
            HostToDevice::SetConfig(cfg) => Some(if self.state == State::Firing {
                DeviceToHost::ConfigRejected(ConfigError::Busy)
            } else {
                match cfg.validate() {
                    Ok(()) => {
                        self.config = cfg;
                        DeviceToHost::ConfigApplied(cfg)
                    }
                    Err(e) => DeviceToHost::ConfigRejected(e),
                }
            }),
            HostToDevice::GetStatus => Some(DeviceToHost::Status(Status {
                fw_version: FW_VERSION,
                armed: self.is_armed(),
                shots_fired: self.shots_fired,
                config: self.config,
            })),
            HostToDevice::Ping(n) => Some(DeviceToHost::Pong(n)),
        }
    }

    /// A trigger edge was seen at `now_us`. Returns the schedule to execute if
    /// (and only if) the sequencer was armed.
    pub fn on_trigger(&mut self, _now_us: u64) -> Option<Schedule> {
        if self.state != State::Armed {
            return None;
        }
        let cfg = self.config;
        let mut starts = [0u32; MAX_PULSES];
        for i in 0..cfg.pulse_count {
            starts[usize::from(i)] = cfg.pulse_start_us(i);
        }
        let schedule = Schedule {
            shot_id: self.next_shot_id,
            pulse_count: cfg.pulse_count,
            pulse_start_us: starts,
            pulse_width_us: cfg.pulse_width_us,
            sequence_duration_us: cfg.sequence_duration_us(),
        };
        self.next_shot_id = self.next_shot_id.wrapping_add(1);
        self.state = State::Firing;
        self.schedule = Some(schedule);
        Some(schedule)
    }

    /// The scheduled sequence finished at `now_us`. `measured_start_us` are
    /// the flash start times the hardware actually captured, in microseconds
    /// after the trigger edge; pass an empty slice to report the planned
    /// times. Returns the `Shot` message for the host, or `None` if no shot
    /// was executing.
    pub fn on_sequence_done(
        &mut self,
        now_us: u64,
        measured_start_us: &[u32],
    ) -> Option<DeviceToHost> {
        if self.state != State::Firing {
            return None;
        }
        let schedule = self.schedule.take()?;
        let n = usize::from(schedule.pulse_count).min(MAX_PULSES);
        let source = if measured_start_us.len() == n {
            measured_start_us
        } else {
            schedule.pulses()
        };
        // Report times relative to flash 0, which is what the host pairs
        // with the ball images in the frame.
        let first = source[0];
        let mut rel = [0u32; MAX_PULSES];
        for (out, &t) in rel.iter_mut().zip(source) {
            *out = t.saturating_sub(first);
        }

        self.shots_fired = self.shots_fired.wrapping_add(1);
        self.state = if self.pending_disarm {
            State::Disarmed
        } else {
            State::Lockout {
                until_us: now_us + u64::from(self.lockout_us),
            }
        };
        self.pending_disarm = false;

        Some(DeviceToHost::Shot(ShotTriggered {
            shot_id: schedule.shot_id,
            pulse_count: schedule.pulse_count,
            pulse_times_us: rel,
        }))
    }

    /// Call periodically (or whenever the clock advances); re-arms after the
    /// lockout expires.
    pub fn tick(&mut self, now_us: u64) {
        if let State::Lockout { until_us } = self.state {
            if now_us >= until_us {
                self.state = State::Armed;
            }
        }
    }
}

#[cfg(test)]
extern crate std;

#[cfg(test)]
mod tests {
    use super::*;

    fn armed() -> Sequencer {
        let mut s = Sequencer::new();
        assert_eq!(s.handle(HostToDevice::Arm), Some(DeviceToHost::Armed));
        s
    }

    #[test]
    fn starts_disarmed_and_ignores_triggers() {
        let mut s = Sequencer::new();
        assert_eq!(s.state(), State::Disarmed);
        assert_eq!(s.on_trigger(0), None);
        assert_eq!(s.state(), State::Disarmed);
    }

    #[test]
    fn trigger_builds_schedule_from_config() {
        let mut s = armed();
        let cfg = StrobeConfig {
            pulse_count: 3,
            pulse_width_us: 40,
            pulse_spacing_us: 1_500,
            trigger_delay_us: 300,
        };
        assert_eq!(
            s.handle(HostToDevice::SetConfig(cfg)),
            Some(DeviceToHost::ConfigApplied(cfg))
        );
        let sched = s.on_trigger(1_000).expect("armed sequencer fires");
        assert_eq!(sched.shot_id, 1);
        assert_eq!(sched.pulses(), &[300, 1_800, 3_300]);
        assert_eq!(sched.pulse_width_us, 40);
        assert_eq!(sched.sequence_duration_us, 3_340);
        assert_eq!(s.state(), State::Firing);
    }

    #[test]
    fn second_trigger_while_firing_is_ignored() {
        let mut s = armed();
        assert!(s.on_trigger(0).is_some());
        assert_eq!(s.on_trigger(10), None);
    }

    #[test]
    fn shot_report_uses_planned_times_when_none_measured() {
        let mut s = armed();
        s.handle(HostToDevice::SetConfig(StrobeConfig {
            trigger_delay_us: 500,
            ..StrobeConfig::default()
        }));
        s.on_trigger(0).unwrap();
        let Some(DeviceToHost::Shot(shot)) = s.on_sequence_done(3_000, &[]) else {
            panic!("expected a shot");
        };
        assert_eq!(shot.shot_id, 1);
        assert_eq!(shot.pulse_count, 3);
        // Relative to flash 0, so the trigger delay drops out.
        assert_eq!(shot.pulses(), &[0, 1_000, 2_000]);
    }

    #[test]
    fn shot_report_prefers_measured_times() {
        let mut s = armed();
        s.on_trigger(0).unwrap();
        let Some(DeviceToHost::Shot(shot)) = s.on_sequence_done(3_000, &[12, 1_015, 2_011]) else {
            panic!("expected a shot");
        };
        assert_eq!(shot.pulses(), &[0, 1_003, 1_999]);
    }

    #[test]
    fn wrong_length_measurement_falls_back_to_plan() {
        let mut s = armed();
        s.on_trigger(0).unwrap();
        let Some(DeviceToHost::Shot(shot)) = s.on_sequence_done(3_000, &[5, 6]) else {
            panic!("expected a shot");
        };
        assert_eq!(shot.pulses(), &[0, 1_000, 2_000]);
    }

    #[test]
    fn lockout_then_rearm_and_shot_ids_increment() {
        let mut s = Sequencer::new().with_lockout_us(100_000);
        s.handle(HostToDevice::Arm);
        s.on_trigger(0).unwrap();
        s.on_sequence_done(3_000, &[]).unwrap();
        assert_eq!(s.state(), State::Lockout { until_us: 103_000 });

        // Triggers during lockout are ignored; tick before expiry changes nothing.
        assert_eq!(s.on_trigger(50_000), None);
        s.tick(102_999);
        assert!(matches!(s.state(), State::Lockout { .. }));

        s.tick(103_000);
        assert_eq!(s.state(), State::Armed);
        let second = s.on_trigger(200_000).unwrap();
        assert_eq!(second.shot_id, 2);
    }

    #[test]
    fn disarm_cancels_lockout_immediately() {
        let mut s = armed();
        s.on_trigger(0).unwrap();
        s.on_sequence_done(3_000, &[]).unwrap();
        assert_eq!(s.handle(HostToDevice::Disarm), Some(DeviceToHost::Disarmed));
        assert_eq!(s.state(), State::Disarmed);
        s.tick(10_000_000);
        assert_eq!(s.state(), State::Disarmed);
    }

    #[test]
    fn disarm_mid_shot_still_reports_the_shot() {
        let mut s = armed();
        s.on_trigger(0).unwrap();
        assert_eq!(s.handle(HostToDevice::Disarm), Some(DeviceToHost::Disarmed));
        // Still executing: the shot is reported, then we end up disarmed.
        assert_eq!(s.state(), State::Firing);
        assert!(matches!(
            s.on_sequence_done(3_000, &[]),
            Some(DeviceToHost::Shot(_))
        ));
        assert_eq!(s.state(), State::Disarmed);
        // A later Arm works normally.
        s.handle(HostToDevice::Arm);
        assert_eq!(s.state(), State::Armed);
    }

    #[test]
    fn arm_during_pending_disarm_cancels_it() {
        let mut s = armed();
        s.on_trigger(0).unwrap();
        s.handle(HostToDevice::Disarm);
        s.handle(HostToDevice::Arm);
        s.on_sequence_done(3_000, &[]).unwrap();
        assert!(matches!(s.state(), State::Lockout { .. }));
    }

    #[test]
    fn set_config_is_validated_and_blocked_mid_shot() {
        let mut s = armed();
        let bad = StrobeConfig {
            pulse_count: 0,
            ..StrobeConfig::default()
        };
        assert_eq!(
            s.handle(HostToDevice::SetConfig(bad)),
            Some(DeviceToHost::ConfigRejected(ConfigError::PulseCount))
        );
        assert_eq!(
            s.config(),
            StrobeConfig::default(),
            "rejected config not applied"
        );

        s.on_trigger(0).unwrap();
        assert_eq!(
            s.handle(HostToDevice::SetConfig(StrobeConfig::default())),
            Some(DeviceToHost::ConfigRejected(ConfigError::Busy))
        );
    }

    #[test]
    fn status_and_ping() {
        let mut s = Sequencer::new();
        let Some(DeviceToHost::Status(st)) = s.handle(HostToDevice::GetStatus) else {
            panic!("expected status");
        };
        assert_eq!(st.fw_version, FW_VERSION);
        assert!(!st.armed);
        assert_eq!(st.shots_fired, 0);

        s.handle(HostToDevice::Arm);
        s.on_trigger(0).unwrap();
        s.on_sequence_done(1, &[]).unwrap();
        let Some(DeviceToHost::Status(st)) = s.handle(HostToDevice::GetStatus) else {
            panic!("expected status");
        };
        assert!(st.armed, "lockout still counts as armed");
        assert_eq!(st.shots_fired, 1);

        assert_eq!(
            s.handle(HostToDevice::Ping(99)),
            Some(DeviceToHost::Pong(99))
        );
    }

    #[test]
    fn sequence_done_without_a_shot_is_a_no_op() {
        let mut s = armed();
        assert_eq!(s.on_sequence_done(0, &[]), None);
    }
}
