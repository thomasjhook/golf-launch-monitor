//! Wire messages between the firmware and the host. `no_std` so the same
//! definitions compile for the microcontroller.
//!
//! Messages are postcard-encoded and COBS-framed (see [`frame`]); every frame
//! ends with a `0x00` delimiter, so a reader can resynchronise after noise.
#![cfg_attr(not(test), no_std)]

use serde::{Deserialize, Serialize};

pub mod frame;

/// Maximum strobe pulses per shot.
pub const MAX_PULSES: usize = 8;

/// Why a [`StrobeConfig`] was rejected.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ConfigError {
    /// `pulse_count` outside `1..=MAX_PULSES`.
    PulseCount,
    /// `pulse_width_us` outside `1..=MAX_PULSE_WIDTH_US`.
    PulseWidth,
    /// Spacing too long, or shorter than twice the pulse width (>50% duty).
    PulseSpacing,
    /// `trigger_delay_us` above `MAX_DELAY_US`.
    TriggerDelay,
    /// Total LED on-time per shot above `MAX_ON_TIME_US`.
    DutyCycle,
    /// The device is mid-shot and cannot change configuration.
    Busy,
}

/// Strobe sequence configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct StrobeConfig {
    pub pulse_count: u8,
    pub pulse_width_us: u16,
    /// Time between the starts of consecutive flashes.
    pub pulse_spacing_us: u32,
    /// Delay from trigger edge to flash 0.
    pub trigger_delay_us: u32,
}

impl StrobeConfig {
    /// Longest allowed flash. Keeps LED heating and eye exposure bounded.
    pub const MAX_PULSE_WIDTH_US: u16 = 500;
    pub const MAX_SPACING_US: u32 = 20_000;
    pub const MAX_DELAY_US: u32 = 20_000;
    /// Cap on total LED on-time per shot.
    pub const MAX_ON_TIME_US: u32 = 2_000;

    pub fn validate(&self) -> Result<(), ConfigError> {
        if self.pulse_count == 0 || usize::from(self.pulse_count) > MAX_PULSES {
            return Err(ConfigError::PulseCount);
        }
        if self.pulse_width_us == 0 || self.pulse_width_us > Self::MAX_PULSE_WIDTH_US {
            return Err(ConfigError::PulseWidth);
        }
        let multi = self.pulse_count > 1;
        if self.pulse_spacing_us > Self::MAX_SPACING_US
            || (multi && self.pulse_spacing_us < 2 * u32::from(self.pulse_width_us))
        {
            return Err(ConfigError::PulseSpacing);
        }
        if self.trigger_delay_us > Self::MAX_DELAY_US {
            return Err(ConfigError::TriggerDelay);
        }
        if u32::from(self.pulse_count) * u32::from(self.pulse_width_us) > Self::MAX_ON_TIME_US {
            return Err(ConfigError::DutyCycle);
        }
        Ok(())
    }

    /// Start of flash `i`, in microseconds after the trigger edge.
    pub fn pulse_start_us(&self, i: u8) -> u32 {
        self.trigger_delay_us + u32::from(i) * self.pulse_spacing_us
    }

    /// Time from the trigger edge until the last flash ends. A camera
    /// exposure must cover at least this long (plus its own trigger latency).
    pub fn sequence_duration_us(&self) -> u32 {
        self.pulse_start_us(self.pulse_count.saturating_sub(1)) + u32::from(self.pulse_width_us)
    }
}

impl Default for StrobeConfig {
    /// Three 50 µs flashes 1 ms apart. Ball images only separate when the ball
    /// moves more than its diameter between flashes, so 1 ms suits ball speeds
    /// above ~43 m/s (96 mph); slower shots (chips, putts) need longer spacing.
    fn default() -> Self {
        Self {
            pulse_count: 3,
            pulse_width_us: 50,
            pulse_spacing_us: 1_000,
            trigger_delay_us: 0,
        }
    }
}

/// Firmware -> host: a shot was triggered and these flashes fired.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct ShotTriggered {
    pub shot_id: u32,
    pub pulse_count: u8,
    /// Flash times in microseconds relative to flash 0 (so the first is 0).
    /// Only the first `pulse_count` entries are meaningful.
    pub pulse_times_us: [u32; MAX_PULSES],
}

impl ShotTriggered {
    pub fn pulses(&self) -> &[u32] {
        &self.pulse_times_us[..usize::from(self.pulse_count).min(MAX_PULSES)]
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct Status {
    pub fw_version: u16,
    pub armed: bool,
    pub shots_fired: u32,
    pub config: StrobeConfig,
}

/// Host -> firmware.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum HostToDevice {
    Arm,
    Disarm,
    SetConfig(StrobeConfig),
    GetStatus,
    Ping(u32),
}

/// Firmware -> host.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum DeviceToHost {
    Armed,
    Disarmed,
    ConfigApplied(StrobeConfig),
    ConfigRejected(ConfigError),
    Status(Status),
    Pong(u32),
    Shot(ShotTriggered),
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_config_is_valid() {
        assert_eq!(StrobeConfig::default().validate(), Ok(()));
    }

    #[test]
    fn validation_rejects_each_bad_field() {
        let ok = StrobeConfig::default();
        let bad = |f: fn(&mut StrobeConfig)| {
            let mut c = ok;
            f(&mut c);
            c.validate()
        };
        assert_eq!(bad(|c| c.pulse_count = 0), Err(ConfigError::PulseCount));
        assert_eq!(bad(|c| c.pulse_count = 9), Err(ConfigError::PulseCount));
        assert_eq!(bad(|c| c.pulse_width_us = 0), Err(ConfigError::PulseWidth));
        assert_eq!(
            bad(|c| c.pulse_width_us = 501),
            Err(ConfigError::PulseWidth)
        );
        // 50% duty limit: spacing must be at least twice the width.
        assert_eq!(
            bad(|c| c.pulse_spacing_us = 99),
            Err(ConfigError::PulseSpacing)
        );
        assert_eq!(
            bad(|c| c.pulse_spacing_us = 20_001),
            Err(ConfigError::PulseSpacing)
        );
        assert_eq!(
            bad(|c| c.trigger_delay_us = 20_001),
            Err(ConfigError::TriggerDelay)
        );
        // 5 x 500 us = 2500 us of LED on-time.
        assert_eq!(
            bad(|c| {
                c.pulse_count = 5;
                c.pulse_width_us = 500;
                c.pulse_spacing_us = 1_000;
            }),
            Err(ConfigError::DutyCycle)
        );
    }

    #[test]
    fn single_pulse_ignores_minimum_spacing() {
        let c = StrobeConfig {
            pulse_count: 1,
            pulse_width_us: 100,
            pulse_spacing_us: 0,
            trigger_delay_us: 0,
        };
        assert_eq!(c.validate(), Ok(()));
    }

    #[test]
    fn timing_helpers() {
        let c = StrobeConfig {
            pulse_count: 3,
            pulse_width_us: 50,
            pulse_spacing_us: 1_000,
            trigger_delay_us: 200,
        };
        assert_eq!(c.pulse_start_us(0), 200);
        assert_eq!(c.pulse_start_us(2), 2_200);
        assert_eq!(c.sequence_duration_us(), 2_250);
    }
}
