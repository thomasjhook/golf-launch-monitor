//! Board-level configuration: the ONE place pins, polarity and timing
//! constants are defined. Wiring is documented in README.md.
//!
//! The pin *numbers* below are used for the panic-time raw register backstop;
//! the typed peripherals taken in [`Io::take`] must be kept in sync by hand.

use embassy_rp::gpio::{Drive, Input, Level, Output, Pull, SlewRate};
use embassy_rp::Peripherals;

// GP2: break-beam trigger input (taken in `Io::take`).
/// GP3: strobe output (logic-level MOSFET gate), active-high.
pub const STROBE_PIN_NUM: u8 = 3;
/// GP4: camera trigger output, active-high.
pub const CAMERA_PIN_NUM: u8 = 4;
// GP25: onboard LED on a Pico 2 (NOT Pico 2 W, where the LED sits on the
// wireless chip); taken in `Io::take`.

/// Trigger is asserted when the pin reads LOW (sensor pulls to ground when
/// the beam is broken, internal pull-up holds it high otherwise).
pub const TRIGGER_ACTIVE_LOW: bool = true;

/// Camera trigger pulse width (microseconds). Held at least this long; may
/// be a few microseconds longer because it is dropped from inside the strobe
/// polling loops.
pub const CAMERA_PULSE_US: u32 = 100;

/// Upper bound on busy-loop iterations while a strobe pulse is high. This is
/// a software backstop that only matters if the 1 MHz timer reads stop
/// advancing; the normal exit is the elapsed-time check.
pub const STROBE_MAX_SPINS: u32 = 1_000_000;

/// Raw pin masks for the panic/hard-fault path (SIO GPIO_OUT_CLR).
pub const OUTPUT_LOW_MASK: u32 = (1 << STROBE_PIN_NUM) | (1 << CAMERA_PIN_NUM);

pub struct Io {
    pub trigger: Input<'static>,
    pub strobe: Output<'static>,
    pub camera: Output<'static>,
    pub led: Output<'static>,
}

impl Io {
    /// Take the pins. Outputs are created LOW. Keep these peripheral names in
    /// sync with the `*_PIN_NUM` consts above.
    pub fn take(p: Peripherals) -> (Self, Remaining) {
        let trigger = Input::new(
            p.PIN_2,
            if TRIGGER_ACTIVE_LOW { Pull::Up } else { Pull::Down },
        );
        let mut strobe = Output::new(p.PIN_3, Level::Low);
        // Fast edges into a MOSFET gate; see README for drive-strength notes.
        strobe.set_drive_strength(Drive::_12mA);
        strobe.set_slew_rate(SlewRate::Fast);
        let camera = Output::new(p.PIN_4, Level::Low);
        let led = Output::new(p.PIN_25, Level::Low);
        (
            Self {
                trigger,
                strobe,
                camera,
                led,
            },
            Remaining {
                usb: p.USB,
                core1: p.CORE1,
            },
        )
    }
}

/// The peripherals left over for other users.
pub struct Remaining {
    pub usb: embassy_rp::Peri<'static, embassy_rp::peripherals::USB>,
    pub core1: embassy_rp::Peri<'static, embassy_rp::peripherals::CORE1>,
}
