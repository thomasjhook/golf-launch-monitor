//! Core 1: the hard-real-time loop. No executor, no interrupts, no async.
//!
//! While the sequencer is armed it busy-polls the trigger pin. On the edge it
//! raises the camera trigger immediately, asks the `Sequencer` for the
//! schedule (tiny critical section), then plays the strobe schedule against
//! the free-running 1 MHz TIMER0, recording when each flash REALLY started.
//! All decisions (timings, validation, state) come from `firmware-core`.

use core::sync::atomic::Ordering;

use embassy_rp::gpio::{Input, Output};
use embassy_rp::pac;
use firmware_core::Schedule;
use protocol::{StrobeConfig, MAX_PULSES};

use crate::board::{CAMERA_PULSE_US, STROBE_MAX_SPINS, TRIGGER_ACTIVE_LOW};
use crate::shared::{with_seq, Measured, ARMED, MEASURED};

/// Low 32 bits of the free-running 1 MHz timer (wraps every ~71 min; we only
/// ever use wrapping differences of a few hundred ms).
#[inline(always)]
fn now32() -> u32 {
    pac::TIMER0.timerawl().read()
}

#[inline(always)]
fn trigger_active(pin: &Input<'static>) -> bool {
    if TRIGGER_ACTIVE_LOW {
        pin.is_low()
    } else {
        pin.is_high()
    }
}

struct Outputs {
    strobe: Output<'static>,
    camera: Output<'static>,
    t0: u32,
    camera_high: bool,
}

impl Outputs {
    /// Drop the camera trigger once its pulse width has elapsed.
    #[inline(always)]
    fn service_camera(&mut self) {
        if self.camera_high && now32().wrapping_sub(self.t0) >= CAMERA_PULSE_US {
            self.camera.set_low();
            self.camera_high = false;
        }
    }

    /// Spin until `offset_us` after the trigger edge.
    #[inline(always)]
    fn wait_until(&mut self, offset_us: u32) {
        while now32().wrapping_sub(self.t0) < offset_us {
            self.service_camera();
        }
    }

    /// One flash. The strobe is guaranteed low on return.
    fn flash(&mut self, width_us: u32) -> u32 {
        let start = now32();
        self.strobe.set_high();
        let mut spins = 0u32;
        while now32().wrapping_sub(start) < width_us && spins < STROBE_MAX_SPINS {
            self.service_camera();
            spins += 1;
        }
        self.strobe.set_low();
        start.wrapping_sub(self.t0)
    }

    fn all_low(&mut self) {
        self.strobe.set_low();
        self.camera.set_low();
        self.camera_high = false;
    }
}

fn play(out: &mut Outputs, sched: &Schedule) -> Measured {
    // Belt and braces: the Sequencer's config is already validated, but never
    // let a corrupted value stretch a pulse beyond the protocol maximum.
    let width = u32::from(sched.pulse_width_us.min(StrobeConfig::MAX_PULSE_WIDTH_US));
    let mut m = Measured {
        count: 0,
        starts_us: [0; MAX_PULSES],
    };
    for &offset in sched.pulses() {
        out.wait_until(offset);
        m.starts_us[m.count] = out.flash(width);
        m.count += 1;
    }
    // Make sure the camera pulse has its full width even for short sequences.
    out.wait_until(CAMERA_PULSE_US);
    out.service_camera();
    out.all_low();
    m
}

/// Core 1 entry point. Never returns.
pub fn run(trigger: Input<'static>, mut strobe: Output<'static>, mut camera: Output<'static>) -> ! {
    strobe.set_low();
    camera.set_low();
    let mut out = Outputs {
        strobe,
        camera,
        t0: 0,
        camera_high: false,
    };
    // Require the beam to be seen intact (inactive) after arming before an
    // active level counts, so arming while the beam is already broken does
    // not fire.
    let mut ready = false;

    loop {
        if !ARMED.load(Ordering::Acquire) {
            ready = false;
            core::hint::spin_loop();
            continue;
        }
        if !trigger_active(&trigger) {
            ready = true;
            continue;
        }
        if !ready {
            continue;
        }
        ready = false;

        // --- critical path -------------------------------------------------
        out.t0 = now32();
        out.camera.set_high();
        out.camera_high = true;
        let now_us = embassy_time::Instant::now().as_micros();
        let sched = with_seq(|s| s.on_trigger(now_us));
        // -------------------------------------------------------------------

        match sched {
            // Lost a race with a disarm/lockout: retract the camera pulse.
            None => out.all_low(),
            Some(s) => {
                let measured = play(&mut out, &s);
                MEASURED.lock(|c| c.set(Some(measured)));
            }
        }
    }
}
