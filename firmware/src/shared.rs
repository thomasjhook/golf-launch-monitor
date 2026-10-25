//! State shared between core 0 (USB / Embassy) and core 1 (hard real time).

use core::cell::{Cell, RefCell};
use core::sync::atomic::AtomicU32;
use core::sync::atomic::AtomicBool;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::channel::Channel;
use firmware_core::Sequencer;
use protocol::{DeviceToHost, MAX_PULSES};

/// The one and only sequencer. `CriticalSectionRawMutex` on embassy-rp (with
/// the `critical-section-impl` feature) takes a hardware spinlock, so it is
/// safe across both cores. Hold it only for a single `Sequencer` call.
pub static SEQ: Mutex<CriticalSectionRawMutex, RefCell<Sequencer>> =
    Mutex::new(RefCell::new(Sequencer::new()));

/// Lock-free mirror of "sequencer state == Armed" so core 1 can poll without
/// touching the spinlock. ONLY written from inside the `SEQ` lock (see
/// [`with_seq`]), so it cannot be left stale by a race between writers.
pub static ARMED: AtomicBool = AtomicBool::new(false);

/// Shots reported to the host since boot (LED feedback only).
pub static SHOT_COUNT: AtomicU32 = AtomicU32::new(0);

/// Flash start times measured by core 1 for the shot it just finished,
/// microseconds after the trigger edge.
#[derive(Clone, Copy)]
pub struct Measured {
    pub count: usize,
    pub starts_us: [u32; MAX_PULSES],
}

/// Mailbox core 1 -> core 0 (single slot; the sequencer cannot start another
/// shot until core 0 has consumed this one, because it stays in `Firing`).
pub static MEASURED: Mutex<CriticalSectionRawMutex, Cell<Option<Measured>>> =
    Mutex::new(Cell::new(None));

/// Messages queued for the host.
pub static OUT: Channel<CriticalSectionRawMutex, DeviceToHost, 8> = Channel::new();

/// Run `f` on the sequencer under the lock and refresh the `ARMED` mirror
/// before releasing it.
pub fn with_seq<R>(f: impl FnOnce(&mut Sequencer) -> R) -> R {
    SEQ.lock(|cell| {
        let mut seq = cell.borrow_mut();
        let r = f(&mut seq);
        ARMED.store(
            seq.state() == firmware_core::State::Armed,
            core::sync::atomic::Ordering::Release,
        );
        r
    })
}
