//! Host-side capture daemon: MCU link + camera + pairing + archive + upload.
//!
//! Real hardware is not available yet; `sim` provides a device running the
//! real firmware `Sequencer` and a synthetic camera that exercise the same
//! code paths. Known limitations: no MCU reconnect (link loss is a fatal
//! error), no real camera backend (`Camera` is the seam), and
//! `link::connect_serial` is untested without a device.

pub mod camera;
pub mod daemon;
pub mod error;
pub mod link;
pub mod pairing;
pub mod service;
pub mod sim;

pub use camera::{Camera, CapturedFrame};
pub use daemon::{run, DaemonConfig, DaemonEvent, EXPOSURE_MARGIN_US};
pub use error::{Error, Result};
pub use link::McuLink;
