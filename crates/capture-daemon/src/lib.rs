//! Host-side capture daemon: MCU link + camera + pairing + archive + upload.
//!
//! Real hardware is not available yet; `sim` provides a device running the
//! real firmware `Sequencer` and a synthetic camera that exercise the same
//! code paths. `run_supervised` reconnects and re-arms after MCU link loss.
//! `camera::RpicamCamera` drives a CSI camera through `rpicam-vid` (untested
//! on real hardware); `link::connect_serial` was verified on a Pico 2.

pub mod camera;
pub mod daemon;
pub mod error;
pub mod link;
pub mod pairing;
pub mod service;
pub mod sim;

pub use camera::{Camera, CapturedFrame};
pub use daemon::{
    run, run_supervised, DaemonConfig, DaemonEvent, ReconnectPolicy, EXPOSURE_MARGIN_US,
};
pub use error::{Error, Result};
pub use link::{LinkConnector, McuLink};
