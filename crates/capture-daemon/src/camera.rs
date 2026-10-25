//! Camera abstraction. One multi-exposure frame is delivered per shot.

use std::future::Future;
use std::time::Instant;

use vision::GrayImage;

use crate::error::Result;

/// A frame as delivered by the camera.
#[derive(Debug, Clone)]
pub struct CapturedFrame {
    pub image: GrayImage,
    /// Monotonic time the frame reached the host (used for shot pairing).
    pub received_at: Instant,
    /// Wall-clock time the frame reached the host (used for archiving).
    pub unix_ms: u64,
}

/// A camera. Used through generics (static dispatch) so native async fns work.
pub trait Camera: Send {
    /// Exposure time measured from the camera trigger. Must cover the whole
    /// strobe sequence or later flashes are missing from the frame.
    fn set_exposure_us(&mut self, us: u32) -> impl Future<Output = Result<()>> + Send;

    /// Wait for the next frame.
    fn next_frame(&mut self) -> impl Future<Output = Result<CapturedFrame>> + Send;
}

pub fn unix_ms_now() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_millis() as u64)
        .unwrap_or(0)
}

/// Never delivers a frame. For bringing up the MCU link before a real camera
/// backend exists: every shot is then reported as `ShotWithoutFrame`.
pub struct NullCamera;

impl Camera for NullCamera {
    async fn set_exposure_us(&mut self, _us: u32) -> Result<()> {
        Ok(())
    }

    async fn next_frame(&mut self) -> Result<CapturedFrame> {
        std::future::pending().await
    }
}
