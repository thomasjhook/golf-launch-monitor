//! Camera abstraction. One multi-exposure frame is delivered per shot.

use std::future::Future;
use std::process::Stdio;
use std::time::Instant;

use tokio::io::AsyncReadExt;
use tokio::process::{Child, ChildStdout, Command};
use vision::GrayImage;

use crate::error::{Error, Result};

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

/// How to run `rpicam-vid` for a (trigger-mode) CSI camera such as the
/// InnoMaker IMX296.
#[derive(Debug, Clone)]
pub struct RpicamConfig {
    /// Executable; overridable so tests can substitute a stub.
    pub bin: String,
    pub width: u32,
    pub height: u32,
    /// Bytes per row of the Y plane in rpicam's YUV420 output. Equals `width`
    /// unless rpicam pads rows (verify on the Pi with a first frame).
    pub stride: u32,
    pub gain: f32,
}

impl RpicamConfig {
    /// IMX296 full frame.
    pub fn imx296() -> Self {
        Self {
            bin: "rpicam-vid".into(),
            width: 1456,
            height: 1088,
            stride: 1456,
            gain: 1.0,
        }
    }

    fn frame_bytes(&self) -> usize {
        // YUV420: full-size Y plane plus two quarter-size chroma planes.
        let y = self.stride as usize * self.height as usize;
        y + y / 2
    }
}

/// Camera driven through `rpicam-vid` raw YUV420 output on stdout; only the
/// luma plane is kept. In external-trigger mode the sensor emits a frame only
/// when triggered, so `next_frame` simply waits for the next one.
///
/// The exposure is passed as `--shutter`, so changing it restarts the child.
pub struct RpicamCamera {
    cfg: RpicamConfig,
    exposure_us: Option<u32>,
    child: Option<(Child, ChildStdout)>,
    /// Partial frame kept across calls so a cancelled `next_frame` loses nothing.
    buf: Vec<u8>,
}

impl RpicamCamera {
    pub fn new(cfg: RpicamConfig) -> Self {
        Self {
            cfg,
            exposure_us: None,
            child: None,
            buf: Vec::new(),
        }
    }

    fn spawn(&self, exposure_us: u32) -> Result<(Child, ChildStdout)> {
        let mut child = Command::new(&self.cfg.bin)
            .args([
                "--nopreview",
                "--timeout",
                "0",
                "--codec",
                "yuv420",
                "--flush",
            ])
            .args(["--width", &self.cfg.width.to_string()])
            .args(["--height", &self.cfg.height.to_string()])
            .args(["--shutter", &exposure_us.to_string()])
            .args(["--gain", &self.cfg.gain.to_string()])
            .args(["--output", "-"])
            .stdin(Stdio::null())
            .stdout(Stdio::piped())
            .kill_on_drop(true)
            .spawn()
            .map_err(|e| Error::Camera(format!("cannot start {}: {e}", self.cfg.bin)))?;
        let stdout = child
            .stdout
            .take()
            .ok_or_else(|| Error::Camera("rpicam stdout unavailable".into()))?;
        Ok((child, stdout))
    }
}

/// Copy the Y plane out of a YUV420 frame, dropping any row padding.
pub fn gray_from_yuv420(buf: &[u8], width: u32, height: u32, stride: u32) -> Result<GrayImage> {
    let (w, h, st) = (width as usize, height as usize, stride as usize);
    if st < w || buf.len() < st * h {
        return Err(Error::Camera(format!(
            "frame buffer too small: {} bytes for {width}x{height} stride {stride}",
            buf.len()
        )));
    }
    let mut data = Vec::with_capacity(w * h);
    for row in buf.chunks(st).take(h) {
        data.extend_from_slice(&row[..w]);
    }
    GrayImage::from_raw(width, height, data).map_err(|e| Error::Camera(format!("{e:?}")))
}

impl Camera for RpicamCamera {
    async fn set_exposure_us(&mut self, us: u32) -> Result<()> {
        if self.exposure_us != Some(us) {
            self.exposure_us = Some(us);
            // Restart on next use with the new --shutter.
            self.child = None;
            self.buf.clear();
        }
        Ok(())
    }

    async fn next_frame(&mut self) -> Result<CapturedFrame> {
        let exposure = self
            .exposure_us
            .ok_or_else(|| Error::Camera("exposure not set before capture".into()))?;
        if self.child.is_none() {
            self.child = Some(self.spawn(exposure)?);
        }
        let need = self.cfg.frame_bytes();
        let (_, stdout) = self.child.as_mut().expect("spawned above");
        while self.buf.len() < need {
            let n = stdout.read_buf(&mut self.buf).await?;
            if n == 0 {
                self.child = None;
                return Err(Error::Camera("rpicam exited".into()));
            }
        }
        // Keep any surplus bytes: they belong to the next frame.
        let rest = self.buf.split_off(need);
        let frame = std::mem::replace(&mut self.buf, rest);
        let received_at = Instant::now();
        let unix_ms = unix_ms_now();
        let image = gray_from_yuv420(&frame, self.cfg.width, self.cfg.height, self.cfg.stride)?;
        Ok(CapturedFrame {
            image,
            received_at,
            unix_ms,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn gray_drops_padding_and_chroma() {
        // 3x2 image, stride 4, then 3 bytes of "chroma".
        let buf = [1, 2, 3, 0, 4, 5, 6, 0, 9, 9, 9, 9];
        let img = gray_from_yuv420(&buf, 3, 2, 4).unwrap();
        assert_eq!(img.data, vec![1, 2, 3, 4, 5, 6]);
    }

    #[test]
    fn gray_rejects_short_buffer() {
        assert!(gray_from_yuv420(&[0; 5], 3, 2, 3).is_err());
    }

    /// Stub "rpicam-vid": writes two frames of 4x2 (+4 chroma bytes) and exits.
    fn stub(dir: &std::path::Path) -> String {
        let path = dir.join("fake-rpicam");
        std::fs::write(
            &path,
            "#!/bin/sh\nprintf '\\001\\001\\001\\001\\001\\001\\001\\001\\000\\000\\000\\000'\nprintf '\\002\\002\\002\\002\\002\\002\\002\\002\\000\\000\\000\\000'\n",
        )
        .unwrap();
        use std::os::unix::fs::PermissionsExt;
        std::fs::set_permissions(&path, std::fs::Permissions::from_mode(0o755)).unwrap();
        path.to_string_lossy().into_owned()
    }

    #[tokio::test]
    async fn reads_consecutive_frames_then_reports_exit() {
        let dir = std::env::temp_dir().join(format!("lm-rpicam-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let mut cam = RpicamCamera::new(RpicamConfig {
            bin: stub(&dir),
            width: 4,
            height: 2,
            stride: 4,
            gain: 1.0,
        });
        assert!(
            cam.next_frame().await.is_err(),
            "exposure must be set first"
        );
        cam.set_exposure_us(2000).await.unwrap();
        assert!(cam
            .next_frame()
            .await
            .unwrap()
            .image
            .data
            .iter()
            .all(|&v| v == 1));
        assert!(cam
            .next_frame()
            .await
            .unwrap()
            .image
            .data
            .iter()
            .all(|&v| v == 2));
        assert!(cam.next_frame().await.is_err());
        let _ = std::fs::remove_dir_all(dir);
    }
}
