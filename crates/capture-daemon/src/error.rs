//! Daemon error type.

use protocol::frame::FrameError;
use protocol::{ConfigError, DeviceToHost};

pub type Result<T, E = Error> = std::result::Result<T, E>;

#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("MCU link I/O error: {0}")]
    Io(#[from] std::io::Error),
    /// The MCU stream hit EOF (e.g. the Pico was unplugged).
    #[error("MCU link closed unexpectedly")]
    LinkClosed,
    #[error("timed out waiting for MCU response to {0}")]
    Timeout(&'static str),
    #[error("could not encode message for the MCU: {0:?}")]
    Encode(FrameError),
    #[error("strobe config is invalid on the host side: {0:?}")]
    InvalidConfig(ConfigError),
    #[error("MCU rejected the strobe config ({0:?}); refusing to arm")]
    ConfigRejected(ConfigError),
    #[error("MCU applied a different config than requested: {0:?}")]
    ConfigMismatch(DeviceToHost),
    #[error("camera error: {0}")]
    Camera(String),
    #[error("shot service error: {}", fmt_status(.0))]
    Service(Box<tonic::Status>),
    #[error("capture store error: {0}")]
    Store(#[from] capture_store::StoreError),
    #[error("setup error: {0}")]
    Setup(String),
}

impl From<tonic::Status> for Error {
    fn from(s: tonic::Status) -> Self {
        Self::Service(Box::new(s))
    }
}

/// `Code: message`, so logs and events carry the gRPC code too.
pub fn fmt_status(s: &tonic::Status) -> String {
    format!("{:?}: {}", s.code(), s.message())
}

impl Error {
    /// True for failures of the MCU link itself (as opposed to bad config, a
    /// dead camera, ...), which a fresh connection may fix.
    pub fn is_link_failure(&self) -> bool {
        matches!(
            self,
            Error::Io(_)
                | Error::LinkClosed
                | Error::Timeout(_)
                | Error::ConfigRejected(ConfigError::Busy)
        )
    }
}
