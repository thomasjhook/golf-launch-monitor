//! COBS framing for postcard messages over a byte stream (USB serial).
//!
//! Each frame is `cobs(postcard(msg)) ++ 0x00`. A reader accumulates bytes
//! until the delimiter, so garbage or a half-sent frame only costs that frame.

use postcard::accumulator::{CobsAccumulator, FeedResult};
use serde::de::DeserializeOwned;
use serde::Serialize;

/// Upper bound on one encoded frame including the delimiter. The largest
/// message (a full `ShotTriggered`) is well under this.
pub const MAX_FRAME_LEN: usize = 64;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FrameError {
    /// Output buffer too small for the encoded message.
    BufferTooSmall,
    /// A frame arrived but did not decode as the expected message.
    Malformed,
    /// Bytes arrived without a delimiter for more than `MAX_FRAME_LEN`.
    Overflow,
}

/// Encode `msg` into `buf`, returning the used prefix (including the
/// trailing delimiter).
pub fn encode<'a, T: Serialize>(msg: &T, buf: &'a mut [u8]) -> Result<&'a mut [u8], FrameError> {
    postcard::to_slice_cobs(msg, buf).map_err(|_| FrameError::BufferTooSmall)
}

/// Incremental frame reader. Feed it whatever bytes the port yields.
pub struct FrameDecoder {
    acc: CobsAccumulator<MAX_FRAME_LEN>,
}

impl Default for FrameDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl FrameDecoder {
    pub const fn new() -> Self {
        Self {
            acc: CobsAccumulator::new(),
        }
    }

    /// Feed bytes; `on_msg` is called once per completed frame, with either
    /// the decoded message or the reason the frame was dropped.
    pub fn feed<T: DeserializeOwned>(
        &mut self,
        mut bytes: &[u8],
        mut on_msg: impl FnMut(Result<T, FrameError>),
    ) {
        while !bytes.is_empty() {
            bytes = match self.acc.feed::<T>(bytes) {
                FeedResult::Consumed => return,
                FeedResult::OverFull(rest) => {
                    on_msg(Err(FrameError::Overflow));
                    rest
                }
                FeedResult::DeserError(rest) => {
                    on_msg(Err(FrameError::Malformed));
                    rest
                }
                FeedResult::Success { data, remaining } => {
                    on_msg(Ok(data));
                    remaining
                }
            };
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{DeviceToHost, HostToDevice, ShotTriggered, StrobeConfig, MAX_PULSES};

    fn collect<T: DeserializeOwned>(
        dec: &mut FrameDecoder,
        bytes: &[u8],
    ) -> Vec<Result<T, FrameError>> {
        let mut out = Vec::new();
        dec.feed(bytes, |m| out.push(m));
        out
    }

    #[test]
    fn round_trip_every_host_message() {
        let msgs = [
            HostToDevice::Arm,
            HostToDevice::Disarm,
            HostToDevice::SetConfig(StrobeConfig::default()),
            HostToDevice::GetStatus,
            HostToDevice::Ping(0xdead_beef),
        ];
        let mut dec = FrameDecoder::new();
        for m in msgs {
            let mut buf = [0u8; MAX_FRAME_LEN];
            let bytes = encode(&m, &mut buf).unwrap();
            assert_eq!(*bytes.last().unwrap(), 0, "frame ends with delimiter");
            assert_eq!(collect::<HostToDevice>(&mut dec, bytes), vec![Ok(m)]);
        }
    }

    #[test]
    fn worst_case_shot_fits_in_a_frame() {
        let shot = DeviceToHost::Shot(ShotTriggered {
            shot_id: u32::MAX,
            pulse_count: MAX_PULSES as u8,
            pulse_times_us: [u32::MAX; MAX_PULSES],
        });
        let mut buf = [0u8; MAX_FRAME_LEN];
        let bytes = encode(&shot, &mut buf).expect("fits in MAX_FRAME_LEN");
        let mut dec = FrameDecoder::new();
        assert_eq!(collect::<DeviceToHost>(&mut dec, bytes), vec![Ok(shot)]);
    }

    #[test]
    fn byte_at_a_time_and_back_to_back_frames() {
        let (a, b) = (HostToDevice::Ping(1), HostToDevice::Arm);
        let mut stream = Vec::new();
        for m in [a, b] {
            let mut buf = [0u8; MAX_FRAME_LEN];
            stream.extend_from_slice(encode(&m, &mut buf).unwrap());
        }
        let mut dec = FrameDecoder::new();
        let mut got = Vec::new();
        for byte in &stream {
            dec.feed::<HostToDevice>(core::slice::from_ref(byte), |m| got.push(m));
        }
        assert_eq!(got, vec![Ok(a), Ok(b)]);

        // Same bytes delivered in one chunk.
        let mut dec = FrameDecoder::new();
        assert_eq!(
            collect::<HostToDevice>(&mut dec, &stream),
            vec![Ok(a), Ok(b)]
        );
    }

    #[test]
    fn recovers_after_garbage() {
        let good = HostToDevice::Ping(7);
        let mut buf = [0u8; MAX_FRAME_LEN];
        let frame = encode(&good, &mut buf).unwrap().to_vec();

        let mut stream = vec![0xff, 0x13, 0x37, 0x00]; // junk, terminated
        stream.extend_from_slice(&frame);
        let mut dec = FrameDecoder::new();
        let got = collect::<HostToDevice>(&mut dec, &stream);
        assert_eq!(got.last(), Some(&Ok(good)), "{got:?}");
        assert!(got[..got.len() - 1].iter().all(Result::is_err));
    }

    #[test]
    fn oversize_garbage_overflows_then_recovers() {
        let mut dec = FrameDecoder::new();
        let junk = [0x55u8; MAX_FRAME_LEN * 3];
        let got = collect::<HostToDevice>(&mut dec, &junk);
        assert!(got.iter().any(|r| r == &Err(FrameError::Overflow)));

        let mut buf = [0u8; MAX_FRAME_LEN];
        let mut stream = vec![0x00];
        stream.extend_from_slice(encode(&HostToDevice::Arm, &mut buf).unwrap());
        let got = collect::<HostToDevice>(&mut dec, &stream);
        assert_eq!(got.last(), Some(&Ok(HostToDevice::Arm)));
    }

    #[test]
    fn encode_reports_small_buffers() {
        let mut tiny = [0u8; 2];
        assert_eq!(
            encode(&HostToDevice::SetConfig(StrobeConfig::default()), &mut tiny),
            Err(FrameError::BufferTooSmall)
        );
    }
}
