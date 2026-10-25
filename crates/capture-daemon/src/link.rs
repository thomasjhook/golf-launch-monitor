//! Host side of the MCU wire protocol (COBS-framed postcard over a byte stream).

use std::collections::VecDeque;
use std::time::Duration;

use protocol::frame::{self, FrameDecoder, MAX_FRAME_LEN};
use protocol::{DeviceToHost, HostToDevice};
use tokio::io::{AsyncRead, AsyncReadExt, AsyncWrite, AsyncWriteExt};

use crate::error::{Error, Result};

pub struct McuLink<T> {
    io: T,
    decoder: FrameDecoder,
    queue: VecDeque<DeviceToHost>,
    malformed: u64,
}

impl<T: AsyncRead + AsyncWrite + Unpin + Send> McuLink<T> {
    pub fn new(io: T) -> Self {
        Self {
            io,
            decoder: FrameDecoder::new(),
            queue: VecDeque::new(),
            malformed: 0,
        }
    }

    /// Number of malformed/oversize frames skipped so far.
    pub fn malformed_frames(&self) -> u64 {
        self.malformed
    }

    pub async fn send(&mut self, msg: HostToDevice) -> Result<()> {
        let mut buf = [0u8; MAX_FRAME_LEN];
        let bytes = frame::encode(&msg, &mut buf).map_err(Error::Encode)?;
        self.io.write_all(bytes).await?;
        self.io.flush().await?;
        Ok(())
    }

    /// Next message from the device. Cancel-safe (so usable in `select!`):
    /// partial frames stay buffered in the decoder. Malformed frames are
    /// logged and skipped. EOF is `Error::LinkClosed`.
    pub async fn recv(&mut self) -> Result<DeviceToHost> {
        let mut buf = [0u8; 256];
        loop {
            if let Some(m) = self.queue.pop_front() {
                return Ok(m);
            }
            let n = self.io.read(&mut buf).await?;
            if n == 0 {
                return Err(Error::LinkClosed);
            }
            let (queue, malformed) = (&mut self.queue, &mut self.malformed);
            self.decoder.feed::<DeviceToHost>(&buf[..n], |r| match r {
                Ok(m) => queue.push_back(m),
                Err(e) => {
                    *malformed += 1;
                    tracing::warn!(error = ?e, "skipping malformed MCU frame");
                }
            });
        }
    }

    /// Send `msg` and wait for the first reply accepted by `matcher`. Other
    /// messages received meanwhile (e.g. a `Shot`) are kept for `recv`.
    pub async fn request(
        &mut self,
        msg: HostToDevice,
        timeout: Duration,
        what: &'static str,
        matcher: impl Fn(&DeviceToHost) -> bool,
    ) -> Result<DeviceToHost> {
        self.send(msg).await?;
        let mut skipped = Vec::new();
        let waited = tokio::time::timeout(timeout, async {
            loop {
                let m = self.recv().await?;
                if matcher(&m) {
                    return Ok::<_, Error>(m);
                }
                skipped.push(m);
            }
        })
        .await;
        for m in skipped.into_iter().rev() {
            self.queue.push_front(m);
        }
        waited.map_err(|_| Error::Timeout(what))?
    }
}

/// Open a USB CDC serial port. USB CDC ignores the baud rate; 115200 is used.
///
/// NOTE: compiles but is untested without a device.
pub fn connect_serial(path: &str) -> Result<McuLink<tokio_serial::SerialStream>> {
    use tokio_serial::SerialPortBuilderExt;
    let port = tokio_serial::new(path, 115_200)
        .open_native_async()
        .map_err(|e| Error::Io(std::io::Error::other(format!("open {path}: {e}"))))?;
    Ok(McuLink::new(port))
}

#[cfg(test)]
mod tests {
    use super::*;
    use protocol::{ShotTriggered, MAX_PULSES};
    use tokio::io::duplex;

    fn encoded(msg: &DeviceToHost) -> Vec<u8> {
        let mut buf = [0u8; MAX_FRAME_LEN];
        frame::encode(msg, &mut buf).unwrap().to_vec()
    }

    fn shot(id: u32) -> DeviceToHost {
        let mut p = [0u32; MAX_PULSES];
        p[1] = 1000;
        p[2] = 2000;
        DeviceToHost::Shot(ShotTriggered {
            shot_id: id,
            pulse_count: 3,
            pulse_times_us: p,
        })
    }

    #[tokio::test]
    async fn host_message_arrives_framed() {
        let (host, mut dev) = duplex(1024);
        let mut link = McuLink::new(host);
        link.send(HostToDevice::Ping(9)).await.unwrap();
        let mut buf = [0u8; 64];
        let n = dev.read(&mut buf).await.unwrap();
        let mut dec = FrameDecoder::new();
        let mut got = vec![];
        dec.feed::<HostToDevice>(&buf[..n], |m| got.push(m));
        assert_eq!(got, vec![Ok(HostToDevice::Ping(9))]);
    }

    #[tokio::test]
    async fn fragmented_reads_and_garbage_between_frames() {
        let (host, mut dev) = duplex(1024);
        let mut link = McuLink::new(host);
        let mut stream = encoded(&DeviceToHost::Pong(1));
        stream.extend_from_slice(&[0xff, 0x13, 0x37, 0x00]); // garbage frame
        stream.extend(encoded(&shot(5)));
        stream.extend_from_slice(&[0x55; 3 * MAX_FRAME_LEN]); // oversize junk
        stream.push(0x00);
        stream.extend(encoded(&DeviceToHost::Armed));

        let writer = tokio::spawn(async move {
            for byte in stream {
                dev.write_all(&[byte]).await.unwrap();
                tokio::task::yield_now().await;
            }
            dev
        });
        assert_eq!(link.recv().await.unwrap(), DeviceToHost::Pong(1));
        assert_eq!(link.recv().await.unwrap(), shot(5));
        assert_eq!(link.recv().await.unwrap(), DeviceToHost::Armed);
        assert!(link.malformed_frames() >= 1);
        drop(writer.await.unwrap());
        assert!(matches!(link.recv().await, Err(Error::LinkClosed)));
    }

    #[tokio::test]
    async fn request_keeps_unrelated_messages_in_order() {
        let (host, mut dev) = duplex(1024);
        let mut link = McuLink::new(host);
        let mut bytes = encoded(&shot(1));
        bytes.extend(encoded(&DeviceToHost::Pong(3)));
        dev.write_all(&bytes).await.unwrap();
        let r = link
            .request(
                HostToDevice::Ping(3),
                Duration::from_millis(200),
                "ping",
                |m| matches!(m, DeviceToHost::Pong(_)),
            )
            .await
            .unwrap();
        assert_eq!(r, DeviceToHost::Pong(3));
        assert_eq!(link.recv().await.unwrap(), shot(1));
    }

    #[tokio::test]
    async fn request_times_out() {
        let (host, _dev) = duplex(1024);
        let mut link = McuLink::new(host);
        let r = link
            .request(
                HostToDevice::GetStatus,
                Duration::from_millis(20),
                "status",
                |_| true,
            )
            .await;
        assert!(matches!(r, Err(Error::Timeout("status"))));
    }
}
