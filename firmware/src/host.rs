//! Core 0: USB CDC-ACM link to the host, shot forwarding, status LED.
//! Thin glue: every decision is made by `firmware_core::Sequencer`.

use core::sync::atomic::Ordering;

use defmt::{info, warn, Debug2Format};
use embassy_futures::select::select;
use embassy_rp::gpio::Output;
use embassy_rp::peripherals::USB;
use embassy_rp::usb::Driver;
use embassy_time::{Instant, Timer};
use embassy_usb::class::cdc_acm::{Receiver, Sender};
use embassy_usb::driver::EndpointError;
use embassy_usb::UsbDevice;
use protocol::frame::{self, FrameDecoder, MAX_FRAME_LEN};
use protocol::{DeviceToHost, HostToDevice};

use crate::shared::{with_seq, ARMED, MEASURED, OUT, SHOT_COUNT};

type UsbDriver = Driver<'static, USB>;

#[embassy_executor::task]
pub async fn usb_task(mut device: UsbDevice<'static, UsbDriver>) -> ! {
    device.run().await
}

fn queue(msg: DeviceToHost) {
    if OUT.try_send(msg).is_err() {
        warn!("host out queue full, dropping message");
    }
}

#[embassy_executor::task]
pub async fn cdc_task(mut tx: Sender<'static, UsbDriver>, mut rx: Receiver<'static, UsbDriver>) -> ! {
    loop {
        rx.wait_connection().await;
        info!("host connected");
        while OUT.try_receive().is_ok() {}
        let mut decoder = FrameDecoder::new();

        let rx_loop = async {
            let mut buf = [0u8; 64];
            loop {
                let n = match rx.read_packet(&mut buf).await {
                    Ok(n) => n,
                    Err(e) => break e,
                };
                decoder.feed::<HostToDevice>(&buf[..n], |res| match res {
                    Ok(msg) => {
                        if let Some(reply) = with_seq(|s| s.handle(msg)) {
                            queue(reply);
                        }
                    }
                    Err(e) => warn!("bad frame: {}", Debug2Format(&e)),
                });
            }
        };
        let tx_loop = async {
            loop {
                let msg = OUT.receive().await;
                let mut buf = [0u8; MAX_FRAME_LEN];
                match frame::encode(&msg, &mut buf) {
                    Ok(f) => {
                        if let Err(e) = tx.write_packet(f).await {
                            break e;
                        }
                    }
                    Err(e) => warn!("encode failed: {}", Debug2Format(&e)),
                }
            }
        };
        let why: EndpointError = match select(rx_loop, tx_loop).await {
            embassy_futures::select::Either::First(e)
            | embassy_futures::select::Either::Second(e) => e,
        };
        warn!("host gone ({}); disarming", Debug2Format(&why));
        // Never stay armed with nobody listening.
        let _ = with_seq(|s| s.handle(HostToDevice::Disarm));
    }
}

/// Periodic housekeeping: forward finished shots and run `Sequencer::tick`.
#[embassy_executor::task]
pub async fn pump_task() -> ! {
    loop {
        // Core 1 cannot wake this executor, so poll. 1 ms is far below the
        // 250 ms lockout and the host's frame cadence.
        Timer::after_millis(1).await;
        let done = MEASURED.lock(|c| c.take());
        let now_us = Instant::now().as_micros();
        let shot = with_seq(|s| {
            let shot = done.and_then(|d| s.on_sequence_done(now_us, &d.starts_us[..d.count]));
            s.tick(now_us);
            shot
        });
        if let Some(msg) = shot {
            SHOT_COUNT.fetch_add(1, Ordering::Relaxed);
            queue(msg);
        }
    }
}

/// Fast blink = armed, slow blink = disarmed, solid for 300 ms after a shot.
#[embassy_executor::task]
pub async fn led_task(mut led: Output<'static>) -> ! {
    let mut seen = SHOT_COUNT.load(Ordering::Relaxed);
    loop {
        let count = SHOT_COUNT.load(Ordering::Relaxed);
        if count != seen {
            seen = count;
            led.set_high();
            Timer::after_millis(300).await;
            continue;
        }
        led.toggle();
        let half_period = if ARMED.load(Ordering::Relaxed) { 100 } else { 800 };
        Timer::after_millis(half_period).await;
    }
}
