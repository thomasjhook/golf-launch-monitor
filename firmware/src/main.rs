//! Launch-monitor firmware for the Raspberry Pi Pico 2 (RP2350).
//!
//! * core 1: hard-real-time trigger -> camera/strobe loop ([`realtime`])
//! * core 0: Embassy executor, USB CDC-ACM, shot forwarding ([`host`])
//!
//! UNVERIFIED ON HARDWARE. See README.md.
#![no_std]
#![no_main]

mod board;
mod host;
mod realtime;
mod shared;

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::multicore::{spawn_core1, Stack};
use embassy_rp::peripherals::USB;
use embassy_rp::usb::{Driver, InterruptHandler};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use static_cell::StaticCell;

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
});

#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [embassy_rp::binary_info::EntryAddr; 4] = [
    embassy_rp::binary_info::rp_program_name!(c"launch-monitor"),
    embassy_rp::binary_info::rp_program_description!(c"Golf launch monitor strobe/camera sequencer"),
    embassy_rp::binary_info::rp_cargo_version!(),
    embassy_rp::binary_info::rp_program_build_attribute!(),
];

static CORE1_STACK: StaticCell<Stack<4096>> = StaticCell::new();
static EXECUTOR0: StaticCell<embassy_rp::executor::Executor> = StaticCell::new();

/// Safety net: on panic or hard fault, force the outputs LOW through raw SIO
/// writes (no driver state needed), then stop. Without this the strobe pin
/// would stay in whatever state it had when the fault hit.
fn force_outputs_low() {
    embassy_rp::pac::SIO
        .gpio_out(0)
        .value_clr()
        .write_value(board::OUTPUT_LOW_MASK);
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    force_outputs_low();
    defmt::error!("panic: {}", defmt::Display2Format(info));
    loop {
        force_outputs_low();
        cortex_m::asm::wfe();
    }
}

#[cortex_m_rt::exception]
unsafe fn HardFault(_ef: &cortex_m_rt::ExceptionFrame) -> ! {
    force_outputs_low();
    loop {
        force_outputs_low();
        cortex_m::asm::wfe();
    }
}

#[cortex_m_rt::entry]
fn main() -> ! {
    let p = embassy_rp::init(Default::default());
    let (io, rest) = board::Io::take(p);
    let board::Io {
        trigger,
        strobe,
        camera,
        led,
    } = io;

    // Core 1: hard real time. Takes ownership of the trigger/strobe/camera pins.
    let stack = CORE1_STACK.init(Stack::new());
    spawn_core1(rest.core1, stack, move || realtime::run(trigger, strobe, camera));

    // USB CDC-ACM.
    let driver = Driver::new(rest.usb, Irqs);
    // 0x1209:0x0001 is the pid.codes shared "test" VID/PID: acceptable for
    // hobby/bench use, not for anything you distribute. Get your own PID
    // (pid.codes is free for open hardware) before sharing devices.
    let mut config = embassy_usb::Config::new(0x1209, 0x0001);
    config.manufacturer = Some("launch-monitor");
    config.product = Some("launch-monitor");
    config.serial_number = Some("0001");
    config.max_power = 100;
    config.max_packet_size_0 = 64;

    static CONFIG_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
    static BOS_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
    static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();
    static CDC_STATE: StaticCell<State> = StaticCell::new();
    let mut builder = embassy_usb::Builder::new(
        driver,
        config,
        CONFIG_DESCRIPTOR.init([0; 256]),
        BOS_DESCRIPTOR.init([0; 256]),
        &mut [],
        CONTROL_BUF.init([0; 64]),
    );
    let class = CdcAcmClass::new(&mut builder, CDC_STATE.init(State::new()), 64);
    let (tx, rx) = class.split();
    let usb = builder.build();

    let executor = EXECUTOR0.init(embassy_rp::executor::Executor::new());
    executor.run(|spawner: Spawner| {
        spawner.spawn(defmt::unwrap!(host::usb_task(usb)));
        spawner.spawn(defmt::unwrap!(host::cdc_task(tx, rx)));
        spawner.spawn(defmt::unwrap!(host::pump_task()));
        spawner.spawn(defmt::unwrap!(host::led_task(led)));
    })
}
