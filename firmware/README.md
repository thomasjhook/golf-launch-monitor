# launch-monitor firmware (Pico 2 / RP2350)

Rust + Embassy firmware: break-beam trigger -> camera trigger + IR strobe
sequence, reported to the host over USB CDC-ACM using the `protocol` crate
(COBS + postcard frames). All decision logic lives in `crates/firmware-core`
(`Sequencer`, unit-tested on the host); this crate is hardware glue only.

> **STATUS: compiles and passes clippy. NEVER RUN ON HARDWARE.** Everything
> about timing, electrical behaviour and USB enumeration is unverified. See the
> bring-up checklist at the end before connecting an LED array or camera.

## Architecture

```
core 0 (Embassy executor)                  core 1 (bare metal, no interrupts)
 usb_task   usb.run()                       loop {
 cdc_task   bytes -> FrameDecoder ->          if !ARMED { spin; continue }
            Sequencer::handle -> OUT queue    wait for trigger inactive, then active
 pump_task  every 1 ms:                       t0 = TIMER0; camera HIGH
            MEASURED -> on_sequence_done      Sequencer::on_trigger  (tiny lock)
            -> Shot -> OUT queue              for each pulse: spin to t0+offset,
            Sequencer::tick                     strobe HIGH, record start, spin width, LOW
 led_task   status LED                        camera LOW after 100 us; post MEASURED
        \________ SEQ: Mutex<CriticalSectionRawMutex, RefCell<Sequencer>> ________/
```

Why this shape: the strobe timing must not depend on an async wake-up, USB
interrupts or the executor. Core 1 busy-polls a GPIO and a 1 MHz hardware
timer, so trigger->camera latency is a handful of instructions and flash
offsets are limited by loop granularity (~tens of ns at 150 MHz), unaffected by
USB traffic on core 0. The embassy-rp `critical-section-impl` takes a hardware
spinlock (SIO spinlock 31) so the `Sequencer` mutex is safe across cores
(verified in the embassy-rp 0.10.0 source). PIO would give cycle-exact timing
and is a reasonable future step, but it would have to re-implement or bypass
`Schedule` playback; the CPU approach reuses `firmware-core` unchanged and
should be good to well under a microsecond of jitter (to be measured).

Details worth knowing:

* `ARMED` is a lock-free mirror of `state == Armed`, written only inside the
  `SEQ` lock. Core 1 polls it. If it is stale-true for an instant (host
  disarms exactly as the beam breaks), core 1 may raise the camera line;
  `on_trigger` then returns `None` and core 1 immediately lowers it. That
  possible brief spurious camera pulse is a deliberate trade for "raise the
  camera before taking the lock".
* Arming while the beam is already broken does not fire: the beam must be
  seen intact (inactive) first.
* The camera pulse is dropped from inside the strobe polling loops, so it is
  100 us plus at most a couple of microseconds.
* Measured flash start times (relative to the trigger edge) go to core 0 via a
  single-slot mailbox, and `Sequencer::on_sequence_done` converts them to the
  `Shot` message (the Sequencer stays `Firing` until then, so the slot cannot
  be overwritten).
* On USB disconnect/error core 0 sends `Disarm` through the Sequencer and
  waits for a new connection. Queued replies are flushed on reconnect.
* No heap, `defmt` over RTT only (needs a probe to see logs; set `DEFMT_LOG`
  in `.cargo/config.toml`).

## Pins (all defined in `src/board.rs`)

| Signal | Pin | Direction | Notes |
|---|---|---|---|
| Break-beam trigger | GP2 | in | internal pull-up; **active-low** (`TRIGGER_ACTIVE_LOW`) |
| Strobe (MOSFET gate) | GP3 | out | active-high, LOW at boot and while disarmed |
| Camera trigger | GP4 | out | active-high, ~100 us pulse (`CAMERA_PULSE_US`) |
| Status LED | GP25 | out | Pico 2 onboard LED (not valid on Pico 2 W) |

LED: fast blink = armed, slow blink = disarmed, solid 300 ms after a shot.

```
 break-beam sensor            Pico 2
  (open-collector/NPN) ----- GP2  (pull-up in MCU; sensor pulls to GND when broken)
  Power the sensor as it requires, but GP2 is a 3.3 V logic input: level-shift
  any 5 V sensor output (the RP2350 GPIO 5 V tolerance depends on supply
  conditions; verify in the datasheet before relying on it).

                              GP3 ---[ 100R ]---+---- MOSFET gate (logic-level, e.g. Vgs(th)<2V)
                                                |
                                              [ 100k ] pull-down (keeps LED OFF while the MCU is in reset/boot)
                                                |
                                               GND
   LED supply (own, e.g. 12V) --- IR LED array --- current-limit R / CC driver --- MOSFET drain
                                                   MOSFET source --- GND (common with Pico GND)
   Flyback: if the array has any inductance (long leads, constant-current
   inductor), add a diode/TVS across the load; bypass the supply at the array.

                              GP4 ---[ level shift / opto ]--- camera trigger input
```

* **Camera trigger level warning:** GP4 is a 3.3 V push-pull output. Many
  industrial cameras expect 5 V or 12/24 V opto-isolated inputs, a sinking
  (open-collector) interface, or a specific current. Do not connect GP4 to such
  an input directly: use an optocoupler / transistor stage that suits the
  camera's I/O spec, and respect its isolation and current rating.
* **Strobe drive strength:** the firmware sets GP3 to 12 mA, fast slew, to
  charge the MOSFET gate quickly. 12 mA is the RP2350 pad maximum; never drive
  an LED directly from the pin, and keep the series gate resistor so edge
  ringing and gate current stay bounded. Lower the drive if you see ringing.
* **Hardware pull-down on the gate is required:** GPIOs float until the
  firmware runs (and during reset/BOOTSEL), and nothing in software can cover
  that.

## Safety / backstops

* Pulse widths come only from `Sequencer`'s validated `StrobeConfig`
  (`pulse_width_us <= 500`, bounded on-time per sequence). Core 1 additionally
  clamps to `StrobeConfig::MAX_PULSE_WIDTH_US`.
* The strobe pin is set low after every flash on the single code path in
  `Outputs::flash`; the high-time wait also has a bounded iteration count
  (`STROBE_MAX_SPINS`), so a stalled timer cannot hold it high.
* Panic and HardFault handlers write the raw SIO `GPIO_OUT_CLR` register to
  force GP3/GP4 low before halting.
* None of that protects against a stuck MOSFET, a watchdog-less lock-up
  mid-pulse, or a gate shorted high. The real backstop must be electrical:
  limit LED current/duty with the supply and driver (current-limited supply,
  RC one-shot or gate driver with a hard max on-time) so a stuck-high pin
  cannot cook the array. A true hardware one-shot via a PWM/PIO state machine
  was considered but is not implemented.

## Camera configuration

The camera exposure must stay open for at least
`StrobeConfig::sequence_duration_us()` (trigger delay + last flash end) **plus
the camera's own trigger-to-exposure-start latency and jitter**. Configure it
for external/hardware trigger mode, exposure time = that sum plus margin, and
a trigger edge/polarity that matches GP4 (after any level shifter). With the
default config (3 flashes, 50 us, 1 ms spacing, 0 delay) that is ~2.05 ms
plus camera latency.

## Build and flash

```sh
export PATH="$HOME/.rustup/toolchains/stable-aarch64-apple-darwin/bin:$PATH"   # this machine only
cd firmware
cargo build --release        # target thumbv8m.main-none-eabihf is the default
cargo clippy --release -- -D warnings
```

Pinned: embassy-rp 0.10.0 (`rp235xa`), embassy-executor 0.10.0, embassy-time
0.5.1, embassy-sync 0.8.0, embassy-usb 0.6.0, defmt 1.0.1, defmt-rtt 1.0.0,
cortex-m 0.7.10, cortex-m-rt 0.7.7. The RP2350 IMAGE_DEF start block is
emitted by embassy-rp itself (`.start_block`, placed by `memory.x`).

Flash (none of these tried here):

* **BOOTSEL, picotool:** hold BOOTSEL while plugging in, then
  `picotool load -u -v -x -t elf target/thumbv8m.main-none-eabihf/release/launch-monitor-firmware`
  (or uncomment that `runner` in `.cargo/config.toml` and use `cargo run --release`).
  Drag-and-drop UF2 works too: `picotool uf2 convert <elf> out.uf2 --family rp2350-arm-s`
  then copy to the RP2350 volume.
* **SWD probe:** uncomment the `probe-rs run --chip RP235x` runner and
  `cargo run --release`; this also gives defmt logs.

Release ELF is ~1.5 MB with debug info; loadable segments are ~53 KB flash and
~8 KB static RAM (core 1 stack is a separate 4 KB).

## UNVERIFIED ON HARDWARE - bring-up checklist

Do these in order, with the LED array and camera DISCONNECTED until step 5.

1. Flash; confirm onboard LED slow-blinks (disarmed). Probe GP3 and GP4 with a
   scope/meter: both must be LOW.
2. Plug into macOS; confirm enumeration as `/dev/cu.usbmodem*` and in
   `system_profiler SPUSBDataType` (VID 1209 / PID 0001, "launch-monitor").
   Confirm unplug/replug and closing the port while armed leaves the device
   disarmed and responsive.
3. Send Ping, GetStatus, SetConfig (valid and invalid), Arm via the host
   tooling; LED should fast-blink when armed.
4. With the sensor wired, break the beam: scope GP2 vs GP4 vs GP3. Measure
   trigger-to-camera-edge latency, camera pulse width (~100 us), trigger-to-
   first-flash vs `trigger_delay_us`, flash spacing and flash width against
   the config. Compare to the `Shot` timestamps reported to the host.
5. Only then add the MOSFET with a dummy load/low-current LED and verify the
   gate waveform (ringing, rise time, pull-down), then the real array with a
   current-limited supply.
6. Verify arming with the beam already broken does not fire, re-arm after the
   250 ms lockout works, and Disarm mid-shot still reports the shot.
7. Stress: hammer the USB link with Ping while shooting and confirm flash
   timing on the scope does not move.
8. Confirm the panic path: force a panic with a probe attached and check GP3
   stays low.
9. Confirm the camera actually captures all flashes within its exposure window.
