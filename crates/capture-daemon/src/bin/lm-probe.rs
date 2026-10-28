//! Bench tool for talking to the Pico directly: ping, status, config, arm,
//! and a live shot listener. Uses the same `McuLink` as the daemon.
//! Only one process can hold the serial port, so stop the daemon first.

use std::process::ExitCode;
use std::time::{Duration, Instant};

use capture_daemon::link::{connect_serial, McuLink};
use clap::{Args, Parser, Subcommand};
use protocol::{DeviceToHost, HostToDevice, StrobeConfig};
use tokio::io::{AsyncRead, AsyncWrite};

const REPLY_TIMEOUT: Duration = Duration::from_secs(2);

#[derive(Parser, Debug)]
#[command(about = "Talk to the launch-monitor MCU over USB serial")]
struct Cli {
    /// Serial device, e.g. /dev/cu.usbmodemXXXX or /dev/ttyACM0.
    #[arg(long, short)]
    port: String,
    #[command(subcommand)]
    cmd: Cmd,
}

#[derive(Args, Debug)]
struct ConfigArgs {
    #[arg(long, default_value_t = 3)]
    pulses: u8,
    #[arg(long, default_value_t = 50)]
    pulse_width_us: u16,
    #[arg(long, default_value_t = 1000)]
    spacing_us: u32,
    #[arg(long, default_value_t = 0)]
    trigger_delay_us: u32,
}

impl ConfigArgs {
    fn config(&self) -> StrobeConfig {
        StrobeConfig {
            pulse_count: self.pulses,
            pulse_width_us: self.pulse_width_us,
            pulse_spacing_us: self.spacing_us,
            trigger_delay_us: self.trigger_delay_us,
        }
    }
}

#[derive(Subcommand, Debug)]
enum Cmd {
    /// Round-trip `Ping`s and report latency.
    Ping {
        #[arg(short = 'n', long, default_value_t = 20)]
        count: u32,
    },
    /// Print firmware version, armed state, shot count and config.
    Status,
    /// Send a `SetConfig` and print whether the device accepted it.
    Config(ConfigArgs),
    /// Arm the strobe, wait for Enter, then disarm.
    Arm,
    /// Force a `Disarm`.
    Disarm,
    /// Apply a config, arm, and print every shot until ctrl-c; then disarm.
    Listen(ConfigArgs),
}

#[tokio::main]
async fn main() -> ExitCode {
    let cli = Cli::parse();
    let mut link = match connect_serial(&cli.port) {
        Ok(l) => l,
        Err(e) => {
            eprintln!("error: {e}");
            return ExitCode::from(2);
        }
    };
    match run(&mut link, cli.cmd).await {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("error: {e}");
            ExitCode::from(1)
        }
    }
}

async fn run<T>(link: &mut McuLink<T>, cmd: Cmd) -> Result<(), String>
where
    T: AsyncRead + AsyncWrite + Unpin + Send,
{
    match cmd {
        Cmd::Ping { count } => ping(link, count).await,
        Cmd::Status => {
            print_status(link).await?;
            Ok(())
        }
        Cmd::Config(a) => set_config(link, a.config()).await,
        Cmd::Arm => {
            arm(link).await?;
            println!("armed; press Enter to disarm");
            let _ = tokio::task::spawn_blocking(|| {
                let mut s = String::new();
                std::io::stdin().read_line(&mut s)
            })
            .await;
            disarm(link).await
        }
        Cmd::Disarm => disarm(link).await,
        Cmd::Listen(a) => listen(link, a.config()).await,
    }
}

async fn ping<T>(link: &mut McuLink<T>, count: u32) -> Result<(), String>
where
    T: AsyncRead + AsyncWrite + Unpin + Send,
{
    let mut rtts = Vec::new();
    for i in 0..count {
        let start = Instant::now();
        link.request(
            HostToDevice::Ping(i),
            REPLY_TIMEOUT,
            "ping",
            |m| matches!(m, DeviceToHost::Pong(n) if *n == i),
        )
        .await
        .map_err(|e| format!("ping {i}: {e}"))?;
        rtts.push(start.elapsed());
    }
    if rtts.is_empty() {
        return Ok(());
    }
    rtts.sort();
    let sum: Duration = rtts.iter().sum();
    println!(
        "{} pings: min {:?} median {:?} mean {:?} max {:?}",
        rtts.len(),
        rtts[0],
        rtts[rtts.len() / 2],
        sum / rtts.len() as u32,
        rtts[rtts.len() - 1]
    );
    Ok(())
}

async fn print_status<T>(link: &mut McuLink<T>) -> Result<protocol::Status, String>
where
    T: AsyncRead + AsyncWrite + Unpin + Send,
{
    let reply = link
        .request(HostToDevice::GetStatus, REPLY_TIMEOUT, "status", |m| {
            matches!(m, DeviceToHost::Status(_))
        })
        .await
        .map_err(|e| e.to_string())?;
    let DeviceToHost::Status(s) = reply else {
        unreachable!("matcher only accepts Status")
    };
    println!(
        "fw v{} armed={} shots_fired={} config={:?}",
        s.fw_version, s.armed, s.shots_fired, s.config
    );
    Ok(s)
}

async fn set_config<T>(link: &mut McuLink<T>, cfg: StrobeConfig) -> Result<(), String>
where
    T: AsyncRead + AsyncWrite + Unpin + Send,
{
    if let Err(e) = cfg.validate() {
        println!("(host-side validation would reject this: {e:?}; sending anyway)");
    }
    let reply = link
        .request(
            HostToDevice::SetConfig(cfg),
            REPLY_TIMEOUT,
            "set-config",
            |m| {
                matches!(
                    m,
                    DeviceToHost::ConfigApplied(_) | DeviceToHost::ConfigRejected(_)
                )
            },
        )
        .await
        .map_err(|e| e.to_string())?;
    match reply {
        DeviceToHost::ConfigApplied(c) => {
            println!("config applied: {c:?}");
            Ok(())
        }
        DeviceToHost::ConfigRejected(e) => Err(format!("config rejected by device: {e:?}")),
        _ => unreachable!("matcher only accepts config replies"),
    }
}

async fn arm<T>(link: &mut McuLink<T>) -> Result<(), String>
where
    T: AsyncRead + AsyncWrite + Unpin + Send,
{
    link.request(HostToDevice::Arm, REPLY_TIMEOUT, "arm", |m| {
        matches!(m, DeviceToHost::Armed)
    })
    .await
    .map(|_| ())
    .map_err(|e| e.to_string())
}

async fn disarm<T>(link: &mut McuLink<T>) -> Result<(), String>
where
    T: AsyncRead + AsyncWrite + Unpin + Send,
{
    link.request(HostToDevice::Disarm, REPLY_TIMEOUT, "disarm", |m| {
        matches!(m, DeviceToHost::Disarmed)
    })
    .await
    .map_err(|e| e.to_string())?;
    println!("disarmed");
    Ok(())
}

async fn listen<T>(link: &mut McuLink<T>, cfg: StrobeConfig) -> Result<(), String>
where
    T: AsyncRead + AsyncWrite + Unpin + Send,
{
    set_config(link, cfg).await?;
    arm(link).await?;
    println!("armed; waiting for shots (ctrl-c to stop)");
    let start = Instant::now();
    let result = loop {
        tokio::select! {
            _ = tokio::signal::ctrl_c() => break Ok(()),
            msg = link.recv() => match msg {
                Ok(DeviceToHost::Shot(s)) => println!(
                    "[{:>8.3}s] shot #{} pulses={} times_us={:?}",
                    start.elapsed().as_secs_f64(),
                    s.shot_id,
                    s.pulse_count,
                    s.pulses()
                ),
                Ok(other) => println!("[{:>8.3}s] {other:?}", start.elapsed().as_secs_f64()),
                Err(e) => break Err(e.to_string()),
            },
        }
    };
    if result.is_ok() {
        disarm(link).await?;
    }
    result
}
