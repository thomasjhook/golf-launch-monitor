use std::path::PathBuf;
use std::process::ExitCode;
use std::time::Duration;

use capture_daemon::daemon::{run, DaemonConfig, DaemonEvent};
use capture_daemon::service::{connect_lazy, install_bench_setup, RetryPolicy, ShotUploader};
use capture_daemon::sim::{demo_truth, SimRig, SimRigConfig};
use clap::Parser;
use protocol::StrobeConfig;
use tokio::sync::{mpsc, oneshot};
use tracing_subscriber::EnvFilter;

#[derive(Parser, Debug)]
#[command(about = "Launch monitor capture daemon (MCU + camera -> shot-service)")]
struct Cli {
    /// MCU backend: `sim` or `serial:/dev/cu.usbmodemXXXX`.
    #[arg(long, default_value = "sim")]
    mcu: String,
    /// Camera backend. Only `sim` exists; a real camera is a TODO.
    #[arg(long, default_value = "sim")]
    camera: String,
    #[arg(
        long,
        env = "SHOT_SERVICE_ADDR",
        default_value = "http://127.0.0.1:8080"
    )]
    service_addr: String,
    /// Archive every paired shot here (before upload).
    #[arg(long)]
    captures_dir: Option<PathBuf>,

    #[arg(long, default_value_t = 3)]
    pulses: u8,
    #[arg(long, default_value_t = 50)]
    pulse_width_us: u16,
    #[arg(long, default_value_t = 1000)]
    spacing_us: u32,
    #[arg(long, default_value_t = 0)]
    trigger_delay_us: u32,

    /// Sim: number of synthetic shots to run, then exit.
    #[arg(long, default_value_t = 3)]
    sim_shots: usize,
    /// Sim: time between shots (must exceed the 250 ms device lockout).
    #[arg(long, default_value_t = 400)]
    sim_interval_ms: u64,
    /// Sim: install the bench camera setup into the service first.
    #[arg(long)]
    install_bench_setup: bool,
}

enum McuKind {
    Sim,
    Serial(String),
}

fn parse_mcu(s: &str) -> Result<McuKind, String> {
    match s {
        "sim" => Ok(McuKind::Sim),
        _ => match s.strip_prefix("serial:") {
            Some(p) if !p.is_empty() => Ok(McuKind::Serial(p.to_string())),
            _ => Err(format!(
                "invalid --mcu {s:?}: expected `sim` or `serial:<path>`"
            )),
        },
    }
}

#[tokio::main]
async fn main() -> ExitCode {
    tracing_subscriber::fmt()
        .with_writer(std::io::stderr)
        .with_env_filter(
            EnvFilter::try_from_default_env().unwrap_or_else(|_| EnvFilter::new("info")),
        )
        .init();
    match real_main(Cli::parse()).await {
        Ok(true) => ExitCode::SUCCESS,
        Ok(false) => ExitCode::from(1),
        Err(e) => {
            eprintln!("error: {e}");
            ExitCode::from(2)
        }
    }
}

async fn ctrl_c() {
    let _ = tokio::signal::ctrl_c().await;
    tracing::info!("ctrl-c received; shutting down");
}

/// Ok(true) when every shot succeeded.
async fn real_main(cli: Cli) -> Result<bool, String> {
    let mcu = parse_mcu(&cli.mcu)?;
    if cli.camera != "sim" {
        return Err(format!(
            "unsupported --camera {:?}: only `sim` exists (real camera backend is a TODO)",
            cli.camera
        ));
    }
    let McuKind::Sim = mcu else {
        let McuKind::Serial(path) = mcu else {
            unreachable!()
        };
        // capture_daemon::link::connect_serial(&path) is ready, but the only
        // camera is the simulated one, which is driven by the simulated MCU.
        return Err(format!(
            "--mcu serial:{path} needs a real camera backend (TODO); --camera sim only works with --mcu sim"
        ));
    };

    let strobe = StrobeConfig {
        pulse_count: cli.pulses,
        pulse_width_us: cli.pulse_width_us,
        pulse_spacing_us: cli.spacing_us,
        trigger_delay_us: cli.trigger_delay_us,
    };
    strobe
        .validate()
        .map_err(|e| format!("invalid strobe flags: {e:?}"))?;
    let rig_cfg = SimRigConfig::default();
    let min_interval_ms = u64::from(rig_cfg.lockout_us) / 1000 + 20;
    if cli.sim_interval_ms < min_interval_ms {
        return Err(format!(
            "--sim-interval-ms must be at least {min_interval_ms} (device lockout)"
        ));
    }
    // Ball must move more than its diameter between flashes.
    if strobe.pulse_count > 1 {
        let min_speed = 0.0427 / (f64::from(strobe.pulse_spacing_us) * 1e-6);
        tracing::info!(
            min_speed_mps = format!("{min_speed:.1}"),
            "flash spacing requires ball speed above"
        );
    }

    let channel = connect_lazy(&cli.service_addr).map_err(|e| e.to_string())?;
    if cli.install_bench_setup {
        install_bench_setup(channel.clone(), RetryPolicy::default())
            .await
            .map_err(|e| format!("installing bench setup failed: {e}"))?;
        tracing::info!("bench setup installed");
    }
    let uploader = ShotUploader::new(channel, RetryPolicy::default());

    let SimRig {
        link,
        camera,
        golfer,
        ..
    } = SimRig::start(rig_cfg);
    let cfg = DaemonConfig {
        strobe,
        captures_dir: cli.captures_dir.clone(),
        ..DaemonConfig::default()
    };
    let (ev_tx, mut ev_rx) = mpsc::unbounded_channel();
    let (stop_tx, stop_rx) = oneshot::channel::<()>();
    let mut daemon = tokio::spawn(run(link, camera, uploader, cfg, ev_tx, async move {
        tokio::select! { () = ctrl_c() => {}, _ = stop_rx => {} }
    }));

    let n = cli.sim_shots;
    let interval = Duration::from_millis(cli.sim_interval_ms);
    let (mut done, mut ok) = (0usize, true);
    println!(
        "{:>4} | {:>17} | {:>17} | {:>17}",
        "shot", "speed m/s (t/m)", "launch deg (t/m)", "azimuth deg (t/m)"
    );
    let deadline = tokio::time::sleep(interval * n as u32 + Duration::from_secs(30));
    tokio::pin!(deadline);
    let mut stop_tx = Some(stop_tx);
    let early = loop {
        if done >= n {
            if let Some(tx) = stop_tx.take() {
                let _ = tx.send(());
            }
        }
        tokio::select! {
            res = &mut daemon => break Some(res),
            () = &mut deadline => { eprintln!("timed out waiting for shots"); ok = false; break None }
            ev = ev_rx.recv() => match ev {
                Some(DaemonEvent::Ready { .. }) => {
                    let g = golfer.clone();
                    tokio::spawn(async move { let _ = g.hit_series(n, interval, demo_truth).await; });
                }
                Some(DaemonEvent::Archived { path, .. }) => tracing::info!(path = %path.display(), "archived"),
                Some(DaemonEvent::ShotProcessed(r)) => {
                    done += 1;
                    let t = demo_truth(r.shot_id.saturating_sub(1) as usize);
                    let l = r.launch.unwrap_or_default();
                    println!(
                        "{:>4} | {:>7.1} / {:>7.1} | {:>7.1} / {:>7.1} | {:>7.1} / {:>7.1}   carry {:.0} m",
                        r.shot_id, t.speed_mps, l.ball_speed_mps, t.launch_angle_deg, l.launch_angle_deg,
                        t.azimuth_deg, l.azimuth_deg, r.flight.map(|f| f.carry_m).unwrap_or(0.0)
                    );
                }
                Some(DaemonEvent::ShotFailed { shot_id, error }) => {
                    done += 1; ok = false;
                    println!("{shot_id:>4} | FAILED: {error}");
                }
                Some(DaemonEvent::FrameWithoutShot) => { done += 1; ok = false; println!("     | frame without shot"); }
                Some(DaemonEvent::ShotWithoutFrame { shot_id }) => { done += 1; ok = false; println!("{shot_id:>4} | shot without frame"); }
                Some(DaemonEvent::LinkError(e)) => { eprintln!("link error: {e}"); ok = false; }
                None => break None,
            },
        }
    };
    let res = match early {
        Some(r) => r,
        None => {
            if let Some(tx) = stop_tx.take() {
                let _ = tx.send(());
            }
            daemon.await
        }
    };
    match res {
        Ok(Ok(())) => Ok(ok),
        Ok(Err(e)) => Err(e.to_string()),
        Err(e) => Err(format!("daemon task failed: {e}")),
    }
}
