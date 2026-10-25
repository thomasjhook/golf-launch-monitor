//! Local test client for shot-service: install a camera setup, push synthetic
//! shots through the real gRPC path, and watch the live result stream.

use std::path::PathBuf;

use clap::{Parser, Subcommand};
use launch_core::FlashOrder;
use launch_proto::pb::calibration_service_client::CalibrationServiceClient;
use launch_proto::pb::shot_service_client::ShotServiceClient;
use launch_proto::pb::{self, CameraSetup, ProcessShotRequest, StreamShotsRequest};
use launch_sim::{bench_calibration, default_noise, render_shot, Truth};
use launch_types::StrobeTiming;

type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;

#[derive(Parser)]
#[command(
    name = "lm-replay",
    about = "Drive shot-service with synthetic shots",
    allow_negative_numbers = true
)]
struct Cli {
    /// shot-service address.
    #[arg(
        long,
        env = "SHOT_SERVICE_ADDR",
        default_value = "http://127.0.0.1:8080"
    )]
    addr: String,
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    /// Install the bench camera setup that the synthetic frames assume.
    Setup,
    /// Render a synthetic shot and process it.
    #[command(allow_negative_numbers = true)]
    Shot {
        #[arg(long, default_value_t = 58.0)]
        speed_mps: f64,
        #[arg(long, default_value_t = 14.0)]
        launch_deg: f64,
        /// Positive = to the right.
        #[arg(long, default_value_t = 0.0)]
        azimuth_deg: f64,
        #[arg(long, default_value_t = 1)]
        shot_id: u32,
        /// Microseconds between strobe flashes (must exceed ball diameter / speed).
        #[arg(long, default_value_t = 1000)]
        spacing_us: u32,
        #[arg(long, default_value_t = 3)]
        flashes: u32,
    },
    /// Replay saved captures (from capture-daemon) through the service.
    Frame {
        /// A capture's .json metadata file, or a directory of captures
        /// (replayed oldest first).
        path: PathBuf,
    },
    /// Print results as shots are processed, until interrupted.
    Watch,
}

fn print_shot(r: &pb::ShotResult) {
    let l = r.launch.unwrap_or_default();
    let f = r.flight.clone().unwrap_or_default();
    println!(
        "shot {:>4} | {:5.1} m/s  launch {:5.1}°  azimuth {:+5.1}° | carry {:6.1} m ({:5.1} yd)  apex {:5.1} m  lateral {:+6.1} m",
        r.shot_id,
        l.ball_speed_mps,
        l.launch_angle_deg,
        l.azimuth_deg,
        f.carry_m,
        f.carry_m * 1.093_61,
        f.apex_height_m,
        f.lateral_m,
    );
}

#[tokio::main]
async fn main() -> Result<()> {
    let cli = Cli::parse();
    let channel = tonic::transport::Channel::from_shared(cli.addr.clone())?
        .connect()
        .await
        .map_err(|e| format!("cannot reach {}: {e}", cli.addr))?;

    match cli.command {
        Command::Setup => {
            let setup = CameraSetup {
                calibration: Some((&bench_calibration([0.0; 5])).into()),
                flash_order: pb::FlashOrder::from(FlashOrder::XDescending) as i32,
            };
            CalibrationServiceClient::new(channel)
                .set_setup(setup)
                .await?;
            println!("bench camera setup installed");
        }
        Command::Shot {
            speed_mps,
            launch_deg,
            azimuth_deg,
            shot_id,
            spacing_us,
            flashes,
        } => {
            let pulses: Vec<u32> = (0..flashes).map(|i| i * spacing_us).collect();
            let truth = Truth {
                speed_mps,
                launch_angle_deg: launch_deg,
                azimuth_deg,
            };
            let frame = render_shot(
                &bench_calibration([0.0; 5]),
                &truth,
                &pulses,
                &default_noise(),
            );
            let request = ProcessShotRequest {
                frame: Some((&frame).into()),
                timing: Some(
                    (&StrobeTiming {
                        shot_id,
                        pulse_times_us: pulses,
                    })
                        .into(),
                ),
            };
            let response = ShotServiceClient::new(channel)
                .process_shot(request)
                .await?;
            println!(
                "truth        | {speed_mps:5.1} m/s  launch {launch_deg:5.1}°  azimuth {azimuth_deg:+5.1}°"
            );
            if let Some(result) = response.into_inner().result {
                print_shot(&result);
            }
        }
        Command::Frame { path } => {
            let metas = if path.is_dir() {
                capture_store::list(&path)?
            } else {
                vec![path]
            };
            if metas.is_empty() {
                return Err("no captures found".into());
            }
            let mut client = ShotServiceClient::new(channel);
            let mut failures = 0;
            for meta in metas {
                let capture = capture_store::load(&meta)?;
                let shot_id = capture.timing.shot_id;
                let request = ProcessShotRequest {
                    frame: Some((&capture.frame).into()),
                    timing: Some((&capture.timing).into()),
                };
                match client.process_shot(request).await {
                    Ok(response) => {
                        if let Some(result) = response.into_inner().result {
                            print_shot(&result);
                        }
                    }
                    Err(status) => {
                        failures += 1;
                        println!("shot {shot_id:>4} | failed: {}", status.message());
                    }
                }
            }
            if failures > 0 {
                return Err(format!("{failures} capture(s) failed").into());
            }
        }
        Command::Watch => {
            let mut stream = ShotServiceClient::new(channel)
                .stream_shots(StreamShotsRequest {})
                .await?
                .into_inner();
            println!("watching for shots (ctrl-c to stop)...");
            while let Some(result) = stream.message().await? {
                print_shot(&result);
            }
        }
    }
    Ok(())
}
