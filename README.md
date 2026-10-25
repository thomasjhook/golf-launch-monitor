# launch-monitor

Hobby golf launch monitor. A global-shutter camera plus a multi-pulse IR strobe
captures one multi-exposure frame per shot; this workspace turns that frame and
the strobe timing into launch conditions and a simulated ball flight.

Build natively with rustup; `rust-toolchain.toml` pins the channel and pulls in
clippy + rustfmt. Docker is only used to package `shot-service`.

```bash
cargo test --workspace
cargo clippy --workspace --all-targets -- -D warnings
cargo fmt --all
make docker-build   # shot-service image
make k8s-render     # kustomize manifests (see docs/DEPLOYMENT.md)
```

## Try it (gRPC)

```bash
cargo run -p shot-service &                 # listens on 0.0.0.0:8080
cargo run -p replay-cli -- --addr http://127.0.0.1:8080 setup
cargo run -p replay-cli -- --addr http://127.0.0.1:8080 shot --speed-mps 62 --launch-deg 12 --azimuth-deg -2
grpcurl -plaintext localhost:8080 list      # reflection is enabled
grpcurl -plaintext localhost:8080 describe launchmonitor.v1.ShotService
```

Planned deployment: Raspberry Pi (capture-daemon, Pico, camera) -> Mac mini
(shot-service + Postgres) <- SwiftUI app. See `docs/ARCHITECTURE.md` and
`docs/SHOT_FLOW.md`.

## Shot history (Postgres)

`shot-service` stores every processed shot in Postgres and serves it through
`ShotService.ListShots` (newest first, keyset paging). With no database
configured it falls back to an in-memory history. Configure with
`DATABASE_URL`, or the libpq variables (`PGHOST`, `PGUSER`, `PGPASSWORD`,
`PGDATABASE`), which is how Kubernetes passes the password from a Secret.
Migrations in `crates/shot-service/migrations` run at startup.

```bash
make db-up                                     # local Postgres (docker compose)
DATABASE_URL=postgres://launch:launch-dev@127.0.0.1:5432/launchmonitor cargo run -p shot-service &
grpcurl -plaintext -d '{"page_size":5}' localhost:8080 launchmonitor.v1.ShotService/ListShots
make test-db                                   # Postgres integration tests
```

## Try it: simulated hardware end to end

The capture daemon drives a simulated device (the real firmware `Sequencer`
over the real wire protocol) and a simulated camera, and uploads to the service:

```bash
cargo run -p shot-service &
cargo run -p capture-daemon -- --mcu sim --camera sim --install-bench-setup \
    --sim-shots 3 --captures-dir ./captures
cargo run -p replay-cli -- frame ./captures     # replay saved captures later
```

## Crates

| Crate | Role |
|---|---|
| `launch-types` | Shared domain types, units, and coordinate frames |
| `protocol` | `no_std` firmware <-> host messages |
| `vision` | Ball detection in a multi-exposure frame, plus a synthetic frame renderer |
| `calibration` | Lens model, pixel -> 3D ball position, JSON calibration file |
| `metrics` | Ball positions/rotations -> speed, launch angle, azimuth, spin |
| `flight-model` | RK4 trajectory with drag + Magnus lift -> carry, apex, total |
| `launch-core` | `Pipeline`: frame + strobe timing -> `ShotResult` |
| `launch-proto` | `.proto` contract (built with pure-Rust `protox`, no `protoc`), tonic stubs, wire <-> domain conversions |
| `launch-sim` | Synthetic multi-exposure shot renderer (tests and replay client) |
| `shot-service` | tonic gRPC server: shot processing, live result stream, shot history (Postgres), camera setup, health, reflection |
| `replay-cli` | `lm-replay`: drive the service with synthetic shots or replay saved captures |
| `firmware-core` | `no_std` shot sequencer state machine (arm/trigger/lockout/config), unit-tested on the host |
| `capture-store` | On-disk capture format (PGM frame + JSON timing) used by the daemon and `lm-replay frame` |
| `capture-daemon` | Host side of the hardware: MCU link, camera, frame/shot pairing, archive, gRPC upload; simulated device and camera included |
| `app/LaunchMonitor` | SwiftUI macOS app (SwiftPM, grpc-swift 2): history + live shots; `scripts/demo.sh` runs the whole stack |
| `firmware/` | Pico 2 (RP2350, embassy-rp) glue, **not** in the workspace; see `firmware/README.md` |

Frames: world is +X downrange, +Y up, +Z golfer's right. See
`crates/launch-types/src/lib.rs` for all conventions.

## Design constraints learned so far

- **Flash spacing must exceed ball diameter / ball speed** (about 0.8 ms at
  55 m/s), or consecutive ball images overlap and cannot be separated. The
  end-to-end test uses 1 ms.
- Spin is not measured yet; `Pipeline` reports zero spin until rotation
  tracking is added. **Carry is therefore unrealistically short for now** (a
  zero-spin ball flies much lower than a real shot); launch speed and angles
  are the numbers to trust.
- `total_m` uses a crude bounce/roll model; once spin is measured, prefer
  `carry_m` over it.

## Status

The firmware builds for the Pico 2 but has never run on hardware (see the
bring-up checklist in `firmware/README.md`). There is no real camera backend
yet: the capture daemon only supports `--camera sim`. Everything is exercised
against synthetic frames; real-world accuracy is unvalidated.
