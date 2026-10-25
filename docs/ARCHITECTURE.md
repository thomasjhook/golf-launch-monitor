# Architecture and deployment plan

Decided layout for the real system. Until the hardware exists, everything still
runs on one Mac with simulated hardware (`scripts/demo.sh`).

```
 at the hitting mat                      always on                       where you sit
┌─────────────────────────┐  gRPC   ┌──────────────────────────┐  gRPC  ┌──────────────────┐
│ Raspberry Pi (Linux)    │ ──────► │ Mac mini                 │ ◄───── │ SwiftUI app      │
│  capture-daemon         │ frame + │  shot-service            │        │ (any Mac on LAN) │
│  Pico 2 (USB serial)    │ timing  │  Postgres                │        └──────────────────┘
│  global-shutter camera  │         │  (Docker Compose or k8s) │
│  ./captures archive     │         └──────────────────────────┘
└─────────────────────────┘
```

## Who runs what

| Machine | Runs | Why |
|---|---|---|
| Raspberry Pi 4/5, 64-bit Linux | `capture-daemon`, the Pico (USB serial), the camera (USB3) | The only machine that must sit next to the hardware. Linux has the best USB3 Vision and serial support. Timing-critical work is on the Pico, so the Pi's speed does not matter. |
| Mac mini | `shot-service` (gRPC), Postgres | Always on, plenty of compute, real disk for shot history, Docker Desktop / Kubernetes for deployment practice. |
| Any Mac on the LAN | SwiftUI app | Connects to the Mac mini (`SHOT_SERVICE_HOST=<mac-mini>`). |
| Windows PC (optional, later) | A bridge to a golf sim (GSPro etc.) | Sims run on Windows. The bridge would subscribe to `StreamShots`. Not built. |

## Network contract
- Pi -> Mac mini: `ShotService.ProcessShot` on port 8080. About 1.3 MB per shot.
  Prefer wired Ethernet for the Pi.
- App -> Mac mini: `ListShots` on connect, then `StreamShots`.
- Everything is plaintext gRPC with no authentication. **LAN only** until TLS and
  a token are added.
- The daemon archives each capture to the Pi's `./captures` before uploading, so a
  Mac mini outage loses nothing that cannot be replayed with `lm-replay frame`.

## Work needed to reach this layout
Done: service, Postgres persistence, app, simulated daemon, firmware (unproven on hardware).

1. Real camera backend in `capture-daemon` (blocked on choosing a camera; see the
   hardware list). Must run on Linux aarch64.
2. Real serial link to the Pico (`--mcu serial:PATH` is rejected today).
3. MCU reconnect and daemon restart handling (systemd unit on the Pi).
4. Build/run `capture-daemon` for aarch64 Linux (build on the Pi, or cross-compile).
5. Point the daemon at the Mac mini: `--service-addr http://<mac-mini>:8080`, and let
   the Mac mini's firewall accept port 8080 from the LAN.
6. Install the camera calibration with `CalibrationService.SetSetup` once the
   physical rig is mounted.
7. Firmware bring-up per `firmware/README.md`.
8. Later: TLS and an auth token; Windows sim bridge.

## Deployment on the Mac mini
`shot-service` and Postgres run either with Docker Compose (`make db-up` plus the
service) or on the Mac's local Kubernetes (`make k8s-apply`, see
`docs/DEPLOYMENT.md`). The Kubernetes manifests have not been applied to a
cluster yet.
