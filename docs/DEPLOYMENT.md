# Deployment

## Pieces
- `crates/shot-service`: gRPC server (`ShotService`, `CalibrationService`, `grpc.health.v1.Health`, reflection). Contract: `crates/launch-proto/proto/`. Env: `LISTEN_ADDR` (default `0.0.0.0:8080`), `RUST_LOG`, `DATA_DIR` (camera setup is persisted to `$DATA_DIR/setup.pb`).
- Shot history: Postgres (`deploy/k8s/postgres.yaml`: StatefulSet, headless Service, 2Gi PVC via `volumeClaimTemplates`, NetworkPolicy allowing only shot-service). shot-service connects with `PGHOST`/`PGUSER`/`PGDATABASE` from the ConfigMap and `PGPASSWORD` from the `postgres-credentials` Secret, retries for up to 90 s while Postgres starts, then applies migrations. If it can't connect it exits (crash-loops) rather than silently dropping history. A DB outage after startup never loses a shot: it is returned and streamed but not stored (error logged).
- `Dockerfile`: multi-stage (cargo-chef dep caching) -> distroless `cc-debian12:nonroot`.
- `deploy/k8s`: kustomize (namespace `launch-monitor`, Deployment, Service :8080 (h2c), gRPC health probes, ConfigMap, PVC `shot-data` 1Gi at `/data`).
- `Makefile`: `build test clippy fmt` (native `cargo`), `docker-build docker-run`, `k8s-render k8s-apply k8s-delete`.

## Quick start
    make docker-build            # native arch (arm64 on Apple Silicon)
    make docker-run              # then: grpcurl -plaintext localhost:8080 list
    make k8s-render              # kubectl kustomize deploy/k8s
    make k8s-apply               # LOCAL clusters only (docker-desktop, kind-*, minikube)
                                 # first creates the namespace and a random postgres-credentials Secret (not in git)

amd64 image: `docker buildx build --platform linux/amd64 -t launch-monitor/shot-service:dev --load .`
With kind: `kind load docker-image launch-monitor/shot-service:dev`; minikube: `minikube image load ...`.
Without a cluster, `kubectl apply --dry-run=client` needs `--validate=false` (no API server for discovery).

See `docs/ARCHITECTURE.md` for the planned layout (Pi: capture-daemon, Pico, camera; Mac mini: shot-service + Postgres).

## IMPORTANT: hardware access on macOS
Docker Desktop on macOS runs Linux inside a VM. USB cameras and the microcontroller serial
device **cannot be passed into containers on a Mac**. So the architecture is split:

1. **Native host-side capture daemon** (runs directly on macOS): talks to the camera and the MCU,
   and sends frames to the service over gRPC (`ShotService.ProcessShot`, e.g. via a forwarded port).
2. **Containerized `shot-service`** (Docker/k8s): processing, replay, storage, API.

On a Linux host this could change: pass devices with `docker run --device /dev/video0 --device /dev/ttyACM0`,
or in k8s use a device plugin (e.g. generic-device-plugin / smarter-device-manager) or a privileged
pod with hostPath `/dev` (plus node pinning). The capture code could then ship in a container too,
though the native-daemon split is still lower-latency and simpler.
