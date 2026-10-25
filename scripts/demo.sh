#!/usr/bin/env bash
# Bring up the whole stack with simulated hardware and the SwiftUI app.
#   scripts/demo.sh [shots] [interval_ms]
# Ctrl-C stops everything.
set -euo pipefail
cd "$(dirname "$0")/.."

SHOTS=${1:-20}
INTERVAL_MS=${2:-3000}
DATA_DIR=$(mktemp -d)
# Shot history lives in the compose Postgres, so it survives between demo runs
# (`make db-reset` wipes it).
make db-up >/dev/null
export DATABASE_URL=${DATABASE_URL:-postgres://launch:launch-dev@127.0.0.1:5432/launchmonitor}
trap 'kill $(jobs -p) 2>/dev/null || true; rm -rf "$DATA_DIR"' EXIT

cargo build -q -p shot-service -p capture-daemon
(cd app/LaunchMonitor && swift build -q)

DATA_DIR="$DATA_DIR" target/debug/shot-service &
until grpcurl -plaintext localhost:8080 grpc.health.v1.Health/Check >/dev/null 2>&1; do sleep 0.2; done

app/LaunchMonitor/.build/debug/LaunchMonitor &
sleep 2   # let the app open its stream before the first shot

target/debug/capture-daemon --mcu sim --camera sim --install-bench-setup \
  --sim-shots "$SHOTS" --sim-interval-ms "$INTERVAL_MS" --captures-dir "$DATA_DIR/captures"
echo "Simulated shots done; close the app window or press Ctrl-C to stop."
wait
