# capture-daemon on the Raspberry Pi

```bash
sudo useradd --system --home /var/lib/launch-monitor --groups dialout launchmonitor
sudo install -m 0755 target/release/capture-daemon /usr/local/bin/
sudo install -D -m 0644 capture-daemon.env /etc/launch-monitor/capture-daemon.env   # edit the address
sudo install -m 0644 capture-daemon.service /etc/systemd/system/
sudo install -m 0644 99-launch-monitor.rules /etc/udev/rules.d/
sudo udevadm control --reload && sudo udevadm trigger
sudo systemctl daemon-reload && sudo systemctl enable --now capture-daemon
journalctl -u capture-daemon -f
```

Behaviour:

- Pico unplugged, reset or not yet plugged in: the daemon retries the serial
  open with backoff (250 ms doubling to 5 s) and re-sends config and `Arm` on
  every reconnect. A shot in flight when the link drops is lost.
- Shot service down: shots are archived first and the upload is retried; see
  `docs/ARCHITECTURE.md`.
- Fatal errors (MCU rejects the config, camera failure) make the process exit
  non-zero and systemd restarts it after `RestartSec`.
- `systemctl stop` sends SIGTERM; the daemon disarms the strobe and finishes
  in-flight uploads before exiting.

Untested on hardware: the unit, the udev rule and the serial path have only
been exercised against the simulator and a missing device.
