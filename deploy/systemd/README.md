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

## Camera (InnoMaker IMX296, `--camera rpicam`)

UNVERIFIED on hardware; do these on the Pi, in order:

1. `/boot/firmware/config.txt`: `camera_auto_detect=0` and `dtoverlay=imx296`
   (add `,cam0` on a Pi 5's CAM0 port), then reboot.
2. `rpicam-still --list-cameras` must list the IMX296.
3. External trigger mode: `echo "options imx296 trigger_mode=1" | sudo tee
   /etc/modprobe.d/imx296.conf`, reboot. Check the InnoMaker manual for the
   trigger polarity/edge and whether exposure follows `--shutter` or the
   trigger pulse width; see firmware/README.md "Camera configuration".
4. Grab one frame by hand and check the size, to confirm the Y-plane stride:
   `rpicam-vid -n -t 1000 --codec yuv420 --width 1456 --height 1088 --shutter 3000 -o /tmp/f.yuv`
   (a triggered camera only writes frames when triggered). Bytes per frame should be
   `1456*1088*1.5 = 2376192`; if it is larger, rpicam pads rows, so pass
   `--cam-stride <padded width>`.
5. Change `--camera none` to `--camera rpicam` in capture-daemon.service.
