# Audix Yaw Hold Bench GUI

This folder is uploaded to ESP32 SPIFFS and served by the Wi-Fi dashboard
firmware.

The page is intentionally narrow:

- safety/setup commands: `stop`, `status`, `quiet`, `i2cscan`, `imu cal`,
  `imu zero`, `imu reset`, `yawhold zero`, and `encoders zero`
- live odometry, IMU, heading-controller, and wheel telemetry from `/api/state`
- heading tests: `yawhold zero`, `yawhold on`, `yawhold off`, `forward`, `backward`, `rotate`
- RPM validation for one wheel or all wheels
- runtime PID controls for only `wheel` and `yaw`
- copy-ready command blocks, raw log, and manual command input

It uses HTTP polling only:

```text
GET /api/ping
GET /api/state
GET /api/history
GET /api/command?line=...
```

## Upload

```powershell
cd "C:\Users\TiBa\Documents\PlatformIO\Projects\esp32_mecanum_yaw_hold"
pio run -e esp32_wifi_base_dashboard -t uploadfs --upload-port COM10
```

## Open

```text
SSID: AudixBench-ESP32
Password: audixbench123
URL: http://192.168.4.1/
```

## Local Layout Preview

The local preview only shows the layout. Live values stay offline unless the
browser can reach the ESP32 APIs.

```powershell
cd "C:\Users\TiBa\Documents\PlatformIO\Projects\esp32_mecanum_yaw_hold\visualizer"
py -m http.server 8765
```

Open:

```text
http://localhost:8765/
```
