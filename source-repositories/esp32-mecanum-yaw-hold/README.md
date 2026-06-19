# ESP32 Mecanum Yaw Hold Bench

Standalone Audix-aligned ESP32 test firmware for mecanum yaw hold, straight-line
heading correction, wheel RPM validation, and a small HTTP dashboard.

This repo is intentionally separate from the real firmware folders:

- read-only reference sandbox: `C:\Users\TiBa\Documents\PlatformIO\Projects\audix_esp32_windows_test`
- read-only final Pi firmware: `C:\Users\TiBa\OneDrive\Spring 26\Design 2\audix_wps\microROS\firmware\esp32_low_level`
- editable yaw-hold test target: `C:\Users\TiBa\Documents\PlatformIO\Projects\esp32_mecanum_yaw_hold`

The old single-file `.ino` test sketch is not the active firmware path here.
This project mirrors the Audix Windows sandbox structure: `src/`, `include/`,
FreeRTOS tasks, shared state, IMU driver, encoder odometry, mecanum kinematics,
wheel PID, HTTP API, and a SPIFFS dashboard.

## What Changed

- Added explicit `imu cal`, `imu zero`, and `imu reset` commands.
- Added `yawhold zero`, `yawhold on`, and `yawhold off`.
- Straight `forward <cm>` and `backward <cm>` now keep heading hold active at
  the captured starting yaw after the distance target finishes.
- Still yaw hold commands chassis `wz` through the normal mecanum inverse
  kinematics path instead of directly poking wheel RPMs.
- Added a zero-output gate in the wheel PID task so tiny target and encoder
  noise near zero does not get boosted into audible motor-deadband ticks.
- Extended `/api/state` with heading and command-path diagnostics.
- Simplified the dashboard to the yaw-hold bench flow: setup, hold/release,
  forward/backward/rotate, RPM validation, wheel/yaw PID tuning, and raw log.

## Build

```powershell
cd "C:\Users\TiBa\Documents\PlatformIO\Projects\esp32_mecanum_yaw_hold"
pio run -e esp32_wifi_base_dashboard
pio run -e esp32_wifi_base_dashboard -t buildfs
```

## Flash Over USB

Replace `COM10` if your ESP32 is on another port.

```powershell
cd "C:\Users\TiBa\Documents\PlatformIO\Projects\esp32_mecanum_yaw_hold"
pio run -e esp32_wifi_base_dashboard -t upload --upload-port COM10
pio run -e esp32_wifi_base_dashboard -t uploadfs --upload-port COM10
```

If you need to find the port:

```powershell
pio device list
```

Serial monitor:

```powershell
pio device monitor -p COM10 -b 115200
```

## Open The GUI

Connect the laptop to:

```text
SSID: AudixBench-ESP32
Password: audixbench123
```

Open:

```text
http://192.168.4.1/
```

Direct API checks:

```text
http://192.168.4.1/api/ping
http://192.168.4.1/api/state
http://192.168.4.1/api/history
http://192.168.4.1/api/command?line=status
```

## Hardware Mapping

Firmware wheel order is `FL, FR, RL, RR`.

| Wheel | Motor label | Motor GPIOs | Encoder label | Encoder GPIOs |
| --- | --- | --- | --- | --- |
| Front Left | MOTD | `17, 18` | ENC2 | `34, 35` |
| Front Right | MOTB | `13, 19` | ENC3 | `33, 32` |
| Rear Left | MOTC | `4, 16` | ENC1 | `39, 36` |
| Rear Right | MOTA | `27, 14` | ENC4 | `25, 26` |

The motor driver is two-input H-bridge style:

- positive command: `IN1 = PWM`, `IN2 = LOW`
- negative command: `IN1 = LOW`, `IN2 = PWM`
- stop: both inputs `LOW`

## Fast Bench Flow

Keep the robot on blocks until wheel direction, encoder mapping, RPM feedback,
and stop behavior are correct.

```text
stop
i2cscan
imu cal
encoders zero
rpm fl 60 2.0
rpm fr 60 2.0
rpm rl 60 2.0
rpm rr 60 2.0
rpms all 60 2.0
yawhold zero
```

Twist the robot by hand and release it. Expected result:

- it returns to the captured yaw
- heading error returns inside `+/-3 deg` without repeated sign-flip hunting
- `heading.commandWz` returns to `0`
- wheel PWM returns to `0` when settled

Then test commanded motion:

```text
forward 20
backward 20
rotate 90
rotate -90
stop
```

`rotate 90` means counter-clockwise. `rotate -90` means clockwise.

## Important Commands

```text
stop
status
quiet
i2cscan
imu cal
imu zero
imu reset
encoders zero
encoders once
encoders on
yawhold on
yawhold off
yawhold zero
forward 20
backward 20
rotate 90
rotate -90
rpm fl 60 2.0
rpms all 60 2.0
pid show
pid wheel 1.20 0.80 0.05
pid yaw 2.00 0.00 0.06
```

## Documentation

- `docs/yaw_hold_test_procedure.md`: hardware validation order and pass/fail criteria.
- `docs/tuning_guide.md`: 10 to 15 minute tuning procedure and symptom map.
- `docs/rtos_porting_notes.md`: how to move only the controller logic into the local RTOS firmware.
- `docs/architecture.md`: module map for this standalone test firmware.

## RTOS Applicability

If this test works, the yaw-hold controller is applicable to the local ESP32 RTOS
firmware. Port only the controller logic: angle wrapping, hold state, deadband,
P plus gyro damping, command clamp, zero-output gate, settle criteria, and
telemetry. Do not copy WebServer, SPIFFS dashboard, Arduino `String` JSON, or
sketch-specific command handlers into the RTOS firmware.
