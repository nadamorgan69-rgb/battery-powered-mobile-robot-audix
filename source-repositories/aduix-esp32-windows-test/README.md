# Audix ESP32 Windows Bench Firmware

This folder is the Windows-only standalone ESP32 testing sandbox.

Use this project when you want to:

- open the ESP32 code directly in VS Code with PlatformIO on Windows
- build and upload over the normal ESP32 USB cable
- upload a local browser UI into ESP32 SPIFFS
- test the ESP32 without the Pi, ROS, or `micro_ros_agent`
- choose a Wi-Fi GUI workflow or a focused USB serial workflow without mixing them
- exercise IMU, encoders, motor outputs, PID motion commands, safety stop,
  RTOS loop-rate reporting, and DRV stepper lift hardware from isolated bench modes

The Windows sandbox IMU path supports MPU6050 and MPU6500/9250/9255-class
accel/gyro chips for bench testing over USB serial.

Do not use this folder as the final Pi-connected submission. The canonical final
firmware stays here:

- `C:\Users\TiBa\OneDrive\Spring 26\Design 2\audix_wps\microROS\firmware\esp32_low_level`

## Open the correct folder

Open this folder itself in VS Code or PlatformIO:

- `C:\Users\TiBa\Documents\PlatformIO\Projects\audix_esp32_windows_test`

That is the folder containing `platformio.ini`.

## Official PlatformIO environments

Wi-Fi dashboard environments:

- `esp32_wifi_base_dashboard`: browser GUI for PID, IMU, encoders, motors, RTOS, and safety.
- `esp32_wifi_stepper_dashboard`: browser GUI for DRV stepper jog, homing, and GPIO4 limit switch.

USB serial-only environments:

- `esp32_serial_pid`: terminal menu for PID/base motion testing.
- `esp32_serial_stepper`: terminal menu for stepper jog, homing, and limit switch testing.
- `esp32_serial_imu_encoder`: terminal menu for IMU and encoder feedback testing.

Compatibility aliases still exist:

- `esp32dev` maps to `esp32_wifi_base_dashboard`.
- `esp32_stepper_wifi` maps to `esp32_wifi_stepper_dashboard`.

## Wi-Fi dashboard workflow

Flash the base dashboard:

```powershell
cd C:\Users\TiBa\Documents\PlatformIO\Projects\audix_esp32_windows_test
pio run -e esp32_wifi_base_dashboard -t upload --upload-port COM10
pio run -e esp32_wifi_base_dashboard -t uploadfs --upload-port COM10
```

Flash the stepper dashboard:

```powershell
cd C:\Users\TiBa\Documents\PlatformIO\Projects\audix_esp32_windows_test
pio run -e esp32_wifi_stepper_dashboard -t upload --upload-port COM10
pio run -e esp32_wifi_stepper_dashboard -t uploadfs --upload-port COM10
```

Then connect to:

- SSID: `AudixBench-ESP32`
- password: `audixbench123`
- browser UI: `http://192.168.4.1/`

For the exact ESP32-only IMU, encoder, motor, RPM, movement, and PID test
sequence, use:

- `docs\esp32_standalone_test_runbook_with_imu.md`

The base dashboard is a simple HTTP-only GUI with live IMU fields, odometry,
encoder/RPM/PWM fields, movement buttons, visual PWM/RPM controls, runtime
`Kp`/`Ki`/`Kd` PID tuning, and a compact raw ESP32 log.

Useful dashboard APIs:

- `http://192.168.4.1/api/ping`
- `http://192.168.4.1/api/state`
- `http://192.168.4.1/api/history`
- `http://192.168.4.1/api/command?line=status`

## Base motor and encoder wiring

Keep motor power disconnected for the first boot after flashing. The base
dashboard uses two-input H-bridge motor control, so `stop` drives both inputs
LOW for each motor.

Wheel order in firmware is `FL, FR, RL, RR`.

This base dashboard mapping intentionally conflicts with the stepper dashboard
on some GPIOs. Flash and wire only one dashboard mode at a time.

| Wheel | Motor label | Motor GPIOs | Encoder label | Encoder GPIOs |
| --- | --- | --- | --- | --- |
| Front Left | MOTD | `17, 18` | ENC2 | `34, 35` |
| Front Right | MOTB | `13, 19` | ENC3 | `33, 32` |
| Rear Left | MOTC | `4, 16` | ENC1 | `39, 36` |
| Rear Right | MOTA | `27, 14` | ENC4 | `25, 26` |

Safe bring-up order:

1. Flash `esp32_wifi_base_dashboard` and `uploadfs`.
2. Boot with motor power disconnected and confirm the Wi-Fi dashboard appears.
3. Run `status`, `encoders zero`, and `encoders on`, then rotate each wheel by hand.
4. Put the robot on blocks before applying motor power.
5. Run `stop`, `enable off`, then test all motors with `motors all 0.30 1.0`.
6. Test one wheel at a time with `motor fl 0.30 1.0`, `motor fr 0.30 1.0`, `motor rl 0.30 1.0`, and `motor rr 0.30 1.0`.
7. Only after wheel order and encoder feedback are correct, run low-power `pidx`, `pidy`, and `pidyaw` tests.

## USB serial workflow

Flash one focused terminal bench:

```powershell
cd C:\Users\TiBa\Documents\PlatformIO\Projects\audix_esp32_windows_test
pio run -e esp32_serial_pid -t upload --upload-port COM10
pio device monitor -p COM10 -b 115200
```

Other serial benches:

```powershell
pio run -e esp32_serial_stepper -t upload --upload-port COM10
pio run -e esp32_serial_imu_encoder -t upload --upload-port COM10
```

Serial-only firmware prints `[wifi] disabled...` and does not create `AudixBench-ESP32`.

## Stepper wiring

- ESP32 `GPIO26` -> DRV `STEP`
- ESP32 `GPIO25` -> DRV `DIR`
- ESP32 `GPIO27` -> DRV `EN`
- ESP32 `GPIO4` -> one side of the home switch
- other side of the home switch -> `GND`
- ESP32 `GND` -> driver logic `GND`
- ESP32 `3V3` -> driver logic VDD only if the breakout exposes logic VDD and accepts 3.3 V logic
- motor supply stays on the driver's own motor-power input
- common ground between ESP32 and driver is required

The stepper variant assumes:

- `EN` is active-low
- the home switch is NO-to-GND and active-low
- `stepper home ...` moves in the configured home direction until the switch is hit

## Dashboard focus

The base Wi-Fi dashboard prioritizes standalone base bring-up:

- IMU scan/status/stream checks
- encoder zero, one-shot, and stream checks
- raw PWM motor checks
- closed-loop wheel RPM checks
- centimeter/degree movement commands
- runtime PID tuning commands
- raw ESP32 log and manual command input

The stepper dashboard remains a separate firmware mode for DRV stepper jog,
homing, and GPIO4 limit-switch tests.

## Serial commands

The serial environments show focused menus. Useful examples:

- PID serial: `status`, `enable on`, `encoders on`, `pidx 0.10 2.0`, `pidy 0.10 2.0`, `pidyaw 0.40 2.0`, `stop`, `enable off`.
- Stepper serial: `status`, `switch`, `enable on`, `jog up 400 2.0`, `jog down 400 2.0`, `home 250 12.0`, `stop`, `enable off`.
- IMU/encoder serial: `i2cscan`, `imu on`, `encoders on`, `rtos on`, `status`, `stop`.

## What this project does not do

- it does not subscribe to `/cmd_vel` or `/robot_enable`
- it does not publish `/odom`, `/imu`, or `/limit_switch` into ROS
- it does not require the Pi or Ubuntu

Those behaviors remain in the final firmware folder only.
