# Audix Stable Base Bench GUI

This folder is uploaded into ESP32 SPIFFS and served directly by the Windows
sandbox firmware.

The active page is intentionally simple:

- stable HTTP polling through `/api/state`
- live IMU yaw, gyro, and accel values
- live odometry x/y/yaw and vx/vy/wz values
- live encoder, RPM, target RPM, and PWM table
- visual PWM and RPM controls
- visual runtime PID tuning for `wheel`, `x`, `y`, and `yaw/z`
- movement buttons for forward, backward, rotate CCW, and rotate CW
- compact raw log
- manual command input

## Flash and open

Use the base dashboard for encoder and PID testing:

```powershell
cd C:\Users\TiBa\Documents\PlatformIO\Projects\audix_esp32_windows_test
pio run -e esp32_wifi_base_dashboard -t upload --upload-port COM10
pio run -e esp32_wifi_base_dashboard -t uploadfs --upload-port COM10
```

Then connect to:

- SSID: `AudixBench-ESP32`
- password: `audixbench123`
- browser: `http://192.168.4.1/`

Useful direct API checks:

```text
http://192.168.4.1/api/ping
http://192.168.4.1/api/state
```

## Encoder first

Start with motor power disconnected:

```text
mode
status
encoders zero
encoders once
encoders on
```

Rotate each wheel by hand and confirm:

- Front Left changes FL / ENC2 / GPIO `34,35`
- Front Right changes FR / ENC3 / GPIO `33,32`
- Rear Left changes RL / ENC1 / GPIO `39,36`
- Rear Right changes RR / ENC4 / GPIO `25,26`

## PID after encoder and single-wheel checks

Keep the robot on blocks:

```text
encoders zero
motors all max 1.0
encoders once
motor fl max 1.0
encoders once
motor fr max 1.0
encoders once
motor rl max 1.0
encoders once
motor rr max 1.0
encoders once
rpm fl 60 2.0
rpm fr 60 2.0
rpm rl 60 2.0
rpm rr 60 2.0
rpms all 60 2.0
rpms all -60 2.0
encoders on
forward 20
encoders once
backward 20
encoders once
rotate 90
encoders once
rotate -90
encoders once
```

PWM is open-loop and uses commands such as:

```text
motor fl 0.30 1.0
motors all max 1.0
```

RPM is closed-loop through encoder feedback and the wheel PID:

```text
rpm fl 60 2.0
rpms all -60 2.0
```

PID values can be changed from the GUI or by command:

```text
pid show
pid wheel 1.20 0.80 0.05
pid x 0.80 0.00 0.02
pid y 0.80 0.00 0.02
pid yaw 1.10 0.00 0.03
```

The one-argument movement commands mean:

```text
forward 20   -> move forward about 20 cm
backward 20  -> move backward about 20 cm
rotate 90    -> rotate counter-clockwise about 90 degrees
rotate -90   -> rotate clockwise about 90 degrees
```

The old velocity/time debug commands are still available:

```text
enable on
forward 0.05 1.0
stop
encoders once
enable on
strafe 0.05 1.0
stop
encoders once
imu on
enable on
rotate 0.20 1.0
stop
encoders once
enable off
```

Use `quiet` any time the serial log is too noisy. It turns off `encoders`, `imu`,
and `rtos` streams without changing the current motor state. Use `stop` if any
wheel moves unexpectedly; it disables motion, turns streams off, and forces motor
outputs low.

If Wi-Fi fails, use VS Code or PlatformIO Serial Monitor:

```powershell
pio device monitor -p COM10 -b 115200
```

## IMU quick test

Use the IMU panel before motor testing:

```text
i2cscan
status
imu on
```

Expected:

- `i2cscan` finds `0x68`
- `status` reports `imuHealthy=yes`
- tilt changes accel
- rotation spikes gyro
- Z rotation changes yaw

Stop the stream with:

```text
imu off
```
