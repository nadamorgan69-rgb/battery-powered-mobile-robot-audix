# ESP32 Standalone IMU Runbook

This is the IMU-focused bring-up runbook for the yaw-hold bench project.

Use this folder:

```powershell
cd "C:\Users\TiBa\Documents\PlatformIO\Projects\esp32_mecanum_yaw_hold"
```

## Flash

```powershell
pio run -e esp32_wifi_base_dashboard
pio run -e esp32_wifi_base_dashboard -t buildfs
pio run -e esp32_wifi_base_dashboard -t upload --upload-port COM10
pio run -e esp32_wifi_base_dashboard -t uploadfs --upload-port COM10
```

If `COM10` is wrong:

```powershell
pio device list
```

## Open

```text
SSID: AudixBench-ESP32
Password: audixbench123
URL: http://192.168.4.1/
```

## IMU Wiring

```text
GPIO21 -> SDA
GPIO22 -> SCL
3V3    -> VCC
GND    -> GND
```

Expected I2C address is `0x68`.

The driver accepts these WHO_AM_I values:

- `0x68`: MPU6050
- `0x70`: MPU6500
- `0x71`: MPU9250
- `0x73`: MPU9255

## IMU Commands

```text
i2cscan
imu cal
imu zero
imu reset
imu on
imu off
status
```

Meanings:

- `imu cal`: recalibrate gyro bias while the robot is still.
- `imu zero`: set the current yaw angle to zero without changing gyro bias.
- `imu reset`: run bias calibration, then zero yaw.
- `imu on`: print IMU stream lines.
- `imu off`: stop IMU stream lines.

Do not run `imu cal` while motors are moving. The firmware refuses calibration
if command velocity, measured wheel speed, or PWM output indicate motion.

## Quick IMU Validation

```text
stop
i2cscan
imu cal
imu zero
status
imu on
```

Pass:

- `i2cscan` finds `0x68`
- `status` shows `imuHealthy=yes`
- tilting changes accel values
- rotating changes gyro values
- rotating around Z changes yaw
- keeping the robot still leaves gyro Z near zero

Stop the stream:

```text
imu off
```

## Yaw Hold Validation

```text
stop
imu cal
imu zero
encoders zero
yawhold zero
```

Twist the robot by hand and release.

Pass:

- heading error returns toward `0 deg`
- heading error finishes inside `+/-3 deg`
- `heading.commandWz` returns to `0`
- `heading.phaseName` becomes `settled`
- wheel PWM returns to `0`
- no repeated ticks are heard when settled

More complete test instructions are in:

- `docs/yaw_hold_test_procedure.md`
- `docs/tuning_guide.md`
