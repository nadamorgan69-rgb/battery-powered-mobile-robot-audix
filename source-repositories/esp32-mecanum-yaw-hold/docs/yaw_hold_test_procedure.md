# Yaw Hold Test Procedure

Use this procedure with the robot lifted on blocks until the RPM and stop tests
pass. Do not run these tests on the floor until wheel direction, encoder mapping,
and stop behavior are correct.

## Build And Flash

```powershell
cd "C:\Users\TiBa\Documents\PlatformIO\Projects\esp32_mecanum_yaw_hold"
pio run -e esp32_wifi_base_dashboard
pio run -e esp32_wifi_base_dashboard -t buildfs
pio run -e esp32_wifi_base_dashboard -t upload --upload-port COM10
pio run -e esp32_wifi_base_dashboard -t uploadfs --upload-port COM10
```

Open the GUI:

```text
SSID: AudixBench-ESP32
Password: audixbench123
URL: http://192.168.4.1/
```

## First Boot

Run:

```text
stop
quiet
status
```

Pass:

- status prints `imuHealthy=yes` after the IMU is initialized
- `cmd=(0, 0, 0)`
- all wheel PWM values are `0`

## IMU Setup

Keep the robot completely still.

```text
i2cscan
imu cal
imu zero
status
```

Pass:

- `i2cscan` finds `0x68`
- `imu cal` reports calibration ok
- `imu zero` sets yaw near `0 deg`
- rotating the robot around Z changes IMU yaw in the GUI

If pressing zero twice used to help, this split is the replacement:

- `imu cal`: estimates gyro bias while still
- `imu zero`: changes the yaw reference only
- `yawhold zero`: zeros yaw and captures `0 deg` as the heading target
- `yawhold on`: captures the current heading target for general hold control

## Encoder And RPM Validation

```text
encoders zero
encoders on
rpm fl 60 2.0
rpm fr 60 2.0
rpm rl 60 2.0
rpm rr 60 2.0
rpms all 60 2.0
rpms all -60 2.0
stop
```

Pass:

- each single-wheel command moves only the selected wheel
- each selected encoder row changes
- measured RPM follows target RPM with no runaway
- `stop` forces PWM to `0`

Fail fast:

- wrong row changes: fix encoder mapping before yaw tests
- raw motion exists but RPM fails: check encoder sign/direction and wheel PID
- wheel keeps ticking after `stop`: stop testing and inspect motor output state

## Still Yaw-Hold Test

```text
stop
imu cal
encoders zero
yawhold zero
```

Twist the robot by hand by about 5 to 15 degrees and release.

Pass:

- `heading.phaseName` changes to `hold_active`
- `heading.yawErrorDeg` moves toward `0` and finishes inside `+/-3 deg`
- `heading.commandWz` has the correct sign and then returns to `0`
- wheel target RPM becomes nonzero while outside `+/-3 deg`
- wheel target RPM returns to `0` when settled
- `heading.phaseName` becomes `settled`
- all wheel PWM values return to `0`
- there are no repeated audible ticks once settled

Fail sign check in under 2 minutes:

1. Run `yawhold zero`.
2. Twist the robot counter-clockwise by hand and hold it.
3. `heading.yawErrorDeg` should show the target is behind the robot.
4. Release it. The first commanded correction must rotate it back toward zero.
5. If it rotates farther away, the yaw correction sign or wheel mixing sign is wrong.

## Straight-Line Test

```text
stop
imu cal
imu zero
encoders zero
forward 20
```

Pass:

- X odometry moves toward about `20 cm`
- yaw error stays small during motion
- when X finishes, heading hold remains active at the original yaw
- after settling, `command.wz`, `heading.commandWz`, wheel targets, and PWM return to zero

Then run:

```text
backward 20
```

Pass criteria are the same, with X moving negative/back toward the starting point.

## Rotation Test

```text
rotate 90
rotate -90
```

Pass:

- `rotate 90` turns counter-clockwise
- `rotate -90` turns clockwise
- yaw error handles wrap around `+/-180`
- final PWM returns to zero

## What To Record

For each failed run, record:

- command sent
- `heading.phaseName`
- `heading.targetYawDeg`
- `heading.yawErrorDeg`
- `heading.commandWz`
- `imu.gyroZ`
- wheel target RPM, measured RPM, and PWM

## Copy-Ready GUI Commands

Paste/send one line at a time through the GUI manual command input.

Clean setup:

```text
stop
yawhold off
quiet
pid wheel 1.20 0.80 0.05
pid yaw 2.00 0.00 0.06
imu cal
yawhold zero
```

If return is too weak:

```text
pid yaw 3.00 0.00 0.08
yawhold zero
```

If it hunts or vibrates:

```text
pid yaw 1.50 0.00 0.08
yawhold zero
```

If it turns the wrong way:

```text
stop
yawhold off
```
