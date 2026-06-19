# Test Firmware Architecture

This project is a standalone yaw-hold bench built from the Audix Windows
sandbox architecture. It is not the final Pi-facing firmware.

## Task Split

- `commandRxTask`: USB serial command parsing plus Wi-Fi dashboard command API.
- `sensorUpdateTask`: encoder sampling, odometry update, IMU update, and sensor state.
- `motionControlTask`: command timeout safety, mecanum inverse kinematics, wheel PID, motor PWM.
- `telemetryTask`: optional serial stream output for IMU, encoders, and task heartbeats.
- `serviceBenchRuntime`: high-level timed motion, target moves, RPM overrides, and yaw hold.

## Main State Objects

- `CommandState`: desired chassis `vx`, `vy`, `wz`, enable flag, command timestamp.
- `WheelState`: target wheel speeds, measured wheel speeds, encoder counts, PWM outputs.
- `OdometryState`: encoder-derived `x`, `y`, `theta`, `vx`, `vy`, and `wtheta`.
- `IMUState`: accel, gyro, and integrated yaw.
- `BenchRuntimeSnapshot`: active bench mode, target move state, RPM override state, heading hold state.
- `HeadingControlState`: public telemetry for heading phase, target, error, gyro Z, command `wz`, and stable time.

## Control Flow

Straight-line move:

```text
forward/backward command
    -> scheduleTargetMove(kX)
    -> capture current IMU yaw as heading target
    -> X PID commands chassis vx from encoder odometry
    -> yaw PID commands chassis wz from IMU yaw error
    -> inverseKinematics(vx, 0, wz)
    -> wheel PID
    -> motor PWM
    -> when X target is reached, keep yawhold active at captured target
```

Still yaw hold:

```text
yawhold on
    -> capture current IMU yaw
    -> clear timed/manual/RPM modes
    -> every control tick compute wz from heading error and gyro Z
    -> publish zero wz after deadband plus settle time
```

Rotation:

```text
rotate <deg>
    -> scheduleTargetMove(kYaw)
    -> use IMU yaw delta with wrapAngleRad
    -> command chassis wz through inverse kinematics
```

## Motor Tick Prevention

The motor driver has a physical deadband lift. Tiny nonzero PID output can become
an audible tick even when the robot is effectively stopped. The wheel control
task therefore gates zero-target wheels:

```text
if abs(target_wheel_rad_s) is tiny and abs(measured_wheel_rad_s) is tiny:
    reset that wheel PID
    force PWM = 0
```

This keeps settled yaw hold quiet without changing normal wheel velocity control.

## HTTP API

The dashboard uses HTTP polling only:

- `GET /api/ping`
- `GET /api/state`
- `GET /api/history`
- `GET /api/command?line=...`

`/api/state` includes a `heading` object:

```json
{
  "holdEnabled": true,
  "settled": false,
  "phaseName": "hold_active",
  "targetYawDeg": 0.0,
  "yawErrorDeg": -4.2,
  "gyroZ": 0.01,
  "commandWz": -0.08,
  "stableMs": 0.0
}
```
