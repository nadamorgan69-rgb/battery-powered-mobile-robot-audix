# RTOS Porting Notes

If this yaw-hold behavior works in the standalone Wi-Fi bench, it can be moved
to the local ESP32 RTOS firmware. Port the controller, not the dashboard.

## Copy These Concepts

- `wrapAngleRad`
- heading target capture
- heading hold active/settled phase state
- deadband check on yaw error
- rate deadband check on gyro Z
- P plus gyro damping through the yaw PID
- clamp on commanded chassis `wz`
- 250 ms settle confirmation
- zero-output gate in the wheel control task
- telemetry for phase, target yaw, yaw error, gyro Z, commanded `wz`, and stable time

## Do Not Copy These Pieces

- Wi-Fi AP setup
- `WebServer`
- SPIFFS dashboard files
- Arduino `String` JSON construction
- one-off bench command parser details
- test-only RPM/manual override scheduling if the RTOS firmware already has its own command layer

## Suggested RTOS Module Shape

Create a small controller that is called from the fixed-period motion task:

```cpp
struct HeadingHoldInput {
    bool enable;
    float target_yaw_rad;
    float current_yaw_rad;
    float gyro_z_rad_s;
    float dt_s;
};

struct HeadingHoldOutput {
    bool settled;
    float yaw_error_rad;
    float command_wz_rad_s;
    float stable_ms;
};
```

The RTOS task should own:

```text
capture target yaw
compute yaw hold command
combine vx/vy/wz
inverse kinematics
wheel PID
motor output
```

The wheel PID should remain wheel-speed tracking only. Do not move heading logic
inside each wheel controller.

## Straight-Line Port

When a straight-line command starts:

```text
target_x = requested distance from encoder odometry
target_yaw = current IMU yaw
```

Every control tick:

```text
x_error = target_x - measured_encoder_x
yaw_error = wrapAngleRad(target_yaw - imu_yaw)
vx = x_pid(x_error)
wz = yaw_hold(yaw_error, gyro_z)
wheel_targets = inverseKinematics(vx, 0, wz)
```

When X reaches tolerance:

```text
vx = 0
keep yaw hold enabled at target_yaw
when heading settled, command exact wz = 0
```

## Still Hold Port

For a manual still-hold command:

```text
yawhold on:
    target_yaw = current imu yaw
    active = true
    stable_ms = 0

yawhold off:
    active = false
    command vx = vy = wz = 0
```

## Zero-Output Gate Port

Place this in the wheel control task immediately before each wheel PID update:

```cpp
if (fabs(target_w_rad_s[i]) < WHEEL_TARGET_ZERO_EPSILON_RAD_S &&
    fabs(measured_w_rad_s[i]) < WHEEL_STOP_EPSILON_RAD_S) {
    wheel_pid[i].reset();
    pwm_output[i] = 0.0f;
    continue;
}
```

This is important because a tiny nonzero PID output can become an audible motor
tick when the motor driver applies its deadband lift.

## RTOS Validation Checklist

- Existing velocity PID behavior still passes.
- Stop command still forces all motor pins low.
- IMU yaw direction matches the standalone bench.
- Straight-line command captures yaw once at command start.
- Hold command settles with `wz = 0` and wheel PWM `0`.
- Telemetry exposes enough values to distinguish sign errors from tuning errors.
