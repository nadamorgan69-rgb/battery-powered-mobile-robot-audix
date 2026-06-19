# Yaw Hold Tuning Guide

This is a 10 to 15 minute tuning flow for the current firmware. Tune on blocks
first, then repeat on the floor at low distance.

## Starting Values

Start from these commands:

```text
pid wheel 1.20 0.80 0.05
pid yaw 2.00 0.00 0.06
```

The controller uses:

- `pid wheel`: wheel velocity tracking
- `pid yaw`: straight-line heading correction, still yaw hold, and rotation
- `HEADING_HOLD_DEADBAND_RAD`: default about 3 degrees
- `HEADING_HOLD_RATE_DEADBAND_RAD_S`: default about 2 deg/s
- `TARGET_MOVE_YAW_HOLD_LIMIT_RAD_S`: caps hold correction speed
- `HEADING_HOLD_MIN_WZ_RAD_S`: minimum return command outside the 3 degree band
- zero-output gate: forces PWM to zero when target wheel speed and measured speed are tiny

## Step 1: Confirm Wheel PID Still Works

```text
stop
encoders zero
rpm fl 60 2.0
rpm fr 60 2.0
rpm rl 60 2.0
rpm rr 60 2.0
rpms all 60 2.0
```

Do not tune yaw until RPM follows target without runaway or wrong-wheel motion.

## Step 2: Calibrate IMU Correctly

Keep the robot still:

```text
stop
imu cal
imu zero
```

Do not run `imu cal` while a wheel is moving. It will bake motion into the gyro
bias and make yaw hold act random.

## Step 3: Still Yaw Hold

```text
yawhold zero
```

Twist the robot 5 to 10 degrees and release. Watch:

- `heading.yawErrorDeg`
- `heading.commandWz`
- wheel PWM values
- audible ticks after `heading.settled = yes`

Adjustment rules:

| Symptom | Change |
| --- | --- |
| Returns too slowly and no overshoot | Increase yaw `Kp` by `0.10` to `0.20`. |
| Corrects fast but overshoots once | Increase yaw `Kd` by `0.01` to `0.02`. |
| Repeated hunting across zero | Lower yaw `Kp` by `0.10`, or raise yaw `Kd` slightly. |
| Motors tick after settled | Confirm wheel PWM is zero; if not, widen heading/rate deadband in `config.hpp`. |
| Commanded `wz` is pinned at the clamp | Reduce yaw `Kp` or raise the clamp only after sign/mapping is confirmed. |
| Twisting away makes it drive farther away | Stop and validate yaw correction sign and mecanum wheel order. |
| Yaw slowly drifts while still | Run `imu cal` while fully still, then `imu zero`. |

Keep yaw `Ki = 0` for these tests. Add integral only after the robot returns
smoothly with P and D.

## Step 4: Straight-Line Heading Hold

```text
stop
imu cal
imu zero
encoders zero
forward 20
```

Expected:

- X position reaches about `20 cm`
- yaw error stays small
- hold remains active after the distance target finishes
- PWM returns to zero after the hold settles

Adjustment rules:

| Symptom | Change |
| --- | --- |
| Drives straight but yaw correction is weak | Increase yaw `Kp` by `0.10`. |
| Wobbles left/right while moving | Increase yaw `Kd` by `0.01`, or lower yaw `Kp`. |
| Distance is wrong but yaw is good | Tune/check X odometry, CPR, wheel radius, and wheel direction before yaw. |
| Raw PWM works but straight-line command fails | Check RPM feedback and wheel PID first. |

## Step 5: Rotation

```text
rotate 90
rotate -90
```

Expected:

- positive angle is counter-clockwise
- negative angle is clockwise
- final heading stops without chatter

Adjustment rules:

| Symptom | Change |
| --- | --- |
| Rotation is too slow | Increase yaw `Kp` or `BENCH_TARGET_YAW_SPEED_RAD_S`. |
| Rotation overshoots | Increase yaw `Kd` or reduce `BENCH_TARGET_YAW_SPEED_RAD_S`. |
| Rotation reaches target but ticks after stop | Check zero-output gate telemetry: wheel target and PWM should become zero. |

## Good Enough Criteria

For the bench test to count as successful:

- a 5 to 10 degree manual twist returns to target without visible repeated hunting
- `heading.yawErrorDeg` settles inside `+/-3 deg`
- `heading.commandWz` returns to `0`
- wheel PWM returns to `0`
- `forward 20` and `backward 20` do not visibly drift orientation
- `rotate 90` and `rotate -90` stop in the right direction
