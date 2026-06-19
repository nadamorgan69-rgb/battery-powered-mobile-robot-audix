# ESP32 Low-Level Architecture

This directory contains the ESP32 low-level firmware for the Audix warehouse
robot hardware path.

## Intended Task Split

- command receive task
- motion control task
- sensor update task
- telemetry task
- quadrature encoder ISR handling

## Ownership Boundary

This firmware owns:
- encoder reading
- IMU reading
- limit switch reading
- mecanum inverse kinematics
- four wheel-speed PID controllers
- raw odometry generation
- PWM and direction output
- command-timeout safety
- telemetry publishing

This firmware does not own:
- EKF
- `arena_roamer.py`
- IR scan conversion
- RViz or other ROS-side debugging tools

## Module Boundaries

- `pid.cpp`: per-wheel PID controller logic
- `mecanum.cpp`: chassis command to wheel target conversion
- `odometry.cpp`: wheel-speed integration into raw odometry
- `motor_driver.cpp`: PWM and direction output abstraction
- `safety.cpp`: command timeout and enable gating
- `microros_transport.cpp`: transport and executor integration points

## Runtime Contract

Pi to ESP32:
- `/cmd_vel` as `geometry_msgs/msg/Twist`
- `/robot_enable` as `std_msgs/msg/Bool`

ESP32 to Pi:
- `/odom` as `nav_msgs/msg/Odometry`
- `/imu` as `sensor_msgs/msg/Imu`
- `/limit_switch` as `std_msgs/msg/Bool`

The firmware intentionally does not publish IR topics in this implementation,
matching the current explicit firmware instruction for this repository task.
