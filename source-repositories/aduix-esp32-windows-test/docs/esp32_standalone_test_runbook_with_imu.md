# ESP32 Standalone Test Runbook With IMU

Use this runbook for ESP32-only testing from Windows/VS Code. It tests the
base Wi-Fi dashboard firmware, IMU, encoders, raw motors, RPM control, movement
commands, and PID velocity debug commands without the Pi or ROS.

Do not use this runbook with the stepper dashboard firmware.

## 1. Flash the correct firmware

```powershell
cd "C:\Users\TiBa\Documents\PlatformIO\Projects\audix_esp32_windows_test"
pio run -e esp32_wifi_base_dashboard
pio run -e esp32_wifi_base_dashboard -t upload --upload-port COM10
pio run -e esp32_wifi_base_dashboard -t uploadfs --upload-port COM10
```

If the ESP32 is not on `COM10`, run:

```powershell
pio device list
```

If upload gives a boot-mode or connect error, hold `BOOT`, start upload again,
and release `BOOT` once uploading starts.

## 2. First boot check

Keep motor power disconnected for the first boot. Open the serial fallback:

```powershell
pio device monitor -p COM10 -b 115200
```

Expected boot lines:

```text
[mode] transport=wifi subsystem=base_dashboard
[wifi] hotspot ready
```

If the ESP32 reboots or reports brownout, disconnect motor power and debug
power/wiring before continuing.

## 3. Open the Wi-Fi dashboard

Connect the laptop to:

```text
SSID: AudixBench-ESP32
Password: audixbench123
```

Open:

```text
http://192.168.4.1/?v=stable-pid
```

No internet is needed. If the page is missing, rerun:

```powershell
pio run -e esp32_wifi_base_dashboard -t uploadfs --upload-port COM10
```

If the old page appears, press `Ctrl + F5`.

Direct API checks:

```text
http://192.168.4.1/api/ping
http://192.168.4.1/api/state
```

## 4. Safety reset first

Send:

```text
stop
quiet
status
```

Expected:

```text
[status] emergency stop applied; streams off; motor pins low.
```

## 5. IMU test

IMU wiring:

```text
ESP32 GPIO21 -> IMU SDA
ESP32 GPIO22 -> IMU SCL
ESP32 3V3    -> IMU VCC
ESP32 GND    -> IMU GND
```

Send:

```text
i2cscan
status
imu on
```

Expected:

```text
i2cscan finds 0x68
status shows imuHealthy=yes
[imu] yaw=... gyro=(...) accel=(...)
```

Physical checks:

```text
Tilt board        -> accel values change
Rotate board      -> gyro values spike
Rotate around Z   -> yaw changes
Keep board still  -> gyro near zero, yaw mostly stable
```

Stop the stream:

```text
imu off
```

If `0x68` does not appear in `i2cscan`, check power, SDA/SCL, GND, and
AD0/address wiring.

## 6. Encoder test before motor power

Keep motor power disconnected if possible, or keep the robot fully lifted on
blocks. Send:

```text
stop
encoders zero
encoders on
```

Expected mapping:

```text
Front Left  -> FL row changes -> ENC2 -> GPIO34, GPIO35
Front Right -> FR row changes -> ENC3 -> GPIO33, GPIO32
Rear Left   -> RL row changes -> ENC1 -> GPIO39, GPIO36
Rear Right  -> RR row changes -> ENC4 -> GPIO25, GPIO26
```

Use this for one clean reading:

```text
encoders once
```

If the wrong row changes, fix encoder wiring or mapping before motor/PID tests.

## 7. Raw motor test

Only do this with the robot on blocks.

```text
stop
encoders zero
encoders once
motors all max 1.0
encoders once
motors all -1.00 1.0
encoders once
```

Then one wheel at a time:

```text
motor fl max 1.0
encoders once
motor fr max 1.0
encoders once
motor rl max 1.0
encoders once
motor rr max 1.0
encoders once
```

Expected:

```text
motor fl -> only front-left wheel moves and FL count changes
motor fr -> only front-right wheel moves and FR count changes
motor rl -> only rear-left wheel moves and RL count changes
motor rr -> only rear-right wheel moves and RR count changes
```

If one-wheel commands move the wrong wheel, stop and debug motor GPIO
wiring/mapping before PID.

## 8. RPM test before distance movement

RPM uses encoders and wheel PID:

```text
stop
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

Expected:

```text
single-wheel RPM moves only the selected wheel
all-wheel RPM updates measured RPM in the GUI
```

If raw PWM works but RPM does not, debug encoder sign/direction or PID tuning.

## 9. Forward, backward, and rotate test

Only continue after raw motor and encoder tests look correct.

Inspect or adjust runtime PID values first:

```text
pid show
pid wheel 1.20 0.80 0.05
pid x 0.80 0.00 0.02
pid yaw 1.10 0.00 0.03
```

```text
stop
encoders zero
encoders on
forward 20
encoders once
backward 20
encoders once
rotate 90
encoders once
rotate -90
encoders once
stop
```

Meaning:

```text
forward 20   -> move forward about 20 cm
backward 20  -> move backward about 20 cm
rotate 90    -> rotate counter-clockwise 90 degrees
rotate -90   -> rotate clockwise 90 degrees
```

Expected result logs:

```text
[move] axis=x target=20.0cm measured=...cm error=...cm
[move] axis=yaw target=90.0deg measured=...deg error=...deg
```

If the robot moves in the wrong direction, fix motor direction, wheel order, or
mecanum orientation before tuning PID.

## 10. PID velocity debug test

Only use this after movement commands behave directionally.

```text
stop
encoders zero
encoders on
imu on
pidx 0.05 1.0
stop
encoders once
pidy 0.05 1.0
stop
encoders once
pidyaw 0.20 1.0
stop
encoders once
imu off
encoders off
stop
```

Expected:

```text
pidx 0.05 1.0    -> odom vx changes most
pidy 0.05 1.0    -> odom vy changes most
pidyaw 0.20 1.0  -> odom wz changes and IMU yaw/gyro-z changes
```

## Command meaning

```text
motor fl max 1.0 -> front-left only, 100% raw PWM, 1 second
motor fl 0.30 1.0 -> front-left only, 30% raw PWM, 1 second
rpm fl 60 2.0 -> front-left wheel, 60 RPM target, 2 seconds, closed loop
pidx 0.05 1.0 -> 0.05 m/s chassis X velocity target, 1 second
```

## Stop conditions

Do not continue if:

```text
motor spins immediately after boot
stop does not force motors low
wrong wheel moves during single-wheel test
encoder count changes on the wrong row
IMU i2cscan does not find 0x68
ESP32 keeps rebooting or browning out
```
