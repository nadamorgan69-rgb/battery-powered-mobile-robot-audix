#pragma once

#include <cstdint>

namespace app {

// -- Firmware mode --------------------------------------------------
constexpr const char* firmwareTransport() {
#if defined(APP_WIFI_DASHBOARD)
    return "wifi";
#else
    return "serial";
#endif
}

constexpr const char* firmwareSubsystem() {
#if defined(APP_WIFI_DASHBOARD) && defined(APP_STEPPER_BENCH)
    return "stepper_dashboard";
#elif defined(APP_WIFI_DASHBOARD) && defined(APP_JOYSTICK_DASHBOARD)
    return "base_joystick_dashboard";
#elif defined(APP_WIFI_DASHBOARD)
    return "base_dashboard";
#elif defined(APP_SERIAL_PID_BENCH)
    return "pid";
#elif defined(APP_SERIAL_STEPPER_BENCH)
    return "stepper";
#elif defined(APP_SERIAL_IMU_ENCODER_BENCH)
    return "imu_encoder";
#else
    return "unknown";
#endif
}

// -- Math constants -------------------------------------------------
constexpr float kPi = 3.14159265358979323846f;
constexpr float kGravityMps2 = 9.80665f;

// -- Serial ---------------------------------------------------------
constexpr uint32_t SERIAL_BAUD = 115200U;

// -- Wi-Fi sandbox console -----------------------------------------
constexpr const char* WIFI_AP_SSID = "AudixBench-ESP32";
constexpr const char* WIFI_AP_PASSWORD = "audixbench123";
constexpr uint8_t WIFI_AP_CHANNEL = 6U;
constexpr uint8_t WIFI_AP_MAX_CLIENTS = 4U;
constexpr uint16_t WIFI_HTTP_PORT = 80U;
constexpr uint16_t WIFI_WS_PORT = 81U;

// -- I2C / IMU ------------------------------------------------------
constexpr int IMU_SDA_PIN = 21;
constexpr int IMU_SCL_PIN = 22;
constexpr uint32_t I2C_FREQUENCY_HZ = 400000U;
constexpr uint8_t MPU6050_I2C_ADDRESS = 0x68U;
constexpr uint32_t IMU_GYRO_BIAS_SAMPLES = 500U;

// -- Limit switch ---------------------------------------------------
constexpr int LIMIT_SWITCH_PIN = 23;
constexpr bool LIMIT_SWITCH_ACTIVE_LOW = true;

// -- Stepper Wi-Fi bench -------------------------------------------
constexpr int STEPPER_STEP_PIN = 26;
constexpr int STEPPER_DIR_PIN = 25;
constexpr int STEPPER_ENABLE_PIN = 27;
constexpr bool STEPPER_ENABLE_ACTIVE_LOW = true;
constexpr bool STEPPER_UP_DIRECTION_HIGH = true;
constexpr int STEPPER_HOME_SWITCH_PIN = 4;
constexpr bool STEPPER_HOME_SWITCH_ACTIVE_LOW = true;
constexpr bool STEPPER_HOME_SWITCH_PULLUP = true;
constexpr float STEPPER_DEFAULT_RATE_HZ = 400.0f;
constexpr float STEPPER_DEFAULT_HOME_RATE_HZ = 250.0f;
constexpr float STEPPER_DEFAULT_HOME_TIMEOUT_SEC = 12.0f;
constexpr bool STEPPER_HOME_DIRECTION_UP = false;
constexpr bool STEPPER_AUTO_DISABLE_ON_IDLE = true;
constexpr uint32_t STEPPER_LOG_INTERVAL_MS = 1000U;
constexpr uint32_t STEPPER_PULSE_WIDTH_US = 12U;

// -- Encoder GPIO pins ----------------------------------------------
// Ordered: FL, FR, RL, RR
// Board labels from the current harness:
// FL=ENC2, FR=ENC3, RL/BL=ENC1, RR/BR=ENC4
constexpr int ENCODER_A_PINS[4] = {34, 33, 39, 25};
constexpr int ENCODER_B_PINS[4] = {35, 32, 36, 26};

// Encoder CPR (counts per revolution of motor output shaft), measured on the current hardware.
constexpr float ENCODER_COUNTS_PER_REV = 4346.8f;

inline float encoderCountToRad(int32_t counts) {
    return static_cast<float>(counts) * (2.0f * kPi / ENCODER_COUNTS_PER_REV);
}

inline float wrapAngleRad(float angle) {
    while (angle > kPi) {
        angle -= 2.0f * kPi;
    }
    while (angle < -kPi) {
        angle += 2.0f * kPi;
    }
    return angle;
}

// -- Motor driver GPIO pins ----------------------------------------
// Two-input H-bridge style, ordered: FL, FR, RL, RR.
// Board labels from the current harness:
// FL=MOTD, FR=MOTB, RL/BL=MOTC, RR/BR=MOTA
constexpr int MOTOR_IN1_PINS[4] = {17, 13, 4, 27};
constexpr int MOTOR_IN2_PINS[4] = {18, 19, 16, 14};
// +1 or -1 to flip physical wiring polarity per wheel
constexpr int MOTOR_POLARITY[4] = {1, -1, 1, -1};

// -- PWM ------------------------------------------------------------
constexpr uint32_t PWM_FREQUENCY = 2000U;
constexpr uint32_t PWM_RESOLUTION_BITS = 10U;
constexpr float PWM_MAX_F = 1023.0f;  // 2^10 - 1
constexpr float MOTOR_DEADBAND = 40.0f;  // counts below which motor stalls

// -- Mecanum kinematics ---------------------------------------------
constexpr float WHEEL_RADIUS_M = 0.0485f;
constexpr float WHEELBASE_HALF_M = 0.090f;  // half of 180 mm wheelbase
constexpr float TRACK_HALF_WIDTH_M = 0.147f;  // half of 294 mm track width

// -- PID gains ------------------------------------------------------
constexpr float PID_KP = 1.2f;
constexpr float PID_KI = 0.8f;
constexpr float PID_KD = 0.05f;
constexpr float PID_OUTPUT_MAX = 1.0f;
constexpr float PID_INTEGRAL_MAX = 50.0f;
constexpr float WHEEL_STOP_EPSILON_RAD_S = 0.05f;

// -- Target movement PID defaults ---------------------------------
constexpr float PID_X_KP = 0.8f;
constexpr float PID_X_KI = 0.0f;
constexpr float PID_X_KD = 0.02f;
constexpr float PID_Y_KP = 0.8f;
constexpr float PID_Y_KI = 0.0f;
constexpr float PID_Y_KD = 0.02f;
constexpr float PID_YAW_KP = 1.1f;
constexpr float PID_YAW_KI = 0.0f;
constexpr float PID_YAW_KD = 0.03f;
constexpr float TARGET_MOVE_LINEAR_SPEED_LIMIT_MPS = 0.12f;
constexpr float TARGET_MOVE_YAW_SPEED_LIMIT_RAD_S = 0.60f;
constexpr float TARGET_MOVE_YAW_HOLD_LIMIT_RAD_S = 0.35f;
constexpr float TARGET_MOVE_LINEAR_TOLERANCE_M = 0.01f;
constexpr float TARGET_MOVE_YAW_TOLERANCE_RAD = 0.035f;
constexpr float TARGET_PID_INTEGRAL_MAX = 2.0f;
constexpr float HEADING_HOLD_DEADBAND_RAD = 0.05236f;  // about 3 degrees
constexpr float HEADING_HOLD_RATE_DEADBAND_RAD_S = 0.035f;  // about 2 deg/s
constexpr float HEADING_HOLD_STABLE_MS = 250.0f;
constexpr float HEADING_HOLD_MIN_WZ_RAD_S = 0.16f;
constexpr float WHEEL_TARGET_ZERO_EPSILON_RAD_S = 0.03f;

// -- IMU-odometry blending -----------------------------------------
constexpr float IMU_YAW_BLEND_ALPHA = 0.05f;  // 0=encoder only, 1=IMU only
constexpr float IMU_YAW_BLEND_LIMIT_RAD = 0.1f;  // max correction per cycle

// -- Timing ---------------------------------------------------------
constexpr uint32_t CONTROL_PERIOD_MS = 10U;
constexpr uint32_t SENSOR_PERIOD_MS = 5U;
constexpr uint32_t TELEMETRY_PERIOD_MS = 50U;
constexpr uint32_t ENCODER_TELEMETRY_PERIOD_MS = 200U;
constexpr uint32_t CMD_TIMEOUT_MS = 500U;
constexpr float BENCH_TARGET_LINEAR_SPEED_MPS = 0.05f;
constexpr float BENCH_TARGET_YAW_SPEED_RAD_S = 0.20f;
constexpr uint32_t BENCH_TARGET_TIMEOUT_EXTRA_MS = 3000U;
constexpr float BENCH_DRIVE_MAX_VX_MPS = 0.25f;
constexpr float BENCH_DRIVE_MAX_VY_MPS = 0.25f;
constexpr float BENCH_DRIVE_MAX_WZ_RAD_S = 1.0f;
constexpr uint32_t BENCH_DRIVE_MAX_HOLD_MS = 500U;

// -- Task config ----------------------------------------------------
constexpr uint32_t COMMAND_TASK_STACK_BYTES = 8192U;
constexpr uint32_t MOTION_TASK_STACK_BYTES = 4096U;
constexpr uint32_t SENSOR_TASK_STACK_BYTES = 4096U;
constexpr uint32_t TELEMETRY_TASK_STACK_BYTES = 4096U;
constexpr uint32_t STEPPER_TASK_STACK_BYTES = 4096U;

constexpr uint32_t COMMAND_TASK_PRIORITY = 5U;
constexpr uint32_t MOTION_TASK_PRIORITY = 4U;
constexpr uint32_t SENSOR_TASK_PRIORITY = 3U;
constexpr uint32_t TELEMETRY_TASK_PRIORITY = 2U;
constexpr uint32_t STEPPER_TASK_PRIORITY = 4U;

}  // namespace app
