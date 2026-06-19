#pragma once

#include <cstddef>
#include <cstdint>

#include "config.hpp"

namespace app {

// -- Wheel index enum -----------------------------------------------
enum WheelIndex : std::size_t {
    kFrontLeft = 0,
    kFrontRight = 1,
    kRearLeft = 2,
    kRearRight = 3,
};
constexpr std::size_t kWheelCount = 4U;

// -- Shared data structs --------------------------------------------
struct CommandState {
    float cmd_vx = 0.0f;
    float cmd_vy = 0.0f;
    float cmd_wz = 0.0f;
    bool robot_enabled = false;
    uint32_t last_cmd_time_ms = 0U;
};

struct WheelState {
    float target_w_rad_s[kWheelCount] = {};
    float measured_w_rad_s[kWheelCount] = {};
    int32_t encoder_counts[kWheelCount] = {};
    float pwm_output[kWheelCount] = {};
};

struct OdometryState {
    float x = 0.0f;
    float y = 0.0f;
    float theta = 0.0f;
    float vx = 0.0f;
    float vy = 0.0f;
    float wtheta = 0.0f;
};

struct IMUState {
    float accel_x = 0.0f;
    float accel_y = 0.0f;
    float accel_z = 0.0f;
    float gyro_x = 0.0f;
    float gyro_y = 0.0f;
    float gyro_z = 0.0f;
    float orientation_z = 0.0f;  // integrated yaw (rad)
};

struct SensorState {
    bool limit_switch_pressed = false;
};

enum class HeadingControlPhase : std::uint8_t {
    kIdle = 0,
    kStraightMove = 1,
    kHoldActive = 2,
    kRotateMove = 3,
    kSettled = 4,
};

struct HeadingControlState {
    bool hold_enabled = false;
    bool settled = true;
    HeadingControlPhase phase = HeadingControlPhase::kIdle;
    float target_yaw = 0.0f;
    float yaw_error = 0.0f;
    float gyro_z = 0.0f;
    float command_wz = 0.0f;
    float stable_ms = 0.0f;
};

enum class PidGainGroup : std::uint8_t {
    kWheel = 0,
    kX = 1,
    kY = 2,
    kYaw = 3,
};

struct PidGains {
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    std::uint32_t version = 0U;
};

struct PidTuningState {
    PidGains wheel;
    PidGains x;
    PidGains y;
    PidGains yaw;
};

// -- Init -----------------------------------------------------------
void initSharedState();

// -- Command --------------------------------------------------------
CommandState getCommandState();
void updateCommandState(const CommandState& state);
void setCommandVelocity(float vx, float vy, float wz, uint32_t timestamp_ms);
void setRobotEnabled(bool enabled);

// -- Wheel ----------------------------------------------------------
WheelState getWheelState();
void setWheelTargets(const float target_w_rad_s[kWheelCount]);
void setMeasuredWheelState(const float measured_w_rad_s[kWheelCount],
                           const int32_t encoder_counts[kWheelCount]);
void setPwmOutputs(const float pwm_output[kWheelCount]);

// -- Odometry -------------------------------------------------------
OdometryState getOdometryState();
void setOdometryState(const OdometryState& state);

// -- IMU ------------------------------------------------------------
IMUState getImuState();
void setImuState(const IMUState& state);

// -- Sensor ---------------------------------------------------------
SensorState getSensorState();
void setLimitSwitchPressed(bool pressed);

// -- Heading control -----------------------------------------------
HeadingControlState getHeadingControlState();
void setHeadingControlState(const HeadingControlState& state);

// -- PID tuning -----------------------------------------------------
PidTuningState getPidTuningState();
PidGains getPidGains(PidGainGroup group);
void setPidGains(PidGainGroup group, float kp, float ki, float kd);

// -- Task entry points ----------------------------------------------
void commandRxTask(void*);
void motionControlTask(void*);
void sensorUpdateTask(void*);
void telemetryTask(void*);

}  // namespace app
