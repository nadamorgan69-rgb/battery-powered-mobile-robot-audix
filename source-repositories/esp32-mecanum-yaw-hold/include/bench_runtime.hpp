#pragma once

#include <cstdint>

#include "shared_state.hpp"

namespace app {

enum TaskHeartbeat : std::uint8_t {
    kCommandTaskHeartbeat = 0U,
    kMotionTaskHeartbeat = 1U,
    kSensorTaskHeartbeat = 2U,
    kTelemetryTaskHeartbeat = 3U,
    kStepperTaskHeartbeat = 4U,
    kTaskHeartbeatCount = 5U,
};

enum class BenchTargetMoveAxis : std::uint8_t {
    kNone = 0U,
    kX = 1U,
    kYaw = 2U,
};

struct BenchRuntimeSnapshot {
    bool imu_stream_enabled = false;
    bool encoder_stream_enabled = false;
    bool rtos_stream_enabled = false;
    bool timed_motion_active = false;
    float timed_vx = 0.0f;
    float timed_vy = 0.0f;
    float timed_wz = 0.0f;
    std::uint32_t motion_or_override_expires_ms = 0U;
    bool manual_override_active = false;
    bool manual_override_auto_disable = false;
    float manual_pwm[kWheelCount] = {};
    bool wheel_speed_override_active = false;
    bool wheel_speed_override_auto_disable = false;
    std::uint8_t wheel_speed_override_mask = 0U;
    float wheel_speed_target_rad_s[kWheelCount] = {};
    float wheel_speed_target_rpm = 0.0f;
    std::uint32_t wheel_speed_override_expires_ms = 0U;
    bool target_move_active = false;
    BenchTargetMoveAxis target_move_axis = BenchTargetMoveAxis::kNone;
    float target_move_delta = 0.0f;
    float target_move_start = 0.0f;
    float target_move_heading_start = 0.0f;
    float target_move_command = 0.0f;
    std::uint32_t target_move_expires_ms = 0U;
    std::uint32_t target_move_last_update_ms = 0U;
    bool heading_hold_active = false;
    float heading_hold_target = 0.0f;
    float heading_hold_stable_ms = 0.0f;
};

struct TaskHeartbeatCounts {
    std::uint32_t counts[kTaskHeartbeatCount] = {};
};

void initBenchRuntime();
void serviceBenchRuntime(std::uint32_t now_ms);
void scheduleTimedMotionCommand(float vx, float vy, float wz, std::uint32_t duration_ms, std::uint32_t now_ms);
void scheduleTargetMove(BenchTargetMoveAxis axis,
                        float target_delta,
                        float command_speed,
                        std::uint32_t timeout_ms,
                        std::uint32_t now_ms);
void scheduleSingleWheelOverride(WheelIndex wheel, float pwm, std::uint32_t duration_ms, std::uint32_t now_ms);
void scheduleAllWheelOverride(float pwm, std::uint32_t duration_ms, std::uint32_t now_ms);
void scheduleWheelSpeedOverride(const float target_rad_s[kWheelCount],
                                std::uint8_t wheel_mask,
                                float target_rpm,
                                std::uint32_t duration_ms,
                                std::uint32_t now_ms);
void setHeadingHoldEnabled(bool enabled, std::uint32_t now_ms);
void clearManualOverride();
void emergencyStopBench(std::uint32_t now_ms);

void setBenchImuStreamEnabled(bool enabled);
void setBenchEncoderStreamEnabled(bool enabled);
void setBenchRtosStreamEnabled(bool enabled);

BenchRuntimeSnapshot getBenchRuntimeSnapshot();
bool getManualOverrideSnapshot(float pwm_out[kWheelCount]);
bool getWheelSpeedOverrideSnapshot(float target_rad_s_out[kWheelCount]);

void markTaskHeartbeat(TaskHeartbeat task);
TaskHeartbeatCounts consumeHeartbeatCounts();

}  // namespace app
