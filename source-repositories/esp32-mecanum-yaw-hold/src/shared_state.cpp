#include "shared_state.hpp"

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

namespace app {

namespace {

CommandState g_command_state;
WheelState g_wheel_state;
OdometryState g_odometry_state;
IMUState g_imu_state;
SensorState g_sensor_state;
HeadingControlState g_heading_control_state;
PidTuningState g_pid_tuning_state;

StaticSemaphore_t g_command_mutex_buffer;
StaticSemaphore_t g_wheel_mutex_buffer;
StaticSemaphore_t g_odometry_mutex_buffer;
StaticSemaphore_t g_imu_mutex_buffer;
StaticSemaphore_t g_sensor_mutex_buffer;
StaticSemaphore_t g_heading_mutex_buffer;
StaticSemaphore_t g_pid_mutex_buffer;

SemaphoreHandle_t g_command_mutex = nullptr;
SemaphoreHandle_t g_wheel_mutex = nullptr;
SemaphoreHandle_t g_odometry_mutex = nullptr;
SemaphoreHandle_t g_imu_mutex = nullptr;
SemaphoreHandle_t g_sensor_mutex = nullptr;
SemaphoreHandle_t g_heading_mutex = nullptr;
SemaphoreHandle_t g_pid_mutex = nullptr;

void ensureMutexes() {
    if (g_command_mutex == nullptr) {
        g_command_mutex = xSemaphoreCreateMutexStatic(&g_command_mutex_buffer);
        g_wheel_mutex = xSemaphoreCreateMutexStatic(&g_wheel_mutex_buffer);
        g_odometry_mutex = xSemaphoreCreateMutexStatic(&g_odometry_mutex_buffer);
        g_imu_mutex = xSemaphoreCreateMutexStatic(&g_imu_mutex_buffer);
        g_sensor_mutex = xSemaphoreCreateMutexStatic(&g_sensor_mutex_buffer);
        g_heading_mutex = xSemaphoreCreateMutexStatic(&g_heading_mutex_buffer);
        g_pid_mutex = xSemaphoreCreateMutexStatic(&g_pid_mutex_buffer);
    }
}

void lockMutex(SemaphoreHandle_t mutex) {
    xSemaphoreTake(mutex, portMAX_DELAY);
}

void unlockMutex(SemaphoreHandle_t mutex) {
    xSemaphoreGive(mutex);
}

}  // namespace

void initSharedState() {
    ensureMutexes();

    const CommandState command_state{};
    const WheelState wheel_state{};
    const OdometryState odometry_state{};
    const IMUState imu_state{};
    const SensorState sensor_state{};
    const HeadingControlState heading_control_state{};
    const PidTuningState pid_tuning_state{
        PidGains{PID_KP, PID_KI, PID_KD, 1U},
        PidGains{PID_X_KP, PID_X_KI, PID_X_KD, 1U},
        PidGains{PID_Y_KP, PID_Y_KI, PID_Y_KD, 1U},
        PidGains{PID_YAW_KP, PID_YAW_KI, PID_YAW_KD, 1U},
    };

    updateCommandState(command_state);
    setWheelTargets(wheel_state.target_w_rad_s);
    setMeasuredWheelState(wheel_state.measured_w_rad_s, wheel_state.encoder_counts);
    setPwmOutputs(wheel_state.pwm_output);
    setOdometryState(odometry_state);
    setImuState(imu_state);
    setLimitSwitchPressed(sensor_state.limit_switch_pressed);
    setHeadingControlState(heading_control_state);
    lockMutex(g_pid_mutex);
    g_pid_tuning_state = pid_tuning_state;
    unlockMutex(g_pid_mutex);
}

CommandState getCommandState() {
    ensureMutexes();
    lockMutex(g_command_mutex);
    const CommandState snapshot = g_command_state;
    unlockMutex(g_command_mutex);
    return snapshot;
}

void updateCommandState(const CommandState& command_state) {
    ensureMutexes();
    lockMutex(g_command_mutex);
    g_command_state = command_state;
    unlockMutex(g_command_mutex);
}

void setCommandVelocity(float vx, float vy, float wz, std::uint32_t timestamp_ms) {
    ensureMutexes();
    lockMutex(g_command_mutex);
    g_command_state.cmd_vx = vx;
    g_command_state.cmd_vy = vy;
    g_command_state.cmd_wz = wz;
    g_command_state.last_cmd_time_ms = timestamp_ms;
    unlockMutex(g_command_mutex);
}

void setRobotEnabled(bool enabled) {
    ensureMutexes();
    lockMutex(g_command_mutex);
    g_command_state.robot_enabled = enabled;
    unlockMutex(g_command_mutex);
}

WheelState getWheelState() {
    ensureMutexes();
    lockMutex(g_wheel_mutex);
    const WheelState snapshot = g_wheel_state;
    unlockMutex(g_wheel_mutex);
    return snapshot;
}

void setWheelTargets(const float target_w_rad_s[kWheelCount]) {
    ensureMutexes();
    lockMutex(g_wheel_mutex);
    for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
        g_wheel_state.target_w_rad_s[wheel] = target_w_rad_s[wheel];
    }
    unlockMutex(g_wheel_mutex);
}

void setMeasuredWheelState(const float measured_w_rad_s[kWheelCount],
                           const std::int32_t encoder_counts[kWheelCount]) {
    ensureMutexes();
    lockMutex(g_wheel_mutex);
    for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
        g_wheel_state.measured_w_rad_s[wheel] = measured_w_rad_s[wheel];
        g_wheel_state.encoder_counts[wheel] = encoder_counts[wheel];
    }
    unlockMutex(g_wheel_mutex);
}

void setPwmOutputs(const float pwm_output[kWheelCount]) {
    ensureMutexes();
    lockMutex(g_wheel_mutex);
    for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
        g_wheel_state.pwm_output[wheel] = pwm_output[wheel];
    }
    unlockMutex(g_wheel_mutex);
}

OdometryState getOdometryState() {
    ensureMutexes();
    lockMutex(g_odometry_mutex);
    const OdometryState snapshot = g_odometry_state;
    unlockMutex(g_odometry_mutex);
    return snapshot;
}

void setOdometryState(const OdometryState& odometry_state) {
    ensureMutexes();
    lockMutex(g_odometry_mutex);
    g_odometry_state = odometry_state;
    unlockMutex(g_odometry_mutex);
}

IMUState getImuState() {
    ensureMutexes();
    lockMutex(g_imu_mutex);
    const IMUState snapshot = g_imu_state;
    unlockMutex(g_imu_mutex);
    return snapshot;
}

void setImuState(const IMUState& imu_state) {
    ensureMutexes();
    lockMutex(g_imu_mutex);
    g_imu_state = imu_state;
    unlockMutex(g_imu_mutex);
}

SensorState getSensorState() {
    ensureMutexes();
    lockMutex(g_sensor_mutex);
    const SensorState snapshot = g_sensor_state;
    unlockMutex(g_sensor_mutex);
    return snapshot;
}

void setLimitSwitchPressed(bool pressed) {
    ensureMutexes();
    lockMutex(g_sensor_mutex);
    g_sensor_state.limit_switch_pressed = pressed;
    unlockMutex(g_sensor_mutex);
}

HeadingControlState getHeadingControlState() {
    ensureMutexes();
    lockMutex(g_heading_mutex);
    const HeadingControlState snapshot = g_heading_control_state;
    unlockMutex(g_heading_mutex);
    return snapshot;
}

void setHeadingControlState(const HeadingControlState& heading_control_state) {
    ensureMutexes();
    lockMutex(g_heading_mutex);
    g_heading_control_state = heading_control_state;
    unlockMutex(g_heading_mutex);
}

PidTuningState getPidTuningState() {
    ensureMutexes();
    lockMutex(g_pid_mutex);
    const PidTuningState snapshot = g_pid_tuning_state;
    unlockMutex(g_pid_mutex);
    return snapshot;
}

PidGains getPidGains(PidGainGroup group) {
    ensureMutexes();
    lockMutex(g_pid_mutex);
    PidGains gains;
    switch (group) {
        case PidGainGroup::kWheel:
            gains = g_pid_tuning_state.wheel;
            break;
        case PidGainGroup::kX:
            gains = g_pid_tuning_state.x;
            break;
        case PidGainGroup::kY:
            gains = g_pid_tuning_state.y;
            break;
        case PidGainGroup::kYaw:
            gains = g_pid_tuning_state.yaw;
            break;
    }
    unlockMutex(g_pid_mutex);
    return gains;
}

void setPidGains(PidGainGroup group, float kp, float ki, float kd) {
    ensureMutexes();
    lockMutex(g_pid_mutex);
    PidGains* target = nullptr;
    switch (group) {
        case PidGainGroup::kWheel:
            target = &g_pid_tuning_state.wheel;
            break;
        case PidGainGroup::kX:
            target = &g_pid_tuning_state.x;
            break;
        case PidGainGroup::kY:
            target = &g_pid_tuning_state.y;
            break;
        case PidGainGroup::kYaw:
            target = &g_pid_tuning_state.yaw;
            break;
    }

    if (target != nullptr) {
        target->kp = kp;
        target->ki = ki;
        target->kd = kd;
        ++target->version;
    }
    unlockMutex(g_pid_mutex);
}

}  // namespace app
