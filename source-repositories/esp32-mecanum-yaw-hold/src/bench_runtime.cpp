#include "bench_runtime.hpp"

#include <cmath>

#include <Arduino.h>

#include "console_sink.hpp"
#include "imu_driver.hpp"
#include "motor_driver.hpp"
#include "pid.hpp"
#include "shared_state.hpp"

namespace app {

namespace {

BenchRuntimeSnapshot g_bench_runtime;
TaskHeartbeatCounts g_heartbeat_counts;
portMUX_TYPE g_bench_mux = portMUX_INITIALIZER_UNLOCKED;
PidController g_target_x_pid;
PidController g_target_yaw_pid;
std::uint32_t g_target_x_pid_version = 0U;
std::uint32_t g_target_yaw_pid_version = 0U;

template <typename Fn>
void withBenchLock(Fn&& fn) {
    portENTER_CRITICAL(&g_bench_mux);
    fn();
    portEXIT_CRITICAL(&g_bench_mux);
}

bool timeReached(std::uint32_t now_ms, std::uint32_t target_ms) {
    return static_cast<std::uint32_t>(now_ms - target_ms) < 0x80000000U;
}

const char* targetMoveAxisName(BenchTargetMoveAxis axis) {
    switch (axis) {
        case BenchTargetMoveAxis::kX:
            return "x";
        case BenchTargetMoveAxis::kYaw:
            return "yaw";
        default:
            return "none";
    }
}

float targetMoveMeasuredDelta(const OdometryState& odometry, BenchTargetMoveAxis axis, float start_value) {
    if (axis == BenchTargetMoveAxis::kYaw) {
        const IMUState imu = getImuState();
        const float yaw = imuDriver().isHealthy() ? imu.orientation_z : odometry.theta;
        return wrapAngleRad(yaw - start_value);
    }
    return odometry.x - start_value;
}

float currentHeadingForTargetMove(const OdometryState& odometry) {
    const IMUState imu = getImuState();
    return imuDriver().isHealthy() ? imu.orientation_z : odometry.theta;
}

void publishHeadingControl(HeadingControlPhase phase,
                           bool hold_enabled,
                           bool settled,
                           float target_yaw,
                           float yaw_error,
                           float gyro_z,
                           float command_wz,
                           float stable_ms) {
    HeadingControlState state;
    state.hold_enabled = hold_enabled;
    state.settled = settled;
    state.phase = phase;
    state.target_yaw = wrapAngleRad(target_yaw);
    state.yaw_error = wrapAngleRad(yaw_error);
    state.gyro_z = gyro_z;
    state.command_wz = command_wz;
    state.stable_ms = stable_ms;
    setHeadingControlState(state);
}

void clearHeadingHoldStateLocked() {
    g_bench_runtime.heading_hold_active = false;
    g_bench_runtime.heading_hold_target = 0.0f;
    g_bench_runtime.heading_hold_stable_ms = 0.0f;
}

float computeHeadingHoldCommand(float target_yaw,
                                float current_yaw,
                                float gyro_z,
                                float dt_seconds,
                                float& stable_ms,
                                bool& settled) {
    const float yaw_error = wrapAngleRad(target_yaw - current_yaw);
    const bool inside_error = std::fabs(yaw_error) <= HEADING_HOLD_DEADBAND_RAD;
    const bool inside_rate = std::fabs(gyro_z) <= HEADING_HOLD_RATE_DEADBAND_RAD_S;

    if (inside_error && inside_rate) {
        stable_ms += dt_seconds * 1000.0f;
        if (stable_ms >= HEADING_HOLD_STABLE_MS) {
            settled = true;
            g_target_yaw_pid.reset();
        }
        return 0.0f;
    }

    stable_ms = 0.0f;
    settled = false;
    float wz = g_target_yaw_pid.update(0.0f, -yaw_error, dt_seconds, true);
    wz = std::clamp(wz, -TARGET_MOVE_YAW_HOLD_LIMIT_RAD_S, TARGET_MOVE_YAW_HOLD_LIMIT_RAD_S);
    if (std::fabs(wz) > 0.0f && std::fabs(wz) < HEADING_HOLD_MIN_WZ_RAD_S) {
        wz = std::copysign(HEADING_HOLD_MIN_WZ_RAD_S, yaw_error);
    }
    return wz;
}

void applyTargetPidGainsIfNeeded(bool reset_controllers = false) {
    const PidTuningState tuning = getPidTuningState();
    if (reset_controllers || tuning.x.version != g_target_x_pid_version) {
        g_target_x_pid.setGains(tuning.x.kp, tuning.x.ki, tuning.x.kd);
        g_target_x_pid.setOutputLimits(-TARGET_MOVE_LINEAR_SPEED_LIMIT_MPS, TARGET_MOVE_LINEAR_SPEED_LIMIT_MPS);
        g_target_x_pid.setIntegralLimits(-TARGET_PID_INTEGRAL_MAX, TARGET_PID_INTEGRAL_MAX);
        g_target_x_pid.reset();
        g_target_x_pid_version = tuning.x.version;
    }
    if (reset_controllers || tuning.yaw.version != g_target_yaw_pid_version) {
        g_target_yaw_pid.setGains(tuning.yaw.kp, tuning.yaw.ki, tuning.yaw.kd);
        g_target_yaw_pid.setOutputLimits(-TARGET_MOVE_YAW_SPEED_LIMIT_RAD_S, TARGET_MOVE_YAW_SPEED_LIMIT_RAD_S);
        g_target_yaw_pid.setIntegralLimits(-TARGET_PID_INTEGRAL_MAX, TARGET_PID_INTEGRAL_MAX);
        g_target_yaw_pid.reset();
        g_target_yaw_pid_version = tuning.yaw.version;
    }
}

void printTargetMoveResult(BenchTargetMoveAxis axis,
                           float target_delta,
                           float measured_delta,
                           bool timeout) {
    if (axis == BenchTargetMoveAxis::kYaw) {
        const float target_deg = target_delta * 180.0f / kPi;
        const float measured_deg = measured_delta * 180.0f / kPi;
        const float error_deg = (target_delta - measured_delta) * 180.0f / kPi;
        consolePrintf(
            "[move] axis=yaw target=%.1fdeg measured=%.1fdeg error=%.1fdeg status=%s\n",
            target_deg,
            measured_deg,
            error_deg,
            timeout ? "timeout" : "done");
        return;
    }

    consolePrintf(
        "[move] axis=%s target=%.1fcm measured=%.1fcm error=%.1fcm status=%s\n",
        targetMoveAxisName(axis),
        target_delta * 100.0f,
        measured_delta * 100.0f,
        (target_delta - measured_delta) * 100.0f,
        timeout ? "timeout" : "done");
}

const char* wheelSpeedMaskName(std::uint8_t mask) {
    static constexpr std::uint8_t kAllWheelMask = 0x0FU;
    if (mask == kAllWheelMask) {
        return "all";
    }
    if (mask == (1U << kFrontLeft)) {
        return "fl";
    }
    if (mask == (1U << kFrontRight)) {
        return "fr";
    }
    if (mask == (1U << kRearLeft)) {
        return "rl";
    }
    if (mask == (1U << kRearRight)) {
        return "rr";
    }
    return "mixed";
}

void clearTargetMoveState() {
    g_bench_runtime.target_move_active = false;
    g_bench_runtime.target_move_axis = BenchTargetMoveAxis::kNone;
    g_bench_runtime.target_move_delta = 0.0f;
    g_bench_runtime.target_move_start = 0.0f;
    g_bench_runtime.target_move_heading_start = 0.0f;
    g_bench_runtime.target_move_command = 0.0f;
    g_bench_runtime.target_move_expires_ms = 0U;
    g_bench_runtime.target_move_last_update_ms = 0U;
}

void clearManualPwmState() {
    g_bench_runtime.manual_override_active = false;
    g_bench_runtime.manual_override_auto_disable = false;
    for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
        g_bench_runtime.manual_pwm[wheel] = 0.0f;
    }
}

void clearWheelSpeedOverrideState() {
    g_bench_runtime.wheel_speed_override_active = false;
    g_bench_runtime.wheel_speed_override_auto_disable = false;
    g_bench_runtime.wheel_speed_override_mask = 0U;
    g_bench_runtime.wheel_speed_target_rpm = 0.0f;
    g_bench_runtime.wheel_speed_override_expires_ms = 0U;
    for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
        g_bench_runtime.wheel_speed_target_rad_s[wheel] = 0.0f;
    }
}

}  // namespace

void initBenchRuntime() {
    withBenchLock([]() {
        g_bench_runtime = BenchRuntimeSnapshot{};
        g_heartbeat_counts = TaskHeartbeatCounts{};
    });
    applyTargetPidGainsIfNeeded(true);
}

void serviceBenchRuntime(std::uint32_t now_ms) {
    float vx = 0.0f;
    float vy = 0.0f;
    float wz = 0.0f;
    bool publish_motion_command = false;
    bool motion_expired = false;
    bool override_expired = false;
    bool override_auto_disable = false;
    bool speed_override_expired = false;
    bool speed_override_auto_disable = false;
    std::uint8_t speed_override_mask = 0U;
    float speed_override_target_rpm = 0.0f;
    bool speed_override_active = false;
    bool target_move_active = false;
    BenchTargetMoveAxis target_axis = BenchTargetMoveAxis::kNone;
    float target_delta = 0.0f;
    float target_start = 0.0f;
    float target_heading_start = 0.0f;
    float target_command = 0.0f;
    std::uint32_t target_expires_ms = 0U;
    std::uint32_t target_last_update_ms = 0U;
    bool heading_hold_active = false;
    float heading_hold_target = 0.0f;
    float heading_hold_stable_ms = 0.0f;

    withBenchLock([&]() {
        if (g_bench_runtime.target_move_active) {
            target_move_active = true;
            target_axis = g_bench_runtime.target_move_axis;
            target_delta = g_bench_runtime.target_move_delta;
            target_start = g_bench_runtime.target_move_start;
            target_heading_start = g_bench_runtime.target_move_heading_start;
            target_command = g_bench_runtime.target_move_command;
            target_expires_ms = g_bench_runtime.target_move_expires_ms;
            target_last_update_ms = g_bench_runtime.target_move_last_update_ms;
        }
        heading_hold_active = g_bench_runtime.heading_hold_active;
        heading_hold_target = g_bench_runtime.heading_hold_target;
        heading_hold_stable_ms = g_bench_runtime.heading_hold_stable_ms;

        if (g_bench_runtime.wheel_speed_override_active) {
            if (timeReached(now_ms, g_bench_runtime.wheel_speed_override_expires_ms)) {
                speed_override_expired = true;
                speed_override_auto_disable = g_bench_runtime.wheel_speed_override_auto_disable;
                speed_override_mask = g_bench_runtime.wheel_speed_override_mask;
                speed_override_target_rpm = g_bench_runtime.wheel_speed_target_rpm;
                clearWheelSpeedOverrideState();
            } else {
                speed_override_active = true;
            }
        }

        if (g_bench_runtime.timed_motion_active) {
            if (timeReached(now_ms, g_bench_runtime.motion_or_override_expires_ms)) {
                g_bench_runtime.timed_motion_active = false;
                g_bench_runtime.timed_vx = 0.0f;
                g_bench_runtime.timed_vy = 0.0f;
                g_bench_runtime.timed_wz = 0.0f;
                motion_expired = true;
            } else {
                publish_motion_command = true;
                vx = g_bench_runtime.timed_vx;
                vy = g_bench_runtime.timed_vy;
                wz = g_bench_runtime.timed_wz;
            }
        }

        if (g_bench_runtime.manual_override_active &&
            timeReached(now_ms, g_bench_runtime.motion_or_override_expires_ms)) {
            override_auto_disable = g_bench_runtime.manual_override_auto_disable;
            clearManualPwmState();
            override_expired = true;
        }
    });

    if (speed_override_active) {
        setCommandVelocity(0.0f, 0.0f, 0.0f, now_ms);
        setRobotEnabled(true);
    }

    if (target_move_active) {
        applyTargetPidGainsIfNeeded();
        const OdometryState odometry = getOdometryState();
        const float measured_delta = targetMoveMeasuredDelta(odometry, target_axis, target_start);
        const float remaining_delta = target_delta - measured_delta;
        const bool reached = std::fabs(remaining_delta) <=
            ((target_axis == BenchTargetMoveAxis::kYaw) ? TARGET_MOVE_YAW_TOLERANCE_RAD : TARGET_MOVE_LINEAR_TOLERANCE_M);
        const bool timed_out = timeReached(now_ms, target_expires_ms);
        const std::uint32_t elapsed_ms = now_ms - target_last_update_ms;
        const float dt_seconds = std::max(0.001f, static_cast<float>(elapsed_ms) / 1000.0f);

        if (reached || timed_out) {
            const bool continue_heading_hold = (target_axis == BenchTargetMoveAxis::kX) && !timed_out;
            g_target_x_pid.reset();
            g_target_yaw_pid.reset();
            withBenchLock([=]() {
                clearTargetMoveState();
                if (continue_heading_hold) {
                    g_bench_runtime.heading_hold_active = true;
                    g_bench_runtime.heading_hold_target = target_heading_start;
                    g_bench_runtime.heading_hold_stable_ms = 0.0f;
                } else {
                    clearHeadingHoldStateLocked();
                }
            });
            setCommandVelocity(0.0f, 0.0f, 0.0f, now_ms);
            setRobotEnabled(continue_heading_hold);
            if (!continue_heading_hold) {
                stopAllMotors();
            }
            const float zero_pwm[kWheelCount] = {0.0f, 0.0f, 0.0f, 0.0f};
            setPwmOutputs(zero_pwm);
            publishHeadingControl(
                continue_heading_hold ? HeadingControlPhase::kSettled : HeadingControlPhase::kIdle,
                continue_heading_hold,
                true,
                target_heading_start,
                0.0f,
                getImuState().gyro_z,
                0.0f,
                0.0f);
            printTargetMoveResult(target_axis, target_delta, measured_delta, timed_out);
        } else {
            if (target_axis == BenchTargetMoveAxis::kYaw) {
                const float wz = g_target_yaw_pid.update(target_delta, measured_delta, dt_seconds, true);
                setCommandVelocity(0.0f, 0.0f, wz, now_ms);
                const float yaw_error = wrapAngleRad(target_delta - measured_delta);
                publishHeadingControl(
                    HeadingControlPhase::kRotateMove,
                    false,
                    false,
                    wrapAngleRad(target_heading_start + target_delta),
                    yaw_error,
                    getImuState().gyro_z,
                    wz,
                    0.0f);
            } else {
                const float vx = g_target_x_pid.update(target_delta, measured_delta, dt_seconds, true);
                const float heading_error = wrapAngleRad(target_heading_start - currentHeadingForTargetMove(odometry));
                float wz = g_target_yaw_pid.update(0.0f, -heading_error, dt_seconds, true);
                wz = std::clamp(wz, -TARGET_MOVE_YAW_HOLD_LIMIT_RAD_S, TARGET_MOVE_YAW_HOLD_LIMIT_RAD_S);
                setCommandVelocity(vx, 0.0f, wz, now_ms);
                publishHeadingControl(
                    HeadingControlPhase::kStraightMove,
                    true,
                    false,
                    target_heading_start,
                    heading_error,
                    getImuState().gyro_z,
                    wz,
                    0.0f);
            }
            withBenchLock([=]() {
                g_bench_runtime.target_move_last_update_ms = now_ms;
            });
            setRobotEnabled(true);
        }
        return;
    }

    if (heading_hold_active && !publish_motion_command && !speed_override_active) {
        applyTargetPidGainsIfNeeded();
        const OdometryState odometry = getOdometryState();
        const IMUState imu = getImuState();
        const float current_heading = currentHeadingForTargetMove(odometry);
        const float dt_seconds = static_cast<float>(CONTROL_PERIOD_MS) / 1000.0f;
        bool settled = false;
        float stable_ms = heading_hold_stable_ms;
        float hold_wz = computeHeadingHoldCommand(
            heading_hold_target,
            current_heading,
            imu.gyro_z,
            dt_seconds,
            stable_ms,
            settled);

        if (settled) {
            hold_wz = 0.0f;
        }

        withBenchLock([=]() {
            if (g_bench_runtime.heading_hold_active) {
                g_bench_runtime.heading_hold_stable_ms = stable_ms;
            }
        });

        setCommandVelocity(0.0f, 0.0f, hold_wz, now_ms);
        setRobotEnabled(true);
        publishHeadingControl(
            settled ? HeadingControlPhase::kSettled : HeadingControlPhase::kHoldActive,
            true,
            settled,
            heading_hold_target,
            wrapAngleRad(heading_hold_target - current_heading),
            imu.gyro_z,
            hold_wz,
            stable_ms);
        return;
    }

    if (publish_motion_command) {
        setCommandVelocity(vx, vy, wz, now_ms);
        publishHeadingControl(HeadingControlPhase::kIdle, false, true, 0.0f, 0.0f, getImuState().gyro_z, wz, 0.0f);
    } else if (motion_expired) {
        setCommandVelocity(0.0f, 0.0f, 0.0f, now_ms);
        publishHeadingControl(HeadingControlPhase::kIdle, false, true, 0.0f, 0.0f, getImuState().gyro_z, 0.0f, 0.0f);
    }

    if (override_expired) {
        stopAllMotors();
        const float zero_pwm[kWheelCount] = {0.0f, 0.0f, 0.0f, 0.0f};
        setPwmOutputs(zero_pwm);
        setCommandVelocity(0.0f, 0.0f, 0.0f, now_ms);
        if (override_auto_disable) {
            setRobotEnabled(false);
        }
    }

    if (speed_override_expired) {
        stopAllMotors();
        const float zero_pwm[kWheelCount] = {0.0f, 0.0f, 0.0f, 0.0f};
        const float zero_targets[kWheelCount] = {0.0f, 0.0f, 0.0f, 0.0f};
        setPwmOutputs(zero_pwm);
        setWheelTargets(zero_targets);
        setCommandVelocity(0.0f, 0.0f, 0.0f, now_ms);
        if (speed_override_auto_disable) {
            setRobotEnabled(false);
        }
        consolePrintf(
            "[rpm] done wheel=%s target=%.1frpm\n",
            wheelSpeedMaskName(speed_override_mask),
            speed_override_target_rpm);
    }
}

void scheduleTimedMotionCommand(float vx, float vy, float wz, std::uint32_t duration_ms, std::uint32_t now_ms) {
    withBenchLock([=]() {
        g_bench_runtime.timed_motion_active = true;
        g_bench_runtime.timed_vx = vx;
        g_bench_runtime.timed_vy = vy;
        g_bench_runtime.timed_wz = wz;
        g_bench_runtime.motion_or_override_expires_ms = now_ms + duration_ms;
        clearManualPwmState();
        clearWheelSpeedOverrideState();
        clearTargetMoveState();
        clearHeadingHoldStateLocked();
    });

    setCommandVelocity(vx, vy, wz, now_ms);
}

void scheduleTargetMove(BenchTargetMoveAxis axis,
                        float target_delta,
                        float command_speed,
                        std::uint32_t timeout_ms,
                        std::uint32_t now_ms) {
    const OdometryState odometry = getOdometryState();
    const float heading_start = currentHeadingForTargetMove(odometry);
    const float start_value = (axis == BenchTargetMoveAxis::kYaw) ? heading_start : odometry.x;
    applyTargetPidGainsIfNeeded(true);

    withBenchLock([=]() {
        g_bench_runtime.target_move_active = true;
        g_bench_runtime.target_move_axis = axis;
        g_bench_runtime.target_move_delta = target_delta;
        g_bench_runtime.target_move_start = start_value;
        g_bench_runtime.target_move_heading_start = heading_start;
        g_bench_runtime.target_move_command = command_speed;
        g_bench_runtime.target_move_expires_ms = now_ms + timeout_ms;
        g_bench_runtime.target_move_last_update_ms = now_ms;
        g_bench_runtime.timed_motion_active = false;
        g_bench_runtime.timed_vx = 0.0f;
        g_bench_runtime.timed_vy = 0.0f;
        g_bench_runtime.timed_wz = 0.0f;
        clearManualPwmState();
        clearWheelSpeedOverrideState();
        clearHeadingHoldStateLocked();
        for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
            g_bench_runtime.manual_pwm[wheel] = 0.0f;
        }
    });

    if (axis == BenchTargetMoveAxis::kYaw) {
        setCommandVelocity(0.0f, 0.0f, command_speed, now_ms);
    } else {
        setCommandVelocity(command_speed, 0.0f, 0.0f, now_ms);
    }
    setRobotEnabled(true);
}

void scheduleSingleWheelOverride(WheelIndex wheel, float pwm, std::uint32_t duration_ms, std::uint32_t now_ms) {
    withBenchLock([=]() {
        g_bench_runtime.manual_override_active = true;
        g_bench_runtime.timed_motion_active = false;
        g_bench_runtime.timed_vx = 0.0f;
        g_bench_runtime.timed_vy = 0.0f;
        g_bench_runtime.timed_wz = 0.0f;
        clearTargetMoveState();
        clearWheelSpeedOverrideState();
        clearHeadingHoldStateLocked();
        g_bench_runtime.motion_or_override_expires_ms = now_ms + duration_ms;
        g_bench_runtime.manual_override_auto_disable = true;
        for (std::size_t index = 0; index < kWheelCount; ++index) {
            g_bench_runtime.manual_pwm[index] = 0.0f;
        }
        g_bench_runtime.manual_pwm[wheel] = pwm;
    });

    setCommandVelocity(0.0f, 0.0f, 0.0f, now_ms);
    setRobotEnabled(true);
}

void scheduleAllWheelOverride(float pwm, std::uint32_t duration_ms, std::uint32_t now_ms) {
    withBenchLock([=]() {
        g_bench_runtime.manual_override_active = true;
        g_bench_runtime.timed_motion_active = false;
        g_bench_runtime.timed_vx = 0.0f;
        g_bench_runtime.timed_vy = 0.0f;
        g_bench_runtime.timed_wz = 0.0f;
        clearTargetMoveState();
        clearWheelSpeedOverrideState();
        clearHeadingHoldStateLocked();
        g_bench_runtime.motion_or_override_expires_ms = now_ms + duration_ms;
        g_bench_runtime.manual_override_auto_disable = true;
        for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
            g_bench_runtime.manual_pwm[wheel] = pwm;
        }
    });

    setCommandVelocity(0.0f, 0.0f, 0.0f, now_ms);
    setRobotEnabled(true);
}

void scheduleWheelSpeedOverride(const float target_rad_s[kWheelCount],
                                std::uint8_t wheel_mask,
                                float target_rpm,
                                std::uint32_t duration_ms,
                                std::uint32_t now_ms) {
    withBenchLock([=]() {
        g_bench_runtime.wheel_speed_override_active = true;
        g_bench_runtime.wheel_speed_override_auto_disable = true;
        g_bench_runtime.wheel_speed_override_mask = wheel_mask;
        g_bench_runtime.wheel_speed_target_rpm = target_rpm;
        g_bench_runtime.wheel_speed_override_expires_ms = now_ms + duration_ms;
        for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
            g_bench_runtime.wheel_speed_target_rad_s[wheel] = target_rad_s[wheel];
        }
        g_bench_runtime.timed_motion_active = false;
        g_bench_runtime.timed_vx = 0.0f;
        g_bench_runtime.timed_vy = 0.0f;
        g_bench_runtime.timed_wz = 0.0f;
        clearTargetMoveState();
        clearManualPwmState();
        clearHeadingHoldStateLocked();
    });

    setCommandVelocity(0.0f, 0.0f, 0.0f, now_ms);
    setRobotEnabled(true);
}

void setHeadingHoldEnabled(bool enabled, std::uint32_t now_ms) {
    if (!enabled) {
        withBenchLock([]() {
            clearHeadingHoldStateLocked();
        });
        applyTargetPidGainsIfNeeded(true);
        setCommandVelocity(0.0f, 0.0f, 0.0f, now_ms);
        setRobotEnabled(false);
        stopAllMotors();
        publishHeadingControl(HeadingControlPhase::kIdle, false, true, 0.0f, 0.0f, getImuState().gyro_z, 0.0f, 0.0f);
        return;
    }

    const OdometryState odometry = getOdometryState();
    const float heading_target = currentHeadingForTargetMove(odometry);
    applyTargetPidGainsIfNeeded(true);

    withBenchLock([=]() {
        g_bench_runtime.heading_hold_active = true;
        g_bench_runtime.heading_hold_target = heading_target;
        g_bench_runtime.heading_hold_stable_ms = 0.0f;
        g_bench_runtime.timed_motion_active = false;
        g_bench_runtime.timed_vx = 0.0f;
        g_bench_runtime.timed_vy = 0.0f;
        g_bench_runtime.timed_wz = 0.0f;
        clearTargetMoveState();
        clearManualPwmState();
        clearWheelSpeedOverrideState();
    });

    setCommandVelocity(0.0f, 0.0f, 0.0f, now_ms);
    setRobotEnabled(true);
    publishHeadingControl(HeadingControlPhase::kSettled, true, true, heading_target, 0.0f, getImuState().gyro_z, 0.0f, 0.0f);
}

void clearManualOverride() {
    withBenchLock([]() {
        clearManualPwmState();
    });
}

void emergencyStopBench(std::uint32_t now_ms) {
    withBenchLock([]() {
        g_bench_runtime.timed_motion_active = false;
        g_bench_runtime.timed_vx = 0.0f;
        g_bench_runtime.timed_vy = 0.0f;
        g_bench_runtime.timed_wz = 0.0f;
        clearTargetMoveState();
        clearManualPwmState();
        clearWheelSpeedOverrideState();
        clearHeadingHoldStateLocked();
    });

    setCommandVelocity(0.0f, 0.0f, 0.0f, now_ms);
    setRobotEnabled(false);
    stopAllMotors();
}

void setBenchImuStreamEnabled(bool enabled) {
    withBenchLock([=]() { g_bench_runtime.imu_stream_enabled = enabled; });
}

void setBenchEncoderStreamEnabled(bool enabled) {
    withBenchLock([=]() { g_bench_runtime.encoder_stream_enabled = enabled; });
}

void setBenchRtosStreamEnabled(bool enabled) {
    withBenchLock([=]() { g_bench_runtime.rtos_stream_enabled = enabled; });
}

BenchRuntimeSnapshot getBenchRuntimeSnapshot() {
    BenchRuntimeSnapshot snapshot;
    withBenchLock([&]() { snapshot = g_bench_runtime; });
    return snapshot;
}

bool getManualOverrideSnapshot(float pwm_out[kWheelCount]) {
    bool active = false;
    withBenchLock([&]() {
        active = g_bench_runtime.manual_override_active;
        for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
            pwm_out[wheel] = g_bench_runtime.manual_pwm[wheel];
        }
    });
    return active;
}

bool getWheelSpeedOverrideSnapshot(float target_rad_s_out[kWheelCount]) {
    bool active = false;
    withBenchLock([&]() {
        active = g_bench_runtime.wheel_speed_override_active;
        for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
            target_rad_s_out[wheel] = g_bench_runtime.wheel_speed_target_rad_s[wheel];
        }
    });
    return active;
}

void markTaskHeartbeat(TaskHeartbeat task) {
    withBenchLock([=]() {
        if (task < kTaskHeartbeatCount) {
            ++g_heartbeat_counts.counts[task];
        }
    });
}

TaskHeartbeatCounts consumeHeartbeatCounts() {
    TaskHeartbeatCounts snapshot;
    withBenchLock([&]() {
        snapshot = g_heartbeat_counts;
        g_heartbeat_counts = TaskHeartbeatCounts{};
    });
    return snapshot;
}

}  // namespace app
