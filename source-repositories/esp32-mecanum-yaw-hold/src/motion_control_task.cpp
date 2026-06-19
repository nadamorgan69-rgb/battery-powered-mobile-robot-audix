#include <algorithm>
#include <cmath>

#include <Arduino.h>
#include <freertos/FreeRTOS.h>

#include "bench_runtime.hpp"
#include "config.hpp"
#include "mecanum.hpp"
#include "motor_driver.hpp"
#include "pid.hpp"
#include "safety.hpp"
#include "shared_state.hpp"

namespace app {

void motionControlTask(void*) {
    PidController wheel_pid[kWheelCount];
    std::uint32_t wheel_pid_version = 0U;
    for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
        wheel_pid[wheel].setGains(PID_KP, PID_KI, PID_KD);
        wheel_pid[wheel].setOutputLimits(-PID_OUTPUT_MAX, PID_OUTPUT_MAX);
        wheel_pid[wheel].setIntegralLimits(-PID_INTEGRAL_MAX, PID_INTEGRAL_MAX);
        wheel_pid[wheel].reset();
    }

    MotionSafety motion_safety;
    bool previous_motion_allowed = false;
    bool previous_manual_override = false;
    bool previous_speed_override = false;
    TickType_t last_wake_time = xTaskGetTickCount();
    const float dt_seconds = static_cast<float>(CONTROL_PERIOD_MS) / 1000.0f;

    for (;;) {
        markTaskHeartbeat(kMotionTaskHeartbeat);
        const std::uint32_t now_ms = millis();
        serviceBenchRuntime(now_ms);

        const PidGains wheel_gains = getPidGains(PidGainGroup::kWheel);
        if (wheel_gains.version != wheel_pid_version) {
            for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
                wheel_pid[wheel].setGains(wheel_gains.kp, wheel_gains.ki, wheel_gains.kd);
                wheel_pid[wheel].reset();
            }
            wheel_pid_version = wheel_gains.version;
        }

        const CommandState command_state = getCommandState();
        const WheelState wheel_state = getWheelState();
        float manual_override_pwm[kWheelCount] = {0.0f, 0.0f, 0.0f, 0.0f};
        const bool manual_override_active =
            getManualOverrideSnapshot(manual_override_pwm) && command_state.robot_enabled;
        float speed_override_target[kWheelCount] = {0.0f, 0.0f, 0.0f, 0.0f};
        const bool speed_override_active =
            getWheelSpeedOverrideSnapshot(speed_override_target) && command_state.robot_enabled;

        if (manual_override_active) {
            if (!previous_manual_override) {
                for (auto& controller : wheel_pid) {
                    controller.reset();
                }
            }

            const float zero_targets[kWheelCount] = {0.0f, 0.0f, 0.0f, 0.0f};
            setWheelTargets(zero_targets);
            writeMotorOutputs(manual_override_pwm);
            setPwmOutputs(manual_override_pwm);

            previous_manual_override = true;
            previous_speed_override = false;
            previous_motion_allowed = false;
            vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
            continue;
        }

        if (previous_manual_override) {
            for (auto& controller : wheel_pid) {
                controller.reset();
            }
            stopAllMotors();
            const float zero_pwm[kWheelCount] = {0.0f, 0.0f, 0.0f, 0.0f};
            setPwmOutputs(zero_pwm);
            previous_manual_override = false;
        }

        if (speed_override_active) {
            if (!previous_speed_override) {
                for (auto& controller : wheel_pid) {
                    controller.reset();
                }
            }

            setWheelTargets(speed_override_target);

            float pwm_output[kWheelCount] = {0.0f, 0.0f, 0.0f, 0.0f};
            for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
                pwm_output[wheel] = wheel_pid[wheel].update(
                    speed_override_target[wheel],
                    wheel_state.measured_w_rad_s[wheel],
                    dt_seconds,
                    true);
                pwm_output[wheel] = std::clamp(pwm_output[wheel], -PID_OUTPUT_MAX, PID_OUTPUT_MAX);
            }

            writeMotorOutputs(pwm_output);
            setPwmOutputs(pwm_output);

            previous_speed_override = true;
            previous_motion_allowed = false;
            vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
            continue;
        }

        if (previous_speed_override) {
            for (auto& controller : wheel_pid) {
                controller.reset();
            }
            stopAllMotors();
            const float zero_pwm[kWheelCount] = {0.0f, 0.0f, 0.0f, 0.0f};
            setPwmOutputs(zero_pwm);
            previous_speed_override = false;
        }

        const bool motion_allowed = motion_safety.motionAllowed(command_state, now_ms);

        if (!motion_allowed) {
            for (auto& controller : wheel_pid) {
                controller.reset();
            }
            const float zero_targets[kWheelCount] = {0.0f, 0.0f, 0.0f, 0.0f};
            const float zero_pwm[kWheelCount] = {0.0f, 0.0f, 0.0f, 0.0f};
            setWheelTargets(zero_targets);
            stopAllMotors();
            setPwmOutputs(zero_pwm);
            previous_motion_allowed = false;
            vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
            continue;
        }

        if (motion_allowed && !previous_motion_allowed) {
            for (auto& controller : wheel_pid) {
                controller.reset();
            }
        }

        ChassisVelocity chassis_command;
        chassis_command.vx = motion_allowed ? command_state.cmd_vx : 0.0f;
        chassis_command.vy = motion_allowed ? command_state.cmd_vy : 0.0f;
        chassis_command.wz = motion_allowed ? command_state.cmd_wz : 0.0f;

        float target_wheel_rad_s[kWheelCount] = {0.0f, 0.0f, 0.0f, 0.0f};
        inverseKinematics(chassis_command, target_wheel_rad_s);
        setWheelTargets(target_wheel_rad_s);

        float pwm_output[kWheelCount] = {0.0f, 0.0f, 0.0f, 0.0f};
        for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
            if (std::fabs(target_wheel_rad_s[wheel]) < WHEEL_TARGET_ZERO_EPSILON_RAD_S &&
                std::fabs(wheel_state.measured_w_rad_s[wheel]) < WHEEL_STOP_EPSILON_RAD_S) {
                wheel_pid[wheel].reset();
                pwm_output[wheel] = 0.0f;
                continue;
            }

            pwm_output[wheel] = wheel_pid[wheel].update(
                target_wheel_rad_s[wheel],
                wheel_state.measured_w_rad_s[wheel],
                dt_seconds,
                motion_allowed);

            if (!motion_allowed && std::fabs(wheel_state.measured_w_rad_s[wheel]) < WHEEL_STOP_EPSILON_RAD_S) {
                wheel_pid[wheel].reset();
                pwm_output[wheel] = 0.0f;
            }

            pwm_output[wheel] = std::clamp(pwm_output[wheel], -PID_OUTPUT_MAX, PID_OUTPUT_MAX);
        }

        writeMotorOutputs(pwm_output);
        setPwmOutputs(pwm_output);

        previous_motion_allowed = motion_allowed;
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
}

}  // namespace app
