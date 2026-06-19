#include <Arduino.h>
#include <freertos/FreeRTOS.h>

#include "bench_runtime.hpp"
#include "console_sink.hpp"
#include "config.hpp"
#include "shared_state.hpp"

namespace app {

void telemetryTask(void*) {
    TickType_t last_wake_time = xTaskGetTickCount();
    std::uint32_t last_rtos_report_ms = millis();
    std::uint32_t last_encoder_report_ms = millis();

    for (;;) {
        markTaskHeartbeat(kTelemetryTaskHeartbeat);
        const std::uint32_t now_ms = millis();

        const BenchRuntimeSnapshot bench = getBenchRuntimeSnapshot();
        if (bench.imu_stream_enabled) {
            const IMUState imu = getImuState();
            consolePrintf(
                "[imu] yaw=%.4f gyro=(%.4f, %.4f, %.4f) accel=(%.4f, %.4f, %.4f)\n",
                imu.orientation_z,
                imu.gyro_x,
                imu.gyro_y,
                imu.gyro_z,
                imu.accel_x,
                imu.accel_y,
                imu.accel_z);
        }

        if (bench.encoder_stream_enabled && (now_ms - last_encoder_report_ms) >= ENCODER_TELEMETRY_PERIOD_MS) {
            last_encoder_report_ms = now_ms;
            const WheelState wheel_state = getWheelState();
            const OdometryState odometry = getOdometryState();
            consolePrintf(
                "[enc] fl=(count=%ld w=%.3f pwm=%.3f) fr=(count=%ld w=%.3f pwm=%.3f) rl=(count=%ld w=%.3f pwm=%.3f) rr=(count=%ld w=%.3f pwm=%.3f) odom=(vx=%.3f vy=%.3f wz=%.3f)\n",
                static_cast<long>(wheel_state.encoder_counts[kFrontLeft]),
                wheel_state.measured_w_rad_s[kFrontLeft],
                wheel_state.pwm_output[kFrontLeft],
                static_cast<long>(wheel_state.encoder_counts[kFrontRight]),
                wheel_state.measured_w_rad_s[kFrontRight],
                wheel_state.pwm_output[kFrontRight],
                static_cast<long>(wheel_state.encoder_counts[kRearLeft]),
                wheel_state.measured_w_rad_s[kRearLeft],
                wheel_state.pwm_output[kRearLeft],
                static_cast<long>(wheel_state.encoder_counts[kRearRight]),
                wheel_state.measured_w_rad_s[kRearRight],
                wheel_state.pwm_output[kRearRight],
                odometry.vx,
                odometry.vy,
                odometry.wtheta);
        } else if (!bench.encoder_stream_enabled) {
            last_encoder_report_ms = now_ms;
        }

        if (bench.rtos_stream_enabled && (now_ms - last_rtos_report_ms) >= 1000U) {
            const TaskHeartbeatCounts heartbeats = consumeHeartbeatCounts();
            const float window_seconds = static_cast<float>(now_ms - last_rtos_report_ms) / 1000.0f;
            last_rtos_report_ms = now_ms;
            if (window_seconds > 0.0f) {
                consolePrintf(
                    "[rtos] cmd=%.1fHz motion=%.1fHz sensor=%.1fHz telemetry=%.1fHz stepper=%.1fHz\n",
                    static_cast<float>(heartbeats.counts[kCommandTaskHeartbeat]) / window_seconds,
                    static_cast<float>(heartbeats.counts[kMotionTaskHeartbeat]) / window_seconds,
                    static_cast<float>(heartbeats.counts[kSensorTaskHeartbeat]) / window_seconds,
                    static_cast<float>(heartbeats.counts[kTelemetryTaskHeartbeat]) / window_seconds,
                    static_cast<float>(heartbeats.counts[kStepperTaskHeartbeat]) / window_seconds);
            }
        } else if (!bench.rtos_stream_enabled) {
            last_rtos_report_ms = now_ms;
            (void)consumeHeartbeatCounts();
        }

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(TELEMETRY_PERIOD_MS));
    }
}

}  // namespace app
