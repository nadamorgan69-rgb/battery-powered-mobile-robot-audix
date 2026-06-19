#include <Arduino.h>
#include <freertos/FreeRTOS.h>

#include "bench_runtime.hpp"
#include "config.hpp"
#include "encoder_isr.hpp"
#include "imu_driver.hpp"
#include "odometry.hpp"
#include "shared_state.hpp"

namespace app {

void sensorUpdateTask(void*) {
    TickType_t last_wake_time = xTaskGetTickCount();
    const float dt_seconds = static_cast<float>(SENSOR_PERIOD_MS) / 1000.0f;

    for (;;) {
        markTaskHeartbeat(kSensorTaskHeartbeat);
        IMUState imu_state = getImuState();
        imuDriver().read(imu_state);
        setImuState(imu_state);

        const bool limit_switch_pressed = LIMIT_SWITCH_ACTIVE_LOW
            ? (digitalRead(LIMIT_SWITCH_PIN) == LOW)
            : (digitalRead(LIMIT_SWITCH_PIN) == HIGH);
        setLimitSwitchPressed(limit_switch_pressed);

        std::int32_t encoder_counts[kWheelCount] = {0, 0, 0, 0};
        readEncoderCounts(encoder_counts);

        float measured_wheel_rad_s[kWheelCount] = {0.0f, 0.0f, 0.0f, 0.0f};
        OdometryState odometry_state;
        odometryTracker().update(
            encoder_counts,
            imu_state,
            dt_seconds,
            measured_wheel_rad_s,
            odometry_state);

        setMeasuredWheelState(measured_wheel_rad_s, encoder_counts);
        setOdometryState(odometry_state);

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(SENSOR_PERIOD_MS));
    }
}

}  // namespace app
