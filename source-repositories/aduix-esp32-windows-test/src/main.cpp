#include <Arduino.h>
#include <cstdint>

#include "bench_runtime.hpp"
#include "console_sink.hpp"
#include "config.hpp"
#include "encoder_isr.hpp"
#include "imu_driver.hpp"
#include "motor_driver.hpp"
#include "odometry.hpp"
#include "shared_state.hpp"
#include "stepper_bench.hpp"
#include "wifi_console.hpp"

namespace {

void printModeLine() {
    app::consolePrintf(
        "[mode] transport=%s subsystem=%s\n",
        app::firmwareTransport(),
        app::firmwareSubsystem());
}

void printBootHint() {
#if defined(APP_WIFI_DASHBOARD)
    app::consolePrintln("Wi-Fi dashboard mode active. Use the browser GUI for normal bench control.");
    app::consolePrintln("USB serial remains available for boot diagnostics only.");
#elif defined(APP_SERIAL_PID_BENCH)
    app::consolePrintln("Serial PID bench active. Type 'help' for PID/base commands.");
#elif defined(APP_SERIAL_STEPPER_BENCH)
    app::consolePrintln("Serial stepper bench active. Type 'help' for stepper, homing, and switch commands.");
#elif defined(APP_SERIAL_IMU_ENCODER_BENCH)
    app::consolePrintln("Serial IMU/encoder bench active. Type 'help' for feedback commands.");
#else
    app::consolePrintln("Unknown bench mode. Check PlatformIO environment flags.");
#endif
}

void printImuResult(bool imu_ready) {
#if defined(APP_BASE_BENCH) || defined(APP_IMU_ENCODER_BENCH)
    if (imu_ready) {
        app::consolePrintf(
            "IMU init OK: %s WHO_AM_I=0x%02X on I2C SDA=%d SCL=%d addr=0x%02X\n",
            app::imuDriver().detectedChipName(),
            app::imuDriver().detectedWhoAmI(),
            app::IMU_SDA_PIN,
            app::IMU_SCL_PIN,
            app::MPU6050_I2C_ADDRESS);
    } else {
        if (app::imuDriver().hasDetectedIdentity()) {
            app::consolePrintf(
                "IMU init FAILED: detected %s WHO_AM_I=0x%02X on I2C SDA=%d SCL=%d addr=0x%02X.\n",
                app::imuDriver().detectedChipName(),
                app::imuDriver().detectedWhoAmI(),
                app::IMU_SDA_PIN,
                app::IMU_SCL_PIN,
                app::MPU6050_I2C_ADDRESS);
        } else {
            app::consolePrintf(
                "IMU init FAILED: no WHO_AM_I response on I2C SDA=%d SCL=%d addr=0x%02X. Check power, wiring, and AD0/address.\n",
                app::IMU_SDA_PIN,
                app::IMU_SCL_PIN,
                app::MPU6050_I2C_ADDRESS);
        }
    }
#else
    (void)imu_ready;
#endif
}

}  // namespace

void setup() {
    Serial.begin(app::SERIAL_BAUD);
    const std::uint32_t serial_wait_start = millis();
    while (!Serial && (millis() - serial_wait_start) < 2000U) {
        delay(10);
    }

    app::initConsoleSink();
    app::initSharedState();
    app::initBenchRuntime();

#if defined(APP_BASE_BENCH) || defined(APP_IMU_ENCODER_BENCH)
    pinMode(app::LIMIT_SWITCH_PIN, app::LIMIT_SWITCH_ACTIVE_LOW ? INPUT_PULLUP : INPUT);
    app::initEncoders();
    app::resetEncoders();
    app::odometryTracker().reset();
    const bool imu_ready = app::imuDriver().begin();
#if defined(APP_BASE_BENCH)
    app::initMotorDriver();
#endif
#elif defined(APP_STEPPER_BENCH)
    app::initStepperBench();
    const bool imu_ready = false;
#else
    const bool imu_ready = false;
#endif

    app::consolePrintln();
    app::consolePrintln("Audix standalone ESP32 bench firmware ready.");
    printModeLine();
    printBootHint();
    printImuResult(imu_ready);
#if defined(APP_STEPPER_BENCH)
    app::consolePrintln("Stepper bench mode active. IMU, encoder, and base-motion hardware paths are intentionally skipped.");
#endif

#if defined(APP_WIFI_DASHBOARD)
    app::initWifiConsole();
#else
    app::consolePrintln("[wifi] disabled in this serial-only firmware environment.");
#endif

    xTaskCreatePinnedToCore(
        app::commandRxTask,
        "command_rx",
        app::COMMAND_TASK_STACK_BYTES,
        nullptr,
        app::COMMAND_TASK_PRIORITY,
        nullptr,
        0);

#if defined(APP_STEPPER_BENCH)
    xTaskCreatePinnedToCore(
        app::stepperBenchTask,
        "stepper_bench",
        app::STEPPER_TASK_STACK_BYTES,
        nullptr,
        app::STEPPER_TASK_PRIORITY,
        nullptr,
        1);
#elif defined(APP_BASE_BENCH)
    xTaskCreatePinnedToCore(
        app::motionControlTask,
        "motion_control",
        app::MOTION_TASK_STACK_BYTES,
        nullptr,
        app::MOTION_TASK_PRIORITY,
        nullptr,
        1);
#endif

#if defined(APP_BASE_BENCH) || defined(APP_IMU_ENCODER_BENCH)
    xTaskCreatePinnedToCore(
        app::sensorUpdateTask,
        "sensor_update",
        app::SENSOR_TASK_STACK_BYTES,
        nullptr,
        app::SENSOR_TASK_PRIORITY,
        nullptr,
        0);
#endif

    xTaskCreatePinnedToCore(
        app::telemetryTask,
        "telemetry",
        app::TELEMETRY_TASK_STACK_BYTES,
        nullptr,
        app::TELEMETRY_TASK_PRIORITY,
        nullptr,
        0);
}

void loop() {
#if defined(APP_WIFI_DASHBOARD)
    app::serviceWifiConsole();
#endif
    vTaskDelay(pdMS_TO_TICKS(5));
}
