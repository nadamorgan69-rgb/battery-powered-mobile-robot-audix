#include "stepper_bench.hpp"

#include <Arduino.h>
#include <cstdint>
#include <freertos/FreeRTOS.h>

#include "bench_runtime.hpp"
#include "config.hpp"
#include "console_sink.hpp"

namespace app {

namespace {

#if defined(APP_STEPPER_BENCH)

struct StepperBenchState {
    bool available = true;
    bool driver_enabled = false;
    bool motion_active = false;
    bool homing_active = false;
    bool direction_up = false;
    bool switch_pressed = false;
    bool enable_active_low = STEPPER_ENABLE_ACTIVE_LOW;
    bool up_direction_high = STEPPER_UP_DIRECTION_HIGH;
    float active_rate_hz = 0.0f;
    std::uint32_t steps_taken = 0U;
    std::uint32_t motion_deadline_ms = 0U;
    std::uint32_t motion_started_ms = 0U;
    std::uint32_t command_token = 0U;
};

StepperBenchState g_stepper_state;
portMUX_TYPE g_stepper_mux = portMUX_INITIALIZER_UNLOCKED;

template <typename Fn>
void withStepperLock(Fn&& fn) {
    portENTER_CRITICAL(&g_stepper_mux);
    fn();
    portEXIT_CRITICAL(&g_stepper_mux);
}

bool timeReached(std::uint32_t now_value, std::uint32_t target_value) {
    return static_cast<std::uint32_t>(now_value - target_value) < 0x80000000U;
}

bool readHomeSwitchRaw() {
    const int raw_level = digitalRead(STEPPER_HOME_SWITCH_PIN);
    return STEPPER_HOME_SWITCH_ACTIVE_LOW ? (raw_level == LOW) : (raw_level == HIGH);
}

void applyEnablePin(bool enabled) {
    const uint8_t active_level = STEPPER_ENABLE_ACTIVE_LOW ? LOW : HIGH;
    const uint8_t inactive_level = STEPPER_ENABLE_ACTIVE_LOW ? HIGH : LOW;
    digitalWrite(STEPPER_ENABLE_PIN, enabled ? active_level : inactive_level);
}

void applyDirectionPin(bool direction_up) {
    const bool direction_high = direction_up ? STEPPER_UP_DIRECTION_HIGH : !STEPPER_UP_DIRECTION_HIGH;
    digitalWrite(STEPPER_DIR_PIN, direction_high ? HIGH : LOW);
}

void pulseStepPin() {
    digitalWrite(STEPPER_STEP_PIN, HIGH);
    delayMicroseconds(STEPPER_PULSE_WIDTH_US);
    digitalWrite(STEPPER_STEP_PIN, LOW);
}

StepperBenchSnapshot toSnapshot(const StepperBenchState& state) {
    StepperBenchSnapshot snapshot;
    snapshot.available = state.available;
    snapshot.driver_enabled = state.driver_enabled;
    snapshot.motion_active = state.motion_active;
    snapshot.homing_active = state.homing_active;
    snapshot.direction_up = state.direction_up;
    snapshot.switch_pressed = state.switch_pressed;
    snapshot.enable_active_low = state.enable_active_low;
    snapshot.up_direction_high = state.up_direction_high;
    snapshot.active_rate_hz = state.active_rate_hz;
    snapshot.steps_taken = state.steps_taken;
    snapshot.motion_started_ms = state.motion_started_ms;
    snapshot.motion_deadline_ms = state.motion_deadline_ms;
    return snapshot;
}

void finalizeMotion(bool disable_driver) {
    bool keep_enabled = false;
    withStepperLock([&]() {
        g_stepper_state.motion_active = false;
        g_stepper_state.homing_active = false;
        g_stepper_state.active_rate_hz = 0.0f;
        if (disable_driver) {
            g_stepper_state.driver_enabled = false;
        }
        keep_enabled = g_stepper_state.driver_enabled;
        ++g_stepper_state.command_token;
    });

    applyEnablePin(keep_enabled);
}

std::uint32_t getMotionStartedMs() {
    std::uint32_t started_ms = 0U;
    withStepperLock([&]() { started_ms = g_stepper_state.motion_started_ms; });
    return started_ms;
}

std::uint32_t getCurrentCommandToken() {
    std::uint32_t token = 0U;
    withStepperLock([&]() { token = g_stepper_state.command_token; });
    return token;
}

std::uint32_t computePeriodUs(float rate_hz) {
    if (rate_hz < 1.0f) {
        rate_hz = 1.0f;
    }

    const float period = 1000000.0f / rate_hz;
    if (period < static_cast<float>(STEPPER_PULSE_WIDTH_US * 2U)) {
        return STEPPER_PULSE_WIDTH_US * 2U;
    }
    return static_cast<std::uint32_t>(period);
}

#endif

}  // namespace

void initStepperBench() {
#if defined(APP_STEPPER_BENCH)
    pinMode(STEPPER_STEP_PIN, OUTPUT);
    digitalWrite(STEPPER_STEP_PIN, LOW);

    pinMode(STEPPER_DIR_PIN, OUTPUT);
    applyDirectionPin(true);

    pinMode(STEPPER_ENABLE_PIN, OUTPUT);
    applyEnablePin(false);

    pinMode(STEPPER_HOME_SWITCH_PIN, STEPPER_HOME_SWITCH_PULLUP ? INPUT_PULLUP : INPUT);

    const bool switch_pressed = readHomeSwitchRaw();
    withStepperLock([&]() {
        g_stepper_state = StepperBenchState{};
        g_stepper_state.switch_pressed = switch_pressed;
    });

    consolePrintf(
        "[stepper] ready step=%d dir=%d en=%d switch=%d active_low=%s enabled=off switch=%s\n",
        STEPPER_STEP_PIN,
        STEPPER_DIR_PIN,
        STEPPER_ENABLE_PIN,
        STEPPER_HOME_SWITCH_PIN,
        STEPPER_HOME_SWITCH_ACTIVE_LOW ? "yes" : "no",
        switch_pressed ? "pressed" : "released");
#endif
}

bool stepperBenchAvailable() {
#if defined(APP_STEPPER_BENCH)
    return true;
#else
    return false;
#endif
}

StepperBenchSnapshot getStepperBenchSnapshot() {
    StepperBenchSnapshot snapshot;
#if defined(APP_STEPPER_BENCH)
    withStepperLock([&]() { snapshot = toSnapshot(g_stepper_state); });
#endif
    return snapshot;
}

void printStepperBenchStatus() {
#if defined(APP_STEPPER_BENCH)
    const StepperBenchSnapshot snapshot = getStepperBenchSnapshot();
    consolePrintf(
        "[stepper] status ready=%s enabled=%s motion=%s homing=%s direction=%s switch=%s rate=%.1f steps=%lu en_active_low=%s up_high=%s\n",
        snapshot.available ? "yes" : "no",
        snapshot.driver_enabled ? "on" : "off",
        snapshot.motion_active ? "yes" : "no",
        snapshot.homing_active ? "yes" : "no",
        snapshot.direction_up ? "up" : "down",
        snapshot.switch_pressed ? "pressed" : "released",
        snapshot.active_rate_hz,
        static_cast<unsigned long>(snapshot.steps_taken),
        snapshot.enable_active_low ? "yes" : "no",
        snapshot.up_direction_high ? "yes" : "no");
#else
    consolePrintln("[stepper] unavailable in this firmware environment.");
#endif
}

bool setStepperBenchEnabled(bool enabled) {
#if defined(APP_STEPPER_BENCH)
    withStepperLock([=]() {
        if (!enabled) {
            g_stepper_state.motion_active = false;
            g_stepper_state.homing_active = false;
            g_stepper_state.active_rate_hz = 0.0f;
        }
        g_stepper_state.driver_enabled = enabled;
        ++g_stepper_state.command_token;
    });
    applyEnablePin(enabled);
    consolePrintf("[stepper] enable=%s\n", enabled ? "on" : "off");
    return true;
#else
    (void)enabled;
    consolePrintln("[stepper] unavailable in this firmware environment.");
    return false;
#endif
}

bool scheduleStepperBenchJog(bool direction_up, float rate_hz, std::uint32_t duration_ms) {
#if defined(APP_STEPPER_BENCH)
    if (rate_hz <= 0.0f || duration_ms == 0U) {
        consolePrintln("[stepper] invalid jog request.");
        return false;
    }

    const std::uint32_t now_ms = millis();
    withStepperLock([=]() {
        g_stepper_state.driver_enabled = true;
        g_stepper_state.motion_active = true;
        g_stepper_state.homing_active = false;
        g_stepper_state.direction_up = direction_up;
        g_stepper_state.active_rate_hz = rate_hz;
        g_stepper_state.steps_taken = 0U;
        g_stepper_state.motion_started_ms = now_ms;
        g_stepper_state.motion_deadline_ms = now_ms + duration_ms;
        ++g_stepper_state.command_token;
    });
    applyDirectionPin(direction_up);
    applyEnablePin(true);
    consolePrintf(
        "[stepper] jog direction=%s rate=%.1f duration=%.2f\n",
        direction_up ? "up" : "down",
        rate_hz,
        static_cast<double>(duration_ms) / 1000.0);
    return true;
#else
    (void)direction_up;
    (void)rate_hz;
    (void)duration_ms;
    consolePrintln("[stepper] unavailable in this firmware environment.");
    return false;
#endif
}

bool scheduleStepperBenchHome(float rate_hz, std::uint32_t timeout_ms) {
#if defined(APP_STEPPER_BENCH)
    if (rate_hz <= 0.0f || timeout_ms == 0U) {
        consolePrintln("[stepper] invalid home request.");
        return false;
    }

    const bool switch_pressed = readHomeSwitchRaw();
    withStepperLock([&]() { g_stepper_state.switch_pressed = switch_pressed; });
    if (switch_pressed) {
        consolePrintln("[stepper] home requested but switch is already pressed.");
        return true;
    }

    const bool direction_up = STEPPER_HOME_DIRECTION_UP;
    const std::uint32_t now_ms = millis();
    withStepperLock([=]() {
        g_stepper_state.driver_enabled = true;
        g_stepper_state.motion_active = true;
        g_stepper_state.homing_active = true;
        g_stepper_state.direction_up = direction_up;
        g_stepper_state.active_rate_hz = rate_hz;
        g_stepper_state.steps_taken = 0U;
        g_stepper_state.motion_started_ms = now_ms;
        g_stepper_state.motion_deadline_ms = now_ms + timeout_ms;
        ++g_stepper_state.command_token;
    });
    applyDirectionPin(direction_up);
    applyEnablePin(true);
    consolePrintf(
        "[stepper] homing direction=%s rate=%.1f timeout=%.2f\n",
        direction_up ? "up" : "down",
        rate_hz,
        static_cast<double>(timeout_ms) / 1000.0);
    return true;
#else
    (void)rate_hz;
    (void)timeout_ms;
    consolePrintln("[stepper] unavailable in this firmware environment.");
    return false;
#endif
}

bool stopStepperBench(bool log_stop) {
#if defined(APP_STEPPER_BENCH)
    finalizeMotion(STEPPER_AUTO_DISABLE_ON_IDLE);
    if (log_stop) {
        consolePrintln("[stepper] stop");
    }
    return true;
#else
    (void)log_stop;
    consolePrintln("[stepper] unavailable in this firmware environment.");
    return false;
#endif
}

void stepperBenchTask(void*) {
#if defined(APP_STEPPER_BENCH)
    TickType_t last_wake_time = xTaskGetTickCount();
    std::uint32_t observed_command_token = getCurrentCommandToken();
    std::uint32_t next_step_due_us = micros();
    std::uint32_t last_progress_log_ms = millis();

    for (;;) {
        markTaskHeartbeat(kStepperTaskHeartbeat);

        const std::uint32_t now_ms = millis();
        const std::uint32_t now_us = micros();
        const bool switch_pressed = readHomeSwitchRaw();

        StepperBenchSnapshot snapshot;
        bool switch_changed = false;
        withStepperLock([&]() {
            if (g_stepper_state.switch_pressed != switch_pressed) {
                g_stepper_state.switch_pressed = switch_pressed;
                switch_changed = true;
            }
            snapshot = toSnapshot(g_stepper_state);
        });

        if (switch_changed) {
            consolePrintf("[stepper] switch=%s\n", switch_pressed ? "pressed" : "released");
        }

        if (snapshot.motion_active) {
            const std::uint32_t current_token = getCurrentCommandToken();
            if (current_token != observed_command_token) {
                observed_command_token = current_token;
                next_step_due_us = now_us;
                last_progress_log_ms = now_ms;
            }

            if (snapshot.homing_active && switch_pressed) {
                const std::uint32_t started_ms = getMotionStartedMs();
                const std::uint32_t steps = snapshot.steps_taken;
                finalizeMotion(STEPPER_AUTO_DISABLE_ON_IDLE);
                consolePrintf(
                    "[stepper] homed steps=%lu elapsed=%.2f\n",
                    static_cast<unsigned long>(steps),
                    static_cast<double>(now_ms - started_ms) / 1000.0);
            } else if (timeReached(now_ms, snapshot.motion_deadline_ms)) {
                const std::uint32_t started_ms = getMotionStartedMs();
                const std::uint32_t steps = snapshot.steps_taken;
                const bool homing = snapshot.homing_active;
                finalizeMotion(STEPPER_AUTO_DISABLE_ON_IDLE);
                if (homing) {
                    consolePrintf(
                        "[stepper] home timeout steps=%lu elapsed=%.2f\n",
                        static_cast<unsigned long>(steps),
                        static_cast<double>(now_ms - started_ms) / 1000.0);
                } else {
                    consolePrintf(
                        "[stepper] finished direction=%s steps=%lu elapsed=%.2f reason=duration\n",
                        snapshot.direction_up ? "up" : "down",
                        static_cast<unsigned long>(steps),
                        static_cast<double>(now_ms - started_ms) / 1000.0);
                }
            } else {
                applyDirectionPin(snapshot.direction_up);
                applyEnablePin(true);

                if (timeReached(now_us, next_step_due_us)) {
                    pulseStepPin();
                    const std::uint32_t period_us = computePeriodUs(snapshot.active_rate_hz);
                    if (static_cast<std::uint32_t>(now_us - next_step_due_us) > (period_us * 4U)) {
                        next_step_due_us = now_us + period_us;
                    } else {
                        next_step_due_us += period_us;
                    }

                    withStepperLock([]() { ++g_stepper_state.steps_taken; });
                }

                if ((now_ms - last_progress_log_ms) >= STEPPER_LOG_INTERVAL_MS) {
                    const StepperBenchSnapshot live_snapshot = getStepperBenchSnapshot();
                    consolePrintf(
                        "[stepper] motion direction=%s mode=%s rate=%.1f steps=%lu switch=%s\n",
                        live_snapshot.direction_up ? "up" : "down",
                        live_snapshot.homing_active ? "home" : "jog",
                        live_snapshot.active_rate_hz,
                        static_cast<unsigned long>(live_snapshot.steps_taken),
                        live_snapshot.switch_pressed ? "pressed" : "released");
                    last_progress_log_ms = now_ms;
                }
            }
        }

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(1));
    }
#else
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
#endif
}

}  // namespace app
