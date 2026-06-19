#pragma once

#include <cstdint>

namespace app {

struct StepperBenchSnapshot {
    bool available = false;
    bool driver_enabled = false;
    bool motion_active = false;
    bool homing_active = false;
    bool direction_up = false;
    bool switch_pressed = false;
    bool enable_active_low = false;
    bool up_direction_high = false;
    float active_rate_hz = 0.0f;
    std::uint32_t steps_taken = 0U;
    std::uint32_t motion_started_ms = 0U;
    std::uint32_t motion_deadline_ms = 0U;
};

void initStepperBench();
bool stepperBenchAvailable();
StepperBenchSnapshot getStepperBenchSnapshot();
void printStepperBenchStatus();
bool setStepperBenchEnabled(bool enabled);
bool scheduleStepperBenchJog(bool direction_up, float rate_hz, std::uint32_t duration_ms);
bool scheduleStepperBenchHome(float rate_hz, std::uint32_t timeout_ms);
bool stopStepperBench(bool log_stop = true);
void stepperBenchTask(void*);

}  // namespace app
