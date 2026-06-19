#include <algorithm>

#include "pid.hpp"

namespace app {

void PidController::setGains(float kp, float ki, float kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void PidController::setOutputLimits(float output_min, float output_max) {
    output_min_ = output_min;
    output_max_ = output_max;
}

void PidController::setIntegralLimits(float integral_min, float integral_max) {
    integral_min_ = integral_min;
    integral_max_ = integral_max;
}

void PidController::reset() {
    integral_ = 0.0f;
    previous_error_ = 0.0f;
    has_previous_error_ = false;
}

float PidController::update(float target, float measured, float dt_seconds, bool enable_integral) {
    const float error = target - measured;
    float derivative = 0.0f;
    if (has_previous_error_ && dt_seconds > 0.0f) {
        derivative = (error - previous_error_) / dt_seconds;
    }

    const float proportional = kp_ * error;
    const float derivative_term = kd_ * derivative;

    float candidate_integral = integral_;
    if (enable_integral && dt_seconds > 0.0f) {
        candidate_integral += error * dt_seconds;
        candidate_integral = std::clamp(candidate_integral, integral_min_, integral_max_);
    }

    const float unclamped_output = proportional + ki_ * candidate_integral + derivative_term;
    const float clamped_output = std::clamp(unclamped_output, output_min_, output_max_);

    if (enable_integral) {
        const bool not_saturated = (unclamped_output == clamped_output);
        const bool relieving_high_saturation = (unclamped_output > output_max_) && (error < 0.0f);
        const bool relieving_low_saturation = (unclamped_output < output_min_) && (error > 0.0f);
        if (not_saturated || relieving_high_saturation || relieving_low_saturation) {
            integral_ = candidate_integral;
        }
    }

    previous_error_ = error;
    has_previous_error_ = true;

    return std::clamp(proportional + ki_ * integral_ + derivative_term, output_min_, output_max_);
}

}  // namespace app
