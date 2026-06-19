#pragma once

namespace app {

class PidController {
public:
    void setGains(float kp, float ki, float kd);
    void setOutputLimits(float output_min, float output_max);
    void setIntegralLimits(float integral_min, float integral_max);
    void reset();

    // enable_integral = false when motion is not allowed (anti-windup)
    float update(float target, float measured, float dt_seconds,
                 bool enable_integral = true);

private:
    float kp_ = 0.0f;
    float ki_ = 0.0f;
    float kd_ = 0.0f;
    float output_min_ = -1.0f;
    float output_max_ = 1.0f;
    float integral_min_ = -1.0f;
    float integral_max_ = 1.0f;
    float integral_ = 0.0f;
    float previous_error_ = 0.0f;
    bool has_previous_error_ = false;
};

}  // namespace app
