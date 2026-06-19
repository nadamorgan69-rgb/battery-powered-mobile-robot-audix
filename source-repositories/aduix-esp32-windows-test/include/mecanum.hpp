#pragma once

#include "shared_state.hpp"
#include "config.hpp"

namespace app {

struct ChassisVelocity {
    float vx = 0.0f;
    float vy = 0.0f;
    float wz = 0.0f;
};

// Converts body velocity to wheel angular velocities (rad/s)
void inverseKinematics(const ChassisVelocity& command,
                       float wheel_rad_s_out[kWheelCount]);

// Converts wheel angular velocities to body velocity
ChassisVelocity forwardKinematics(const float wheel_rad_s[kWheelCount]);

}  // namespace app
