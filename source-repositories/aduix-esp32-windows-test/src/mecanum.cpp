#include "mecanum.hpp"

namespace app {

void inverseKinematics(const ChassisVelocity& command, float wheel_rad_s_out[kWheelCount]) {
    const float k = WHEELBASE_HALF_M + TRACK_HALF_WIDTH_M;
    const float inv_r = 1.0f / WHEEL_RADIUS_M;

    // Sign convention matches ROS base_link:
    // +vx forward, +vy left, +wz counter-clockwise, wheels ordered FL/FR/RL/RR.
    wheel_rad_s_out[kFrontLeft] = inv_r * (command.vx - command.vy - k * command.wz);
    wheel_rad_s_out[kFrontRight] = inv_r * (command.vx + command.vy + k * command.wz);
    wheel_rad_s_out[kRearLeft] = inv_r * (command.vx + command.vy - k * command.wz);
    wheel_rad_s_out[kRearRight] = inv_r * (command.vx - command.vy + k * command.wz);
}

ChassisVelocity forwardKinematics(const float wheel_rad_s[kWheelCount]) {
    const float k = WHEELBASE_HALF_M + TRACK_HALF_WIDTH_M;
    ChassisVelocity chassis_velocity;

    chassis_velocity.vx = (WHEEL_RADIUS_M / 4.0f) * (
        wheel_rad_s[kFrontLeft] +
        wheel_rad_s[kFrontRight] +
        wheel_rad_s[kRearLeft] +
        wheel_rad_s[kRearRight]);

    chassis_velocity.vy = (WHEEL_RADIUS_M / 4.0f) * (
        -wheel_rad_s[kFrontLeft] +
        wheel_rad_s[kFrontRight] +
        wheel_rad_s[kRearLeft] -
        wheel_rad_s[kRearRight]);

    chassis_velocity.wz = (WHEEL_RADIUS_M / (4.0f * k)) * (
        -wheel_rad_s[kFrontLeft] +
        wheel_rad_s[kFrontRight] -
        wheel_rad_s[kRearLeft] +
        wheel_rad_s[kRearRight]);

    return chassis_velocity;
}

}  // namespace app
