#include "odometry.hpp"

#include <algorithm>
#include <cmath>

#include "config.hpp"
#include "mecanum.hpp"

namespace app {

namespace {

OdometryTracker g_odometry_tracker;

}  // namespace

OdometryTracker& odometryTracker() {
    return g_odometry_tracker;
}

void OdometryTracker::reset() {
    initialized_ = false;
    for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
        previous_counts_[wheel] = 0;
    }
    state_ = OdometryState{};
}

void OdometryTracker::update(const std::int32_t encoder_counts[kWheelCount],
                             const IMUState& imu_state,
                             float dt_seconds,
                             float measured_w_rad_s_out[kWheelCount],
                             OdometryState& odometry_state_out) {
    if (!initialized_) {
        for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
            previous_counts_[wheel] = encoder_counts[wheel];
            measured_w_rad_s_out[wheel] = 0.0f;
        }
        state_ = OdometryState{};
        state_.theta = imu_state.orientation_z;
        initialized_ = true;
        odometry_state_out = state_;
        return;
    }

    float wheel_delta_rad[kWheelCount] = {0.0f, 0.0f, 0.0f, 0.0f};
    for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
        const std::int32_t delta_counts = encoder_counts[wheel] - previous_counts_[wheel];
        previous_counts_[wheel] = encoder_counts[wheel];
        wheel_delta_rad[wheel] =
            encoderCountToRad(delta_counts) * static_cast<float>(MOTOR_POLARITY[wheel]);
        measured_w_rad_s_out[wheel] = (dt_seconds > 0.0f)
            ? wheel_delta_rad[wheel] / dt_seconds
            : 0.0f;
    }

    const ChassisVelocity chassis_velocity = forwardKinematics(measured_w_rad_s_out);
    const float previous_theta = state_.theta;
    const float encoder_theta = wrapAngleRad(previous_theta + chassis_velocity.wz * dt_seconds);
    const float imu_theta_error = wrapAngleRad(imu_state.orientation_z - encoder_theta);
    const float bounded_error = std::clamp(imu_theta_error, -IMU_YAW_BLEND_LIMIT_RAD, IMU_YAW_BLEND_LIMIT_RAD);
    const float blended_theta = wrapAngleRad(encoder_theta + IMU_YAW_BLEND_ALPHA * bounded_error);
    const float heading_for_integration = wrapAngleRad(previous_theta + 0.5f * wrapAngleRad(blended_theta - previous_theta));

    state_.x += (chassis_velocity.vx * std::cos(heading_for_integration) -
                 chassis_velocity.vy * std::sin(heading_for_integration)) * dt_seconds;
    state_.y += (chassis_velocity.vx * std::sin(heading_for_integration) +
                 chassis_velocity.vy * std::cos(heading_for_integration)) * dt_seconds;
    state_.theta = blended_theta;
    state_.vx = chassis_velocity.vx;
    state_.vy = chassis_velocity.vy;
    state_.wtheta = chassis_velocity.wz;

    odometry_state_out = state_;
}

const OdometryState& OdometryTracker::state() const {
    return state_;
}

}  // namespace app
