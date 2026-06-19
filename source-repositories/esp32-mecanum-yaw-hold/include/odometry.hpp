#pragma once

#include <cstdint>

#include "shared_state.hpp"

namespace app {

class OdometryTracker {
public:
    void reset();
    void update(const int32_t encoder_counts[kWheelCount],
                const IMUState& imu_state,
                float dt_seconds,
                float measured_w_rad_s_out[kWheelCount],
                OdometryState& odometry_state_out);
    const OdometryState& state() const;

private:
    bool initialized_ = false;
    int32_t previous_counts_[kWheelCount] = {};
    OdometryState state_ = {};
};

OdometryTracker& odometryTracker();

}  // namespace app
