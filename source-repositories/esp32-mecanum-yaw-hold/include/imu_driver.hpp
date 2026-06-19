#pragma once

#include <cstddef>
#include <cstdint>

#include "shared_state.hpp"

namespace app {

class ImuDriver {
public:
    bool begin();
    bool read(IMUState& imu_state);
    bool isHealthy() const;
    bool hasDetectedIdentity() const;
    std::uint8_t detectedWhoAmI() const;
    const char* detectedChipName() const;
    bool calibrateGyroBias();
    void zeroYaw();

private:
    bool writeRegister(uint8_t reg, uint8_t value);
    bool readRegisters(uint8_t start_reg, uint8_t* buffer, std::size_t length);
    bool initialized_ = false;
    bool healthy_ = false;
    bool has_detected_identity_ = false;
    std::uint8_t detected_who_am_i_ = 0U;
    float yaw_rad_ = 0.0f;
    float gyro_bias_x_rad_s_ = 0.0f;
    float gyro_bias_y_rad_s_ = 0.0f;
    float gyro_bias_z_rad_s_ = 0.0f;
    uint32_t last_update_us_ = 0U;
};

ImuDriver& imuDriver();

}  // namespace app
