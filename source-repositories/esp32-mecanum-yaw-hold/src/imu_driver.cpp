#include "imu_driver.hpp"

#include <Arduino.h>
#include <Wire.h>

#include "config.hpp"

namespace app {

namespace {

constexpr std::uint8_t kWhoAmIReg = 0x75U;
constexpr std::uint8_t kPwrMgmt1Reg = 0x6BU;
constexpr std::uint8_t kConfigReg = 0x1AU;
constexpr std::uint8_t kGyroConfigReg = 0x1BU;
constexpr std::uint8_t kAccelConfigReg = 0x1CU;
constexpr std::uint8_t kAccelXoutHReg = 0x3BU;

constexpr std::uint8_t kMpu6050WhoAmI = 0x68U;
constexpr std::uint8_t kMpu6500WhoAmI = 0x70U;
constexpr std::uint8_t kMpu9250WhoAmI = 0x71U;
constexpr std::uint8_t kMpu9255WhoAmI = 0x73U;
constexpr float kAccelLsbPerG = 8192.0f;   // +/-4 g
constexpr float kGyroLsbPerDegPerSec = 65.5f;  // +/-500 dps
constexpr float kDegToRad = kPi / 180.0f;

ImuDriver g_imu_driver;

std::int16_t combineBigEndian(std::uint8_t msb, std::uint8_t lsb) {
    return static_cast<std::int16_t>((static_cast<std::uint16_t>(msb) << 8U) | lsb);
}

const char* chipNameForWhoAmI(std::uint8_t who_am_i) {
    switch (who_am_i) {
        case kMpu6050WhoAmI:
            return "MPU6050";
        case kMpu6500WhoAmI:
            return "MPU6500";
        case kMpu9250WhoAmI:
            return "MPU9250";
        case kMpu9255WhoAmI:
            return "MPU9255";
        default:
            return "Unknown IMU";
    }
}

bool isSupportedWhoAmI(std::uint8_t who_am_i) {
    switch (who_am_i) {
        case kMpu6050WhoAmI:
        case kMpu6500WhoAmI:
        case kMpu9250WhoAmI:
        case kMpu9255WhoAmI:
            return true;
        default:
            return false;
    }
}

}  // namespace

ImuDriver& imuDriver() {
    return g_imu_driver;
}

bool ImuDriver::begin() {
    Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN, I2C_FREQUENCY_HZ);
    delay(50);

    std::uint8_t who_am_i = 0U;
    has_detected_identity_ = readRegisters(kWhoAmIReg, &who_am_i, 1U);
    detected_who_am_i_ = has_detected_identity_ ? who_am_i : 0U;
    healthy_ = has_detected_identity_ && isSupportedWhoAmI(who_am_i);
    if (!healthy_) {
        initialized_ = false;
        return false;
    }

    healthy_ = writeRegister(kPwrMgmt1Reg, 0x00U)
        && writeRegister(kConfigReg, 0x03U)
        && writeRegister(kGyroConfigReg, 0x08U)
        && writeRegister(kAccelConfigReg, 0x08U);

    if (!healthy_) {
        initialized_ = false;
        return false;
    }

    yaw_rad_ = 0.0f;
    gyro_bias_x_rad_s_ = 0.0f;
    gyro_bias_y_rad_s_ = 0.0f;
    gyro_bias_z_rad_s_ = 0.0f;
    last_update_us_ = micros();
    initialized_ = true;

    calibrateGyroBias();
    return healthy_;
}

bool ImuDriver::read(IMUState& imu_state) {
    if (!initialized_) {
        return false;
    }

    std::uint8_t buffer[14] = {0};
    healthy_ = readRegisters(kAccelXoutHReg, buffer, sizeof(buffer));
    if (!healthy_) {
        return false;
    }

    const std::int16_t raw_ax = combineBigEndian(buffer[0], buffer[1]);
    const std::int16_t raw_ay = combineBigEndian(buffer[2], buffer[3]);
    const std::int16_t raw_az = combineBigEndian(buffer[4], buffer[5]);
    const std::int16_t raw_gx = combineBigEndian(buffer[8], buffer[9]);
    const std::int16_t raw_gy = combineBigEndian(buffer[10], buffer[11]);
    const std::int16_t raw_gz = combineBigEndian(buffer[12], buffer[13]);

    const float accel_scale = kGravityMps2 / kAccelLsbPerG;
    const float gyro_scale = kDegToRad / kGyroLsbPerDegPerSec;

    imu_state.accel_x = static_cast<float>(raw_ax) * accel_scale;
    imu_state.accel_y = static_cast<float>(raw_ay) * accel_scale;
    imu_state.accel_z = static_cast<float>(raw_az) * accel_scale;
    imu_state.gyro_x = static_cast<float>(raw_gx) * gyro_scale - gyro_bias_x_rad_s_;
    imu_state.gyro_y = static_cast<float>(raw_gy) * gyro_scale - gyro_bias_y_rad_s_;
    imu_state.gyro_z = static_cast<float>(raw_gz) * gyro_scale - gyro_bias_z_rad_s_;

    const std::uint32_t now_us = micros();
    const float dt_seconds = (last_update_us_ == 0U)
        ? 0.0f
        : static_cast<float>(now_us - last_update_us_) * 1.0e-6f;
    last_update_us_ = now_us;

    if (dt_seconds > 0.0f && dt_seconds < 0.25f) {
        yaw_rad_ = wrapAngleRad(yaw_rad_ + imu_state.gyro_z * dt_seconds);
    }

    // The MPU6050 provides no absolute yaw reference here, so orientation_z is
    // gyro-integrated yaw and will drift over time until the Pi-side EKF fuses
    // it with other sources.
    imu_state.orientation_z = yaw_rad_;
    return true;
}

bool ImuDriver::isHealthy() const {
    return healthy_;
}

bool ImuDriver::hasDetectedIdentity() const {
    return has_detected_identity_;
}

std::uint8_t ImuDriver::detectedWhoAmI() const {
    return detected_who_am_i_;
}

const char* ImuDriver::detectedChipName() const {
    if (!has_detected_identity_) {
        return "No IMU response";
    }
    return chipNameForWhoAmI(detected_who_am_i_);
}

void ImuDriver::zeroYaw() {
    yaw_rad_ = 0.0f;
    last_update_us_ = micros();
}

bool ImuDriver::writeRegister(std::uint8_t reg, std::uint8_t value) {
    Wire.beginTransmission(MPU6050_I2C_ADDRESS);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

bool ImuDriver::readRegisters(std::uint8_t start_reg, std::uint8_t* buffer, std::size_t length) {
    Wire.beginTransmission(MPU6050_I2C_ADDRESS);
    Wire.write(start_reg);
    if (Wire.endTransmission(false) != 0) {
        return false;
    }

    const std::size_t received = Wire.requestFrom(
        static_cast<int>(MPU6050_I2C_ADDRESS),
        static_cast<int>(length),
        static_cast<int>(true));
    if (received != length) {
        return false;
    }

    for (std::size_t index = 0; index < length; ++index) {
        buffer[index] = static_cast<std::uint8_t>(Wire.read());
    }
    return true;
}

bool ImuDriver::calibrateGyroBias() {
    if (!initialized_ || !healthy_) {
        return false;
    }

    float bias_x = 0.0f;
    float bias_y = 0.0f;
    float bias_z = 0.0f;
    std::uint32_t samples_collected = 0U;

    for (std::uint32_t sample = 0; sample < IMU_GYRO_BIAS_SAMPLES; ++sample) {
        std::uint8_t buffer[14] = {0};
        if (!readRegisters(kAccelXoutHReg, buffer, sizeof(buffer))) {
            continue;
        }

        const std::int16_t raw_gx = combineBigEndian(buffer[8], buffer[9]);
        const std::int16_t raw_gy = combineBigEndian(buffer[10], buffer[11]);
        const std::int16_t raw_gz = combineBigEndian(buffer[12], buffer[13]);
        const float gyro_scale = kDegToRad / kGyroLsbPerDegPerSec;

        bias_x += static_cast<float>(raw_gx) * gyro_scale;
        bias_y += static_cast<float>(raw_gy) * gyro_scale;
        bias_z += static_cast<float>(raw_gz) * gyro_scale;
        ++samples_collected;
        delay(2);
    }

    if (samples_collected == 0U) {
        return false;
    }

    const float inv_samples = 1.0f / static_cast<float>(samples_collected);
    gyro_bias_x_rad_s_ = bias_x * inv_samples;
    gyro_bias_y_rad_s_ = bias_y * inv_samples;
    gyro_bias_z_rad_s_ = bias_z * inv_samples;
    last_update_us_ = micros();
    return true;
}

}  // namespace app
