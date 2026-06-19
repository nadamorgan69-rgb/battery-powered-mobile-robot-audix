#include "motor_driver.hpp"

#include <algorithm>
#include <cmath>

#include <Arduino.h>
#include <driver/gpio.h>
#include <driver/ledc.h>

namespace app {

namespace {

constexpr ledc_channel_t kIn1LedcChannels[kWheelCount] = {
    LEDC_CHANNEL_0,
    LEDC_CHANNEL_2,
    LEDC_CHANNEL_4,
    LEDC_CHANNEL_6,
};

constexpr ledc_channel_t kIn2LedcChannels[kWheelCount] = {
    LEDC_CHANNEL_1,
    LEDC_CHANNEL_3,
    LEDC_CHANNEL_5,
    LEDC_CHANNEL_7,
};

void setChannelDuty(ledc_channel_t channel, std::uint32_t duty) {
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, channel, duty);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, channel);
}

void forcePinLow(int pin) {
    gpio_set_level(static_cast<gpio_num_t>(pin), 0);
}

void forceSingleMotorPinsLow(WheelIndex wheel) {
    setChannelDuty(kIn1LedcChannels[wheel], 0U);
    setChannelDuty(kIn2LedcChannels[wheel], 0U);
    forcePinLow(MOTOR_IN1_PINS[wheel]);
    forcePinLow(MOTOR_IN2_PINS[wheel]);
}

void configurePwmChannel(int pin, ledc_channel_t channel) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);

    ledc_channel_config_t channel_config = {};
    channel_config.gpio_num = pin;
    channel_config.speed_mode = LEDC_HIGH_SPEED_MODE;
    channel_config.channel = channel;
    channel_config.intr_type = LEDC_INTR_DISABLE;
    channel_config.timer_sel = LEDC_TIMER_0;
    channel_config.duty = 0;
    channel_config.hpoint = 0;
    ledc_channel_config(&channel_config);
}

void writeSingleMotor(WheelIndex wheel, float pwm_command) {
    const float polarity_adjusted = pwm_command * static_cast<float>(MOTOR_POLARITY[wheel]);
    const float clamped = std::clamp(polarity_adjusted, -1.0f, 1.0f);
    float magnitude = std::fabs(clamped) * PWM_MAX_F;

    if (magnitude > 0.0f && magnitude < MOTOR_DEADBAND) {
        magnitude = MOTOR_DEADBAND;
    }
    magnitude = std::min(magnitude, PWM_MAX_F);
    if (magnitude < 1.0f) {
        magnitude = 0.0f;
    }

    const auto duty = static_cast<std::uint32_t>(magnitude);
    const ledc_channel_t in1_channel = kIn1LedcChannels[wheel];
    const ledc_channel_t in2_channel = kIn2LedcChannels[wheel];

    if (duty == 0U) {
        forceSingleMotorPinsLow(wheel);
        return;
    }

    if (clamped > 0.0f) {
        setChannelDuty(in2_channel, 0U);
        setChannelDuty(in1_channel, duty);
    } else {
        setChannelDuty(in1_channel, 0U);
        setChannelDuty(in2_channel, duty);
    }
}

}  // namespace

void initMotorDriver() {
    ledc_timer_config_t timer_config = {};
    timer_config.speed_mode = LEDC_HIGH_SPEED_MODE;
    timer_config.duty_resolution = static_cast<ledc_timer_bit_t>(PWM_RESOLUTION_BITS);
    timer_config.timer_num = LEDC_TIMER_0;
    timer_config.freq_hz = PWM_FREQUENCY;
    timer_config.clk_cfg = LEDC_AUTO_CLK;
    ledc_timer_config(&timer_config);

    for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
        configurePwmChannel(MOTOR_IN1_PINS[wheel], kIn1LedcChannels[wheel]);
        configurePwmChannel(MOTOR_IN2_PINS[wheel], kIn2LedcChannels[wheel]);
    }
}

void writeMotorOutputs(const float pwm_output[kWheelCount]) {
    for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
        writeSingleMotor(static_cast<WheelIndex>(wheel), pwm_output[wheel]);
    }
}

void stopAllMotors() {
    forceMotorPinsLow();
}

void forceMotorPinsLow() {
    for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
        forceSingleMotorPinsLow(static_cast<WheelIndex>(wheel));
    }
}

}  // namespace app
