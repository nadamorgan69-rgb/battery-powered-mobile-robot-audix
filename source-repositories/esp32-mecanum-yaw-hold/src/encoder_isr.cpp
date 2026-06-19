#include "encoder_isr.hpp"

#include <Arduino.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>

namespace app {

namespace {

volatile std::int32_t g_encoder_counts[kWheelCount] = {0, 0, 0, 0};
volatile std::uint8_t g_previous_states[kWheelCount] = {0, 0, 0, 0};
portMUX_TYPE g_encoder_mux[kWheelCount] = {
    portMUX_INITIALIZER_UNLOCKED,
    portMUX_INITIALIZER_UNLOCKED,
    portMUX_INITIALIZER_UNLOCKED,
    portMUX_INITIALIZER_UNLOCKED,
};

constexpr std::int8_t kQuadratureLookup[16] = {
    0, -1, 1, 0,
    1, 0, 0, -1,
    -1, 0, 0, 1,
    0, 1, -1, 0,
};

void configureEncoderInput(int pin) {
    if (pin >= 34) {
        pinMode(pin, INPUT);
        return;
    }
    pinMode(pin, INPUT_PULLUP);
}

std::uint8_t IRAM_ATTR readEncoderState(WheelIndex wheel) {
    const auto a_level = static_cast<std::uint8_t>(
        gpio_get_level(static_cast<gpio_num_t>(ENCODER_A_PINS[wheel])) != 0);
    const auto b_level = static_cast<std::uint8_t>(
        gpio_get_level(static_cast<gpio_num_t>(ENCODER_B_PINS[wheel])) != 0);
    return static_cast<std::uint8_t>((a_level << 1U) | b_level);
}

void IRAM_ATTR updateEncoderFromPins(WheelIndex wheel) {
    portENTER_CRITICAL_ISR(&g_encoder_mux[wheel]);
    const std::uint8_t current_state = readEncoderState(wheel);
    const std::uint8_t transition = static_cast<std::uint8_t>((g_previous_states[wheel] << 2U) | current_state);
    g_encoder_counts[wheel] += kQuadratureLookup[transition];
    g_previous_states[wheel] = current_state;
    portEXIT_CRITICAL_ISR(&g_encoder_mux[wheel]);
}

void IRAM_ATTR handleFrontLeftEncoder() {
    updateEncoderFromPins(kFrontLeft);
}

void IRAM_ATTR handleFrontRightEncoder() {
    updateEncoderFromPins(kFrontRight);
}

void IRAM_ATTR handleRearLeftEncoder() {
    updateEncoderFromPins(kRearLeft);
}

void IRAM_ATTR handleRearRightEncoder() {
    updateEncoderFromPins(kRearRight);
}

}  // namespace

void initEncoders() {
    for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
        configureEncoderInput(ENCODER_A_PINS[wheel]);
        configureEncoderInput(ENCODER_B_PINS[wheel]);
        g_previous_states[wheel] = readEncoderState(static_cast<WheelIndex>(wheel));
    }

    attachInterrupt(digitalPinToInterrupt(ENCODER_A_PINS[kFrontLeft]), handleFrontLeftEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_B_PINS[kFrontLeft]), handleFrontLeftEncoder, CHANGE);

    attachInterrupt(digitalPinToInterrupt(ENCODER_A_PINS[kFrontRight]), handleFrontRightEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_B_PINS[kFrontRight]), handleFrontRightEncoder, CHANGE);

    attachInterrupt(digitalPinToInterrupt(ENCODER_A_PINS[kRearLeft]), handleRearLeftEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_B_PINS[kRearLeft]), handleRearLeftEncoder, CHANGE);

    attachInterrupt(digitalPinToInterrupt(ENCODER_A_PINS[kRearRight]), handleRearRightEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_B_PINS[kRearRight]), handleRearRightEncoder, CHANGE);
}

void readEncoderCounts(std::int32_t counts_out[kWheelCount]) {
    for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
        portENTER_CRITICAL(&g_encoder_mux[wheel]);
        counts_out[wheel] = g_encoder_counts[wheel];
        portEXIT_CRITICAL(&g_encoder_mux[wheel]);
    }
}

std::int32_t readEncoderCount(WheelIndex wheel) {
    std::int32_t count = 0;
    portENTER_CRITICAL(&g_encoder_mux[wheel]);
    count = g_encoder_counts[wheel];
    portEXIT_CRITICAL(&g_encoder_mux[wheel]);
    return count;
}

void resetEncoders() {
    for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
        portENTER_CRITICAL(&g_encoder_mux[wheel]);
        g_encoder_counts[wheel] = 0;
        g_previous_states[wheel] = readEncoderState(static_cast<WheelIndex>(wheel));
        portEXIT_CRITICAL(&g_encoder_mux[wheel]);
    }
}

}  // namespace app
