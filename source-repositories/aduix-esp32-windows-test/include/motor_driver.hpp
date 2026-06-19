#pragma once

#include "shared_state.hpp"

namespace app {

void initMotorDriver();
void writeMotorOutputs(const float pwm_output[kWheelCount]);
void stopAllMotors();
void forceMotorPinsLow();

}  // namespace app
