#pragma once

#include <cstdint>

#include "shared_state.hpp"
#include "config.hpp"

namespace app {

void initEncoders();
void resetEncoders();
void readEncoderCounts(int32_t counts_out[kWheelCount]);
int32_t readEncoderCount(WheelIndex wheel);

}  // namespace app
