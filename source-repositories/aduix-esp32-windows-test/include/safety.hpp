#pragma once

#include <cstdint>

#include "shared_state.hpp"
#include "config.hpp"

namespace app {

class MotionSafety {
public:
    // Returns true only when robot is enabled AND command is fresh
    bool motionAllowed(const CommandState& command,
                       uint32_t now_ms) const;
};

}  // namespace app
