#include "safety.hpp"

namespace app {

bool MotionSafety::motionAllowed(const CommandState& command, std::uint32_t now_ms) const {
    if (!command.robot_enabled) {
        return false;
    }
    return (now_ms - command.last_cmd_time_ms) < CMD_TIMEOUT_MS;
}

}  // namespace app
