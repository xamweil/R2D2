#pragma once

#include "topics.hpp"

#include <string>

namespace ui_bridge {

// Writes the {"type":"robot_state","payload":{...}} message for the latest
// values of all topics with in_state into out (cleared first, capacity kept).
// Same shape as the Python ui_bridge snapshot. Message data is only
// re-serialized when a new message arrived since the last call, so this must
// only be called from one thread.
void build_robot_state_json(TopicRegistry &topics, double stale_sec,
                            std::string &out);

// Builds the {"type":"hello","payload":{...}} message sent on connect.
std::string build_hello_json(double state_hz, double stale_sec);

} // namespace ui_bridge
