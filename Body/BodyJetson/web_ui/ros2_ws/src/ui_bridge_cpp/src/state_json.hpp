#pragma once

#include "bridge_node.hpp"

#include <string>

namespace ui_bridge {

// Builds the {"type":"robot_state","payload":{...}} message from the latest
// values in the store. Same shape as the Python ui_bridge snapshot.
std::string build_robot_state_json(const TelemetryStore &store,
                                   double stale_sec);

// Builds the {"type":"hello","payload":{...}} message sent on connect.
std::string build_hello_json(double state_hz, double stale_sec);

} // namespace ui_bridge
