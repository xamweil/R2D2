#include "teleop_control/TeleopControlNode.hpp"

#include <rclcpp/logging.hpp>

#include <algorithm>
#include <cmath>

namespace TeleopControl {
using namespace std::chrono_literals;

TeleopControlNode ::TeleopControlNode() : Node("teleop_control_node") {
    const auto qos =
        rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();

    joySub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/teleop", qos, [this](sensor_msgs::msg::Joy::ConstSharedPtr msg) {
            // RCLCPP_DEBUG_STREAM(get_logger(),
            // sensor_msgs::msg::to_yaml(*msg));
            latestJoy_ = std::move(msg);
        });

    motorCommandPublisher_ =
        this->create_publisher<serial_msg::msg::MotorCommand>("/drive_command",
                                                              qos);
    publishTimer_ = create_wall_timer(20ms, [this] { publishCmd(); });

    RCLCPP_INFO(get_logger(), "%s initialized", get_name());
};

void TeleopControlNode::publishCmd() {

    if (!latestJoy_ || latestJoy_->axes.size() < 2)
        return;

    auto x = latestJoy_->axes[0];
    auto y = latestJoy_->axes[1];

    // RCLCPP_INFO(get_logger(), "%f %f", x, y);

    constexpr float kMaxRpm = 300.0F;
    // const float x = latestJoy_->axes[0];
    // const float y = latestJoy_->axes[1];

    const float left = std::clamp(y - x, -1.0F, 1.0F);
    const float right = std::clamp(y + x, -1.0F, 1.0F);

    serial_msg::msg::MotorCommand cmd;
    using Vel = decltype(cmd.velocity)::value_type;

    auto add = [&](int id, float speed, bool mirrored) {
        cmd.ids.push_back(id);
        cmd.enable.push_back(true);
        cmd.direction.push_back((speed < 0.0F) != mirrored);
        cmd.angle_set.push_back(false);
        cmd.velocity_set.push_back(true);
        cmd.angle.push_back(0.0);
        cmd.velocity.push_back(static_cast<Vel>(std::abs(speed) * kMaxRpm));
    };

    for (int id : {11, 13})
        add(id, left, false);
    for (int id : {15, 17})
        add(id, right, true);

    motorCommandPublisher_->publish(cmd);
}

} // namespace TeleopControl
