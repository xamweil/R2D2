#pragma once

#include "topics.hpp"

#include <behavior_msgs/action/follow_track.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <serial_msg/msg/motor_command.hpp>
#include <serial_msg/srv/device_command.hpp>

#include <functional>
#include <string>

namespace ui_bridge {

class BridgeNode : public rclcpp::Node {
public:
    explicit BridgeNode(
        TopicRegistry &topics,
        const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
    //////////////////////////////////////////////////////////////////
    /// Commands, Actions
    //////////////////////////////////////////////////////////////////

    // TODO:

    // Command queue
    // std::deque<PendingCommand> cmd_queue_;
    // std::mutex cmd_mutex_;

    // ROS clients (created lazily)
    rclcpp::Publisher<serial_msg::msg::MotorCommand>::SharedPtr motor_cmd_pub_;
    rclcpp::Client<serial_msg::srv::DeviceCommand>::SharedPtr
        device_cmd_client_;

    rclcpp_action::Client<behavior_msgs::action::FollowTrack>::SharedPtr
        follow_track_client_;
    rclcpp_action::ClientGoalHandle<
        behavior_msgs::action::FollowTrack>::SharedPtr active_follow_goal_;

    void
    handle_publish(const std::string &body,
                   const std::function<void(const std::string &)> &respond);

    void handle_call_service(
        const std::string &body,
        const std::function<void(const std::string &)> &respond);

    void handle_action_start(
        const std::string &body,
        const std::function<void(const std::string &)> &respond);

    void handle_action_cancel(
        const std::string &body,
        const std::function<void(const std::string &)> &respond);
};

} // namespace ui_bridge
