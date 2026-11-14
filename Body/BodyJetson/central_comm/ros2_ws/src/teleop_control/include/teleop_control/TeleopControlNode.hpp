#ifndef DRIVE_CONTROL_NODE_HPP
#define DRIVE_CONTROL_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "serial_msg/msg/motor_command.hpp"

#include <sensor_msgs/msg/joy.hpp>

namespace TeleopControl {
class TeleopControlNode : public rclcpp::Node {
public:
    TeleopControlNode();

private:
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joySub_;
    sensor_msgs::msg::Joy::ConstSharedPtr latestJoy_;
    void joyCallback(sensor_msgs::msg::Joy::ConstSharedPtr msg);

    rclcpp::Publisher<serial_msg::msg::MotorCommand>::SharedPtr
        motorCommandPublisher_;
    rclcpp::TimerBase::SharedPtr publishTimer_;
    void publishCmd();
};
} // namespace TeleopControl

#endif
