#ifndef DRIVE_CONTROL_NODE_HPP
#define DRIVE_CONTROL_NODE_HPP

#include "rclcpp/rclcpp.hpp"

#include "serial_msg/msg/motor_command.hpp"

#include "serial_msg/msg/ddsm115_feedback.hpp"

#include "DriveMotor.hpp"
#include "RS485Com.hpp"

#include <memory>
#include <vector>

namespace DriveControl
{

class DriveControlNode : public rclcpp::Node
{
public:
    DriveControlNode();

private:

    struct FeedbackPublisher
        {
            DriveMotor* motor;

            rclcpp::Publisher<serial_msg::msg::DDSM115Feedback>::SharedPtr publisher;
        };

    RS485Com rs485_;

    std::vector<std::unique_ptr<DriveMotor>> motors_;
    std::vector<FeedbackPublisher> feedbackPublishers_;

    rclcpp::Subscription<serial_msg::msg::MotorCommand>::SharedPtr
        motorCommandSub_;

    rclcpp::TimerBase::SharedPtr communicationTimer_;
    rclcpp::TimerBase::SharedPtr feedbackTimer_;

    void motorCommandCallback(const serial_msg::msg::MotorCommand::SharedPtr msg);

    void communicationUpdate();

    void publishFeedback();

    DriveMotor* findMotorByCommandId(uint8_t commandId);

    bool validateMotorCommand(const serial_msg::msg::MotorCommand& msg) const;
};

}

#endif