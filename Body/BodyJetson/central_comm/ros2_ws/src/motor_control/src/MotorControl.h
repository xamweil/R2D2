#pragma once

#include "motor_protocol.h"
#include "SerialPort.h"

#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>
#include <rclcpp/utilities.hpp>
#include <serial_msg/msg/motor_command.hpp>

class MotorControl : public rclcpp::Node {
public:
    explicit MotorControl();

private:
    void cmd_callback(const serial_msg::msg::MotorCommand &msg);
    void send_frame();

    std::array<uint8_t, 1 + motor_protocol::FRAME_SIZE_BYTES> buf_{}; // 1 byte SOF + frame data
    motor_protocol::Frame frame_{};
    rclcpp::Subscription<serial_msg::msg::MotorCommand>::SharedPtr cmd_sub_;
    rclcpp::TimerBase::SharedPtr send_timer_;
    SerialPort serial_;
};
