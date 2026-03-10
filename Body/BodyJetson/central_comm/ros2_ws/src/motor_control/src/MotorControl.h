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
    static constexpr std::chrono::milliseconds SEND_COOLDOWN_MS{50};
    static constexpr std::chrono::seconds RECONNECT_INTERVAL_S{3};

    std::array<uint8_t, 1 + motor_protocol::FRAME_SIZE_BYTES> buf_{}; // 1 byte SOF + frame data
    motor_protocol::Frame frame_{};
    bool dirty_ = false;
    rclcpp::Subscription<serial_msg::msg::MotorCommand>::SharedPtr cmd_sub_;
    rclcpp::TimerBase::SharedPtr cooldown_timer_;
    rclcpp::TimerBase::SharedPtr reconnect_timer_;
    SerialPort serial_;

    void cmd_callback(const serial_msg::msg::MotorCommand &msg);
    void schedule_send();
    void on_cooldown_expire();
    void send_frame();
    void try_serial_reconnect();
};
