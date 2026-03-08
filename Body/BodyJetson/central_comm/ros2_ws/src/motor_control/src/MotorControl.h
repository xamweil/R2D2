#pragma once

#include "SerialPort.h"

#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>
#include <rclcpp/utilities.hpp>
#include <serial_msg/msg/motor_command.hpp>

#include <cstddef>
#include <cstdint>

struct MotorCommand {
    bool enabled;
    bool direction;
    uint32_t frequency;

    // 1 byte for enabled and direction, 4 bytes for frequency
    static constexpr size_t SIZE = 5;

    bool serialize(uint8_t *buf, size_t size) const;
};

static constexpr size_t NUM_MOTORS = 6;
static constexpr size_t MOTOR_COMMAND_FRAME_SIZE =
    NUM_MOTORS * MotorCommand::SIZE; // 30 bytes

struct MotorCommandFrame {
    MotorCommand mid_foot;
    MotorCommand head;
    MotorCommand left_shoulder;
    MotorCommand right_shoulder;
    MotorCommand left_foot;
    MotorCommand right_foot;

    bool serialize(uint8_t *buf, size_t size) const;
};

class MotorControl : public rclcpp::Node {
public:
    explicit MotorControl();

private:
    void cmd_callback(const serial_msg::msg::MotorCommand &msg);
    void send_frame();
    void connect_serial();

    // Buffer for sending the frame over serial (1 byte SOF=0xAA + frame data)
    std::array<uint8_t, 1 + MOTOR_COMMAND_FRAME_SIZE> buf_{};

    MotorCommandFrame frame_{};
    rclcpp::Subscription<serial_msg::msg::MotorCommand>::SharedPtr cmd_sub_;
    rclcpp::TimerBase::SharedPtr send_timer_;
    SerialPort serial_;
};
