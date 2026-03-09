#include "MotorControl.h"

#include "SerialPort.h"

#include <rclcpp/logging.hpp>

#include <cstdint>

MotorControl::MotorControl() : Node("motor_control") {
    serial_.connect("/dev/ttyACM0");

    if (!serial_.is_connected()) {
        RCLCPP_WARN(this->get_logger(), "Serial: failed to connect to pico");
    } else {
        RCLCPP_INFO(this->get_logger(), "Serial: connected to pico");
    }

    // Initialize all motors disabled
    frame_ = MotorCommandFrame{};

    constexpr int queue_size = 10;
    cmd_sub_ = this->create_subscription<serial_msg::msg::MotorCommand>(
        "/motor_command", queue_size,
        [this](const serial_msg::msg::MotorCommand &msg) {
            this->cmd_callback(msg);
        });

    send_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        [this]() { this->send_frame(); });
}

void MotorControl::cmd_callback(const serial_msg::msg::MotorCommand &msg) {
    if (msg.id >= 6) {
        RCLCPP_WARN(this->get_logger(), "Invalid motor id: %u (must be 0-5)", msg.id);
        return;
    }

    MotorCommand *cmds[] = {&frame_.mid_foot,      &frame_.head,
                            &frame_.left_shoulder, &frame_.right_shoulder,
                            &frame_.left_foot,     &frame_.right_foot};

    cmds[msg.id]->enabled   = msg.enable;
    cmds[msg.id]->direction = msg.direction;
    cmds[msg.id]->frequency = msg.frequency;
}

void MotorControl::send_frame() {
    if (!serial_.is_connected())
        return;

    uint8_t buf[31];
    buf[0] = 0xAA;
    frame_.serialize(buf + 1);

    if (!serial_.write_data(buf, 31))
        RCLCPP_WARN(this->get_logger(), "Serial write failed");
}
