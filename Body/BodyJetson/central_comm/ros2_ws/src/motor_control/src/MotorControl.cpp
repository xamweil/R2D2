#include "MotorControl.h"

#include <rclcpp/logging.hpp>

#include <cstdint>
#include <cstring>

MotorControl::MotorControl() : Node("motor_control") {
    serial_.connect("/dev/ttyACM0");

    if (!serial_.isConnected()) {
        RCLCPP_WARN(this->get_logger(), "Serial: failed to connect to pico");
    } else {
        RCLCPP_INFO(this->get_logger(), "Serial: connected to pico");
    }

    constexpr int queue_size = 10;
    subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", queue_size,
        [this](const sensor_msgs::msg::Joy &msg) { this->joy_callback(msg); });
}

void MotorControl::joy_callback(const sensor_msgs::msg::Joy &msg) {
    static constexpr uint8_t NUM_BUTTONS = 13;
    static constexpr uint8_t NUM_AXES = 8;

    if (!serial_.isConnected())
        return;

    if (msg.buttons.size() != NUM_BUTTONS || msg.axes.size() != NUM_AXES) {
        RCLCPP_WARN(
            this->get_logger(),
            "Invalid joy message size: buttons=%zu, axes=%zu (expected %u, %u)",
            msg.buttons.size(), msg.axes.size(), NUM_BUTTONS, NUM_AXES);
        return;
    }

    uint16_t buttons = 0;
    for (size_t i = 0; i < msg.buttons.size(); ++i) {
        if (msg.buttons[i] > 0)
            buttons += 1 << i;
    }

    std::memcpy(&packet_[1], &buttons, sizeof(buttons));

    static constexpr size_t axes_offset = 1 + sizeof(buttons);
    std::memcpy(&packet_[axes_offset], msg.axes.data(),
                msg.axes.size() * sizeof(msg.axes[0]));

    if (serial_.writeData(packet_.data(), packet_.size())) {
        RCLCPP_INFO(this->get_logger(), "Serial write success");
    } else {
        RCLCPP_WARN(this->get_logger(), "Serial write failed");
    }
}
