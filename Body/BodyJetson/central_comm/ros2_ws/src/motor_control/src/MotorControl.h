#pragma once

#include "SerialPort.h"

#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/utilities.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <cstdint>

class MotorControl : public rclcpp::Node {
public:
    explicit MotorControl();

private:
    static constexpr uint8_t NUM_BUTTONS = 14;
    static constexpr uint8_t NUM_AXES = 8;
    static constexpr uint8_t PKT_SIZE =
        1 + 2 +
        (NUM_AXES * 4); // SOF (1 byte), Buttons (4 bytes), Axes (6 * 4 bytes)
    void joy_callback(const sensor_msgs::msg::Joy &msg);

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    SerialPort serial_;
    std::array<uint8_t, PKT_SIZE> packet_ = {0xAA};
};
