#include "MotorControl.h"

#include "SerialPort.h"

#include <rclcpp/logging.hpp>

MotorControl::MotorControl() : Node("motor_control") {
    serial_.connect("/dev/ttyACM0");

    if (!serial_.is_connected()) {
        RCLCPP_WARN(this->get_logger(), "Serial: failed to connect to pico");
    } else {
        RCLCPP_INFO(this->get_logger(), "Serial: connected to pico");
    }

    constexpr int queue_size = 10;
    cmd_sub_ = this->create_subscription<serial_msg::msg::MotorCommand>(
        "/motor_command", queue_size,
        [this](const serial_msg::msg::MotorCommand &msg) {
            this->cmd_callback(msg);
        });

    // send_timer_ = this->create_wall_timer(std::chrono::milliseconds(50),
    //                                       [this]() { this->send_frame(); });
}

void MotorControl::cmd_callback(const serial_msg::msg::MotorCommand &msg) {
    for (size_t i = 0; i < msg.ids.size(); ++i) {
        const uint8_t id = msg.ids[i];
        if (id >= motor_protocol::MOTOR_COUNT) {
            RCLCPP_WARN(this->get_logger(),
                        "Invalid motor id: %u (must be 0-5)", id);
            continue;
        }

        auto &m = frame_.motors[id];
        m.enable = msg.enable[i];
        m.direction = msg.direction[i];
        m.angle_set = msg.angle_set[i];
        m.velocity_set = msg.velocity_set[i];
        m.angle_value = msg.angle[i];
        m.velocity_value = msg.velocity[i];
    }

    send_frame();
}

void MotorControl::send_frame() {
    if (!serial_.is_connected())
        return;

    buf_[0] = 0xAA;
    if (!frame_.serialize(&buf_[1], buf_.size() - 1)) {
        RCLCPP_WARN(this->get_logger(), "Failed to serialize frame");
        return;
    }

    if (!serial_.write_data(buf_.data(), buf_.size()))
        RCLCPP_WARN(this->get_logger(), "Serial write failed");

    // motor_protocol::Frame frame{};
    // if (!frame.deserialize(&buf_[1], buf_.size() - 1)) {
    //     RCLCPP_WARN(this->get_logger(), "Failed to deserialize frame");
    //     return;
    // }

    // RCLCPP_INFO(this->get_logger(), "Frame:\n%s", frame.to_string().c_str());
}
