#include "MotorControl.h"

#include "SerialPort.h"

#include <rclcpp/logging.hpp>

#include <array>
#include <cstddef>

MotorControl::MotorControl() : Node("motor_control") {
    connect_serial();

    frame_ = MotorCommandFrame{};

    constexpr int queue_size = 10;
    cmd_sub_ = this->create_subscription<serial_msg::msg::MotorCommand>(
        "/motor_command", queue_size,
        [this](const serial_msg::msg::MotorCommand &msg) {
            this->cmd_callback(msg);
        });

    send_timer_ = this->create_wall_timer(std::chrono::milliseconds(50),
                                          [this]() { this->send_frame(); });
}

void MotorControl::cmd_callback(const serial_msg::msg::MotorCommand &msg) {
    switch (msg.id) {
    case 0:
        frame_.mid_foot.enabled = msg.enable;
        frame_.mid_foot.direction = msg.direction;
        frame_.mid_foot.frequency = msg.frequency;
        break;
    case 1:
        frame_.head.enabled = msg.enable;
        frame_.head.direction = msg.direction;
        frame_.head.frequency = msg.frequency;
        break;
    case 2:
        frame_.left_shoulder.enabled = msg.enable;
        frame_.left_shoulder.direction = msg.direction;
        frame_.left_shoulder.frequency = msg.frequency;
        break;
    case 3:
        frame_.right_shoulder.enabled = msg.enable;
        frame_.right_shoulder.direction = msg.direction;
        frame_.right_shoulder.frequency = msg.frequency;
        break;
    case 4:
        frame_.left_foot.enabled = msg.enable;
        frame_.left_foot.direction = msg.direction;
        frame_.left_foot.frequency = msg.frequency;
        break;
    case 5:
        frame_.right_foot.enabled = msg.enable;
        frame_.right_foot.direction = msg.direction;
        frame_.right_foot.frequency = msg.frequency;
        break;
    default:
        RCLCPP_WARN(this->get_logger(), "Invalid motor id: %u (must be 0-5)",
                    msg.id);
        return;
    }
}

void MotorControl::send_frame() {
    if (!serial_.is_connected()) {
        RCLCPP_ERROR(this->get_logger(), "Serial not connected");
        connect_serial();
        return;
    }

    buf_[0] = 0xAA;
    if (!frame_.serialize(&buf_[1], buf_.size() - 1)) {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to serialize motor command frame");
        return;
    }

    if (!serial_.write_data(buf_.data(), buf_.size())) {
        RCLCPP_ERROR(this->get_logger(), "Failed to write motor command frame");
        connect_serial();
    }
}

void MotorControl::connect_serial() {
    if (serial_.is_connected()) {
        return;
    }

    constexpr auto retry_delay = std::chrono::seconds(1);
    while (!serial_.connect("/dev/ttyUSB0")) {
        RCLCPP_INFO(this->get_logger(),
                    "Serial failed to connect, retrying in 1 second...");
        std::this_thread::sleep_for(retry_delay);
    }
}

// Serializes the motor command into a buffer. Returns true if successful, false
// otherwise.
bool MotorCommand::serialize(uint8_t *buf, size_t size) const {
    if (size != MotorCommand::SIZE) {
        return false;
    }
    // NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    buf[0] = (enabled ? 1 : 0) | (direction ? 2 : 0);
    buf[1] = (frequency >> 0) & 0xFF;
    buf[2] = (frequency >> 8) & 0xFF;
    buf[3] = (frequency >> 16) & 0xFF;
    buf[4] = (frequency >> 24) & 0xFF;
    // NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)

    return true;
}

// Serializes the motor command frame into a buffer. Returns true if successful,
// false otherwise.
bool MotorCommandFrame::serialize(uint8_t *buf, size_t size) const {
    if (size != MOTOR_COMMAND_FRAME_SIZE) {
        return false;
    }

    bool success = false;
    for (size_t i = 0; i < NUM_MOTORS; i++) {
        // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
        uint8_t *b = buf + (i * MotorCommand::SIZE);
        // clang-format off
        switch (i) {
            case 0: success = mid_foot.serialize(b, MotorCommand::SIZE);
            case 1: success = head.serialize(b, MotorCommand::SIZE);
            case 2: success = left_shoulder.serialize(b, MotorCommand::SIZE);
            case 3: success = right_shoulder.serialize(b, MotorCommand::SIZE);
            case 4: success = left_foot.serialize(b, MotorCommand::SIZE);
            case 5: success = right_foot.serialize(b, MotorCommand::SIZE);
            default: return false;
        }
        // clang-format on
        if (!success) {
            return false;
        }
    }

    return success;
}
