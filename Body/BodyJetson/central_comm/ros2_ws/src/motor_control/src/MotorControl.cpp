#include "MotorControl.h"

#include "SerialPort.h"

#include <rclcpp/logging.hpp>

MotorControl::MotorControl() : Node("motor_control") {
    serial_.connect("/dev/ttyACM0");

    if (!serial_.is_connected()) {
        RCLCPP_WARN(this->get_logger(),
                    "Serial: failed to connect to pico, will retry");
        reconnect_timer_ = this->create_wall_timer(
            RECONNECT_INTERVAL_S, [this]() { this->try_serial_reconnect(); });
    } else {
        RCLCPP_INFO(this->get_logger(), "Serial: connected to pico");
    }

    // TODO: is this the right queue size?
    constexpr int queue_size = 10;
    cmd_sub_ = this->create_subscription<serial_msg::msg::MotorCommand>(
        "/motor_command", queue_size,
        [this](const serial_msg::msg::MotorCommand &msg) {
            this->cmd_callback(msg);
        });
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

    schedule_send();
}

// Send immediately on the first message, then enforce a minimum interval
// (SEND_COOLDOWN) between sends. Messages arriving during cooldown update
// frame_ but the send is deferred until the timer fires.
void MotorControl::schedule_send() {
    dirty_ = true;
    if (!cooldown_timer_) {
        // No cooldown active: send right away and start the cooldown timer.
        send_frame();
        cooldown_timer_ = this->create_wall_timer(
            SEND_COOLDOWN_MS, [this]() { this->on_cooldown_expire(); });
    }
}

void MotorControl::on_cooldown_expire() {
    if (dirty_) {
        // New data arrived during cooldown: send it and keep the timer
        // running so we don't send again too soon.
        send_frame();
    } else {
        // Nothing changed since the last send: stop the timer so the next
        // message triggers an immediate send via schedule_send().
        cooldown_timer_.reset();
    }
}

void MotorControl::send_frame() {
    dirty_ = false;

    if (!serial_.is_connected())
        return;

    buf_[0] = 0xAA;
    if (!frame_.serialize(&buf_[1], buf_.size() - 1)) {
        RCLCPP_WARN(this->get_logger(), "Failed to serialize frame");
        return;
    }

    if (!serial_.write_data(buf_.data(), buf_.size())) {
        RCLCPP_WARN(this->get_logger(),
                    "Serial write failed, starting reconnect");
        serial_.disconnect();
        if (!reconnect_timer_) {
            reconnect_timer_ =
                this->create_wall_timer(RECONNECT_INTERVAL_S, [this]() {
                    this->try_serial_reconnect();
                });
        }
        return;
    }

    RCLCPP_DEBUG(this->get_logger(), "Frame sent:\n%s",
                 frame_.to_string().c_str());
}

void MotorControl::try_serial_reconnect() {
    if (serial_.reconnect()) {
        RCLCPP_INFO(this->get_logger(), "Serial: reconnected to pico");
        reconnect_timer_.reset();
    } else {
        RCLCPP_WARN(this->get_logger(),
                    "Serial: reconnect failed, retrying...");
    }
}
