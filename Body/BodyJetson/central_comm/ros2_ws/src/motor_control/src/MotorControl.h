#pragma once

#include "SerialPort.h"

#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>
#include <rclcpp/utilities.hpp>
#include <serial_msg/msg/motor_command.hpp>

#include <cstdint>

struct MotorCommand {
    bool enabled;
    bool direction;
    uint32_t frequency;

    void serializeTo(uint8_t *buf) const {
        buf[0] = (enabled ? 1 : 0) | (direction ? 2 : 0);
        buf[1] = (frequency >> 0) & 0xFF;
        buf[2] = (frequency >> 8) & 0xFF;
        buf[3] = (frequency >> 16) & 0xFF;
        buf[4] = (frequency >> 24) & 0xFF;
    }

    static MotorCommand deserializeFrom(const uint8_t *buf) {
        MotorCommand cmd{};
        cmd.enabled = buf[0] & 0x01;
        cmd.direction = ((buf[0] & 0x02) != 0);
        cmd.frequency = (uint32_t)buf[1] | (uint32_t)buf[2] << 8 |
                        (uint32_t)buf[3] << 16 | (uint32_t)buf[4] << 24;
        return cmd;
    }

    static constexpr size_t SIZE = 5;
};

static constexpr size_t MOTOR_COMMAND_FRAME_SIZE =
    6 * MotorCommand::SIZE; // 30 bytes

struct MotorCommandFrame {
    MotorCommand mid_foot;
    MotorCommand head;
    MotorCommand left_shoulder;
    MotorCommand right_shoulder;
    MotorCommand left_foot;
    MotorCommand right_foot;

    void serialize(uint8_t *buf) const {
        const MotorCommand *cmds[] = {&mid_foot,      &head,
                                      &left_shoulder, &right_shoulder,
                                      &left_foot,     &right_foot};
        for (size_t i = 0; i < 6; i++) {
            cmds[i]->serializeTo(buf + (i * MotorCommand::SIZE));
        }
    }

    static bool deserialize(const uint8_t *data, size_t len,
                            MotorCommandFrame &frame) {
        if (len < MOTOR_COMMAND_FRAME_SIZE)
            return false;
        MotorCommand *cmds[] = {&frame.mid_foot,      &frame.head,
                                &frame.left_shoulder, &frame.right_shoulder,
                                &frame.left_foot,     &frame.right_foot};
        for (size_t i = 0; i < 6; i++) {
            *cmds[i] =
                MotorCommand::deserializeFrom(data + (i * MotorCommand::SIZE));
        }
        return true;
    }
};

class MotorControl : public rclcpp::Node {
public:
    explicit MotorControl();

private:
    void cmd_callback(const serial_msg::msg::MotorCommand &msg);
    void send_frame();

    MotorCommandFrame frame_{};
    rclcpp::Subscription<serial_msg::msg::MotorCommand>::SharedPtr cmd_sub_;
    rclcpp::TimerBase::SharedPtr send_timer_;
    SerialPort serial_;
};
