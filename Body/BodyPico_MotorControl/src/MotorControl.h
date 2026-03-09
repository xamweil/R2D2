#pragma once

#include <Arduino.h>
#include <RP2040_PWM.h>

#include <array>
#include <cstddef>
#include <cstdint>

static constexpr size_t NUM_BUTTONS = 14;
static constexpr size_t NUM_AXES = 8;

/*
 * BUTTONS:
 *  0 CROSS,
 *  1 CIRCLE,
 *  2 SQUARE,
 *  3 TRIANGLE,
 *  4 LEFT SHOULDER,
 *  5 RIGHT SHOULDER,
 *  6 LEFT TRIGGER,
 *  7 RIGHT TRIGGER,
 *  8 SHARE,
 *  9 OPTIONS,
 * 10 HOME,
 * 11 LEFT STICK,
 * 12 RIGHT STICK,
 * 13 TOUCHPAD
 *
 * AXES:
 *  0 LEFT STICK X,
 *  1 LEFT STICK Y,
 *  2 LEFT TRIGGER,
 *  3 RIGHT STICK X,
 *  4 RIGHT STICK Y,
 *  5 RIGHT TRIGGER,
 *  6 DPAD X,
 *  7 DPAD Y
 */
struct ControllerState {
    std::array<bool, NUM_BUTTONS> buttons{};
    std::array<bool, NUM_BUTTONS> prev_buttons{};
    std::array<float, NUM_AXES> axes{};
    std::array<float, NUM_AXES> prev_axes{};
};

struct MotorConfig {
    uint8_t enable_pin;
    uint8_t pulse_pin;
    uint8_t direction_pin;
    int32_t steps_per_rev;
};

class Motor {
private:
    RP2040_PWM stepper_;
    uint8_t enable_pin_;
    uint8_t direction_pin_;
    uint8_t pulse_pin_;
    int32_t steps_per_rev_;
    bool direction_ = true;
    int32_t frequency_ = 500;
    bool enabled_ = false;

    // Position tracking
    int32_t step_count_ = 0;
    int32_t step_fraction_ = 0; // sub-step accumulator (units: 1/1000 step)
    uint32_t last_tick_ms_ = 0;
    bool position_mode_ = false;
    int32_t target_steps_ = 0;
    uint16_t speed_steps_per_sec_ = 500;

public:
    explicit Motor(const MotorConfig &config);
    void set_frequency(int32_t frequency);
    void set_direction(bool direction);
    void set_enabled(bool enabled);
    void toggle_direction();
    void toggle_enabled();
    void tick();
    void set_target(int32_t target_steps, uint16_t speed);
    [[nodiscard]] int32_t get_frequency() const;
    [[nodiscard]] bool get_direction() const;
    [[nodiscard]] bool is_enabled() const;
    [[nodiscard]] int32_t get_step_count() const;
    [[nodiscard]] int32_t get_steps_per_rev() const;
};

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

struct HeadMotorCommand {
    bool enabled;
    int16_t target_angle_decideg;  // tenths of a degree, e.g. 900 = 90.0°
    uint16_t speed_steps_per_sec;

    static HeadMotorCommand deserializeFrom(const uint8_t *buf) {
        HeadMotorCommand cmd{};
        cmd.enabled = buf[0] & 0x01;
        cmd.target_angle_decideg =
            (int16_t)((uint16_t)buf[1] | ((uint16_t)buf[2] << 8));
        cmd.speed_steps_per_sec =
            (uint16_t)buf[3] | ((uint16_t)buf[4] << 8);
        return cmd;
    }

    static constexpr size_t SIZE = 5;
};

static constexpr size_t MOTOR_COMMAND_FRAME_SIZE =
    6 * MotorCommand::SIZE; // 30 bytes

struct MotorCommandFrame {
    MotorCommand mid_foot;
    HeadMotorCommand head;
    MotorCommand left_shoulder;
    MotorCommand right_shoulder;
    MotorCommand left_foot;
    MotorCommand right_foot;

    void serialize(uint8_t *buf) const {
        mid_foot.serializeTo(buf + 0 * MotorCommand::SIZE);
        // HeadMotorCommand serialization
        buf[1 * MotorCommand::SIZE + 0] = head.enabled ? 1 : 0;
        buf[1 * MotorCommand::SIZE + 1] = (uint8_t)(head.target_angle_decideg & 0xFF);
        buf[1 * MotorCommand::SIZE + 2] = (uint8_t)((head.target_angle_decideg >> 8) & 0xFF);
        buf[1 * MotorCommand::SIZE + 3] = (uint8_t)(head.speed_steps_per_sec & 0xFF);
        buf[1 * MotorCommand::SIZE + 4] = (uint8_t)((head.speed_steps_per_sec >> 8) & 0xFF);
        left_shoulder.serializeTo(buf + 2 * MotorCommand::SIZE);
        right_shoulder.serializeTo(buf + 3 * MotorCommand::SIZE);
        left_foot.serializeTo(buf + 4 * MotorCommand::SIZE);
        right_foot.serializeTo(buf + 5 * MotorCommand::SIZE);
    }

    static bool deserialize(const uint8_t *data, size_t len,
                            MotorCommandFrame &frame) {
        if (len < MOTOR_COMMAND_FRAME_SIZE)
            return false;
        frame.mid_foot = MotorCommand::deserializeFrom(data + 0 * MotorCommand::SIZE);
        frame.head = HeadMotorCommand::deserializeFrom(data + 1 * MotorCommand::SIZE);
        frame.left_shoulder = MotorCommand::deserializeFrom(data + 2 * MotorCommand::SIZE);
        frame.right_shoulder = MotorCommand::deserializeFrom(data + 3 * MotorCommand::SIZE);
        frame.left_foot = MotorCommand::deserializeFrom(data + 4 * MotorCommand::SIZE);
        frame.right_foot = MotorCommand::deserializeFrom(data + 5 * MotorCommand::SIZE);
        return true;
    }
};

struct Motors {
    Motor mid_foot;
    Motor head;
    Motor left_shoulder;
    Motor right_shoulder;
    Motor left_foot;
    Motor right_foot;
};

struct MotorsConfigs {
    MotorConfig mid_foot;
    MotorConfig head;
    MotorConfig left_shoulder;
    MotorConfig right_shoulder;
    MotorConfig left_foot;
    MotorConfig right_foot;
};

class MotorControl {
public:
    uint32_t last_update_{};
    ControllerState controller_state_;
    Motors motors_;

    MotorCommandFrame command;
    MotorCommandFrame command_prev;

    explicit MotorControl(const MotorsConfigs &motors_configs);
    void tick();
    void update(std::array<uint8_t, MOTOR_COMMAND_FRAME_SIZE> &buf);
};
