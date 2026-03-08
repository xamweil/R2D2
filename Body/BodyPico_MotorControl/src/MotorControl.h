#pragma once

#include <Arduino.h>
#include <RP2040_PWM.h>

#include <array>
#include <cstddef>
#include <cstdint>

struct MotorConfig {
    uint8_t enable_pin;
    uint8_t pulse_pin;
    uint8_t direction_pin;
    uint8_t step_size;
};

class Motor {
private:
    RP2040_PWM stepper_;
    uint8_t enable_pin_;
    uint8_t direction_pin_;
    uint8_t pulse_pin_;
    uint8_t step_size_;
    bool direction_ = true;
    int32_t frequency_ = 500;
    bool enabled_ = false;

public:
    explicit Motor(const MotorConfig &config);
    void set_frequency(int32_t frequency);
    void set_direction(bool direction);
    void set_enabled(bool enabled);
    void toggle_direction();
    void toggle_enabled();
    [[nodiscard]] int32_t get_frequency() const;
    [[nodiscard]] bool get_direction() const;
    [[nodiscard]] bool is_enabled() const;
};

struct MotorCommand {
    bool enabled;
    bool direction;
    uint32_t frequency;

    bool deserialize(const uint8_t *buf, size_t len);

    static constexpr size_t SIZE = 5;
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

    bool deserialize(const uint8_t *buf, size_t len);
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
    Motors motors_;

    MotorCommandFrame command{};
    MotorCommandFrame command_prev{};

    explicit MotorControl(const MotorsConfigs &motors_configs);
    void update(std::array<uint8_t, MOTOR_COMMAND_FRAME_SIZE> &buf);
};
