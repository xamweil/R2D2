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

    explicit MotorControl(const MotorsConfigs &motors_configs);
    void update();
};
