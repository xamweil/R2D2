#include "MotorControl.h"

#include <Arduino.h>

#include <cstdint>

Motor::Motor(const MotorConfig &config)
    : enable_pin_(config.enable_pin),
      direction_pin_(config.direction_pin),
      pulse_pin_(config.pulse_pin),
      step_size_(config.step_size),
      stepper_(config.pulse_pin, 500, 0, false) {

    pinMode(pulse_pin_, OUTPUT);
    pinMode(enable_pin_, OUTPUT);
    pinMode(direction_pin_, OUTPUT);

    setEnabled(false);
}

void Motor::setFrequency(int32_t frequency) {
    frequency_ = frequency;

    if (frequency == 0) {
        stepper_.setPWM(pulse_pin_, 500, 0);
    } else {
        stepper_.setPWM(pulse_pin_, static_cast<float>(frequency), 50);
    }
};

void Motor::setDirection(bool direction) {
    direction_ = direction;
    digitalWrite(direction_pin_, direction ? HIGH : LOW);
}

void Motor::setEnabled(bool enabled) {
    enabled_ = enabled;
    digitalWrite(enable_pin_, enabled ? LOW : HIGH);
}

[[nodiscard]] bool Motor::getDirection() const {
    return direction_;
}

[[nodiscard]] bool Motor::isEnabled() const {
    return enabled_;
}

[[nodiscard]] int32_t Motor::getFrequency() const {
    return frequency_;
}

void MotorControl::update() {
    const uint32_t now = millis();
    const uint32_t dt = now - last_update_;
    last_update_ = now;

    // NOLINTBEGIN(cppcoreguidelines-pro-bounds-constant-array-index)
    for (size_t i = 0; i < controller_state.buttons.size(); ++i) {
        const bool &button = controller_state.buttons[i];
        bool &prev_button = controller_state.prev_buttons[i];

        if (button == prev_button)
            continue;

        prev_button = button;

        if (button) {
            switch (i) {
            case 0: // CROSS
                motor_left_foot.setFrequency(motor_left_foot.getFrequency() +
                                             100);
                motor_right_foot.setFrequency(motor_right_foot.getFrequency() +
                                              100);
                break;
            case 1: // CIRCLE
                motor_left_foot.setFrequency(0);
                motor_right_foot.setFrequency(0);
                break;
            case 2: // SQUARE
                motor_left_foot.setDirection(!motor_left_foot.getDirection());
                motor_right_foot.setDirection(!motor_right_foot.getDirection());
                break;
            case 3: // TRIANGLE
                motor_left_foot.setEnabled(!motor_left_foot.isEnabled());
                motor_right_foot.setEnabled(!motor_right_foot.isEnabled());
                break;
            default:
                break;
            }
        }
    }

    float x = 0;
    float y = 0;
    float head_x = 0;

    uint32_t freq_min = 500;
    uint32_t freq_max = 5000;
    uint32_t ramp_time = 2;
    float max_change_per_second = 1.0 / ramp_time;
    float max_change = max_change_per_second * dt;

    float difference = 0;

    for (size_t i = 0; i < controller_state.axes.size(); ++i) {
        switch (i) {
        case 0: // LEFT STICK X
            x = controller_state.axes[i];
            break;
        case 1: // LEFT STICK Y
            y = controller_state.axes[i];
            break;
        case 3: // RIGHT STICK X
            head_x = controller_state.axes[i];
            break;
        default:
            break;
        }
    }

    auto angle = std::atan2(y, x) * (180 / PI);

    // NOLINTEND(cppcoreguidelines-pro-bounds-constant-array-index)
}
