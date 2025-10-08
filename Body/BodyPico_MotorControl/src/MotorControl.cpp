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
    set_enabled(false);
}

void Motor::set_frequency(int32_t frequency) {
    frequency_ = frequency;

    if (frequency == 0) {
        stepper_.setPWM(pulse_pin_, 500, 0);
    } else {
        stepper_.setPWM(pulse_pin_, static_cast<float>(frequency), 50);
    }
};

void Motor::set_direction(bool direction) {
    direction_ = direction;
    digitalWrite(direction_pin_, direction ? HIGH : LOW);
}

void Motor::set_enabled(bool enabled) {
    enabled_ = enabled;
    digitalWrite(enable_pin_, enabled ? LOW : HIGH);
}

[[nodiscard]] bool Motor::get_direction() const {
    return direction_;
}

[[nodiscard]] bool Motor::is_enabled() const {
    return enabled_;
}

[[nodiscard]] int32_t Motor::get_frequency() const {
    return frequency_;
}

MotorControl::MotorControl(Motors &motors) : motors_(motors) {
    motors_.left_foot.set_direction(false);
    motors_.right_foot.set_direction(true);
    motors_.left_shoulder.set_direction(true);
    motors_.right_shoulder.set_direction(false);

    // motors_.head.set_enabled(true);
    // motors_.mid_foot.set_enabled(true);
    motors_.left_shoulder.set_enabled(true);
    motors_.right_shoulder.set_enabled(true);
    // motors_.right_shoulder.set_frequency(0);
    // motors_.left_shoulder.set_frequency(0);
    // motors_.left_foot.set_enabled(true);
    // motors_.right_foot.set_enabled(true);
}

void MotorControl::update() {
    const uint32_t now = millis();
    const uint32_t dt = now - last_update_;
    last_update_ = now;

    for (size_t i = 0; i < controller_state_.buttons.size(); ++i) {
        const bool &button = controller_state_.buttons[i];
        bool &prev_button = controller_state_.prev_buttons[i];

        if (button == prev_button)
            continue;

        prev_button = button;

        if (button) {
            switch (i) {
            case 0: // CROSS
                motors_.left_foot.set_frequency(
                    motors_.left_foot.get_frequency() + 100);
                motors_.right_foot.set_frequency(
                    motors_.right_foot.get_frequency() + 100);
                break;
            case 1: // CIRCLE
                motors_.left_foot.set_frequency(0);
                motors_.right_foot.set_frequency(0);
                break;
            case 2: // SQUARE
                motors_.left_foot.set_direction(
                    !motors_.left_foot.get_direction());
                motors_.right_foot.set_direction(
                    !motors_.right_foot.get_direction());
                break;
            case 3: // TRIANGLE
                // motors_.head.set_enabled(!motors_.head.is_enabled());
                // motors_.mid_foot.set_enabled(!motors_.mid_foot.is_enabled());
                motors_.left_shoulder.set_enabled(
                    !motors_.left_shoulder.is_enabled());
                motors_.right_shoulder.set_enabled(
                    !motors_.right_shoulder.is_enabled());
                // motors_.left_foot.set_enabled(!motors_.left_foot.is_enabled());
                // motors_.right_foot.set_enabled(
                //     !motors_.right_foot.is_enabled());
                break;
            default:
                break;
            }
        }
    }

    for (size_t i = 0; i < controller_state_.axes.size(); ++i) {
        switch (i) {
        case 6: // DPAD X
            if (controller_state_.axes[i] > 0) {
                motors_.left_shoulder.set_direction(true);
                motors_.right_shoulder.set_direction(false);
                motors_.left_shoulder.set_frequency(100);
                motors_.right_shoulder.set_frequency(100);
            } else if (controller_state_.axes[i] < 0) {
                motors_.left_shoulder.set_direction(false);
                motors_.right_shoulder.set_direction(true);
                motors_.left_shoulder.set_frequency(100);
                motors_.right_shoulder.set_frequency(100);
            } else {
                motors_.left_shoulder.set_frequency(0);
                motors_.right_shoulder.set_frequency(0);
            }
        default:
            break;
        }
    }

    // float x = 0;
    // float y = 0;
    // float head_x = 0;
    //
    // uint32_t freq_min = 500;
    // uint32_t freq_max = 5000;
    // uint32_t ramp_time = 2;
    // float max_change_per_second = 1.0 / ramp_time;
    // float max_change = max_change_per_second * dt;
    //
    // float difference = 0;
    //
    // for (size_t i = 0; i < controller_state_.axes.size(); ++i) {
    //     switch (i) {
    //     case 0: // LEFT STICK X
    //         x = controller_state_.axes[i];
    //         break;
    //     case 1: // LEFT STICK Y
    //         y = controller_state_.axes[i];
    //         break;
    //     case 3: // RIGHT STICK X
    //         head_x = controller_state_.axes[i];
    //         break;
    //     default:
    //         break;
    //     }
    // }
    //
    // auto angle = std::atan2(y, x) * (180 / PI);

    // NOLINTEND(cppcoreguidelines-pro-bounds-constant-array-index)
}
