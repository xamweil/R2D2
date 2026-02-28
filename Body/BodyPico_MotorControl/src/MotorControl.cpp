#include "MotorControl.h"

#include <Arduino.h>

#include <cstdint>

Motor::Motor(const MotorConfig &config)
    : enable_pin_(config.enable_pin),
      direction_pin_(config.direction_pin),
      pulse_pin_(config.pulse_pin),
      steps_per_rev_(config.steps_per_rev),
      stepper_(config.pulse_pin, 100, 0, false) {
    pinMode(pulse_pin_, OUTPUT);
    pinMode(enable_pin_, OUTPUT);
    pinMode(direction_pin_, OUTPUT);
    set_enabled(false);
    last_tick_ms_ = millis();
}

void Motor::set_frequency(int32_t frequency) {
    frequency_ = frequency;

    if (frequency == 0) {
        stepper_.setPWM(pulse_pin_, 100, 0);
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

void Motor::toggle_direction() {
    set_direction(!direction_);
}

void Motor::toggle_enabled() {
    set_enabled(!enabled_);
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

[[nodiscard]] int32_t Motor::get_step_count() const {
    return step_count_;
}

[[nodiscard]] int32_t Motor::get_steps_per_rev() const {
    return steps_per_rev_;
}

void Motor::tick() {
    uint32_t now = millis();
    uint32_t dt = now - last_tick_ms_;
    last_tick_ms_ = now;

    if (enabled_ && frequency_ > 0 && dt > 0) {
        int32_t sign = direction_ ? 1 : -1;
        step_fraction_ += sign * (int32_t)dt * frequency_;
        int32_t steps = step_fraction_ / 1000;
        step_fraction_ -= steps * 1000;
        step_count_ += steps;
    }

    if (position_mode_) {
        int32_t error = target_steps_ - step_count_;
        if (error >= -1 && error <= 1) {
            set_enabled(false);
            position_mode_ = false;
        } else {
            bool new_dir = (error > 0);
            if (new_dir != direction_) {
                set_direction(new_dir);
            }
        }
    }
}

void Motor::set_target(int32_t target_steps, uint16_t speed) {
    target_steps_ = target_steps;
    speed_steps_per_sec_ = speed;
    position_mode_ = true;

    int32_t error = target_steps_ - step_count_;
    set_direction(error > 0);
    set_frequency(speed);
    set_enabled(true);
    last_tick_ms_ = millis();
}

MotorControl::MotorControl(const MotorsConfigs &motors_configs)
    : motors_({
          .mid_foot = Motor(motors_configs.mid_foot),
          .head = Motor(motors_configs.head),
          .left_shoulder = Motor(motors_configs.left_shoulder),
          .right_shoulder = Motor(motors_configs.right_shoulder),
          .left_foot = Motor(motors_configs.left_foot),
          .right_foot = Motor(motors_configs.right_foot),
      }) {

    motors_.left_foot.set_direction(false);
    motors_.right_foot.set_direction(true);
    motors_.left_shoulder.set_direction(true);
    motors_.right_shoulder.set_direction(false);

    motors_.head.set_enabled(false);
    motors_.mid_foot.set_enabled(false);
    motors_.left_shoulder.set_enabled(false);
    motors_.right_shoulder.set_enabled(false);
    motors_.left_foot.set_enabled(false);
    motors_.right_foot.set_enabled(false);
}

void MotorControl::tick() {
    motors_.head.tick();
}

void MotorControl::update(std::array<uint8_t, MOTOR_COMMAND_FRAME_SIZE> &buf) {
    command_prev = command;

    // TODO: handle err
    MotorCommandFrame::deserialize(buf.data(), buf.size(), command);

    const auto &head = command.head;
    int32_t target_steps =
        (int32_t)head.target_angle_decideg * motors_.head.get_steps_per_rev() / 3600;
    motors_.head.set_target(target_steps, head.speed_steps_per_sec);
    motors_.head.set_enabled(head.enabled);
}
