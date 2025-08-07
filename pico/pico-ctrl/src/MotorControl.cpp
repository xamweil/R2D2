#include "MotorControl.h"

#include <Arduino.h>
#include <RP2040_PWM.h>
#include <cstdint>

MotorControl::MotorControl(const MotorConfig &config)
    : enable_pin_(config.enable_pin), direction_pin_(config.direction_pin),
      pulse_pin_(config.pulse_pin),
      stepper_(config.pulse_pin, config.frequency, config.dutycycle,
               config.phaseCorrect) {
    pinMode(enable_pin_, OUTPUT);
    pinMode(direction_pin_, OUTPUT);
    // RP2024_PWM handles setup for pul_pin

    enable();
}

void MotorControl::setSpeed(int speed) {
    if (speed == 0) {
        stepper_.setPWM(pulse_pin_, 500, 0);
    } else {
        digitalWrite(direction_pin_, (speed < 0));
        stepper_.setPWM(pulse_pin_, abs(speed), 50);
    }
}

void MotorControl::move() { sample_move(); }

void MotorControl::sample_move() {
    setSpeed(1000);
    delay(3000);

    // Stop before reversing
    setSpeed(0);
    delay(3000);

    // Reversing
    setSpeed(-500);
    delay(3000);

    // Stop before reversing
    setSpeed(0);
    delay(3000);
}

void MotorControl::enable() { digitalWrite(enable_pin_, LOW); }
void MotorControl::disable() { digitalWrite(enable_pin_, HIGH); }
