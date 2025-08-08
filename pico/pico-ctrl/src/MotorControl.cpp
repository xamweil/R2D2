#include "MotorControl.h"

#include <Arduino.h>
#include <RP2040_PWM.h>
#include <cstdint>

void manualMove(uint8_t direction_pin_, uint8_t pulse_pin_);

MotorControl::MotorControl(const MotorConfig &config)
    : enable_pin_(config.enable_pin), direction_pin_(config.direction_pin),
      pulse_pin_(config.pulse_pin) {

    // stepper_ = new RP2040_PWM(config.pulse_pin, config.frequency,
    // config.dutycycle, config.phaseCorrect);
    pinMode(pulse_pin_, OUTPUT); // RP2024_PWM handles setup for pulse_pin

    pinMode(enable_pin_, OUTPUT);
    pinMode(direction_pin_, OUTPUT);

    enable();
}

void MotorControl::setSpeed(int speed) {
    if (speed == 0) {
        stepper_->setPWM(pulse_pin_, 500, 0);
    } else {
        digitalWrite(direction_pin_, (speed < 0));
        stepper_->setPWM(pulse_pin_, abs(speed), 50);
    }
}

void MotorControl::move() {
    // sampleMove();
    manualMove(direction_pin_, pulse_pin_);
}

void MotorControl::sampleMove() {
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

void manualMove(uint8_t direction_pin_, uint8_t pulse_pin_) {
    digitalWrite(direction_pin_, HIGH);

    int Index = 0;

    for (Index = 0; Index < 5000; Index++) {
        digitalWrite(pulse_pin_, HIGH);
        delayMicroseconds(500);
        digitalWrite(pulse_pin_, LOW);
        delayMicroseconds(500);
    }
    delay(1000);

    digitalWrite(direction_pin_, LOW);

    for (Index = 0; Index < 5000; Index++) {
        digitalWrite(pulse_pin_, HIGH);
        delayMicroseconds(500);
        digitalWrite(pulse_pin_, LOW);
        delayMicroseconds(500);
    }
    delay(1000);
}

void MotorControl::enable() { digitalWrite(enable_pin_, LOW); }
void MotorControl::disable() { digitalWrite(enable_pin_, HIGH); }
