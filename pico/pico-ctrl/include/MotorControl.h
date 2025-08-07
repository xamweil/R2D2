#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#include <Arduino.h>
#include <RP2040_PWM.h>
#include <cstdint>

struct MotorConfig {
    uint8_t enable_pin;
    uint8_t direction_pin;
    uint8_t pulse_pin;
    float frequency = 500;
    float dutycycle = 0;
    bool phaseCorrect = false;
};

struct MotorControl {
    RP2040_PWM stepper_;
    uint8_t enable_pin_;
    uint8_t direction_pin_;
    uint8_t pulse_pin_;

    MotorControl(const MotorConfig &config);

    void enable();
    void disable();

    void move();
    void sample_move();

    void setSpeed(int speed);
};

#endif // MOTORCONTROL_H
