#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#include <Arduino.h>
#include <RP2040_PWM.h>
#include <cstdint>

struct MotorConfig {
    uint8_t ena_pin;
    uint8_t dir_pin;
    uint8_t pul_pin;
};

struct MotorControl {
    RP2040_PWM servo_;
    uint8_t ena_pin_;
    uint8_t dir_pin_;
    uint8_t pul_pin_;

    MotorControl(const MotorConfig &config);

    void enable();
    void disable();

    void move();
    void sample_move();
};

#endif // MOTORCONTROL_H
