#include <Arduino.h>
#include <cstdint>
#include "MotorControl.h"

MotorControl::MotorControl(const MotorConfig &config)
    : ena_pin_(config.ena_pin), dir_pin_(config.dir_pin), pul_pin_(config.pul_pin), servo_(5, 5, 5) {
    pinMode(ena_pin_, OUTPUT);
    pinMode(dir_pin_, OUTPUT);
    pinMode(pul_pin_, OUTPUT);
    enable();
}

void MotorControl::move() { sample_move(); }

void MotorControl::sample_move() {
    uint16_t index = 0;

    digitalWrite(dir_pin_, HIGH);

    Serial.println("FIRST LOOP\n");

    for (index = 0; index < 5000; index++) {
        digitalWrite(pul_pin_, HIGH);
        delayMicroseconds(500);
        digitalWrite(pul_pin_, LOW);
        delayMicroseconds(500);
    }
    delay(1000);

    digitalWrite(dir_pin_, LOW);

    Serial.print("SECOND LOOP\n");

    for (index = 0; index < 5000; index++) {
        digitalWrite(pul_pin_, HIGH);
        delayMicroseconds(500);
        digitalWrite(pul_pin_, LOW);
        delayMicroseconds(500);
    }
    delay(1000);
}

void MotorControl::enable() { digitalWrite(ena_pin_, LOW); }
void MotorControl::disable() { digitalWrite(ena_pin_, HIGH); }
