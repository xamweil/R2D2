#include "SerialProcessor.h"

#include "MotorControl.h"

#include <Arduino.h>

#include <cassert>
#include <cstddef>
#include <cstring>

SerialProcessor::SerialProcessor(MotorControl *motor_control)
    : motor_control_(motor_control) {
}

void SerialProcessor::listen() {
    while (true) {
        while (Serial.available() == 0) {
        }

        if (Serial.read() == SOF_)
            break;
    }

    while (Serial.available() < buffer_.size()) {
    }

    size_t bytes_read = Serial.readBytes(buffer_.data(), buffer_.size());
    if (bytes_read != buffer_.size()) {
        return;
    }

    motor_control_->update(buffer_);
    Serial.println("PACKET PROCESSED!");
}
