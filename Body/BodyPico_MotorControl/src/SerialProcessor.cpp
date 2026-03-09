#include "SerialProcessor.h"

#include "MotorControl.h"

#include <Arduino.h>

#include <cstddef>
#include <cstdint>

SerialProcessor::SerialProcessor(MotorControl *motor_control)
    : motor_control_(motor_control) {
}

void SerialProcessor::process() {
    switch (state_) {
    case State::WAIT_SOF:
        if (Serial.available() > 0) {
            uint8_t b = (uint8_t)Serial.read();
            if (b == SOF_) {
                state_ = State::WAIT_PACKET;
            }
        }
        break;

    case State::WAIT_PACKET:
        if (Serial.available() >= (int)buffer_.size()) {
            Serial.readBytes(buffer_.data(), buffer_.size());
            state_ = State::PROCESS;
        }
        break;

    case State::PROCESS:
        motor_control_->update(buffer_);
        Serial.println("PACKET PROCESSED!");
        state_ = State::WAIT_SOF;
        break;
    }
}
