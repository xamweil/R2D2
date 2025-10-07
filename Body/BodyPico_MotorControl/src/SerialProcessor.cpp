#include "SerialProcessor.h"

#include <Arduino.h>

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstring>

SerialProcessor::SerialProcessor(const MotorControl &motor_control)
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

    uint16_t buttons_raw = 0;
    std::memcpy(&buttons_raw, buffer_.data(), sizeof(buttons_raw));
    auto &buttons = motor_control_.controller_state.buttons;
    for (size_t i = 0; i < buttons.size(); ++i) {
        buttons[i] = (buttons_raw & (1 << i)) != 0;
    }

    auto &axes = motor_control_.controller_state.axes;
    std::memcpy(axes.data(), &buffer_[sizeof(buttons_raw)],
                NUM_AXES * sizeof(axes[0]));

    motor_control_.update();
}
