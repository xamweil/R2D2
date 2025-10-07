#pragma once

#include "MotorControl.h"

#include <array>
#include <cstdint>

class SerialProcessor {
private:
    static constexpr uint8_t SOF_ = 0xAA;
    // Buttons (uint16) + Axes (NUM_AXES * float)
    static constexpr size_t BUF_SIZE_ = 2 + (NUM_AXES * 4);

    std::array<uint8_t, BUF_SIZE_> buffer_ = {};
    MotorControl motor_control_;

public:
    explicit SerialProcessor(const MotorControl &motor_control);

    void listen();
};
