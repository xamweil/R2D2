#pragma once

#include "MotorControl.h"

#include <array>
#include <cstdint>

class SerialProcessor {
private:
    static constexpr uint8_t SOF_ = 0xAA;

    std::array<uint8_t, MOTOR_COMMAND_FRAME_SIZE> buffer_ = {};
    MotorControl *motor_control_;

public:
    explicit SerialProcessor(MotorControl *motor_control);
    void listen();
};
