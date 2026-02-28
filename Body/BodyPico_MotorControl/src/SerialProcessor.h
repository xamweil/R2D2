#pragma once

#include "MotorControl.h"

#include <array>
#include <cstdint>

class SerialProcessor {
private:
    static constexpr uint8_t SOF_ = 0xAA;

    enum class State { WAIT_SOF, WAIT_PACKET, PROCESS };

    std::array<uint8_t, MOTOR_COMMAND_FRAME_SIZE> buffer_ = {};
    MotorControl *motor_control_;
    State state_ = State::WAIT_SOF;

public:
    explicit SerialProcessor(MotorControl *motor_control);
    void process();
};
