#pragma once

#include "HeadMotor.h"

#include <array>
#include <cstdint>



class SerialProcessor {
public:
    static constexpr size_t MOTOR_COUNT = 6;
    static constexpr size_t FRAME_SIZE = 34;

    SerialProcessor();

    void setup();
    void process();
    void updateMotors();

private:
    std::array<uint8_t, FRAME_SIZE> buffer_{};
    size_t index_ = 0;

    std::array<HeadMotor, MOTOR_COUNT> motors_;

    // Safety mask for development
    std::array<bool, MOTOR_COUNT> motor_allowed_ = {
        false,  // mid
        true,   // head
        false,  // shoulder_l
        false,  // shoulder_r
        false,  // drive_l
        false   // drive_r
    };

    void _handleFrame();
};