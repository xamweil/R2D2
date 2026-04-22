#pragma once

#include "HeadMotor.h"
#include "ShoulderMotor.h"

#include <array>
#include <cstdint>

struct Motors 
{
    HeadMotor head;
    ShoulderMotor shoulderLeft;
    ShoulderMotor shoulderRight;
    HeadMotor middleLeg;

    
};

class SerialProcessor {
    public:
        static constexpr size_t MOTOR_COUNT = 4;
        static constexpr size_t FRAME_SIZE = 34;

        SerialProcessor();

        void setup();
        void process();
        void updateMotors();

    private:
        std::array<uint8_t, FRAME_SIZE> buffer_{};
        size_t index_ = 0;

        Motors motors_;

        // Safety mask for development
        std::array<bool, MOTOR_COUNT> motor_allowed_ = {
            false,  // mid
            true,   // head
            false,  // shoulder_l
            false,  // shoulder_r
        };

        void _handleFrame();
};