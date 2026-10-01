#pragma once

#include "MotorBase.h"
#include "HeadMotor.h"
#include "ShoulderMotor.h"
#include "MPUData.h"

#include <array>
#include <cstddef>
#include <cstdint>

struct Motors
{
    // Order matches motor index mapping:
    // 0 = middleLeg
    // 1 = head
    // 2 = shoulderLeft
    // 3 = shoulderRight
    HeadMotor middleLeg;
    HeadMotor head;
    ShoulderMotor shoulderLeft;
    ShoulderMotor shoulderRight;
};

class SerialProcessor {
    public:
        static constexpr size_t MOTOR_COUNT = 4;
        static constexpr size_t IMU_COUNT = 6;

        static constexpr uint8_t MOTOR_SOF = 0xAA;
        static constexpr uint8_t IMU_SOF   = 0xAB;

        static constexpr size_t MOTOR_FRAME_SIZE = 34;  // payload only, after SOF
        static constexpr size_t IMU_FRAME_SIZE   = 60;  // 6 * (ax, ay, az, ts_ms) = 6 * 10
        static constexpr size_t MAX_FRAME_SIZE   = IMU_FRAME_SIZE;

        enum ImuIndex : size_t {
        IMU_HEAD = 0,
        IMU_BODY = 1,
        IMU_LEG_L_FOOT = 2,
        IMU_LEG_L_LEG  = 3,
        IMU_LEG_R_FOOT = 4,
        IMU_LEG_R_LEG  = 5,
    };

        SerialProcessor();

        void setup();
        void process();
        void updateMotors();

    private:
        std::array<uint8_t, MAX_FRAME_SIZE> buffer_{};
        size_t index_ = 0;
        size_t expected_size_ = 0;
        uint8_t active_sof_ = 0;

        std::array<MPUData, IMU_COUNT> imus_{};

        Motors motors_;
        std::array<MotorBase*, MOTOR_COUNT> motor_ptrs_{};

        // Safety mask for development
        std::array<bool, MOTOR_COUNT> motor_allowed_ = {
            false,  // mid
            true,   // head
            true,  // shoulder_l
            true,  // shoulder_r
        };

        void _resetParser();
        void _stopMotorForSafety(size_t motorIndex);

        void _handleMotorFrame();
        void _handleImuFrame();
};