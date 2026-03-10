#include "SerialProcessor.h"

#include <Arduino.h>
#include <cstring>
// ============================================================================
// MOTOR PIN MAP (RP2040 Pico)
//
// Motor Index Mapping
// 0 : M_mid
// 1 : M_head
// 2 : M_shoulder_l
// 3 : M_shoulder_r
// 4 : M_drive_l
// 5 : M_drive_r
//
// Pin Layout
//
//                ENABLE   STEP(PULSE)   DIR
// M_mid          GP2      GP6           GP21
// M_head         GP3      GP7           GP20
// M_shoulder_l   GP4      GP8           GP19
// M_shoulder_r   GP4      GP9           GP18
// M_drive_l      GP5      GP10          GP17
// M_drive_r      GP5      GP11          GP16
//
// Notes
// - ENABLE pins are shared per motor group
// - LOW = motor enabled (driver logic)
// - STEP pins must support PWM
// - DIR pins are standard GPIO
// ============================================================================
SerialProcessor::SerialProcessor()
    : motors_{
          HeadMotor(2, 6, 21, 200),  // mid
          HeadMotor(3, 7, 20, 200 * 8),  // head
          HeadMotor(4, 8, 19, 200),  // shoulder_l
          HeadMotor(4, 9, 18, 200),  // shoulder_r
          HeadMotor(5, 10, 17, 200), // drive_l
          HeadMotor(5, 11, 16, 200)  // drive_r
      } {}

void SerialProcessor::setup() {

    for (size_t i = 0; i < MOTOR_COUNT; ++i) {
        motors_[i].setup();
        motors_[i].setEnabled(false);
    }

    // only head motor active
    motors_[1].setEnabled(true);
    motors_[1].homing();
}

void SerialProcessor::process() {

    while (Serial.available() > 0) {

        buffer_[index_++] = (uint8_t)Serial.read();

        if (index_ < FRAME_SIZE)
            continue;

        _handleFrame();

        index_ = 0;
    }
}

void SerialProcessor::updateMotors()
{
    for (size_t i = 0; i < MOTOR_COUNT; ++i)
    {
        if (!motor_allowed_[i])
        {
            motors_[i].setEnabled(false);
            motors_[i].setTargetVelocity(0);
        }

        motors_[i].update();
    }
}

void SerialProcessor::_handleFrame() {

    uint32_t control =
        (uint32_t)buffer_[0] |
        ((uint32_t)buffer_[1] << 8) |
        ((uint32_t)buffer_[2] << 16) |
        ((uint32_t)buffer_[3] << 24);

    for (size_t i = 0; i < MOTOR_COUNT; ++i) {

        HeadMotor &motor = motors_[i];

        uint32_t bits = (control >> (i * 4)) & 0x0F;

        bool enable = bits & (1 << 0);
        bool direction = bits & (1 << 1);
        bool angle_set = bits & (1 << 2);
        bool velocity_set = bits & (1 << 3);

        motor.setEnabled(enable);
        motor.setDirection(direction);

        size_t off = 4 + (i * 5);

        float angle;
        std::memcpy(&angle, &buffer_[off], sizeof(float));

        uint8_t velocity = buffer_[off + 4];
        if (velocity > 100)
            velocity = 100;

        if (angle_set) {
            motor.setAngleMode(true);
            motor.setTargetAngle(angle);
        } else {
            motor.setAngleMode(false);
        }

        if (velocity_set) {
            motor.setTargetVelocity(velocity);
        }
    }
}