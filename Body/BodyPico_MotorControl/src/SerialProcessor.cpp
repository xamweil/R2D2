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
//
// Pin Layout
//
//                ENABLE   STEP(PULSE)   DIR
// M_mid          GP2      GP6           GP21
// M_head         GP3      GP7           GP20
// M_shoulder_l   GP4      GP8           GP19
// M_shoulder_r   GP4      GP9           GP18
//
// Notes
// - ENABLE pins are shared per motor group
// - LOW = motor enabled (driver logic)
// - STEP pins must support PWM
// - DIR pins are standard GPIO
// ============================================================================

SerialProcessor::SerialProcessor()
    : imus_{},
      motors_{
          HeadMotor(2, 6, 21, 200),                              // middleLeg (index 0)
          HeadMotor(3, 7, 20, 200 * 8),                          // head      (index 1)
          ShoulderMotor(4, 8, 19, 200, imus_[IMU_BODY], imus_[IMU_LEG_L_LEG], 35, -15), // left
          ShoulderMotor(4, 9, 18, 200, imus_[IMU_BODY], imus_[IMU_LEG_R_LEG], 35, -15)  // right
      },
      motor_ptrs_{
          &motors_.middleLeg,
          &motors_.head,
          &motors_.shoulderLeft,
          &motors_.shoulderRight
      }
{
}

void SerialProcessor::setup() {
    for (size_t i = 0; i < MOTOR_COUNT; ++i) {
        motor_ptrs_[i]->setup();
        motor_ptrs_[i]->setEnabled(false);
    }

    // Only head motor active for now
    if (motor_allowed_[1]) {
        motors_.head.setEnabled(true);
        motors_.head.homing();
    }

    _resetParser();
}

void SerialProcessor::_resetParser() {
    index_ = 0;
    expected_size_ = 0;
    active_sof_ = 0;
}

void SerialProcessor::process() {
    while (Serial.available() > 0) {
        const uint8_t b = static_cast<uint8_t>(Serial.read());

        // Waiting for SOF
        if (active_sof_ == 0) {
            if (b == MOTOR_SOF) {
                active_sof_ = MOTOR_SOF;
                expected_size_ = MOTOR_FRAME_SIZE;
                index_ = 0;
            } else if (b == IMU_SOF) {
                active_sof_ = IMU_SOF;
                expected_size_ = IMU_FRAME_SIZE;
                index_ = 0;
            } else {
                // Ignore junk until a valid SOF arrives
            }
            continue;
        }

        // Collect payload bytes
        if (index_ < buffer_.size()) {
            buffer_[index_] = b;
            ++index_;
        } else {
            // Should never happen, but recover cleanly
            _resetParser();
            continue;
        }

        // Frame complete
        if (index_ == expected_size_) {
            if (active_sof_ == MOTOR_SOF) {
                _handleMotorFrame();
            } else if (active_sof_ == IMU_SOF) {
                _handleImuFrame();
            }

            _resetParser();
        }
    }
}

void SerialProcessor::updateMotors() {
    for (size_t i = 0; i < MOTOR_COUNT; ++i) {
        if (!motor_allowed_[i]) {
            _stopMotorForSafety(i);
        }

        motor_ptrs_[i]->update();
    }
}

void SerialProcessor::_stopMotorForSafety(size_t motorIndex) {
    MotorBase* motor = motor_ptrs_[motorIndex];
    motor->setEnabled(false);
    motor->setTargetVelocity(0);

    // Only HeadMotor has angle/velocity mode switching
    if (motorIndex == 0) {
        motors_.middleLeg.setAngleMode(false);
    } else if (motorIndex == 1) {
        motors_.head.setAngleMode(false);
    }
}

void SerialProcessor::_handleMotorFrame() {
    const uint32_t control =
        static_cast<uint32_t>(buffer_[0]) |
        (static_cast<uint32_t>(buffer_[1]) << 8) |
        (static_cast<uint32_t>(buffer_[2]) << 16) |
        (static_cast<uint32_t>(buffer_[3]) << 24);

    // Only first 4 motors are handled on the Pico
    for (size_t i = 0; i < MOTOR_COUNT; ++i) {
        MotorBase* motor = motor_ptrs_[i];

        // Safety lock for development-disabled motors
        if (!motor_allowed_[i]) {
            _stopMotorForSafety(i);
            continue;
        }

        const uint32_t bits = (control >> (i * 4)) & 0x0F;

        const bool enable       = (bits & (1U << 0)) != 0;
        const bool direction    = (bits & (1U << 1)) != 0;
        const bool angle_set    = (bits & (1U << 2)) != 0;
        const bool velocity_set = (bits & (1U << 3)) != 0;

        motor->setEnabled(enable);
        motor->setDirection(direction);

        const size_t off = 4 + (i * 5);

        float angle = 0.0f;
        std::memcpy(&angle, &buffer_[off], sizeof(float));

        uint8_t velocity = buffer_[off + 4];
        if (velocity > 100) {
            velocity = 100;
        }

        // Mid + head are HeadMotor and support angle mode / velocity mode
        if (i == 0 || i == 1) {
            HeadMotor* hm = (i == 0) ? &motors_.middleLeg : &motors_.head;

            if (angle_set) {
                hm->setAngleMode(true);
                hm->setTargetAngle(angle);

                if (velocity_set) {
                    hm->setTargetVelocity(velocity);
                } else {
                    hm->setTargetVelocity(50);
                }
            } else if (velocity_set) {
                hm->setAngleMode(false);
                hm->setTargetVelocity(velocity);
            } else {
                hm->setAngleMode(false);
                hm->setTargetVelocity(0);
            }
        }
        // Shoulders are always angle-controlled
        else {
            ShoulderMotor* sm = (i == 2) ? &motors_.shoulderLeft
                                         : &motors_.shoulderRight;

            if (angle_set) {
                sm->setTargetAngle(angle);

                if (velocity_set) {
                    sm->setTargetVelocity(velocity);
                } else {
                    sm->setTargetVelocity(50);
                }
            } else {
                // velocity-only makes no sense for shoulders
                sm->setTargetVelocity(0);
            }
        }
    }
}

void SerialProcessor::_handleImuFrame() {
    // Frame layout:
    // 6 IMUs, each 10 bytes:
    // int16 ax, int16 ay, int16 az, uint32 ts_ms

    for (size_t i = 0; i < IMU_COUNT; ++i) {
        const size_t off = i * 10;

        const int16_t accel_x = static_cast<int16_t>(
            static_cast<uint16_t>(buffer_[off + 0]) |
            (static_cast<uint16_t>(buffer_[off + 1]) << 8));

        const int16_t accel_y = static_cast<int16_t>(
            static_cast<uint16_t>(buffer_[off + 2]) |
            (static_cast<uint16_t>(buffer_[off + 3]) << 8));

        const int16_t accel_z = static_cast<int16_t>(
            static_cast<uint16_t>(buffer_[off + 4]) |
            (static_cast<uint16_t>(buffer_[off + 5]) << 8));

        const uint32_t ts_ms =
            static_cast<uint32_t>(buffer_[off + 6]) |
            (static_cast<uint32_t>(buffer_[off + 7]) << 8) |
            (static_cast<uint32_t>(buffer_[off + 8]) << 16) |
            (static_cast<uint32_t>(buffer_[off + 9]) << 24);

        // ts_ms == 0 means invalid / missing / stale
        imus_[i].setData(accel_x, accel_y, accel_z, ts_ms);
    }
}