#ifndef DRIVE_MOTOR_HPP
#define DRIVE_MOTOR_HPP
/*
* |     Motor       |   ID  |   Note
* | Left foot back  |   11  |   
* | Left foot middle|   12  | No motor yet, maybe in future
* | Left foot front |   13  | 
* | Middle foot back|   13  | No motor yet, maybe in future
* |Middle foot front|   13  | No motor yet, probably in future
* | Right foot back |   15  |   
* |Right foot middle|   16  | No motor yet, maybe in future
* | Right foot front|   17  | 
*/

#include "DDSM115.hpp"

#include <cstdint>

enum class MotorGroup : uint8_t
{
    LEFT_FOOT,
    MIDDLE_FOOT,
    RIGHT_FOOT
};

struct DriveMotorConfig
{
    uint8_t commandId;      // ID used by ROS MotorCommand
    uint8_t motorId;        // Physical DDSM115 RS485 ID

    const char* name;
    MotorGroup group;

    bool inverted;
};

class DriveMotor
{
    public:
        explicit DriveMotor(const DriveMotorConfig& config);

        void init();

        bool setMode(uint8_t mode);

        bool setVelocity(int16_t rpm);
        bool setPosition(float angleDeg);
        bool setCurrent(int16_t current);

        void brakeOn();
        void brakeOff();
        void stop();

        // Metadata
        uint8_t commandId() const;
        uint8_t motorId() const;
        const char* name() const;
        MotorGroup group() const;
        bool inverted() const;

        // Access to the underlying motor
        DDSM115& ddsm();
        const DDSM115& ddsm() const;

    private:
        DriveMotorConfig config_;
        DDSM115 motor_;
};


// -----------------------------------------------------------------------------
// Installed motors
// -----------------------------------------------------------------------------
//
// commandId = ID used by higher-level ROS commands
// motorId   = physical DDSM115 RS485 address
//
// Adding/removing a physical motor should only require changing this table.
//
// ROS command ID / RS485 motor ID / name / group / inverted
static constexpr DriveMotorConfig DRIVE_MOTOR_CONFIGS[] =
{
    // Left foot
    {11, 11, "left_foot_back",  MotorGroup::LEFT_FOOT, false},
    {13, 13, "left_foot_front", MotorGroup::LEFT_FOOT, false},

    // Middle foot
    // Add once physical motors and unique RS485 IDs are assigned.

    // Right foot
    {15, 15, "right_foot_back",  MotorGroup::RIGHT_FOOT, false},
    {17, 17, "right_foot_front", MotorGroup::RIGHT_FOOT, false},
};
#endif