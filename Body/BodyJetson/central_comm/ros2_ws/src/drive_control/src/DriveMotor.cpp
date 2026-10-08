#include "drive_control/DriveMotor.hpp"

DriveMotor::DriveMotor(const DriveMotorConfig& config)
    : config_(config),
      motor_(config.motorId)
{
}

void DriveMotor::init()
{
    motor_.init();
}

bool DriveMotor::setMode(uint8_t mode)
{
    return motor_.setMode(mode);
}

bool DriveMotor::setVelocity(int16_t rpm)
{
    if (config_.inverted)
    {
        rpm = -rpm;
    }

    return motor_.setVelocity(rpm);
}

bool DriveMotor::setPosition(float angleDeg)
{
    if (config_.inverted)
    {
        angleDeg = -angleDeg;
    }

    return motor_.setPosition(angleDeg);
}

bool DriveMotor::setCurrent(int16_t current)
{
    if (config_.inverted)
    {
        current = -current;
    }

    return motor_.setCurrent(current);
}

void DriveMotor::brakeOn()
{
    motor_.brakeOn();
}

void DriveMotor::brakeOff()
{
    motor_.brakeOff();
}

void DriveMotor::stop()
{
    motor_.stop();
}

uint8_t DriveMotor::commandId() const
{
    return config_.commandId;
}

uint8_t DriveMotor::motorId() const
{
    return config_.motorId;
}

const char* DriveMotor::name() const
{
    return config_.name;
}

MotorGroup DriveMotor::group() const
{
    return config_.group;
}

bool DriveMotor::inverted() const
{
    return config_.inverted;
}

DDSM115& DriveMotor::ddsm()
{
    return motor_;
}

const DDSM115& DriveMotor::ddsm() const
{
    return motor_;
}