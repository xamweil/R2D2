#include "drive_control/DDSM115.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

DDSM115::DDSM115(uint8_t id)
    : id_(id),
      mode_(MODE_SPEED_LOOP),
      modeChangePending_(false),
      velocity_(0),
      current_(0),
      position_(0),
      errorCode_(0)
{
    std::memset(driveFrame_, 0, FRAME_SIZE);
    std::memset(modeFrame_, 0, FRAME_SIZE);
}

uint8_t DDSM115::id() const
{
    return id_;
}

uint8_t DDSM115::mode() const
{
    return mode_;
}

int16_t DDSM115::velocity() const
{
    return velocity_;
}

float DDSM115::current() const
{
    return static_cast<float>(current_)* 8.0f / 32767.0f;
}

float DDSM115::position() const
{
    return static_cast<float>(position_) * 360.0f / 32767.0f;
}

void DDSM115::init()
{
    setMode(MODE_SPEED_LOOP);
    brakeOn();
}

bool DDSM115::hasPendingModeChange() const
{
    return modeChangePending_;
}

const uint8_t* DDSM115::driveFrame() const
{
    return driveFrame_;
}

const uint8_t* DDSM115::modeFrame() const
{
    return modeFrame_;
}

void DDSM115::markModeChangeSent()
{
    modeChangePending_ = false;
}

bool DDSM115::setMode(uint8_t mode)
{
    if (mode != MODE_CURRENT_LOOP &&
        mode != MODE_SPEED_LOOP &&
        mode != MODE_POSITION_LOOP)
    {
        return false;
    }

    buildModeCommand(mode);

    mode_ = mode;
    modeChangePending_ = true;

    return true;
}

void DDSM115::buildModeCommand(uint8_t mode)
{
    std::memset(modeFrame_, 0, FRAME_SIZE);

    modeFrame_[0] = id_;
    modeFrame_[1] = CMD_MODE;
    modeFrame_[9] = mode;
}

bool DDSM115::setVelocity(int16_t rpm)
{
    if (mode_ != MODE_SPEED_LOOP)
    {
        return false;
    }

    buildVelocityCommand(rpm);
    return true;
}

void DDSM115::buildVelocityCommand(int16_t rpm)
{
    rpm = std::clamp<int16_t>(rpm, -MAX_RPM, MAX_RPM);

    std::memset(driveFrame_, 0, FRAME_SIZE);

    driveFrame_[0] = id_;
    driveFrame_[1] = CMD_DRIVE;

    driveFrame_[2] = static_cast<uint8_t>((rpm >> 8) & 0xFF);
    driveFrame_[3] = static_cast<uint8_t>(rpm & 0xFF);

    // driveFrame_[4] = 0
    // driveFrame_[5] = 0

    // Acceleration time
    driveFrame_[6] = 0x00;

    // Brake off while driving
    driveFrame_[7] = BRAKE_OFF;

    // driveFrame_[8] = 0

    driveFrame_[9] = crc8(driveFrame_, 9);
}

void DDSM115::stop()
{
    if (mode_ != MODE_SPEED_LOOP)
    {
        return;
    }

    buildVelocityCommand(0);
}

void DDSM115::brakeOn()
{
    if (mode_ != MODE_SPEED_LOOP)
    {
        return;
    }

    buildBrakeCommand(true);
}

void DDSM115::brakeOff()
{
    if (mode_ != MODE_SPEED_LOOP)
    {
        return;
    }

    buildBrakeCommand(false);
}

void DDSM115::buildBrakeCommand(bool enabled)
{
    std::memset(driveFrame_, 0, FRAME_SIZE);

    driveFrame_[0] = id_;
    driveFrame_[1] = CMD_DRIVE;

    // Velocity = 0
    driveFrame_[2] = 0x00;
    driveFrame_[3] = 0x00;

    driveFrame_[6] = 0x00;
    driveFrame_[7] = enabled ? BRAKE_ON : BRAKE_OFF;

    driveFrame_[9] = crc8(driveFrame_, 9);
}

bool DDSM115::setCurrent(int16_t current)
{
    if (mode_ != MODE_CURRENT_LOOP)
    {
        return false;
    }

    buildCurrentCommand(current);
    return true;
}

void DDSM115::buildCurrentCommand(int16_t current)
{
    // current = std::clamp<int16_t>(current, -MAX_CURRENT, MAX_CURRENT);

    std::memset(driveFrame_, 0, FRAME_SIZE);

    driveFrame_[0] = id_;
    driveFrame_[1] = CMD_DRIVE;

    driveFrame_[2] = static_cast<uint8_t>((current >> 8) & 0xFF);
    driveFrame_[3] = static_cast<uint8_t>(current & 0xFF);

    driveFrame_[7] = BRAKE_OFF;

    driveFrame_[9] = crc8(driveFrame_, 9);
}

bool DDSM115::setPosition(float angleDeg)
{
    if (mode_ != MODE_POSITION_LOOP)
    {
        return false;
    }

    buildPositionCommand(angleDeg);
    return true;
}

void DDSM115::buildPositionCommand(float angleDeg)
{
    angleDeg = std::fmod(angleDeg, 360.0f);

    if (angleDeg < 0.0f)
    {
        angleDeg += 360.0f;
    }

    const uint16_t rawPosition =
        static_cast<uint16_t>(
            std::lround((angleDeg / 360.0f) * 32767.0f)
        );

    std::memset(driveFrame_, 0, FRAME_SIZE);

    driveFrame_[0] = id_;
    driveFrame_[1] = CMD_DRIVE;

    driveFrame_[2] = static_cast<uint8_t>((rawPosition >> 8) & 0xFF);
    driveFrame_[3] = static_cast<uint8_t>(rawPosition & 0xFF);

    driveFrame_[9] = crc8(driveFrame_, 9);
}

uint8_t DDSM115::crc8(
    const uint8_t* data,
    std::size_t length)
{
    uint8_t crc = 0x00;

    for (std::size_t i = 0; i < length; ++i)
    {
        crc ^= data[i];

        for (uint8_t bit = 0; bit < 8; ++bit)
        {
            if (crc & 0x01)
            {
                crc = static_cast<uint8_t>(
                    (crc >> 1) ^ 0x8C
                );
            }
            else
            {
                crc >>= 1;
            }
        }
    }

    return crc;
}

bool DDSM115::processFeedback(
    const uint8_t* data,
    std::size_t length)
{
    if (data == nullptr || length != FRAME_SIZE)
    {
        return false;
    }

    if (data[0] != id_)
    {
        return false;
    }

    if (crc8(data, 9) != data[9])
    {
        return false;
    }

    mode_ = data[1];

    const uint16_t rawCurrent =
        (static_cast<uint16_t>(data[2]) << 8) |
        static_cast<uint16_t>(data[3]);

    current_ = static_cast<int16_t>(rawCurrent);

    const uint16_t rawVelocity =
        (static_cast<uint16_t>(data[4]) << 8) |
        static_cast<uint16_t>(data[5]);

    velocity_ = static_cast<int16_t>(rawVelocity);

    position_ =
        (static_cast<uint16_t>(data[6]) << 8) |
        static_cast<uint16_t>(data[7]);

    errorCode_ = data[8];

    return true;
}

uint8_t DDSM115::errorCode() const
{
    return errorCode_;
}