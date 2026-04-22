#ifndef SHOULDER_MOTOR_H
#define SHOULDER_MOTOR_H

#include "MotorBase.h"
#include "MPUData.h"

class ShoulderMotor : public MotorBase {
public:
    ShoulderMotor(uint8_t enablePin,
                  uint8_t pulsePin,
                  uint8_t directionPin,
                  int32_t stepsPerRev,
                  MPUData& bodyMpu,
                  MPUData& legMpu,
                  int16_t minDeltaZ,
                  int16_t maxDeltaZ,
                  uint64_t maxMpuAgeMs = 100);

    void setup() override;
    void update() override;

private:
    bool _motionAllowed();
    void _updateAngleControl();

private:
    MPUData& _bodyMpu;
    MPUData& _legMpu;

    int16_t _minDeltaZ;
    int16_t _maxDeltaZ;
    uint64_t _maxMpuAgeMs;

    bool _correctionMode = false;
    float _angleTolerance = 1.0f;
};

#endif