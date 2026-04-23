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
                float minRelativeAngleDeg,
                float maxRelativeAngleDeg,
                uint64_t maxMpuAgeMs = 100);

    void setup() override;
    void update() override;
    float getMinAngle() const;
    float getMaxAngle() const;

private:
    bool _motionAllowed();
    void _updateAngleControl();

    float _computeRelativeAngleDeg() const;
    bool _imuDataValid() const;

    MPUData& _bodyMpu;
    MPUData& _legMpu;

    float _minRelativeAngleDeg;
    float _maxRelativeAngleDeg;
    uint64_t _maxMpuAgeMs; // not in use yet, for later sanity checks. 

    bool _correctionMode = false;
    float _angleTolerance = 1.0f;
};

#endif