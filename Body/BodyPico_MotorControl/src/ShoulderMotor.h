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

    void resetAngleAccumulator();
    void updateAngleAccumulator();
    bool angleMeanReady() const;
    float getAngleMeanDeg() const;
    bool angleConsistent() const;
    void tryHoming();
    bool isHomed() const;
    bool isMoving();

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

    static constexpr uint16_t MIN_ANGLE_SAMPLES = 10;

    bool _angleMeanReady = false;
    uint32_t _angleSampleCount = 0;

    uint32_t _lastAccumulatedBodyTs = 0;
    uint32_t _lastAccumulatedLegTs = 0;

    float _angleMeanDeg = 0.0f;
    bool _homed = false;
    uint8_t _homingAttemts = 0;
    float _prevError = 0;
    bool _homingActive = false;

    bool _correctionMode = false;
    float _angleTolerance = 1.0f;
};

#endif