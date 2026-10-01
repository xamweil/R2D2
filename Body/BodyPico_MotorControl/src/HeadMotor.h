#ifndef HEAD_MOTOR_H
#define HEAD_MOTOR_H

#include "MotorBase.h"

class HeadMotor : public MotorBase {
public:
    HeadMotor(uint8_t enablePin,
              uint8_t pulsePin,
              uint8_t directionPin,
              int32_t stepsPerRev,
              uint8_t sensorPin = 28);

    void setup() override;
    void update() override;

    void setAngleMode(bool angleMode);
    void homing();

private:
    void _updateVelocityMode();
    void _updateAngleMode();

private:
    uint8_t _sensorPin;
    bool _angleMode = false;
    bool _correctionMode = false;
    float _angleTolerance = 1.0f;
};

#endif