#ifndef MOTOR_BASE_H
#define MOTOR_BASE_H

#include <Arduino.h>
#include <RP2040_PWM.h>
#include <math.h>
#include "pico/stdlib.h"

class MotorBase {
public:
    MotorBase(uint8_t enablePin,
              uint8_t pulsePin,
              uint8_t directionPin,
              int32_t stepsPerRev,
              float transmissionRatio);

    virtual ~MotorBase() = default;

    virtual void setup() = 0;
    virtual void update() = 0;

    void setTargetAngle(float targetAngle);
    void setTargetVelocity(uint8_t targetVelocity);
    void setEnabled(bool enabled);
    void setDirection(bool direction);

    int16_t getTargetAngle() const;
    int16_t getTargetVelocity() const;
    int16_t getCurrentAngle() const;
    int16_t getCurrentVelocity() const;

protected:
    void _setupBase();
    void _applyMotorState();
    void _stepUpdate(uint64_t dt_us);
    void _angleUpdate();

    uint32_t _velocityToFrequency(uint8_t velocity) const;
    uint8_t _frequencyToVelocity(uint32_t frequency) const;

    float _directionalAngleError(bool direction, float current, float target) const;
    float _signedShortestAngleError(float current, float target) const;
    float _angleToSteps(float angleDeg) const;
    float _brakingSteps() const;

    void _rampFrequencyTo(uint32_t targetFreq);
    void _brakeToStop();

    static float _normalizeAngle(float angle);

    uint8_t _enablePin;
    uint8_t _pulsePin;
    uint8_t _directionPin;

    RP2040_PWM* pwm;

    int32_t _stepsPerRev;
    float _transmissionRatio;

    uint64_t _lastUpdate = 0;

    bool _enabled = false;
    bool _direction = false;   // true = left/increasing angle, false = right/decreasing angle

    uint32_t _frequency = 0;

    float _targetAngle = 0.0f;      // deg
    uint8_t _targetVelocity = 0;    // 0..100
    float _currentAngle = 0.0f;     // deg
    uint8_t _currentVelocity = 0;   // 0..100

    uint32_t _numSteps = 0;
    uint32_t _numStepsOld = 0;
    float _stepRemainder = 0.0f;

    uint32_t _dfMax = 50;
    uint32_t _maxFrequency = 5000;
    uint32_t _minFrequency = 50;
};

#endif