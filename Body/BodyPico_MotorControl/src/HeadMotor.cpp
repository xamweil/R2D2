#include "HeadMotor.h"

HeadMotor::HeadMotor(uint8_t enablePin,
                     uint8_t pulsePin,
                     uint8_t directionPin,
                     int32_t stepsPerRev,
                     uint8_t sensorPin)
    : MotorBase(enablePin, pulsePin, directionPin, stepsPerRev, 9.954545454f),
      _sensorPin(sensorPin) {}

void HeadMotor::setAngleMode(bool angleMode) {
    _angleMode = angleMode;
}

void HeadMotor::setup() {
    _setupBase();
    pinMode(_sensorPin, INPUT);
}

void HeadMotor::homing() {
    _frequency = 500;
    _direction = false;
    _enabled = true;
    _applyMotorState();

    while (digitalRead(_sensorPin) == HIGH) {
        // wait for home sensor
    }

    _currentAngle = 228.0f;
    _targetAngle = 0.0f;
    _targetVelocity = 50;

    _numSteps = 0;
    _numStepsOld = 0;
    _stepRemainder = 0.0f;
    _correctionMode = false;

    _direction = true;
    _angleMode = true;
    _frequency = 0;
    _applyMotorState();
}

void HeadMotor::update() {
    uint64_t now = time_us_64();
    uint64_t dt_us = now - _lastUpdate;

    if (dt_us < 10000ULL) return;

    _stepUpdate(dt_us);
    _angleUpdate();

    if (_angleMode) _updateAngleMode();
    else _updateVelocityMode();

    _applyMotorState();
}

void HeadMotor::_updateVelocityMode() {
    uint32_t targetFreq = _velocityToFrequency(_targetVelocity);
    _rampFrequencyTo(targetFreq);
}

void HeadMotor::_updateAngleMode() {
    float shortestErr = _signedShortestAngleError(_currentAngle, _targetAngle);

    bool desiredDirection = (shortestErr > 0.0f);

    // if moving and desired direction changed -> always brake first
    if (_frequency > 0 && desiredDirection != _direction) {
        _brakeToStop();
        return;
    }

    // once stopped, adopt new direction
    if (_frequency == 0) {
        _direction = desiredDirection;
    }

    float dirErrorDeg = _directionalAngleError(_direction, _currentAngle, _targetAngle);
    float remainingSteps = _angleToSteps(dirErrorDeg);
    float brakeSteps = _brakingSteps();
    uint32_t targetFreq = _velocityToFrequency(_targetVelocity);

    if (fabs(shortestErr) <= _angleTolerance && _frequency == 0) {
        return;
    }

    if (remainingSteps <= _angleToSteps(_angleTolerance)) {
        _brakeToStop();
        return;
    }

    if (remainingSteps <= brakeSteps) {
        _brakeToStop();
    } else {
        _rampFrequencyTo(targetFreq);
        if (_frequency > 0 && _frequency < _minFrequency) {
            _frequency = _minFrequency;
        }
    }
}