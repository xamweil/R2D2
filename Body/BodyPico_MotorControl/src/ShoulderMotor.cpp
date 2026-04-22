#include "ShoulderMotor.h"

ShoulderMotor::ShoulderMotor(uint8_t enablePin,
                             uint8_t pulsePin,
                             uint8_t directionPin,
                             int32_t stepsPerRev,
                             MPUData& bodyMpu,
                             MPUData& legMpu,
                             int16_t minDeltaZ,
                             int16_t maxDeltaZ,
                             uint64_t maxMpuAgeMs)
    : MotorBase(enablePin, pulsePin, directionPin, stepsPerRev, 10.0f),
      _bodyMpu(bodyMpu),
      _legMpu(legMpu),
      _minDeltaZ(minDeltaZ),
      _maxDeltaZ(maxDeltaZ),
      _maxMpuAgeMs(maxMpuAgeMs) {}

void ShoulderMotor::setup() {
    _setupBase();
}

bool ShoulderMotor::_motionAllowed() {
    if (_bodyMpu.getAge() > _maxMpuAgeMs) return false;
    if (_legMpu.getAge() > _maxMpuAgeMs) return false;

    int32_t deltaZ =
        static_cast<int32_t>(_bodyMpu.data.accel_z) -
        static_cast<int32_t>(_legMpu.data.accel_z);

    if (deltaZ < _minDeltaZ) return false;
    if (deltaZ > _maxDeltaZ) return false;

    return true;
}

void ShoulderMotor::update() {
    uint64_t now = time_us_64();
    uint64_t dt_us = now - _lastUpdate;

    if (dt_us < 10000ULL) return;

    _stepUpdate(dt_us);
    _angleUpdate();

    if (!_motionAllowed()) {
        _brakeToStop();
        _applyMotorState();
        return;
    }

    _updateAngleControl();
    _applyMotorState();
}

void ShoulderMotor::_updateAngleControl() {
    float dirErrorDeg = _directionalAngleError(_direction, _currentAngle, _targetAngle);
    float shortestErr = _signedShortestAngleError(_currentAngle, _targetAngle);

    float remainingSteps = _angleToSteps(dirErrorDeg);
    float brakeSteps = _brakingSteps();

    uint32_t targetFreq = _velocityToFrequency(_targetVelocity);

    if (fabs(shortestErr) <= _angleTolerance && _frequency == 0) {
        _correctionMode = false;
        return;
    }

    bool targetBehind;
    if (_direction) {
        targetBehind = (shortestErr < 0.0f);
    } else {
        targetBehind = (shortestErr > 0.0f);
    }

    if (!_correctionMode && targetBehind) {
        _correctionMode = true;
    }

    if (_correctionMode) {
        if (_frequency > 0) {
            _brakeToStop();
            return;
        }

        _direction = (shortestErr > 0.0f);
        _correctionMode = false;
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