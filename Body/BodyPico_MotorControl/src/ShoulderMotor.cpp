#include "ShoulderMotor.h"

ShoulderMotor::ShoulderMotor(uint8_t enablePin,
                            uint8_t pulsePin,
                            uint8_t directionPin,
                            int32_t stepsPerRev,
                            MPUData& bodyMpu,
                            MPUData& legMpu,
                            float minRelativeAngleDeg,
                            float maxRelativeAngleDeg,
                            uint64_t maxMpuAgeMs)
    : MotorBase(enablePin, pulsePin, directionPin, stepsPerRev, 10.0f),
      _bodyMpu(bodyMpu),
      _legMpu(legMpu),
      _minRelativeAngleDeg(minRelativeAngleDeg),
      _maxRelativeAngleDeg(maxRelativeAngleDeg),
      _maxMpuAgeMs(maxMpuAgeMs) {}

void ShoulderMotor::setup() {
    _setupBase();
}

float ShoulderMotor::_computeRelativeAngleDeg() const {
    const float by = static_cast<float>(_bodyMpu.data.accel_y);
    const float bz = static_cast<float>(_bodyMpu.data.accel_z);

    const float ly = static_cast<float>(_legMpu.data.accel_y);
    const float lz = static_cast<float>(_legMpu.data.accel_z);

    const float bodyNorm = sqrtf(by * by + bz * bz);
    const float legNorm  = sqrtf(ly * ly + lz * lz);

    if (bodyNorm <= 1e-6f || legNorm <= 1e-6f) {
        return 180.0f;
    }

    const float dot = by * ly + bz * lz;
    const float det = by * lz - bz * ly;

    return atan2f(det, dot) * 180.0f / PI;
}

bool ShoulderMotor::_imuDataValid() const {
    if (_bodyMpu.data.t == 0) return false;
    if (_legMpu.data.t == 0) return false;
    return true;
}

bool ShoulderMotor::_motionAllowed() {
    if (!_imuDataValid()) return false;

    const float angleDeg = _computeRelativeAngleDeg();
    return angleDeg >= _minRelativeAngleDeg &&
           angleDeg <= _maxRelativeAngleDeg;
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