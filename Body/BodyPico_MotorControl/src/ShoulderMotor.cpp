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

bool ShoulderMotor::isHomed() const {
    return _homed;
}

bool ShoulderMotor::isMoving(){
    return _frequency != 0;
}

void ShoulderMotor::tryHoming() {
    constexpr float HOME_ACCEPT_ERROR_DEG = 1.0f;
    constexpr float HOME_IMPROVEMENT_EPS_DEG = 1.0f;
    constexpr uint8_t HOME_VELOCITY = 20;

    if (!_imuDataValid() || !angleMeanReady()) return;

    float measuredAngle = getAngleMeanDeg();

    const float anchorA = 0.0f;
    const float anchorB = _normalizeAngle(_minRelativeAngleDeg);

    const float homeTarget =
        (fabs(_normalizeAngle(measuredAngle) - anchorA) <= fabs(_normalizeAngle(measuredAngle) - anchorB))
            ? anchorA
            : anchorB;

    float angleError = _signedShortestAngleError(measuredAngle, homeTarget);
    
    if (fabs(angleError) <= HOME_ACCEPT_ERROR_DEG) {
        _currentAngle = homeTarget;
        _targetAngle = homeTarget;
        _numSteps = 0;
        _numStepsOld = 0;
        _stepRemainder = 0.0f;
        _frequency = 0;
        _correctionMode = false;
        _homed = true;
        setEnabled(false);
        _homingActive = false;
        _homingAttemts = 0;
        return;
    }

    else if (fabs(angleError) > fabs(_minRelativeAngleDeg)) {
        while (true) {
            Serial.println("SHOULDER HOMING ERROR: correction too large");
            delay(1000);
        }
    }

    else if (_homingAttemts > 0){
        if (fabs(angleError) > fabs(_prevError) + HOME_IMPROVEMENT_EPS_DEG) {
            while (true) {
                Serial.println("SHOULDER HOMING ERROR: correction worsened angle");
                delay(1000);
            }
        } 
    }    
    _prevError = angleError;

    _currentAngle = measuredAngle;
    _targetAngle = homeTarget;
    _targetVelocity = HOME_VELOCITY;
    _correctionMode = false;
    setEnabled(true);
    _homingActive = true;
    _homingAttemts += 1;
    
}

float ShoulderMotor::getMinAngle() const {
    return _minRelativeAngleDeg;
}

float ShoulderMotor::getMaxAngle() const {
    return _maxRelativeAngleDeg;
}

void ShoulderMotor::resetAngleAccumulator() {
    _angleMeanReady = false;
    _angleSampleCount = 0;
    _lastAccumulatedBodyTs = 0;
    _lastAccumulatedLegTs = 0;
    _angleMeanDeg = 0.0f;
}

void ShoulderMotor::updateAngleAccumulator() {
    if (!_imuDataValid()) return;

    const uint32_t bodyTs = _bodyMpu.data.t;
    const uint32_t legTs = _legMpu.data.t;

    // Only use fresh sensor pairs.
    if (bodyTs == _lastAccumulatedBodyTs &&
        legTs == _lastAccumulatedLegTs) {
        return;
    }

    _lastAccumulatedBodyTs = bodyTs;
    _lastAccumulatedLegTs = legTs;

    const float angleDeg = _computeRelativeAngleDeg();

    // Running mean:
    // mean_n = mean_(n-1) + (x - mean_(n-1)) / n
    if (_angleSampleCount < UINT32_MAX) {
        ++_angleSampleCount;
    } else {
        // Prevent overflow. Keep current mean and continue as if window is very large.
        _angleSampleCount = MIN_ANGLE_SAMPLES;
    }

    _angleMeanDeg += (angleDeg - _angleMeanDeg) /
                     static_cast<float>(_angleSampleCount);

    if (_angleSampleCount >= MIN_ANGLE_SAMPLES) {
        _angleMeanReady = true;
    }
}

bool ShoulderMotor::angleMeanReady() const {
    return _angleMeanReady;
}

float ShoulderMotor::getAngleMeanDeg() const {
    return _normalizeAngle(-_angleMeanDeg);
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
    if (_homingActive) return true;

    if (angleConsistent()) return true;

    const bool currentAngleInRange =
        (_currentAngle >= _normalizeAngle(_minRelativeAngleDeg)) ||
        (_currentAngle <= _normalizeAngle(_maxRelativeAngleDeg));

    if (_frequency != 0 && currentAngleInRange) return true;

    if (_angleSampleCount > 20) {
        _currentAngle = getAngleMeanDeg();
        return true;
    }

    return false;
}

bool ShoulderMotor::angleConsistent() const {
    if (!_imuDataValid()) return false;
    if (!angleMeanReady()) return false;

    constexpr float MAX_ANGLE_DISAGREEMENT_DEG = 2.0f;

    const float err = _signedShortestAngleError(_currentAngle, getAngleMeanDeg());
    return fabs(err) <= MAX_ANGLE_DISAGREEMENT_DEG;
}

void ShoulderMotor::update() {
    uint64_t now = time_us_64();
    uint64_t dt_us = now - _lastUpdate;

    if (dt_us < 10000ULL) return;
    if (_frequency==0) updateAngleAccumulator();
    else if (_angleSampleCount != 0) resetAngleAccumulator();
    
    if (!_homed && _frequency == 0){
        tryHoming();
    }
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