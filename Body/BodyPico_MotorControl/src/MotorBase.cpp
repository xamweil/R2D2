#include "MotorBase.h"

MotorBase::MotorBase(uint8_t enablePin,
                     uint8_t pulsePin,
                     uint8_t directionPin,
                     int32_t stepsPerRev,
                     float transmissionRatio)
    : _enablePin(enablePin),
      _pulsePin(pulsePin),
      _directionPin(directionPin),
      pwm(new RP2040_PWM(pulsePin, 100.0f, 0.0f, false)),
      _stepsPerRev(stepsPerRev),
      _transmissionRatio(transmissionRatio),
      _lastUpdate(time_us_64()) {}

void MotorBase::setTargetAngle(float targetAngle) {
    _targetAngle = _normalizeAngle(targetAngle);
}

void MotorBase::setTargetVelocity(uint8_t targetVelocity) {
    if (targetVelocity > 100) targetVelocity = 100;
    _targetVelocity = targetVelocity;
}

void MotorBase::setEnabled(bool enabled) {
    _enabled = enabled;
}

void MotorBase::setDirection(bool direction) {
    _direction = direction;
}

int16_t MotorBase::getTargetAngle() const {
    return static_cast<int16_t>(_targetAngle * 100.0f);
}

int16_t MotorBase::getTargetVelocity() const {
    return static_cast<int16_t>(_targetVelocity);
}

int16_t MotorBase::getCurrentAngle() const {
    return static_cast<int16_t>(_currentAngle * 100.0f);
}

int16_t MotorBase::getCurrentVelocity() const {
    return static_cast<int16_t>(_currentVelocity);
}

void MotorBase::_setupBase() {
    pinMode(_enablePin, OUTPUT);
    pinMode(_pulsePin, OUTPUT);
    pinMode(_directionPin, OUTPUT);

    // Adjust if your driver enable polarity is different.
    digitalWrite(_enablePin, _enabled ? LOW : HIGH); 
    digitalWrite(_pulsePin, LOW);
    digitalWrite(_directionPin, _direction ? HIGH : LOW);

    delay(10);
}

void MotorBase::_applyMotorState() {
    digitalWrite(_directionPin, _direction ? HIGH : LOW);

    if (!_enabled || _frequency == 0) {
        pwm->setPWM(_pulsePin, 1000.0f, 0.0f);
        digitalWrite(_enablePin, HIGH);   // disabled
    } else {
        pwm->setPWM(_pulsePin, static_cast<float>(_frequency), 50.0f);
        digitalWrite(_enablePin, LOW);    // enabled
    }

    _lastUpdate = time_us_64();
    _currentVelocity = _frequencyToVelocity(_frequency);
}

void MotorBase::_stepUpdate(uint64_t dt_us) {
    float stepsFloat =
        (static_cast<float>(dt_us) * static_cast<float>(_frequency)) / 1000000.0f
        + _stepRemainder;

    uint32_t wholeSteps = static_cast<uint32_t>(stepsFloat);
    _stepRemainder = stepsFloat - static_cast<float>(wholeSteps);
    _numSteps += wholeSteps;
}

void MotorBase::_angleUpdate() {
    if (_frequency == 0) return;

    uint32_t deltaSteps = _numSteps - _numStepsOld;
    if (deltaSteps == 0) return;

    float deltaAngle =
        static_cast<float>(deltaSteps) * 360.0f /
        (static_cast<float>(_stepsPerRev) * _transmissionRatio);

    if (_direction) {
        _currentAngle += deltaAngle;
    } else {
        _currentAngle -= deltaAngle;
    }

    _currentAngle = _normalizeAngle(_currentAngle);
    _numStepsOld = _numSteps;
}

uint32_t MotorBase::_velocityToFrequency(uint8_t velocity) const {
    if (velocity > 100) velocity = 100;
    return static_cast<uint32_t>(
        (static_cast<uint32_t>(_maxFrequency) * static_cast<uint32_t>(velocity)) / 100UL
    );
}

uint8_t MotorBase::_frequencyToVelocity(uint32_t frequency) const {
    return static_cast<uint8_t>((100UL * frequency) / _maxFrequency);
}

float MotorBase::_directionalAngleError(bool direction, float current, float target) const {
    current = _normalizeAngle(current);
    target = _normalizeAngle(target);

    if (direction) {
        return _normalizeAngle(target - current);
    } else {
        return _normalizeAngle(current - target);
    }
}

float MotorBase::_signedShortestAngleError(float current, float target) const {
    float err = _normalizeAngle(target) - _normalizeAngle(current);
    while (err < -180.0f) err += 360.0f;
    while (err >= 180.0f) err -= 360.0f;
    return err;
}

float MotorBase::_angleToSteps(float angleDeg) const {
    return angleDeg * static_cast<float>(_stepsPerRev) * _transmissionRatio / 360.0f;
}

float MotorBase::_brakingSteps() const {
    return (static_cast<float>(_frequency) * static_cast<float>(_frequency)) /
           (2.0f * (100.0f * static_cast<float>(_dfMax)));
}

void MotorBase::_rampFrequencyTo(uint32_t targetFreq) {
    if (_frequency < targetFreq) {
        uint32_t df = targetFreq - _frequency;
        if (df > _dfMax) df = _dfMax;
        _frequency += df;
    } else if (_frequency > targetFreq) {
        uint32_t df = _frequency - targetFreq;
        if (df > _dfMax) df = _dfMax;
        _frequency -= df;
    }
}

void MotorBase::_brakeToStop() {
    if (_frequency > _dfMax) _frequency -= _dfMax;
    else _frequency = 0;
}

float MotorBase::_normalizeAngle(float angle) {
    while (angle < 0.0f) angle += 360.0f;
    while (angle >= 360.0f) angle -= 360.0f;
    return angle;
}