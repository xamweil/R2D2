#include "HeadMotor.h"

HeadMotor::HeadMotor(uint8_t enablePin, uint8_t pulsePin, uint8_t directionPin, int32_t stepsPerRev)
    : _enablePin(enablePin),
     _pulsePin(pulsePin), 
     _directionPin(directionPin), 
     _stepsPerRev(stepsPerRev),
     _enable(true),
     _direction(false),
     _lastUpdate(time_us_64()),
     pwm(pulsePin, 100.0f, 0.0f, false){
}

void HeadMotor::setTargetAngle(float targetAngle) {
    _targetAngle = targetAngle;
}
void HeadMotor::setTargetVelocity(uint8_t targetVelocity) {
    _targetVelocity = targetVelocity;
}
void HeadMotor::setEnabled(bool enabled) {
    _enable = !enabled;
}
void HeadMotor::setAngleMode(bool angleMode) {
    _angleMode = angleMode;
}
void HeadMotor::setDirection(bool direction) {
    _direction = direction;
}

int16_t HeadMotor::getTargetVelocity() const {
    return _targetVelocity;
}
int16_t HeadMotor::getCurrentAngle() const {
    return (int16_t)(_currentAngle * 100);
}
int16_t HeadMotor::getCurrentVelocity() const {
    return _currentVelocity;
}

void HeadMotor::setup() {
    pinMode(_enablePin, OUTPUT);
    pinMode(_pulsePin, OUTPUT);
    pinMode(_directionPin, OUTPUT);
    pinMode(_sensorPin, INPUT);
    digitalWrite(_enablePin, _enable ? HIGH : LOW); // Disable motor by default
    digitalWrite(_pulsePin, LOW);
    digitalWrite(_directionPin, _direction ? HIGH : LOW);
    delay(10);
}

void HeadMotor::_setMotorState() {
    digitalWrite(_enablePin, _enable ? HIGH : LOW);
    digitalWrite(_directionPin, _direction ? HIGH : LOW);

    if (_frequency == 0) {
        pwm.setPWM(_pulsePin, 1000.0f, 0.0f);  // set to 0 over duty cycle not frequency
        setEnabled(false); // Disable motor to reduce holding current
    } else {
        pwm.setPWM(_pulsePin, (float)_frequency, 50.0f);
        setEnabled(true);
    }

    _lastUpdate = time_us_64();
}

void HeadMotor::homing() {
    _frequency = 500; // Set a low speed for homing
    _direction = false; 
    setEnabled(true);
    _setMotorState();
    while (digitalRead(_sensorPin) == HIGH) {
        // Wait until the sensor is triggered
    }
    _currentAngle = 228;
    _targetAngle = 0;
    _targetVelocity = 50;

    _numSteps = 0;
    _numStepsOld = 0;
    _stepRemainder = 0.0f;
    _correctionMode = false;

    _direction = true;   
    _angleMode = true;
    _frequency = 0;
    _setMotorState();
    
}


void HeadMotor::update() {
    uint64_t now = time_us_64();
    uint64_t dt_us = now - _lastUpdate;

    if (dt_us < 10000ULL) return;

    _stepUpdate(dt_us);
    _angleUpdate();

    if (_angleMode) _updateAngleMode();
    else _updateVelocityMode();

    _setMotorState();
    _currentVelocity = _frequencyToVelocity(_frequency);
}

void HeadMotor::_updateVelocityMode() {
    uint32_t targetFreq = _velocityToFrequency(_targetVelocity);
    _rampFrequencyTo(targetFreq);
}

void HeadMotor::_updateAngleMode() {
    float dirErrorDeg = _directionalAngleError(_direction, _currentAngle, _targetAngle);
    float shortestErr = _signedShortestAngleError(_currentAngle, _targetAngle);

    float remainingSteps = _angleToSteps(dirErrorDeg);
    float brakeSteps = _brakingSteps();

    uint32_t targetFreq = _velocityToFrequency(_targetVelocity);

    // Close enough and essentially stopped
    if (fabs(shortestErr) <= _angleTolerance && _frequency == 0) {
        _correctionMode = false;
        return;
    }

    // If target is behind current motion direction enters correction logic
    bool targetBehind;
    if (_direction) { // left
        targetBehind = (shortestErr < 0.0f);
    } else {          // right
        targetBehind = (shortestErr > 0.0f);
    }

    if (!_correctionMode && targetBehind) {
        _correctionMode = true;
    }

    if (_correctionMode) {
        // First brake to zero
        if (_frequency > 0) {
            _brakeToStop();
            return;
        }

        // Once stopped, choose shortest correction direction
        _direction = (shortestErr > 0.0f);
        _correctionMode = false;
        return;
    }

    // Normal directed move: brake when needed
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


static float normalizeAngle(float a) {
    while (a < 0.0f) a += 360.0f;
    while (a >= 360.0f) a -= 360.0f;
    return a;
}

float HeadMotor::_directionalAngleError(bool direction, float current, float target) {
    current = normalizeAngle(current);
    target  = normalizeAngle(target);

    if (direction) { // left, angle increases
        return normalizeAngle(target - current);
    } else {         // right, angle decreases
        return normalizeAngle(current - target);
    }
}

float HeadMotor::_brakingSteps() {
    // update every 10 ms, _dfMax is Hz change per update
    return ((float)_frequency * (float)_frequency) / (2.0f * (100.0f * _dfMax));
}

float HeadMotor::_signedShortestAngleError(float current, float target) {
    float err = normalizeAngle(target) - normalizeAngle(current);
    while (err < -180.0f) err += 360.0f;
    while (err >= 180.0f) err -= 360.0f;
    return err;
}

void HeadMotor::_rampFrequencyTo(uint32_t targetFreq) {
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

void HeadMotor::_brakeToStop() {
    if (_frequency > _dfMax) _frequency -= _dfMax;
    else _frequency = 0;
}

void HeadMotor::_stepUpdate(uint64_t dt_us) {
    float stepsFloat = ((float)dt_us * (float)_frequency) / 1000000.0f + _stepRemainder;
    uint32_t wholeSteps = (uint32_t)stepsFloat;
    _stepRemainder = stepsFloat - wholeSteps;
    _numSteps += wholeSteps;
}

uint32_t HeadMotor::_velocityToFrequency(uint8_t velocity) {
    if (velocity > 100) velocity = 100;
    return (uint32_t)((_maxFrequency * (uint32_t)velocity) / 100UL);
}

uint8_t HeadMotor::_frequencyToVelocity(uint32_t frequency){
    return (uint8_t)((100UL * frequency) / _maxFrequency);
}
float HeadMotor::_angleToSteps(float angleDeg) {
    return angleDeg * _stepsPerRev * _transmissionRatio/ 360.0f;
}

void HeadMotor::_angleUpdate() {
    if (_frequency == 0) return;

    uint32_t deltaSteps = _numSteps - _numStepsOld;
    if (deltaSteps == 0) return;

    float deltaAngle = (float)deltaSteps * 360.0f / (_stepsPerRev*_transmissionRatio);

    if (_direction) {   // left: increasing angle
        _currentAngle += deltaAngle;
    } else {            // right: decreasing angle
        _currentAngle -= deltaAngle;
    }

    _currentAngle = normalizeAngle(_currentAngle);
    _numStepsOld = _numSteps;
}