#ifndef HEAD_MOTOR_H
#define HEAD_MOTOR_H

#include <Arduino.h>
#include <RP2040_PWM.h>
#include <math.h>
#include "pico/stdlib.h"

class HeadMotor {
    public:
        HeadMotor(uint8_t enablePin, uint8_t pulsePin, uint8_t directionPin, int32_t stepsPerRev);
        void setTargetAngle(float targetAngle);
        void setTargetVelocity(uint8_t targetVelocity);
        void setEnabled(bool enabled);
        void setAngleMode(bool angleMode);
        void setDirection(bool direction);

        int16_t getTargetAngle() const;
        int16_t getTargetVelocity() const;
        int16_t getCurrentAngle() const;
        int16_t getCurrentVelocity() const;

        void setup();
        void update();
        void homing();
        

    private:
        void _angleUpdate();
        void _setMotorState();
        void _stepUpdate(uint64_t dt_us);

        uint32_t _velocityToFrequency(uint8_t velocity);
        uint8_t _frequencyToVelocity(uint32_t frequency);
        
        float _directionalAngleError(bool direction, float current, float target);
        float _signedShortestAngleError(float current, float target);
        float _angleToSteps(float angleDeg);
        float _brakingSteps();

        void _updateVelocityMode();
        void _updateAngleMode();
        void _rampFrequencyTo(uint32_t targetFreq);
        void _brakeToStop();

        uint8_t _enablePin;
        uint8_t _pulsePin;
        uint8_t _directionPin;
        uint8_t _sensorPin = 28;

        RP2040_PWM pwm; 

        int32_t _stepsPerRev;

        uint64_t _lastUpdate;
        bool _enable;
        bool _direction;

        uint32_t _frequency = 0;

        float _targetAngle = 0; // deg
        uint8_t _targetVelocity = 0; // 0-100
        float _currentAngle = 0;   // deg
        uint8_t _currentVelocity = 0; // 0-100

        bool _angleMode = false;
        bool _correctionMode = false;

        float _angleTolerance = 1.0f;   // deg

        uint32_t _numSteps = 0; // number of steps since calculation started
        uint32_t _numStepsOld = 0; // number of steps at last update
        float _stepRemainder = 0.0f;
        float _transmissionRatio = 15.9525;

        uint32_t _dfMax = 50; //maximum Frequency step per time
        uint32_t _maxFrequency = 5000;
        uint32_t _minFrequency = 50;
};
#endif