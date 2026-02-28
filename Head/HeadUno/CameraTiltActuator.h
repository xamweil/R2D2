#ifndef CAMERATILTACTUATOR_H
#define CAMERATILTACTUATOR_H

#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>


class CameraTiltActuator {
    public: 
        CameraTiltActuator(Adafruit_PWMServoDriver &driver, uint8_t channel, uint16_t posDefault, 
        uint16_t posLow, uint16_t minPos, uint16_t maxPos);
        
        void update();
        uint8_t defaultPos();
        uint8_t lowPos();
        uint8_t setDefaultPos(uint16_t posDefault);
        uint8_t setLowPos(uint16_t posLow);
        uint8_t setPos(uint16_t pos);
        uint8_t getPos();

        private:
        bool _checkBound(uint16_t pos);

        bool _pendingOff = false;
        unsigned long _offAtMs = 0;
        uint16_t _minPos;
        uint16_t _maxPos;

        Adafruit_PWMServoDriver &_driver;
        uint8_t _channel;
        uint16_t _posDefault;
        uint16_t _posLow;
        uint16_t _pos;


};
#endif
