#include "CameraTiltActuator.h"


CameraTiltActuator::CameraTiltActuator(Adafruit_PWMServoDriver &driver, uint8_t channel, uint16_t posDefault, uint16_t posLow,
                uint16_t minPos, uint16_t max_pos): 
                _driver(driver), _channel(channel), _posDefault(posDefault), _posLow(posLow), _pos(map(posDefault, 103, 512, 0, 180)),
                _minPos(minPos), _maxPos(max_pos){}

void CameraTiltActuator::update(){
    if (_pendingOff && (long)(millis() - _offAtMs) >= 0){
        _driver.setPWM(_channel, 0, 0);
        _pendingOff = false;
    }
}

uint8_t CameraTiltActuator::defaultPos(){
    _driver.setPWM(_channel, 0, _posDefault);
    _pendingOff = true;
    _offAtMs = millis() + 500;
    _pos = map(_posDefault, 103, 512, 0, 180);
    return 0x00;
}

uint8_t CameraTiltActuator::lowPos(){
    _driver.setPWM(_channel, 0, _posLow);
    _pendingOff = true;
    _offAtMs = millis() + 500;
    _pos = map(_posLow, 103, 512, 0, 180);
    return 0x00;
}

uint8_t CameraTiltActuator::setPos(uint16_t pos){
    if (!_checkBound(pos)) return 0x31;
    _driver.setPWM(_channel, 0, map(pos, 0, 180, 103, 512));
    _pendingOff = true;
    _offAtMs = millis() + 500;
    _pos = pos;
    return 0x00;
}

uint8_t CameraTiltActuator::getPos(){
    Serial.println(_pos);
    return 0x00;
}

uint8_t CameraTiltActuator::setDefaultPos(uint16_t posDefault){
    if (!_checkBound(posDefault)) return 0x31;
    _posDefault = map(posDefault, 0, 180, 103, 512);
    defaultPos();
    return 0x00;
}

uint8_t CameraTiltActuator::setLowPos(uint16_t posLow){
    if (!_checkBound(posLow)) return 0x31;
    _posLow = map(posLow, 0, 180, 103, 512);
    lowPos();
    return 0x00;
}

bool CameraTiltActuator::_checkBound(uint16_t pos){
    if (pos < _minPos || pos > _maxPos){
        return false;
    }
    else return true;
}