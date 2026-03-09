#include "Lid.h"

Lid::Lid(Adafruit_PWMServoDriver &driver, uint8_t channel, uint16_t closePos, uint16_t openPos, bool isInverted) 
  : _driver(driver), _channel(channel), _closePos(closePos), _openPos(openPos), _isInverted(isInverted){
}

void Lid::update(){
    if (_pendingOff && (long)(millis() - _offAtMs) >=0){
        _driver.setPWM(_channel, 0, 0);
        _pendingOff = false;
    }
}
uint8_t Lid::openLid(){
  if (!_isInverted){
    _driver.setPWM(_channel, 0, _openPos);
  }
  else{
    _driver.setPWM(_channel, 0, _closePos);
  }
  _pendingOff = true;
  _offAtMs = millis() + 500;
  return 0x00;
}

uint8_t Lid::closeLid(){
  if (!_isInverted){
    _driver.setPWM(_channel, 0, _closePos);
  }
  else {
    _driver.setPWM(_channel, 0, _openPos);
  }
  _pendingOff = true;
  _offAtMs = millis() + 500;
  return 0x00;
}

uint8_t Lid::setOpenPos(uint16_t openPos) {
  if (!_isInverted){
    _openPos = map(openPos, 0, 180, 103, 512); 
  }
  else {
    _closePos = map(openPos, 0, 180, 103, 512);
  }
  openLid();
  return 0x00;
  
}

uint8_t Lid::setClosePos(uint16_t closePos) {
  if (!_isInverted){
    _closePos = map(closePos, 0, 180, 103, 512);
  }
  else {
    _openPos = map(closePos, 0, 180, 103, 512);
  }
  closeLid(); 
  return 0x00;
}

uint8_t Lid::getPositions(){
  Serial.print(F("Open position: "));
  Serial.println(map(_openPos, 103, 512, 0, 180));
  Serial.print(F("Close position: "));
  Serial.print(map(_closePos, 103, 512, 0, 180));
  return 0x00;
}

