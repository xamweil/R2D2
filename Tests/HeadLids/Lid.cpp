#include "Lid.h"

Lid::Lid(Adafruit_PWMServoDriver &driver, uint8_t channel, uint16_t closePos, uint16_t openPos, bool isInverted) 
  : _driver(driver), _channel(channel), _closePos(closePos), _openPos(openPos), _isInverted(isInverted){
}

String Lid::openLid(){
  if (!_isInverted){
    _driver.setPWM(_channel, 0, _openPos);
  }
  else{
    _driver.setPWM(_channel, 0, _closePos);
  }
  delay(500);
  _driver.setPWM(_channel, 0, 0);
  return "Lid opened on channel: " + String(_channel);
}

String Lid::closeLid(){
  if (!_isInverted){
    _driver.setPWM(_channel, 0, _closePos);
  }
  else {
    _driver.setPWM(_channel, 0, _openPos);
  }
  delay(500);
  _driver.setPWM(_channel, 0, 0);
  return "Lid closed on channel: " + String(_channel);
}

String Lid::setOpenPos(uint16_t openPos) {
  if (!_isInverted){
    _openPos = map(openPos, 0, 180, 103, 512); 
  }
  else {
    _closePos = map(openPos, 0, 180, 103, 512);
  }
  openLid();
  return "Set open position on channel: " + String(_channel);
  
}

String Lid::setClosePos(uint16_t closePos) {
  if (!_isInverted){
    _closePos = map(closePos, 0, 180, 103, 512);
  }
  else {
    _openPos = map(closePos, 0, 180, 103, 512);
  }
  closeLid(); 
  return "Set close position on channel: " + String(_channel);
}

String Lid::getPositions(){
  Serial.print("Open position: ");
  Serial.println(map(_openPos, 103, 512, 0, 180));
  Serial.print("Close position: ");
  Serial.print(map(_closePos, 103, 512, 0, 180));
  return "";
}

