#include "Nob.h"
#include <math.h> 

uint16_t Nob::degToTick(uint16_t deg) {
  // Arduino map: maps [0…180] → [SERVO_MIN…SERVO_MAX]
  return map(deg, 0, 180, SERVO_MIN, SERVO_MAX);
}

uint16_t Nob::tickToDeg(uint16_t tick) {
  // inverse mapping
  return map(tick, SERVO_MIN, SERVO_MAX, 0, 180);
}

Nob::Nob(Adafruit_PWMServoDriver &driver, uint8_t channelX, uint8_t channelY, uint16_t centreX, uint16_t centreY, uint16_t maxTiltDeg) :
_driver(driver), _channelX(channelX), _channelY(channelY){

  // compute centre in ticks
  _cenX = degToTick(centreX);
  _cenY = degToTick(centreY);

  uint16_t maxTickDelta = map(maxTiltDeg, 0, 180, 0, SERVO_MAX - SERVO_MIN);
  _rMax2 = uint32_t(maxTickDelta) * maxTickDelta;
  _rMax2 = uint32_t(maxTickDelta) * maxTickDelta;

  // initialize last pos at centre
  _x = _cenX;
  _y = _cenY;


}
uint8_t Nob::setPosX(int16_t dx) {
  // map user° → servo ticks
  uint16_t xTick = degToTick(tickToDeg(_cenX)+dx);

  // check new X against current Y
  if (!isWithinBounds(xTick, _y)) {
    return 0x21;  //Position is out of bounds, choose smaller values
  }

  // drive only X-axis servo
  _driver.setPWM(_channelX, 0, xTick);

  // store new X
  _x = xTick;

  return 0x00;
}

uint8_t Nob::setPosY(int16_t dy) {
  // map user° → servo ticks
  uint16_t yTick = degToTick(tickToDeg(_cenY)+dy);

  // check new Y against current X
  if (!isWithinBounds(_x, yTick)) {
    return 0x21;  //Position is out of bounds, choose smaller values
  }

  // drive only Y-axis servo
  _driver.setPWM(_channelY, 0, yTick);

  // store new Y
  _y = yTick;

  return 0x00;
}

uint8_t Nob::setPos(int16_t dx, int16_t dy){
  uint16_t xTick = degToTick(tickToDeg(_cenX)+dx);
  uint16_t yTick = degToTick(tickToDeg(_cenY)+dy);

  if (!isWithinBounds(xTick, yTick)) {
    return 0x21; //Position is out of bounds, choose smaller values
  }

   // actually drive the servos
  _driver.setPWM(_channelX, 0, xTick);
  _driver.setPWM(_channelY, 0, yTick);

  // store for getPos()
  _x = xTick;
  _y = yTick;

  return 0x00;
}

bool Nob::isWithinBounds(uint16_t x, uint16_t y) const {
  int32_t dx = int32_t(x) - int32_t(_cenX);
  int32_t dy = int32_t(y) - int32_t(_cenY);
  uint32_t dist2 = uint32_t(dx*dx) + uint32_t(dy*dy);
  return dist2 <= _rMax2;
}

uint8_t Nob::getPos() const {
  // back-map servo ticks → user angles
  uint16_t angleX = tickToDeg(_x);
  uint16_t angleY = tickToDeg(_y);
  Serial.println(String(angleX) + "°, " + String(angleY) + "°");
  return 0x00;
}

uint8_t Nob::runCircle() {
  uint16_t delayMs =30;
  uint16_t steps = 200;
  // radius in ticks
  float r = sqrt(float(_rMax2));

  for (uint16_t i = 0; i < steps; ++i) {
    // current angle [0…2π)
    float theta = 4.0f * PI * float(i) / float(steps);

    // compute each axis in servo‐ticks
    uint16_t xTick = _cenX + uint16_t(r * cos(theta));
    uint16_t yTick = _cenY + uint16_t(r * sin(theta));

    // drive both servos
    _driver.setPWM(_channelX, 0, xTick);
    _driver.setPWM(_channelY, 0, yTick);

    delay(delayMs);
  }
  delay(200);
  _driver.setPWM(_channelX, 0, 0);

  return 0x00;
}