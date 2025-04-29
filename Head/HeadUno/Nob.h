#ifndef NOB_H
#define NOB_H

#include <Adafruit_PWMServoDriver.h>

class Nob {
  public:
  Nob(Adafruit_PWMServoDriver &driver, uint8_t channelX, uint8_t channelY, uint16_t centreX, uint16_t centreY, uint16_t maxTiltDeg);

    uint8_t setPos(int16_t dx, int16_t dy);
    uint8_t getPos() const;
    uint8_t setPosX(int16_t dx);
    uint8_t setPosY(int16_t dy);
    uint8_t runCircle();


  private:
    Adafruit_PWMServoDriver &_driver;

    static constexpr uint16_t SERVO_MIN    = 103;   ///< tick for 0°
    static constexpr uint16_t SERVO_MAX    = 512;   ///< tick for 180°
    uint8_t _channelX, _channelY;
    uint16_t _cenX, _cenY; //centre in servo‐ticks
    uint16_t _x, _y;  // Last position in servo‐ticks
    uint16_t _rMax2;  //squared max‐radius in servo‐ticks

    bool isWithinBounds(uint16_t newX, uint16_t newY) const;
    /// map degrees → ticks
    static uint16_t degToTick(uint16_t deg);
    /// map ticks → degrees
    static uint16_t tickToDeg(uint16_t tick);


};
#endif