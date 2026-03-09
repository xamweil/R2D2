#ifndef LID_H
#define LID_H

#include <Adafruit_PWMServoDriver.h>

class Lid {
  public:
    Lid(Adafruit_PWMServoDriver &driver, uint8_t channel, uint16_t closePos, uint16_t openPos, bool _isInverted);

    uint8_t openLid();
    uint8_t closeLid();
    uint8_t setOpenPos(uint16_t openPos);
    uint8_t setClosePos(uint16_t closePos);
    uint8_t getPositions();

    void update();
  private:
    unsigned long _offAtMs = 0;
     bool _pendingOff = false;

    Adafruit_PWMServoDriver &_driver;
    uint8_t _channel;
    uint16_t _openPos;
    uint16_t _closePos;
    bool _isInverted;

};
#endif