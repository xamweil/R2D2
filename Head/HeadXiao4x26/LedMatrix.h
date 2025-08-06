#ifndef MATRIX_H
#define MATRIX_H

#include <Arduino.h>

class LedMatrix {
  public:
    LedMatrix();
    void randomLights(uint8_t ratOn=70, uint8_t ratChange=10, uint8_t dt = 150);
    void displayOff();
    void displayOn();
    void setLeds(bool frame[4][26]);
  private:
    static constexpr int SRCLK = 1;
    static constexpr int RCLK = 2;
    static constexpr int SER = 3;

    int dt = 300;
    int ledBuffer[30] = {1};
    void writeToRegister(const int bits[8]);
    void resetLedBuffer();
};

#endif