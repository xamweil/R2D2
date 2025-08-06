#ifndef MPU6500_H
#define MPU6500_H

#include <Arduino.h>
#include <Wire.h>


class MPU6500 {
  public:
    struct Sample {
      int16_t ax, ay, az;
      int16_t gx, gy, gz;
      uint32_t timestamp;
    };

    MPU6500(uint8_t address = 0x68);

    bool begin();
    void update();
    uint8_t available();
    Sample read();

    void setSampleRate(uint16_t hz);
    void setAccelRange(uint8_t range);  // // 0=2g, 1=4g, 2=8g, 3=16g
    void setGyroRange(uint8_t range);   // 0=250, 1=500, 2=1000, 3=2000 deg/s
    int16_t readTemperature();
    void getPackedSample(uint8_t* out);

  private:
    void writeRegister(uint8_t reg, uint8_t value);
    uint8_t readRegister(uint8_t reg);
    void readRegisters(uint8_t reg, uint8_t* buf, uint8_t len);
    uint16_t readFIFOCount();
    void resetFIFO();
    void resetSignalPaths();
    void configureSensor();

    static const uint8_t BUF_SIZE = 64;
    Sample buffer[BUF_SIZE];
    uint8_t head = 0;
    uint8_t tail = 0;

    uint8_t i2c_addr;
    uint16_t sampleRateHz = 100;
};

#endif
