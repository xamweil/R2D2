#pragma once
#include <Arduino.h>
#include <Wire.h>

class MPU6500 {
public:
  struct Sample {
    int16_t  ax, ay, az;
    int16_t  gx, gy, gz;
    uint32_t timestamp_ms;
  };

  explicit MPU6500(uint8_t address);
  bool     begin();
  void     setSampleRate(uint16_t hz);     // default 100 Hz
  void     setAccelRange(uint8_t fs_sel);  // 0: ±2g, 1: ±4g, 2: ±8g, 3: ±16g
  void     setGyroRange(uint8_t fs_sel);   // 0: ±250, 1: ±500, 2: ±1000, 3: ±2000 dps
  int16_t  readTemperature();

  // No FIFO: update() reads the latest registers once and stores one sample.
  void     update();
  uint8_t  available() const { return has_sample_ ? 1 : 0; }
  void     getPackedSample(uint8_t* out16); // packs LE: ax..gz + u32 timestamp

private:
  // Registers
  static constexpr uint8_t MPU_PWR_MGMT_1   = 0x6B;
  static constexpr uint8_t MPU_SMPLRT_DIV   = 0x19;
  static constexpr uint8_t MPU_CONFIG       = 0x1A;  // Gyro DLPF
  static constexpr uint8_t MPU_GYRO_CONFIG  = 0x1B;
  static constexpr uint8_t MPU_ACCEL_CONFIG = 0x1C;
  static constexpr uint8_t MPU_TEMP_OUT_H   = 0x41;
  static constexpr uint8_t MPU_ACCEL_XOUT_H = 0x3B;  // 14-byte burst to GYRO_ZOUT_L
  static constexpr uint8_t MPU_INT_ENABLE  = 0x38;
  static constexpr uint8_t MPU_INT_STATUS  = 0x3A;

  uint8_t  i2c_addr_;
  uint16_t sampleRateHz_ = 50;  // default 50 Hz
  Sample   last_{};
  bool     has_sample_ = false;
  uint32_t last_read_us_ = 0;

  // I2C helpers
  void     writeRegister(uint8_t reg, uint8_t value);
  uint8_t  readRegister(uint8_t reg);
  bool     readRegisters(uint8_t reg, uint8_t* buf, uint8_t len);
};
