#include "MPU6500.h"
#include <sys/time.h>

static inline uint32_t now_ms()
{
    struct timeval tv;
    gettimeofday(&tv, nullptr);                 // SNTP steers this clock
    return (uint32_t)(tv.tv_sec * 1000ULL + tv.tv_usec / 1000ULL);
}

MPU6500::MPU6500(uint8_t address) : i2c_addr_(address) {}

bool MPU6500::begin() {
  Wire.begin();
  // There are long I2C wires inside the leg, 100 kHz is safer:
  Wire.setClock(100000);

  // Wake up device (use internal osc; 0x01 for PLL with X gyro is also fine)
  writeRegister(MPU_PWR_MGMT_1, 0x00);
  delay(10);

  setSampleRate(sampleRateHz_);
  setAccelRange(0);  // ±2g
  setGyroRange(0);   // ±250 dps

  writeRegister(MPU_INT_ENABLE, 0x01);
  return true;
}

void MPU6500::setSampleRate(uint16_t hz) {
  sampleRateHz_ = hz;
  // With DLPF on, internal rate is 1 kHz. sample_rate = 1000 / (1 + SMPLRT_DIV)
  uint8_t div = (hz >= 1 && hz <= 1000) ? (uint8_t)(1000 / hz - 1) : 9; // ~100 Hz
  writeRegister(MPU_SMPLRT_DIV, div);
  // Enable DLPF with a modest bandwidth (CONFIG=1 → ~184 Hz gyro BW)
  writeRegister(MPU_CONFIG, 0x01);
}

void MPU6500::setAccelRange(uint8_t range) {
  // ACCEL_CONFIG bits [4:3] = FS_SEL
  writeRegister(MPU_ACCEL_CONFIG, (range & 0x03) << 3);
}

void MPU6500::setGyroRange(uint8_t range) {
  // GYRO_CONFIG bits [4:3] = FS_SEL
  writeRegister(MPU_GYRO_CONFIG, (range & 0x03) << 3);
}

int16_t MPU6500::readTemperature() {
  uint8_t d[2];
  if (!readRegisters(MPU_TEMP_OUT_H, d, 2)) return 0;
  return (int16_t)((int16_t(d[0]) << 8) | d[1]);
}

void MPU6500::update() {
  uint32_t now_us = micros();
  if ((uint32_t)(now_us - last_read_us_) < (1000*1/sampleRateHz_)) {
    has_sample_ = false;
    return;
  }

  // Only read when new data is ready
  uint8_t ist = readRegister(MPU_INT_STATUS);
  if ((ist & 0x01) == 0) {          // DATA_RDY_INT not set
    has_sample_ = false;
    return;
  }
  
  // Burst read the freshest sample directly from data regs (14 bytes):
  // ACCEL_X/Y/Z (6), TEMP(2), GYRO_X/Y/Z(6)
  uint8_t d[14];
  if (!readRegisters(MPU_ACCEL_XOUT_H, d, sizeof(d))) {
    has_sample_ = false;
    return;
  }

  // Big-endian in regs → convert to int16
  auto be16 = [&](int i) -> int16_t { return (int16_t)((int16_t(d[i]) << 8) | d[i+1]); };

  last_.ax = be16(0);
  last_.ay = be16(2);
  last_.az = be16(4);
  // temp at d[6..7] (ignored here)
  last_.gx = be16(8);
  last_.gy = be16(10);
  last_.gz = be16(12);

  last_.timestamp_ms = now_ms();  
  has_sample_ = true;
  last_read_us_ = now_us;
}

void MPU6500::getPackedSample(uint8_t* out) {
  // Caller should check available() first; if not, still packs zeros.
  const int16_t vals[6] = { last_.ax, last_.ay, last_.az, last_.gx, last_.gy, last_.gz };
  uint8_t* p = out;

  // little-endian pack
  for (int i = 0; i < 6; ++i) {
    int16_t v = vals[i];
    *p++ = (uint8_t)(v & 0xFF);
    *p++ = (uint8_t)((v >> 8) & 0xFF);
  }
  uint32_t t = last_.timestamp_ms;
  *p++ = (uint8_t)(t & 0xFF);
  *p++ = (uint8_t)((t >> 8) & 0xFF);
  *p++ = (uint8_t)((t >> 16) & 0xFF);
  *p++ = (uint8_t)((t >> 24) & 0xFF);

  // consume the single-sample latch
  has_sample_ = false;
}

void MPU6500::writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(i2c_addr_);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

uint8_t MPU6500::readRegister(uint8_t reg) {
  Wire.beginTransmission(i2c_addr_);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(i2c_addr_, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0;
}

bool MPU6500::readRegisters(uint8_t reg, uint8_t* buf, uint8_t len) {
  Wire.beginTransmission(i2c_addr_);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  uint8_t got = Wire.requestFrom(i2c_addr_, len);
  if (got != len) {
    for (uint8_t i = 0; i < got && i < len; ++i) buf[i] = Wire.read();
    return false;
  }
  for (uint8_t i = 0; i < len; ++i) buf[i] = Wire.read();
  return true;
}
