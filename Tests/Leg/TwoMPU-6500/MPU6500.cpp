#include "MPU6500.h"

#define MPU_PWR_MGMT_1     0x6B
#define MPU_SMPLRT_DIV     0x19
#define MPU_CONFIG         0x1A
#define MPU_GYRO_CONFIG    0x1B
#define MPU_ACCEL_CONFIG   0x1C
#define MPU_FIFO_EN        0x23
#define MPU_INT_ENABLE     0x38
#define MPU_USER_CTRL      0x6A
#define MPU_FIFO_COUNT_H   0x72
#define MPU_FIFO_R_W       0x74
#define MPU_TEMP_OUT_H     0x41
#define MPU_SIGNAL_PATH_RESET 0x68

MPU6500::MPU6500(uint8_t address) : i2c_addr(address) {}

bool MPU6500::begin() {
  Wire.begin();
  writeRegister(MPU_PWR_MGMT_1, 0x00);
  delay(10);
  resetSignalPaths();
  configureSensor();
  return true;
}

void MPU6500::resetSignalPaths() {
  writeRegister(MPU_SIGNAL_PATH_RESET, 0x07);
  delay(10);
}

void MPU6500::configureSensor() {
  setSampleRate(sampleRateHz);
  setAccelRange(0); // ±2g
  setGyroRange(0);  // ±250°/s

  resetFIFO();

  writeRegister(MPU_USER_CTRL, 0x40); // Enable FIFO
  writeRegister(MPU_FIFO_EN, 0x78);   // Enable gyro + accel
}

void MPU6500::resetFIFO() {
  writeRegister(MPU_USER_CTRL, 0x04); // Reset FIFO
  delay(10);
}

void MPU6500::setSampleRate(uint16_t hz) {
  sampleRateHz = hz;
  uint8_t div = 1000 / hz - 1;
  writeRegister(MPU_SMPLRT_DIV, div);
  writeRegister(MPU_CONFIG, 0x01);  // DLPF enabled
}

void MPU6500::setAccelRange(uint8_t range) {
  writeRegister(MPU_ACCEL_CONFIG, (range & 0x03) << 3);
}

void MPU6500::setGyroRange(uint8_t range) {
  writeRegister(MPU_GYRO_CONFIG, (range & 0x03) << 3);
}

void MPU6500::update() {
  uint16_t fifoCount = readFIFOCount();
  while (fifoCount >= 12) {
    uint8_t data[12];
    readRegisters(MPU_FIFO_R_W, data, 12);

    Sample s;
    s.ax = (data[0] << 8) | data[1];
    s.ay = (data[2] << 8) | data[3];
    s.az = (data[4] << 8) | data[5];
    s.gx = (data[6] << 8) | data[7];
    s.gy = (data[8] << 8) | data[9];
    s.gz = (data[10] << 8) | data[11];
    s.timestamp = millis();

    buffer[head] = s;
    head = (head + 1) % BUF_SIZE;
    if (head == tail) tail = (tail + 1) % BUF_SIZE;

    fifoCount -= 12;
  }
}

uint8_t MPU6500::available() {
  return (head >= tail) ? (head - tail) : (BUF_SIZE - tail + head);
}

MPU6500::Sample MPU6500::read() {
  Sample s = {0};
  if (head != tail) {
    s = buffer[tail];
    tail = (tail + 1) % BUF_SIZE;
  }
  return s;
}

int16_t MPU6500::readTemperature() {
  uint8_t data[2];
  readRegisters(MPU_TEMP_OUT_H, data, 2);
  return (int16_t)((data[0] << 8) | data[1]);
}

uint16_t MPU6500::readFIFOCount() {
  uint8_t h = readRegister(MPU_FIFO_COUNT_H);
  uint8_t l = readRegister(MPU_FIFO_COUNT_H + 1);
  return (h << 8) | l;
}

void MPU6500::writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(i2c_addr);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

uint8_t MPU6500::readRegister(uint8_t reg) {
  Wire.beginTransmission(i2c_addr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(i2c_addr, (uint8_t)1);
  return Wire.read();
}

void MPU6500::readRegisters(uint8_t reg, uint8_t* buf, uint8_t len) {
  Wire.beginTransmission(i2c_addr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(i2c_addr, len);
  for (uint8_t i = 0; i < len; ++i) {
    buf[i] = Wire.read();
  }
}
