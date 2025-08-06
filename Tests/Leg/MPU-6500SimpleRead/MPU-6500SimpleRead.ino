#include <Wire.h>

// I2C address of MPU-6500
#define MPU_ADDR 0x68

// Register addresses
#define ACCEL_XOUT_H 0x3B
#define PWR_MGMT_1   0x6B
#define ACCEL_CONFIG 0x1C
#define GYRO_CONFIG  0x1B

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Wake up MPU
  writeRegister(PWR_MGMT_1, 0x00);
  delay(100);

  // Optional: configure full-scale ranges
  writeRegister(ACCEL_CONFIG, 0x00);  // ±2g
  writeRegister(GYRO_CONFIG, 0x00);   // ±250°/s

  Serial.println("MPU6500 direct read test started.");
}

void loop() {
  int16_t ax, ay, az, gx, gy, gz;
  uint8_t data[14];

  readRegisters(ACCEL_XOUT_H, data, 14);

  ax = (data[0] << 8) | data[1];
  ay = (data[2] << 8) | data[3];
  az = (data[4] << 8) | data[5];
  // data[6,7] = temperature (skipped)
  gx = (data[8] << 8) | data[9];
  gy = (data[10] << 8) | data[11];
  gz = (data[12] << 8) | data[13];

  float fax = ax / 16384.0;
  float fay = ay / 16384.0;
  float faz = az / 16384.0;

  float fgx = gx / 131.0;
  float fgy = gy / 131.0;
  float fgz = gz / 131.0;

  Serial.print("a[g]: ");
  Serial.print(fax); Serial.print(", ");
  Serial.print(fay); Serial.print(", ");
  Serial.print(faz);

  Serial.print(" | g[°/s]: ");
  Serial.print(fgx); Serial.print(", ");
  Serial.print(fgy); Serial.print(", ");
  Serial.println(fgz);

  delay(500);  // slow down for readability
}

void writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

void readRegisters(uint8_t reg, uint8_t* buf, uint8_t len) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, len);
  for (uint8_t i = 0; i < len; ++i) {
    buf[i] = Wire.read();
  }
}
