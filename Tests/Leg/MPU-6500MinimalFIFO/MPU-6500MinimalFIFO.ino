#include <Wire.h>

#define MPU_ADDR       0x68
#define BUF_SIZE       64

#define PWR_MGMT_1     0x6B
#define ACCEL_CONFIG   0x1C
#define GYRO_CONFIG    0x1B
#define CONFIG         0x1A
#define SMPLRT_DIV     0x19
#define FIFO_EN        0x23
#define USER_CTRL      0x6A
#define FIFO_COUNT_H   0x72
#define FIFO_R_W       0x74
#define SIGNAL_PATH_RESET 0x68

struct Sample {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  uint32_t timestamp;
};

Sample buffer[BUF_SIZE];
uint8_t head = 0;
uint8_t tail = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(100);

  writeRegister(PWR_MGMT_1, 0x00);
  delay(10);

  writeRegister(SIGNAL_PATH_RESET, 0x07);
  delay(10);

  writeRegister(ACCEL_CONFIG, 0x00);  // ±2g
  writeRegister(GYRO_CONFIG, 0x00);   // ±250°/s
  writeRegister(SMPLRT_DIV, 9);       // 100 Hz
  writeRegister(CONFIG, 0x01);        // DLPF enabled

  writeRegister(USER_CTRL, 0x04);     // FIFO reset
  delay(10);
  writeRegister(USER_CTRL, 0x40);     // FIFO enable
  writeRegister(FIFO_EN, 0x78);       // Enable gyro + accel

  Serial.println("FIFO Ring Buffer test started.");
}

void loop() {
  fillBufferFromFIFO();
  dumpBufferToSerial();
  delay(200);
}

void fillBufferFromFIFO() {
  uint16_t fifoCount = readFIFOCount();
  while (fifoCount >= 12) {
    uint8_t data[12];
    readRegisters(FIFO_R_W, data, 12);

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
    if (head == tail) {
      tail = (tail + 1) % BUF_SIZE; // Overwrite oldest
    }

    fifoCount -= 12;
  }
}

void dumpBufferToSerial() {
  while (tail != head) {
    Sample s = buffer[tail];
    tail = (tail + 1) % BUF_SIZE;

    Serial.print("a[g]: ");
    Serial.print(s.ax / 16384.0); Serial.print(", ");
    Serial.print(s.ay / 16384.0); Serial.print(", ");
    Serial.print(s.az / 16384.0);

    Serial.print(" | g[°/s]: ");
    Serial.print(s.gx / 131.0); Serial.print(", ");
    Serial.print(s.gy / 131.0); Serial.print(", ");
    Serial.print(s.gz / 131.0);

    Serial.print(" | t[ms]: ");
    Serial.println(s.timestamp);
  }
}

uint16_t readFIFOCount() {
  uint8_t h = readRegister(FIFO_COUNT_H);
  uint8_t l = readRegister(FIFO_COUNT_H + 1);
  return (h << 8) | l;
}

void writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

uint8_t readRegister(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 1);
  return Wire.read();
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
