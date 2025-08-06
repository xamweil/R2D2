#include "MPU6500.h"

MPU6500 mpu;  // default address 0x68

void setup() {
  Serial.begin(115200);
  delay(500);  // give USB time

  if (!mpu.begin()) {
    Serial.println("MPU6500 initialization failed!");
    while (true);
  }

  Serial.println("MPU6500 initialized. Reading FIFO...");
}

void loop() {
  mpu.update();

  while (mpu.available()) {
    MPU6500::Sample s = mpu.read();

    float ax = s.ax / 16384.0;
    float ay = s.ay / 16384.0;
    float az = s.az / 16384.0;

    float gx = s.gx / 131.0;
    float gy = s.gy / 131.0;
    float gz = s.gz / 131.0;

    Serial.print("a[g]: ");
    Serial.print(ax); Serial.print(", ");
    Serial.print(ay); Serial.print(", ");
    Serial.print(az);

    Serial.print(" | g[°/s]: ");
    Serial.print(gx); Serial.print(", ");
    Serial.print(gy); Serial.print(", ");
    Serial.print(gz);

    Serial.print(" | t[ms]: ");
    Serial.println(s.timestamp);
  }

  delay(200);  // chunked read
}
