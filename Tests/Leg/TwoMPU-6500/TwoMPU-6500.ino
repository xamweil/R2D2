#include "MPU6500.h"

MPU6500 mpu1(0x68);  // default address 0x68
MPU6500 mpu2(0x69);

void setup() {
  Serial.begin(115200);
  delay(500);  // give USB time

  if (!mpu1.begin()) {
    Serial.println("MPU6500 initialization failed!");
    while (true);
  }

  Serial.println("MPU6500-1 initialized. Reading FIFO...");
  if (!mpu2.begin()) {
    Serial.println("MPU6500-2 initialization failed!");
  }
}

void loop() {
  mpu1.update();
  Serial.println("MPU-1");
  while (mpu1.available()) {
    MPU6500::Sample s = mpu1.read();

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

  delay(500);  // chunked read
  mpu2.update();
  Serial.println("MPU-2");
  while (mpu2.available()) {
    MPU6500::Sample s = mpu2.read();

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

  delay(500);  // chunked read
}
