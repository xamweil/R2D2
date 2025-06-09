#include "Matrix.h"
#include "delay.h"
#include "wiring_constants.h"
#include "wiring_digital.h"

static int SRCLK = 1;
static int RCLK = 2;
static int SER = 3;

bool checkSOF() { return Serial.available() && Serial.peek() == 0xAA; }

Matrix::Matrix() {
  pinMode(SRCLK, OUTPUT);
  pinMode(RCLK, OUTPUT);
  pinMode(SER, OUTPUT);
  digitalWrite(SRCLK, LOW);
  digitalWrite(RCLK, LOW);
  digitalWrite(SER, LOW);
}

void Matrix::writeToRegister(const int bits[30]) {
  for (int i = 29; i >= 0; i--) {
    digitalWrite(SER, bits[i]);
    digitalWrite(SRCLK, HIGH);
    digitalWrite(SRCLK, LOW);
  }

  digitalWrite(RCLK, HIGH);
  digitalWrite(RCLK, LOW);
}

void Matrix::resetArr() {
  for (int i = 0; i < 30; i++) {
    arr[i] = 1;
  }
  writeToRegister(arr);
}

void Matrix::randomLights(uint8_t ratOn, uint8_t ratChange, uint8_t dt_random) {
  bool leds[4][26];
  unsigned long lastUpdate = millis();

  // set ratOn% of the LED's on
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 26; j++) {
      leds[i][j] = (random(100) < ratOn);
    }
  }

  while (true) {
    // ratChange% of LED's are changed every iteration

    for (int i = 0; i < 4; i++) {
      arr[i] = 0;

      for (int j = 4; j < 30; j++) {
        arr[j] = !leds[i][j];
      }

      writeToRegister(arr);
      delay(dt);
      arr[i] = 1;
    }

    if (millis() - lastUpdate >= dt_random) {
      lastUpdate = millis();
      for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 26; j++) {
          if (random(100) < ratChange)
            leds[i][j] = !leds[i][j];
        }
      }
    }

    if (checkSOF()) {
      resetArr();
      break;
    }
  }
}

void Matrix::displayOff() {
  for (int i = 4; i < 30; i++) {
    arr[i] = 1;
  }

  for (int i = 0; i < 4; i++) {
    arr[i] = 0;

    writeToRegister(arr);
    delay(dt);
    arr[i] = 1;
  }
}

void Matrix::displayOn() {
  for (int i = 4; i < 30; i++) {
    arr[i] = 0;
  }

  while (true) {
    for (int i = 0; i < 4; i++) {
      arr[i] = 0;

      writeToRegister(arr);
      delay(dt);
      arr[i] = 1;
    }

    if (checkSOF()) {
      resetArr();
      break;
    }
  }
}

void Matrix::setLeds(bool frame[4][26]) {
  while (true) {
    for (int i = 0; i < 4; i++) {
      arr[i] = 0;
      for (int j = 4; j < 30; j++) {
        arr[j] = !frame[i][j];
      }

      writeToRegister(arr);
      delay(dt);
      arr[i] = 1;
    }

    if (checkSOF()) {
      resetArr();
      break;
    }
  }
}
