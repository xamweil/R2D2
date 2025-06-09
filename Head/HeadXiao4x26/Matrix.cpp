#include "Matrix.h"
#include "Arduino.h"
#include "wiring_constants.h"
#include "wiring_digital.h"

static int SRCLK = 1;
static int RCLK = 2;
static int SER = 3;

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

void Matrix::setLeds(bool frame[4][26]) {
  for (int i = 0; i < 4; i++) {
    arr[i] = 0;
    for (int j = 4; j < 30; j++) {
      arr[j] = !frame[i][j];
    }

    writeToRegister(arr);
    delay(300);
    arr[i] = 1;
  }
}