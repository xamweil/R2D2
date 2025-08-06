#include <Arduino.h>

const int LED_PIN = 22;

const int ENA_PIN = 22;
const int DIR_PIN = 20;
const int PUL_PIN = 21;

void setup() {
  pinMode(ENA_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(PUL_PIN, OUTPUT);
}

void loop() { sample_pwm(); }
