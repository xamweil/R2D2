#include <Arduino.h>

void sample_digital(int LED_PIN) {
  digitalWrite(LED_PIN, HIGH);
  delay(1000);
  digitalWrite(LED_PIN, LOW);
  delay(1000);
}

void sample_pwm(int LED_PIN) {
  int brightness = 0;
  int fadeAmount = 5;
  analogWrite(LED_PIN, brightness);

  brightness = brightness + fadeAmount;

  if (brightness <= 0 || brightness >= 255) {
    fadeAmount = -fadeAmount;
  }

  delay(30);
}
