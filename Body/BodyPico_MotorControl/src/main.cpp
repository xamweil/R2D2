#include "MotorControl.h"
#include "SerialProcessor.h"
#include "debug.h"

#include <Arduino.h>

namespace {
// clang-format off
Motors motors = {
    .mid_foot = Motor({
      .enable_pin = 2, // 4
      .pulse_pin = 6, // 9
      .direction_pin = 21 , // 27
      .step_size = 1
    }),
    .head = Motor({
      .enable_pin = 3, // 5
      .pulse_pin = 7, // 10
      .direction_pin = 20, // 26
      .step_size = 1
    }),
    .left_shoulder = Motor({
      .enable_pin = 4, //6 
      .pulse_pin = 8, // 11
      .direction_pin = 19, // 25
      .step_size = 1
    }),
    .right_shoulder = Motor({
      .enable_pin = 4, // 6
      .pulse_pin = 9, // 12
      .direction_pin = 18, // 24
      .step_size = 1
    }),
    .left_foot = Motor({
      .enable_pin = 5, // 7
      .pulse_pin = 10, // 14
      .direction_pin = 17, // 22
      .step_size = 1
    }),
    .right_foot = Motor({
      .enable_pin = 5, // 7
      .pulse_pin = 11, // 15
      .direction_pin = 16, // 21
      .step_size = 1
    }),
};
MotorControl motor_control(motors);
SerialProcessor serial_processor(motor_control);
} // namespace

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    Serial.begin(115200);
    while (!Serial) { /**/ }
    delay(100);
    DBG_PRINTLN("SETUP COMLETE");
}

void loop() {
    serial_processor.listen();
}
