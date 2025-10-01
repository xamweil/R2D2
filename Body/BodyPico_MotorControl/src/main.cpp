#include "MotorControl.h"
#include "SerialProcessor.h"
#include "debug.h"

#include <Arduino.h>

namespace {
// clang-format off
MotorControl motor_control = {
    .controller_state = ControllerState{},
    .motor_mid_foot = Motor({
      .enable_pin = 4,
      .pulse_pin = 14,
      .direction_pin = 27,
      .step_size = 1
    }),
    .motor_head = Motor({
      .enable_pin = 5,
      .pulse_pin = 15,
      .direction_pin = 26,
      .step_size = 1
    }),
    .motor_left_shoulder = Motor({
      .enable_pin = 6,
      .pulse_pin = 16,
      .direction_pin = 25,
      .step_size = 1
    }),
    .motor_right_shoulder = Motor({
      .enable_pin = 6,
      .pulse_pin = 17,
      .direction_pin = 24,
      .step_size = 1
    }),
    .motor_left_foot = Motor({
      .enable_pin = 7,
      .pulse_pin = 32,
      .direction_pin = 22,
      .step_size = 1
    }),
    .motor_right_foot = Motor({
      .enable_pin = 7,
      .pulse_pin = 31,
      .direction_pin = 21,
      .step_size = 1
    }),
};
// clang-format on

SerialProcessor serial_processor(motor_control);
} // namespace

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    Serial.begin(115200);
    while (!Serial) {
    }
    delay(100);
    DBG_PRINTLN("SETUP COMLETE");
}

void loop() {
    serial_processor.listen();
}
