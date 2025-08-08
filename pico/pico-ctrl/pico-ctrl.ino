#include <Arduino.h>

// builtin led is 25
static constexpr uint8_t LED_PIN = 22;

void setup() {
    // pinMode(LED_PIN, OUTPUT);
    // pinMode(LED_BUILTIN, OUTPUT);

    for (int i = 0; i < 41; i++) {
        pinMode(i, OUTPUT);
        digitalWrite(i, HIGH);
    }

    Serial.begin(115200);
    while (!Serial) {
    }

    delay(100);

    Serial.print("SETUP COMPLETE");
}

void loop() {
    // digitalWrite(LED_PIN, HIGH);
    // digitalWrite(LED_BUILTIN, HIGH);
    // delay(1000);
    // digitalWrite(LED_PIN, LOW);
    // digitalWrite(LED_BUILTIN, LOW);
    // delay(1000);
}
