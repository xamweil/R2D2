#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
#include "Lid.h"
#include "CommandProcessor.h"

Adafruit_PWMServoDriver driver = Adafruit_PWMServoDriver(0x40);

CommandProcessor cmdProcessor(driver);

String command;

void setup() {
  Serial.begin(19200);
  driver.begin();
  driver.setPWMFreq(50);
}

void loop() {
  if (Serial.available()){
    command = Serial.readStringUntil('\n');
    command.trim();   // Remove any trailing newline or spaces
    delay(200);
  }
  if (command.length() > 0) {
    String feedback = cmdProcessor.processCommand(command);
    Serial.println(feedback);
    command = "";
  }
}
