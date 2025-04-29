#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
Adafruit_PWMServoDriver driver1 = Adafruit_PWMServoDriver(0x41);

String command;

void setup() {
  Serial.begin(19200);
  driver1.begin();
  driver1.setPWMFreq(50);
  delay(500);
  Serial.println("Starting");

}

void loop() {
  if (Serial.available()){
    command = Serial.readStringUntil('\n');
    command.trim();   // Remove any trailing newline or spaces
    delay(200);
  }
  long pos = random(0,180);

  if (command.length() > 0) {
    if (command.equals("c1")){
      driver1.setPWM(0, 0, map(pos, 0, 180, 103, 512));
    }
    Serial.println(command);
    command = "";
  }

}
