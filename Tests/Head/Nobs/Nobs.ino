#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
#include "Nob.h"
#include "Lid.h"
#include "SerialProcessor.h"

Adafruit_PWMServoDriver driver1 = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver driver2 = Adafruit_PWMServoDriver(0x41);

SerialProcessor serialProcessor(driver1, driver2);



void setup() {
  Serial.begin(57600);
  driver1.begin();
  driver2.begin();
  driver1.setPWMFreq(50);
  driver2.setPWMFreq(50);
  delay(500);

}

void loop() {
  uint8_t ret = 0xFF;
  while(ret==0xFF){
    ret = serialProcessor.listen();
  }
  Serial.write(ret);
  
}
