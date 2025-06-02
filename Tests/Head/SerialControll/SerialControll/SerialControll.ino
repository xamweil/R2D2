#include "CommandLib.h"

CommandLib commandlib(3);
String command;
void setup() {
  Serial.begin(9600);
  delay(200);
  
  Serial.println("ready");

}

void loop() {
  if (Serial.available()){
    command = Serial.readStringUntil('\n');
    command.trim();   // Remove any trailing newline or spaces
  }
    if (command.length() > 0) {
      commandlib.processCommand(command);
    }

}
