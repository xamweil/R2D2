#ifndef COMMAND_PROCESSOR_H
#define COMMAND_PROCESSOR_H

#include <Arduino.h>
#include "Lid.h"
#include <Adafruit_PWMServoDriver.h>

class CommandProcessor {
public:
  // Constructor takes a reference to the driver.
  CommandProcessor(Adafruit_PWMServoDriver &driver);

  // processCommand may modify the string (it removes parameters), so we pass by non-const reference.
  String processCommand(String &command);

private:
  
  Adafruit_PWMServoDriver &_driver;
  uint16_t _servoMin;
  uint16_t _servoMax;


  // Lid objects for all channels.
  Lid lid1_1;
  Lid lid1_2;
  Lid lid1_3;
  Lid lid1_4;
  Lid lid1_5;
  Lid lid1_6;

  Lid lid2_1;
  Lid lid2_2;
  Lid lid2_3;
  Lid lid2_4;
  Lid lid2_5;

  uint16_t _stringToUint16(const String &input);
  bool _hadParantheses(const String &command);


  String _runSetCommand(const String &command, uint16_t param);
  String _runCommand(const String &command);
  String _openLid(Lid* lid);
  String _closeLid(Lid* lid);
  String _systemTest();

  String _error;

  size_t _numCommands;      // Number of commands in the table (if needed)
};

#endif 
