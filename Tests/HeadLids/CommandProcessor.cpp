#include "CommandProcessor.h"

CommandProcessor::CommandProcessor(Adafruit_PWMServoDriver &driver) :
  _driver(driver),
  _servoMin(103),
  _servoMax(512),
  lid1_1(driver, 0, map(40, 0, 180,   _servoMin, _servoMax), map(179, 0, 180,  _servoMin, _servoMax), false),
  lid1_2(driver, 1, map(0, 0, 180,   _servoMin, _servoMax), map(120, 0, 180,   _servoMin, _servoMax), false),
  lid1_3(driver, 2, map(10, 0, 180,   _servoMin, _servoMax), map(120, 0, 180,   _servoMin, _servoMax), false),
  lid1_4(driver, 3, map(0, 0, 180,   _servoMin, _servoMax), map(110, 0, 180,   _servoMin, _servoMax), false),
  lid1_5(driver, 4, map(8, 0, 180,   _servoMin, _servoMax), map(125, 0, 180,   _servoMin, _servoMax), false),
  lid1_6(driver, 5, map(10, 0, 180,   _servoMin, _servoMax), map(130, 0, 180,   _servoMin, _servoMax), false),
  lid2_1(driver, 6, map(20, 0, 180,   _servoMin, _servoMax), map(180, 0, 180,   _servoMin, _servoMax), true),
  lid2_2(driver, 7, map(50, 0, 180,   _servoMin, _servoMax), map(145, 0, 180,   _servoMin, _servoMax), true),
  lid2_3(driver, 8, map(30, 0, 180,   _servoMin, _servoMax), map(145, 0, 180,   _servoMin, _servoMax), true),
  lid2_4(driver, 9, map(20, 0, 180,   _servoMin, _servoMax), map(150, 0, 180,   _servoMin, _servoMax), true),
  lid2_5(driver, 10, map(10, 0, 180,   _servoMin, _servoMax), map(145, 0, 180,   _servoMin, _servoMax), true)
{
}

String CommandProcessor::processCommand(String &command) {  
  if (command.equals("system_test")) {
    return _systemTest();
  }

  if (_hadParantheses(command)) {
    int openParen = command.indexOf('(');
    int closeParen = command.indexOf(')');
    String paramStr = command.substring(openParen + 1, closeParen);
    command = command.substring(0, openParen);
    uint16_t param = _stringToUint16(paramStr);
    if (param == 65535) {
      return _error;
    }
    return _runSetCommand(command, param);
  } else {
    return _runCommand(command);
  }
}

String CommandProcessor::_systemTest(){
  Serial.println("System test started...");
  int dt = 300; // Delay in milliseconds.
  
  // Open lids.
  _runCommand("lid1_1.open");
  delay(dt);
  _runCommand("lid1_2.open");
  delay(dt);
  _runCommand("lid1_3.open");
  delay(dt);
  _runCommand("lid1_4.open");
  delay(dt);
  _runCommand("lid1_5.open");
  delay(dt);
  _runCommand("lid1_6.open");
  delay(dt);
  _runCommand("lid2_1.open");
  delay(dt);
  _runCommand("lid2_2.open");
  delay(dt);
  _runCommand("lid2_3.open");
  delay(dt);
  _runCommand("lid2_4.open");
  delay(dt);
  _runCommand("lid2_5.open");
  delay(dt);

  // Close lids.
  _runCommand("lid1_1.close");
  delay(dt);
  _runCommand("lid1_2.close");
  delay(dt);
  _runCommand("lid1_3.close");
  delay(dt);
  _runCommand("lid1_4.close");
  delay(dt);
  _runCommand("lid1_5.close");
  delay(dt);
  _runCommand("lid1_6.close");
  delay(dt);
  _runCommand("lid2_1.close");
  delay(dt);
  _runCommand("lid2_2.close");
  delay(dt);
  _runCommand("lid2_3.close");
  delay(dt);
  _runCommand("lid2_4.close");
  delay(dt);
  _runCommand("lid2_5.close");
  delay(dt);
    
  return "System test done";
}

bool CommandProcessor::_hadParantheses(const String &command) {
  int openParen = command.indexOf('(');
  int closeParen = command.indexOf(')');
  return (openParen != -1 && closeParen != -1 && closeParen > openParen);
}

String CommandProcessor::_runCommand(const String &command) {
  Serial.println(command);
  if (command.equals("lid1_1.open")) {
    return lid1_1.openLid(); 
  } else if (command.equals("lid1_1.close")) {
    return lid1_1.closeLid();
  } else if (command.equals("lid1_2.open")) {
    return lid1_2.openLid();
  } else if (command.equals("lid1_2.close")) {
    return lid1_2.closeLid();
  } else if (command.equals("lid1_3.open")) {
    return lid1_3.openLid();
  } else if (command.equals("lid1_3.close")) {
    return lid1_3.closeLid();
  } else if (command.equals("lid1_4.open")) {
    return lid1_4.openLid();
  } else if (command.equals("lid1_4.close")) {
    return lid1_4.closeLid();
  } else if (command.equals("lid1_5.open")) {
    return lid1_5.openLid();
  } else if (command.equals("lid1_5.close")) {
    return lid1_5.closeLid();
  } else if (command.equals("lid1_6.open")) {
    return lid1_6.openLid();
  } else if (command.equals("lid1_6.close")) {
    return lid1_6.closeLid();
  } else if (command.equals("lid2_1.open")) {
    return lid2_1.openLid();
  } else if (command.equals("lid2_1.close")) {
    return lid2_1.closeLid();
  } else if (command.equals("lid2_2.open")) {
    return lid2_2.openLid();
  } else if (command.equals("lid2_2.close")) {
    return lid2_2.closeLid();
  } else if (command.equals("lid2_3.open")) {
    return lid2_3.openLid();
  } else if (command.equals("lid2_3.close")) {
    return lid2_3.closeLid();
  } else if (command.equals("lid2_4.open")) {
    return lid2_4.openLid();
  } else if (command.equals("lid2_4.close")) {
    return lid2_4.closeLid();
  } else if (command.equals("lid2_5.open")) {
    return lid2_5.openLid();
  } else if (command.equals("lid2_5.close")) {
    return lid2_5.closeLid();
  } else if (command.equals("lid1_1.getPositions")){
    return lid1_1.getPositions();
  } else if (command.equals("lid1_2.getPositions")){
    return lid1_2.getPositions();
  } else if (command.equals("lid1_3.getPositions")){
    return lid1_3.getPositions();
  } else if (command.equals("lid1_4.getPositions")){
    return lid1_4.getPositions();
  } else if (command.equals("lid1_5.getPositions")){
    return lid1_5.getPositions();
  } else if (command.equals("lid1_6.getPositions")){
    return lid1_6.getPositions();
  } else if (command.equals("lid2_1.getPositions")){
    return lid2_1.getPositions();
  } else if (command.equals("lid2_2.getPositions")){
    return lid2_2.getPositions();
  } else if (command.equals("lid2_3.getPositions")){
    return lid2_3.getPositions();
  } else if (command.equals("lid2_4.getPositions")){
    return lid2_4.getPositions();
  } else if (command.equals("lid2_5.getPositions")){
    return lid2_5.getPositions();
  } else {
    return "Command not found";
  }
}



String CommandProcessor::_runSetCommand(const String &command, uint16_t param) {
  Serial.print(command);
  Serial.println(param);
  // Open position commands
  if (command.equals("lid1_1.setOpenPos")) {
    return lid1_1.setOpenPos(param);
  } else if (command.equals("lid1_2.setOpenPos")) {
    return lid1_2.setOpenPos(param);
  } else if (command.equals("lid1_3.setOpenPos")) {
    return lid1_3.setOpenPos(param);
  } else if (command.equals("lid1_4.setOpenPos")) {
    return lid1_4.setOpenPos(param);
  } else if (command.equals("lid1_5.setOpenPos")) {
    return lid1_5.setOpenPos(param);
  } else if (command.equals("lid1_6.setOpenPos")) {
    return lid1_6.setOpenPos(param);
  } else if (command.equals("lid2_1.setOpenPos")) {
    return lid2_1.setOpenPos(param);
  } else if (command.equals("lid2_2.setOpenPos")) {
    return lid2_2.setOpenPos(param);
  } else if (command.equals("lid2_3.setOpenPos")) {
    return lid2_3.setOpenPos(param);
  } else if (command.equals("lid2_4.setOpenPos")) {
    return lid2_4.setOpenPos(param);
  } else if (command.equals("lid2_5.setOpenPos")) {
    return lid2_5.setOpenPos(param);
  }
  // Close position commands
  else if (command.equals("lid1_1.setClosePos")) {
    return lid1_1.setClosePos(param);
  } else if (command.equals("lid1_2.setClosePos")) {
    return lid1_2.setClosePos(param);
  } else if (command.equals("lid1_3.setClosePos")) {
    return lid1_3.setClosePos(param);
  } else if (command.equals("lid1_4.setClosePos")) {
    return lid1_4.setClosePos(param);
  } else if (command.equals("lid1_5.setClosePos")) {
    return lid1_5.setClosePos(param);
  } else if (command.equals("lid1_6.setClosePos")) {
    return lid1_6.setClosePos(param);
  } else if (command.equals("lid2_1.setClosePos")) {
    return lid2_1.setClosePos(param);
  } else if (command.equals("lid2_2.setClosePos")) {
    return lid2_2.setClosePos(param);
  } else if (command.equals("lid2_3.setClosePos")) {
    return lid2_3.setClosePos(param);
  } else if (command.equals("lid2_4.setClosePos")) {
    return lid2_4.setClosePos(param);
  } else if (command.equals("lid2_5.setClosePos")) {
    return lid2_5.setClosePos(param);
  } else {
    return "Command not found";
  }
}


uint16_t CommandProcessor::_stringToUint16(const String &input) {
  if (input.length() == 0) {
    _error = "Input string is empty";
    return 65535;
  }
  for (int i = 0; i < input.length(); i++) {
    if (!isDigit(input.charAt(i))) {
      _error = "Non-digit character encountered: " + String(input.charAt(i));
      return 65535;
    }
  }
  
  long num = input.toInt();
  if (num < 0 || num > 65535) {
    _error = "Number out of range (0-65535): " + String(num);
    return 65535;
  }
  
  _error = "";
  return (uint16_t) num;
}
