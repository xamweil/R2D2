#include "SerialProcessor.h"

SerialProcessor::SerialProcessor(Adafruit_PWMServoDriver &driver1, Adafruit_PWMServoDriver &driver2):
  _driver1(driver1),
  _driver2(driver2),
  _servoMin(103),
  _servoMax(512),

  lid1_1(driver1, 0, map(40, 0, 180,   _servoMin, _servoMax), map(179, 0, 180,  _servoMin, _servoMax), false),
  lid1_2(driver1, 1, map(0, 0, 180,   _servoMin, _servoMax), map(120, 0, 180,   _servoMin, _servoMax), false),
  lid1_3(driver1, 2, map(10, 0, 180,   _servoMin, _servoMax), map(120, 0, 180,   _servoMin, _servoMax), false),
  lid1_4(driver1, 3, map(0, 0, 180,   _servoMin, _servoMax), map(110, 0, 180,   _servoMin, _servoMax), false),
  lid1_5(driver1, 4, map(8, 0, 180,   _servoMin, _servoMax), map(125, 0, 180,   _servoMin, _servoMax), false),
  lid1_6(driver1, 5, map(10, 0, 180,   _servoMin, _servoMax), map(130, 0, 180,   _servoMin, _servoMax), false),
  lid2_1(driver1, 6, map(20, 0, 180,   _servoMin, _servoMax), map(180, 0, 180,   _servoMin, _servoMax), true),
  lid2_2(driver1, 7, map(50, 0, 180,   _servoMin, _servoMax), map(145, 0, 180,   _servoMin, _servoMax), true),
  lid2_3(driver1, 8, map(30, 0, 180,   _servoMin, _servoMax), map(145, 0, 180,   _servoMin, _servoMax), true),
  lid2_4(driver1, 9, map(20, 0, 180,   _servoMin, _servoMax), map(150, 0, 180,   _servoMin, _servoMax), true),
  lid2_5(driver1, 10, map(10, 0, 180,   _servoMin, _servoMax), map(145, 0, 180,   _servoMin, _servoMax), true),

  nob1(driver1, 11, 12, 83, 88, 15),
  nob2(driver1, 13, 14, 93, 90, 15),
  nob3(driver2, 0, 1, 90, 93, 15),

  SOF(0xAA)


  {

  }

uint8_t SerialProcessor::listen(){
  // Wait for start-of-frame
  if (!Serial.available()) return 0xFF; // No Message
  if (Serial.peek() != SOF) {
    Serial.read();       // discard noise
    return 0xFF;         // No Message
  }
  Serial.read();         // consume SOF

  // Read LEN
  while (Serial.available() < 1) { /* wait */ }
  uint8_t len = Serial.read();
  if (len < 3 || len > MAX_PKT) return 0x05;   // sanity check

  uint8_t packet[MAX_PKT];
  // Read the rest of the frame
  while (Serial.available() < len) { /* wait */ }
  Serial.readBytes(packet, len);
  uint8_t crc = SOF ^ len; 
  for (uint8_t i = 0; i < len - 1; i++) {
    crc ^= packet[i];
  } 
  if (crc!=packet[len-1]) return 0x01; //Error: bad checksum
  
  return processPacket(packet, len);
}


uint8_t SerialProcessor::processPacket(uint8_t *packet, uint8_t len){
  uint8_t classID = packet[0];
  uint8_t funcID = packet[1];
  uint8_t payloadLen = len -3;
  
  const uint8_t* payload = &packet[2];

  Nob* nob = nullptr;
  Lid* lid = nullptr;

  switch(classID) {
    // Lids
    case 0x01: 
      lid = &lid1_1;
      break;
    case 0x02:
      lid = &lid1_2;
      break;
    case 0x03:
      lid = &lid1_3;
      break;
    case 0x04:
      lid = &lid1_4;
      break;
    case 0x05:
      lid = &lid1_5;
      break;
    case 0x06:
      lid = &lid2_1;
      break;
    case 0x07:
      lid = &lid2_2;
      break;
    case 0x08:
      lid = &lid2_3;
      break;
    case 0x09:
      lid = &lid2_4;
      break;
    case 0x0A:
      lid = &lid2_5;
      break;

    //Nobs
    case 0x10:
      nob = &nob1;
      break;
    case 0x11:
      nob = &nob2;
      break;
    case 0x12:
      nob = &nob3;
      break;
    default:
      return 0x01;  //Error: unknown class object
  }
  if (lid) {
    switch(funcID){
        case 0x01:
          return lid->openLid();
        case 0x02:
          return lid->closeLid();
        case 0x03:
          if (payloadLen==2){
            return lid->setOpenPos((payload[0] << 8) | payload[1]);
          }
          else{
            return 0x04; //Error: bad payload
          }
        case 0x04:
          if (payloadLen==2){
            return lid->setClosePos((payload[0] << 8) | payload[1]);
          }
          else{
            return 0x04; //Error: bad payload
          }
        case 0x05:
          return lid->getPositions();                  
        default:
          return 0x02;  //Error: unknown function
      }
  } 
  else if(nob){
    switch(funcID){
      case 0x01:
        if (payloadLen==4){
          return nob->setPos(static_cast<int16_t>( (uint16_t(payload[0]) << 8) | payload[1] ), static_cast<int16_t>( (uint16_t(payload[2]) << 8) | payload[3] ));
        }
        else{
          return 0x04; //Error: bad payload
        }
      case 0x02:
        if (payloadLen==2){
          return nob->setPosX(static_cast<int16_t>( (uint16_t(payload[0]) << 8) | payload[1] ));
        }
        else{
          return 0x04; //Error: bad payload
        }
      case 0x03:
        if (payloadLen==2){
          return nob->setPosY(static_cast<int16_t>( (uint16_t(payload[0]) << 8) | payload[1] ));
        }
        else{
          return 0x04; //Error: bad payload
        }
      case 0x04:
        return nob->getPos();
      case 0x05:
        return nob->runCircle();
      default:
          return 0x02;  //Error: unknown function
    }
  
  }

}

