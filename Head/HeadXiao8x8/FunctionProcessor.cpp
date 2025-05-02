#include "FunctionProcessor.h"

FunctionProcessor::FunctionProcessor():
  matrix(){
  }

uint8_t FunctionProcessor::processPacket(uint8_t *packet, uint8_t len){
  uint8_t classID = packet[0];
  uint8_t funcID = packet[1];
  uint8_t payloadLen = len -3;

  const uint8_t* payload = &packet[2];
  Max7219 *m = nullptr;
  switch(classID){
    case 0x01:
      m = &matrix;
      break;
    default:
      return 0x02;
  }
  if (m){
    switch(funcID){
      case 0x01:{                              // randomLights
          // defaults
          uint8_t rateOn     = 35;
          uint8_t rateChange = 10;
          uint8_t dt         = 150;

          // overwrite
          if (payloadLen >= 1) rateOn     = payload[0];
          if (payloadLen >= 2) rateChange = payload[1];
          if (payloadLen >= 3) dt         = payload[2];

          m->randomLights(rateOn, rateChange, dt);
          return 0x00;                          // success
      } 
      case 0x02:
        m->displayOn();
        return 0x00;
      
      case 0x03:
        m->displayOff();
        return 0x00;

      case 0x04:{
        if (payloadLen < 8) return 0x04;              // error: Bad payload
    
        bool frame[8][8];                  // local 8×8 buffer
    
        for (uint8_t row = 0; row < 8; ++row) {
          uint8_t byte = payload[row];           // one row
          for (uint8_t col = 0; col < 8; ++col) {
              frame[row][col] = (byte & (1 << col)) != 0;   
          }
        }
        m->setLeds(frame);
        return 0x00;
      }
      case 0x05:
        if (payloadLen!=3) return 0x04;
        m->setSingleLed(payload[0], payload[1], payload[2]);
        return 0x00;
      default:
        return 0x03;
        
    }
  }
  
}
