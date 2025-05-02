#ifndef SERIAL_PROCESSOR_H
#define SERIAL_PROCESSOR_H

#include <Arduino.h>
#include "FunctionProcessor.h"

class SerialProcessor{
  public:
    SerialProcessor();
    uint8_t listen();
    FunctionProcessor fctProc;

   private:
    static constexpr uint8_t SOF = 0xAA;
    static constexpr uint8_t MAX_PKT = 32;
    uint8_t lastPkt[MAX_PKT];
    uint8_t lastLen = 0;
    
};

#endif
