#ifndef FUNCTION_PROCESSOR_H
#define FUNCTION_PROCESSOR_H

#include<Arduino.h>
#include "Max7219.h"

class FunctionProcessor{
  public:
    FunctionProcessor();
    uint8_t processPacket(uint8_t *packet, uint8_t len);

  private:
    Max7219 matrix;
    
};


#endif
