#ifndef FUNCTION_PROCESSOR_H
#define FUNCTION_PROCESSOR_H

#include "LedMatrix.h"

class FunctionProcessor {
public:
  FunctionProcessor();
  void processPacket(uint8_t *packet, uint8_t len, bool newPacket);
private:
  LedMatrix ledMatrix;
};

#endif
