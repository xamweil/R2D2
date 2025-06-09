#ifndef FUNCTION_PROCESSOR_H
#define FUNCTION_PROCESSOR_H

#include <stdint.h>
#include "Matrix.h"

class FunctionProcessor {
public:
    FunctionProcessor();
    uint8_t processPacket(uint8_t *packet, uint8_t len);

private:
    Matrix matrix;
};

#endif
