#pragma once

#ifndef NDEBUG

#define ASSERT(condition)                                                      \
    do {                                                                       \
        if (!(condition)) {                                                    \
            Serial.print("Assertion failed: ");                                \
            Serial.print(#condition);                                          \
            Serial.print(" at ");                                              \
            Serial.print(__FILE__);                                            \
            Serial.print(":");                                                 \
            Serial.println(__LINE__);                                          \
            while (1)                                                          \
                ;                                                              \
        }                                                                      \
    } while (0)

#define DBG_INIT() Serial.begin(9600)
#define DBG_WAIT() while (!Serial)
#define DBG_PRINT(x) Serial.print(x)
#define DBG_PRINTLN(x) Serial.println(x)
#define DBG_PRINTF(x, y) Serial.printf(x, y)

#else

#define ASSERT(condition) ((void)0)
#define DBG_PRINT(x)
#define DBG_PRINTLN(x)
#define DBG_PRINTF(x, y)

#endif
