#ifndef DEBUG_H
#define DEBUG_H

#if DEBUG
  // #define DBG_INIT() Serial.begin(9600)
  // #define DBG_WAIT() while(!Serial)
  #define DBG_PRINT(x) Serial.print(x)
  #define DBG_PRINTLN(x) Serial.println(x)
  #define DBG_PRINTF(x, y) Serial.printf(x, y)
#else
  // #define DBG_INIT()
  // #define DBG_WAIT()
  #define DBG_PRINT(x)
  #define DBG_PRINTLN(x)
  #define DBG_PRINTF(x, y)
#endif

#endif // DEBUG_H
