#ifndef MAX7219_H
#define MAX7219_H

#include <Arduino.h>
#include <LedControl.h>

class Max7219{
  public: 
    Max7219();
    bool leds[8][8];

    void displayOff();
    void randomLights(uint8_t ratOn, uint8_t ratChange, uint8_t dt);
    void displayOn();
    void setLeds(const bool (&newLeds)[8][8]);
    void setSingleLed(const uint8_t row, const uint8_t col, const bool led);
    
   private:
    int DIN;
    int CS;
    int CLK;
    LedControl lc;
    
   
    
    

    
};
#endif
