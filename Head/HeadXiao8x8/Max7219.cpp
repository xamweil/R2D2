#include "Max7219.h"

Max7219::Max7219():
leds{0},
DIN(10),
CS(7),
CLK(8),
lc(DIN, CLK, CS, 0){
  lc.shutdown(0, false);
  lc.setIntensity(0, 16);
  lc.clearDisplay(0);
  
}

void Max7219::displayOff(){
  lc.clearDisplay(0);
}

void Max7219::randomLights(uint8_t ratOn=70, uint8_t ratChange=10, uint8_t dt=150){
  //set ratOn% of the LED's on
  for(int row = 0; row < 8; row++) {
    for (int col = 0; col < 8; col++){
      leds[row][col] = (random(100) < ratOn);
    }
  }
  //runs untill new command arrives
  while(!Serial.available()){
    //ratChange% of LED's are changed every iteration
    for (int row = 0; row < 8; row++) {
      for (int col = 0; col < 8; col++) {
        if (random(100) < 10) {
          leds[row][col] = !leds[row][col];
        }
      }
    }
    // Update display 
      for (int row = 0; row < 8; row++) {
        for (int col = 0; col < 8; col++) {
          lc.setLed(0, row, col, leds[row][col]);
        }
      }
      delay(dt);
  }
}


void Max7219::displayOn(){
  for (int row = 0; row < 8; row++) {
    for (int col = 0; col < 8; col++) {
       leds[row][col] = true;
       lc.setLed(0, row, col, leds[row][col]);
    }
  }
}
void Max7219::setLeds(const bool (&newLeds)[8][8]){
  for (int row = 0; row < 8; row++) {
    for (int col = 0; col < 8; col++) {
      leds[row][col] = newLeds[row][col];
      lc.setLed(0, row, col, leds[row][col]);
    }
  }
}

void Max7219::setSingleLed(const uint8_t row, const uint8_t col, const bool led){
      leds[row][col] = led;
      lc.setLed(0, row, col, leds[row][col]); 
}
