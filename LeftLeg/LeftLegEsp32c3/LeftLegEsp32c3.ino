#include <WiFi.h>
#include "secrets.h"
#include "TCPprocessor.h"

IPAddress localIP(192, 168, 66, 10);
IPAddress gateway(192, 168, 66, 1);
IPAddress subnet(255, 255, 255, 0);

/*
* Set samplerate here:
*/
int SAMPLE_RATE = 50;

TCPprocessor* tcp;

uint8_t taster1 = D0;
uint8_t taster2 = D1;

int i = 0;

uint8_t out[16]; // Buffer for sensor data

Stepper stepper = Stepper(2048, 10, 8, 9, 7);

void setup() {
  Serial.begin(115200);
  // Start TCP connection

  tcp = new TCPprocessor(localIP, gateway, subnet, stepper, SAMPLE_RATE, 5010);
  tcp->begin();
  pinMode(taster1, INPUT_PULLUP);
  pinMode(taster2, INPUT_PULLUP);
}

void loop(){
    
    // listen for incoming packets
    tcp->listen();

    // Update buffer
    tcp->mpuLeg.update();
    tcp->mpuFoot.update();
    
    // send sensor data
    
    while (tcp->mpuLeg.available()) {          // <= don’t send zeros
        tcp->mpuLeg.getPackedSample(out);
        tcp->sendPacket(16, 0x01, 0x02, out);
    }
    while (tcp->mpuFoot.available()) {
        tcp->mpuFoot.getPackedSample(out);
        tcp->sendPacket(16, 0x02, 0x02, out);
    }

    // Taster signals to jetson
    if (digitalRead(taster1) == LOW) {
        tcp->sendPacket(0, 0x04, 0x01, nullptr);
    }
    else if (digitalRead(taster2) == LOW) {
        tcp->sendPacket(0, 0x04, 0x02, nullptr);
    }
    
}

    
