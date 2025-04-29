void setup() {
  // Do nothing
}

void loop() {
  // Do nothing
}

/*
#include <SoftwareSerial.h>
#include <ld2410.h>

#define MONITOR_BAUD 115200
#define RADAR_BAUD 256000 // Lower baud rate for testing

// Define the RX and TX pins for SoftwareSerial
#define RADAR_RX_PIN 0
#define RADAR_TX_PIN 1

SoftwareSerial RADAR_SERIAL(RADAR_RX_PIN, RADAR_TX_PIN); // RX, TX

ld2410 radar;

uint32_t lastReading = 0;
bool radarConnected = false;

void setup(void)
{
  Serial.begin(MONITOR_BAUD); // Feedback over Serial Monitor
  RADAR_SERIAL.begin(RADAR_BAUD); // UART for monitoring the radar

  delay(500);
  Serial.print(F("\nConnect LD2410 radar TX to GPIO:"));
  Serial.println(RADAR_RX_PIN);
  Serial.print(F("Connect LD2410 radar RX to GPIO:"));
  Serial.println(RADAR_TX_PIN);
  Serial.print(F("LD2410 radar sensor initialising: "));
  if(radar.begin(RADAR_SERIAL))
  {
    Serial.println(F("OK"));
    Serial.print(F("LD2410 firmware version: "));
    Serial.print(radar.firmware_major_version);
    Serial.print('.');
    Serial.print(radar.firmware_minor_version);
    Serial.print('.');
    Serial.println(radar.firmware_bugfix_version, HEX);
  }
  else
  {
    Serial.println(F("not connected"));
  }
}

void loop()
{
  radar.read();
  if(radar.isConnected() && millis() - lastReading > 1000)  // Report every 1000ms
  {
    lastReading = millis();
    if(radar.presenceDetected())
    {
      if(radar.stationaryTargetDetected())
      {
        Serial.print(F("Stationary target: "));
        Serial.print(radar.stationaryTargetDistance());
        Serial.print(F("cm energy:"));
        Serial.print(radar.stationaryTargetEnergy());
        Serial.print(' ');
      }
      if(radar.movingTargetDetected())
      {
        Serial.print(F("Moving target: "));
        Serial.print(radar.movingTargetDistance());
        Serial.print(F("cm energy:"));
        Serial.print(radar.movingTargetEnergy());
      }
      Serial.println();
    }
    else
    {
      Serial.println(F("No target"));
    }
  }
}



*/

/*
#define MONITOR_SERIAL 115200
#define RADAR_SERIAL 256000
#define RADAR_RX_PIN 0
#define RADAR_TX_PIN 1
#include <SoftwareSerial.h>
#include <ld2410.h>
ld2410 radar;

uint32_t lastReading = 0;
bool radarConnected = false;

void setup(void)
{
  MONITOR_SERIAL.begin(115200); //Feedback over Serial Monitor
  //radar.debug(MONITOR_SERIAL); //Uncomment to show debug information from the library on the Serial Monitor. By default this does not show sensor reads as they are very frequent.
  #if defined(ESP32)
    RADAR_SERIAL.begin(256000, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN); //UART for monitoring the radar
  #elif defined(__AVR_ATmega32U4__)
    RADAR_SERIAL.begin(256000); //UART for monitoring the radar
  #endif
  delay(500);
  MONITOR_SERIAL.print(F("\nConnect LD2410 radar TX to GPIO:"));
  MONITOR_SERIAL.println(RADAR_RX_PIN);
  MONITOR_SERIAL.print(F("Connect LD2410 radar RX to GPIO:"));
  MONITOR_SERIAL.println(RADAR_TX_PIN);
  MONITOR_SERIAL.print(F("LD2410 radar sensor initialising: "));
  if(radar.begin(RADAR_SERIAL))
  {
    MONITOR_SERIAL.println(F("OK"));
    MONITOR_SERIAL.print(F("LD2410 firmware version: "));
    MONITOR_SERIAL.print(radar.firmware_major_version);
    MONITOR_SERIAL.print('.');
    MONITOR_SERIAL.print(radar.firmware_minor_version);
    MONITOR_SERIAL.print('.');
    MONITOR_SERIAL.println(radar.firmware_bugfix_version, HEX);
  }
  else
  {
    MONITOR_SERIAL.println(F("not connected"));
  }
}

void loop()
{
  radar.read();
  if(radar.isConnected() && millis() - lastReading > 1000)  //Report every 1000ms
  {
    lastReading = millis();
    if(radar.presenceDetected())
    {
      if(radar.stationaryTargetDetected())
      {
        Serial.print(F("Stationary target: "));
        Serial.print(radar.stationaryTargetDistance());
        Serial.print(F("cm energy:"));
        Serial.print(radar.stationaryTargetEnergy());
        Serial.print(' ');
      }
      if(radar.movingTargetDetected())
      {
        Serial.print(F("Moving target: "));
        Serial.print(radar.movingTargetDistance());
        Serial.print(F("cm energy:"));
        Serial.print(radar.movingTargetEnergy());
      }
      Serial.println();
    }
    else
    {
      Serial.println(F("No target"));
    }
  }
}
*/