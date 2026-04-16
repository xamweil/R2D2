#include <Arduino.h>
#include <HardwareSerial.h>
#include <IPAddress.h>

#include "button.h"
#include "radar.h"
#include "micro_ros_node.h"
#include "secrets.h"

#define MONITOR_BAUD 115200
#define RADAR_BAUD 256000

#define RADAR_RX_PIN 4         // D2
#define RADAR_TX_PIN 5         // D3

#define TASTER_1_PIN 6         // D4
#define TASTER_2_PIN 7         // D5

#define TASTER_1_ID 1
#define TASTER_2_ID 2

#define RADAR_PUBLISH_INTERVAL_MS 200

#define AGENT_IP_1 192
#define AGENT_IP_2 168
#define AGENT_IP_3 66
#define AGENT_IP_4 2
#define AGENT_PORT 8888

#define NODE_NAME "leg_l_node"
#define TASTER_TOPIC "/leg_l/taster_event"
#define RADAR_TOPIC "/leg_l/radar"
#define TASTER_FRAME_ID "leg_l_taster"
#define RADAR_FRAME_ID "leg_l_radar"

HardwareSerial RadarSerial(1);

Button button1(TASTER_1_PIN, TASTER_1_ID);
Button button2(TASTER_2_PIN, TASTER_2_ID);

Radar radar(
  RadarSerial,
  RADAR_RX_PIN,
  RADAR_TX_PIN,
  RADAR_BAUD
);

MicroRosNode ros_node(
  WIFI_SSID,
  WIFI_PASSWORD,
  IPAddress(AGENT_IP_1, AGENT_IP_2, AGENT_IP_3, AGENT_IP_4),
  AGENT_PORT,
  NODE_NAME,
  TASTER_TOPIC,
  RADAR_TOPIC,
  TASTER_FRAME_ID,
  RADAR_FRAME_ID
);

unsigned long last_radar_publish_ms = 0;

void setup() {
  Serial.begin(MONITOR_BAUD);
  delay(1000);

  Serial.println();
  Serial.println("Left leg micro-ROS firmware booting...");

  button1.begin();
  button2.begin();

  radar.begin();
  Serial.print("Radar connected: ");
  Serial.println(radar.isConnected() ? "yes" : "no");

  if (!ros_node.begin()) {
    Serial.println("micro-ROS init failed");
    while (true) {
      delay(100);
    }
  }

  Serial.println("micro-ROS init done");
}

void loop() {
  button1.update();
  button2.update();

  if (button1.wasPressed()) {
    ros_node.publishButtonPress(button1.buttonId());
  }

  if (button2.wasPressed()) {
    ros_node.publishButtonPress(button2.buttonId());
    
  }

  radar.update();

  const unsigned long now = millis();
  if ((now - last_radar_publish_ms) >= RADAR_PUBLISH_INTERVAL_MS) {
    const RadarReading reading = radar.getReading();

    ros_node.publishRadar(reading);

    last_radar_publish_ms = now;
  }

  delay(5);
}
