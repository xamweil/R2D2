#include <Arduino.h>
#include <WiFi.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>

#include "secrets.h"

#define MONITOR_BAUD 115200

IPAddress agent_ip(192, 168, 66, 2);
size_t agent_port = 8888;

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

void fail_loop()
{
  while (true) {
    delay(100);
  }
}

void setup()
{
  Serial.begin(MONITOR_BAUD);
  delay(1000);
  Serial.println("micro-ROS WiFi test booting...");

  set_microros_wifi_transports(
    (char*)WIFI_SSID,
    (char*)WIFI_PASSWORD,
    agent_ip,
    agent_port
  );

  delay(2000);

  allocator = rcl_get_default_allocator();

  rcl_ret_t rc = rclc_support_init(&support, 0, NULL, &allocator);
  if (rc != RCL_RET_OK) {
    Serial.printf("rclc_support_init failed: %d\n", (int)rc);
    fail_loop();
  }

  rc = rclc_node_init_default(&node, "leg_l_test_node", "", &support);
  if (rc != RCL_RET_OK) {
    Serial.printf("rclc_node_init_default failed: %d\n", (int)rc);
    fail_loop();
  }

  rc = rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "/leg_l/test_counter"
  );
  if (rc != RCL_RET_OK) {
    Serial.printf("publisher init failed: %d\n", (int)rc);
    fail_loop();
  }

  msg.data = 0;
  Serial.println("micro-ROS init done");
}

void loop()
{
  rcl_ret_t rc = rcl_publish(&publisher, &msg, NULL);
  if (rc == RCL_RET_OK) {
    Serial.printf("Published: %ld\n", (long)msg.data);
    msg.data++;
  } else {
    Serial.printf("Publish failed: %d\n", (int)rc);
  }

  delay(1000);
}