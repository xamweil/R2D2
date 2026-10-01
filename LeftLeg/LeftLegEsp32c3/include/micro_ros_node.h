#pragma once

#include <Arduino.h>
#include <IPAddress.h>

#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>

#include <leg_msg/msg/taster_event.h>
#include <leg_msg/msg/ld2410_state.h>

#include "radar.h"

class MicroRosNode {
    public:
        MicroRosNode(const char* wifi_ssid,
                const char* wifi_password,
                IPAddress agent_ip,
                size_t agent_port,
                const char* node_name,
                const char* taster_topic,
                const char* radar_topic,
                const char* taster_frame_id,
                const char* radar_frame_id);

        bool begin();
        bool publishButtonPress(uint8_t button_id);
        bool publishRadar(const RadarReading& reading);

    private:
        const char* wifi_ssid_;
        const char* wifi_password_;
        IPAddress agent_ip_;
        size_t agent_port_;

        const char* node_name_;
        const char* taster_topic_;
        const char* radar_topic_;
        const char* taster_frame_id_;
        const char* radar_frame_id_;

        rcl_allocator_t allocator_;
        rclc_support_t support_;
        rcl_node_t node_;
        rcl_publisher_t taster_pub_;
        rcl_publisher_t radar_pub_;

        leg_msg__msg__TasterEvent taster_msg_;
        leg_msg__msg__Ld2410State radar_msg_;

        bool initialized_;

        bool initHeader(std_msgs__msg__Header& header, const char* frame_id);
        void setHeaderStamp(std_msgs__msg__Header& header);
};