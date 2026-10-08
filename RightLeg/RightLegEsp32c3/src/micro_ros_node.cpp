#include "micro_ros_node.h"

#include <rosidl_runtime_c/string_functions.h>

MicroRosNode::MicroRosNode(const char* wifi_ssid,
                           const char* wifi_password,
                           IPAddress agent_ip,
                           size_t agent_port,
                           const char* node_name,
                           const char* taster_topic,
                           const char* radar_topic,
                           const char* taster_frame_id,
                           const char* radar_frame_id): 
            wifi_ssid_(wifi_ssid),
            wifi_password_(wifi_password),
            agent_ip_(agent_ip),
            agent_port_(agent_port),
            node_name_(node_name),
            taster_topic_(taster_topic),
            radar_topic_(radar_topic),
            taster_frame_id_(taster_frame_id),
            radar_frame_id_(radar_frame_id),
            allocator_(rcl_get_default_allocator()),
            initialized_(false) {}

bool MicroRosNode::initHeader(std_msgs__msg__Header& header, const char* frame_id) {
    header.stamp.sec = 0;
    header.stamp.nanosec = 0;
    return rosidl_runtime_c__String__assign(&header.frame_id, frame_id);
}

void MicroRosNode::setHeaderStamp(std_msgs__msg__Header& header) {
    const uint32_t ms = millis();
    header.stamp.sec = static_cast<int32_t>(ms / 1000);
    header.stamp.nanosec = static_cast<uint32_t>(ms % 1000) * 1000000UL;
}

bool MicroRosNode::begin() {
    set_microros_wifi_transports(const_cast<char*>(wifi_ssid_), const_cast<char*>(wifi_password_), agent_ip_, agent_port_);

    delay(2000);

    if (rclc_support_init(&support_, 0, nullptr, &allocator_) != RCL_RET_OK) {
        return false;
    }

    if (rclc_node_init_default(&node_, node_name_, "", &support_) != RCL_RET_OK) {
        return false;
    }

    if (rclc_publisher_init_default(
            &taster_pub_,
            &node_,
            ROSIDL_GET_MSG_TYPE_SUPPORT(leg_msg, msg, TasterEvent),
            taster_topic_) != RCL_RET_OK) {
        return false;
    }

    if (rclc_publisher_init_default(
            &radar_pub_,
            &node_,
            ROSIDL_GET_MSG_TYPE_SUPPORT(leg_msg, msg, Ld2410State),
            radar_topic_) != RCL_RET_OK) {
        return false;
    }

    leg_msg__msg__TasterEvent__init(&taster_msg_);
    leg_msg__msg__Ld2410State__init(&radar_msg_);

    if (!initHeader(taster_msg_.header, taster_frame_id_)) {
        return false;
    }

    if (!initHeader(radar_msg_.header, radar_frame_id_)) {
        return false;
    }

    initialized_ = true;
    return true;

}

bool MicroRosNode::publishButtonPress(uint8_t button_id) {
    if (!initialized_) {
        return false;
    }

    setHeaderStamp(taster_msg_.header);
    taster_msg_.button_id = button_id;

    return rcl_publish(&taster_pub_, &taster_msg_, nullptr) == RCL_RET_OK;
}

bool MicroRosNode::publishRadar(const RadarReading& reading) {
    if (!initialized_) {
        return false;
    }

    setHeaderStamp(radar_msg_.header);
    radar_msg_.presence = reading.presence;
    radar_msg_.moving_distance_cm = reading.moving_distance_cm;
    radar_msg_.moving_energy = reading.moving_energy;
    radar_msg_.stationary_distance_cm = reading.stationary_distance_cm;
    radar_msg_.stationary_energy = reading.stationary_energy;

    return rcl_publish(&radar_pub_, &radar_msg_, nullptr) == RCL_RET_OK;
}