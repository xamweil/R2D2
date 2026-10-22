#pragma once

#include "topic.hpp"

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <tcp_msg/msg/mpu6500_sample.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

namespace ui_bridge {

// Keys used by code that accesses a topic directly.
inline constexpr const char *CAMERA_IMAGE_KEY = "image_raw_compressed";

// The list of topics the bridge subscribes to. Each entry creates a
// subscription and (unless in_state is false) an entry in robot_state.
//
// TopicSpec<Msg>{key, ros_topic [, serializer [, in_state [, qos]]]}
//
// The serializer defaults to write_json (msg_json.hpp), which writes every
// field of any message type. Pass a custom
// void (*)(const Msg &, std::string &out) to send something else.
inline auto make_topics() {
    using Imu = tcp_msg::msg::MPU6500Sample;
    using CameraInfo = sensor_msgs::msg::CameraInfo;
    using CompressedImage = sensor_msgs::msg::CompressedImage;
    using Detections = vision_msgs::msg::Detection2DArray;

    return make_registry(
        // imu
        TopicSpec<Imu>{"imu_left_foot", "/leg_l/imu/foot"},
        TopicSpec<Imu>{"imu_left_leg", "/leg_l/imu/leg"},
        TopicSpec<Imu>{"imu_right_foot", "/leg_r/imu/foot"},
        TopicSpec<Imu>{"imu_right_leg", "/leg_r/imu/leg"},
        TopicSpec<Imu>{"imu_body", "/Body/mpu"},
        // camera
        TopicSpec<CameraInfo>{"camera_info", "/relay/camera/camera_info"},
        TopicSpec<CompressedImage>{CAMERA_IMAGE_KEY,
                                   "/relay/camera/image_raw/compressed"},
        // tracking
        TopicSpec<Detections>{"scene_detections",
                              "/scene_understanding/detections"},
        TopicSpec<Detections>{"tracking_tracks", "/tracking/tracks"});
}

using TopicRegistry = decltype(make_topics());

} // namespace ui_bridge
