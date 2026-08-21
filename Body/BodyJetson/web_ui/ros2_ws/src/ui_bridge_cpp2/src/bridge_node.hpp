#pragma once

#include "slot.hpp"

#include <behavior_msgs/action/follow_track.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <serial_msg/msg/motor_command.hpp>
#include <serial_msg/srv/device_command.hpp>
#include <tcp_msg/msg/mpu6500_sample.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include <functional>
#include <string>

namespace ui_bridge {

struct TelemetryStore {
    // imu
    Slot<tcp_msg::msg::MPU6500Sample> imu_left_foot;
    Slot<tcp_msg::msg::MPU6500Sample> imu_left_leg;
    Slot<tcp_msg::msg::MPU6500Sample> imu_right_foot;
    Slot<tcp_msg::msg::MPU6500Sample> imu_right_leg;
    Slot<tcp_msg::msg::MPU6500Sample> imu_body;
    // camera
    Slot<sensor_msgs::msg::CameraInfo> camera_info;
    Slot<sensor_msgs::msg::CompressedImage> compressed_image;
    // tracking
    Slot<vision_msgs::msg::Detection2DArray> scene_detections;
    Slot<vision_msgs::msg::Detection2DArray> tracking_tracks;
};

class BridgeNode : public rclcpp::Node {
public:
    explicit BridgeNode(
        TelemetryStore &store,
        const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
    //////////////////////////////////////////////////////////////////
    /// subscriptions
    //////////////////////////////////////////////////////////////////

    TelemetryStore &store_;

    // imu subscriptions
    rclcpp::Subscription<tcp_msg::msg::MPU6500Sample>::SharedPtr
        imu_left_foot_sub_;
    rclcpp::Subscription<tcp_msg::msg::MPU6500Sample>::SharedPtr
        imu_left_leg_sub_;
    rclcpp::Subscription<tcp_msg::msg::MPU6500Sample>::SharedPtr
        imu_right_foot_sub_;
    rclcpp::Subscription<tcp_msg::msg::MPU6500Sample>::SharedPtr
        imu_right_leg_sub_;
    rclcpp::Subscription<tcp_msg::msg::MPU6500Sample>::SharedPtr imu_body_sub_;

    // camera subscriptions
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr
        camera_info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr
        compressed_image_sub_;

    // detection subscriptions
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr
        scene_detections_sub_;
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr
        tracking_tracks_sub_;

    //////////////////////////////////////////////////////////////////
    /// Commands, Actions
    //////////////////////////////////////////////////////////////////

    // TODO:

    // Command queue
    // std::deque<PendingCommand> cmd_queue_;
    // std::mutex cmd_mutex_;

    // ROS clients (created lazily)
    rclcpp::Publisher<serial_msg::msg::MotorCommand>::SharedPtr motor_cmd_pub_;
    rclcpp::Client<serial_msg::srv::DeviceCommand>::SharedPtr
        device_cmd_client_;

    rclcpp_action::Client<behavior_msgs::action::FollowTrack>::SharedPtr
        follow_track_client_;
    rclcpp_action::ClientGoalHandle<
        behavior_msgs::action::FollowTrack>::SharedPtr active_follow_goal_;

    void
    handle_publish(const std::string &body,
                   const std::function<void(const std::string &)> &respond);

    void handle_call_service(
        const std::string &body,
        const std::function<void(const std::string &)> &respond);

    void handle_action_start(
        const std::string &body,
        const std::function<void(const std::string &)> &respond);

    void handle_action_cancel(
        const std::string &body,
        const std::function<void(const std::string &)> &respond);
};

} // namespace ui_bridge
