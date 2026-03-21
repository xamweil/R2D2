#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <tcp_msg/msg/mpu6500_sample.hpp>

#include <functional>
#include <mutex>
#include <string>

namespace ui_bridge_cpp {

struct ImuEntry {
    tcp_msg::msg::MPU6500Sample msg;
    double recv_time = 0.0;
    bool received = false;
};

struct SensorState {
    static constexpr size_t NUM_IMU = 5;
    static constexpr const char *IMU_NAMES[NUM_IMU] = {
        "imu_left_foot", "imu_left_leg", "imu_right_foot", "imu_right_leg",
        "imu_body"};
    static constexpr double STALE_SEC = 2.0;
    ImuEntry imu[NUM_IMU];
};

class BridgeNode : public rclcpp::Node {
public:
    explicit BridgeNode(
        const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    sensor_msgs::msg::CompressedImage::ConstSharedPtr get_latest_image();

    int get_port() const {
        return port_;
    }
    const std::string &get_doc_root() const {
        return doc_root_;
    }

    void set_broadcast(std::function<void(const std::string &)> broadcast_fn);
    void set_binary_broadcast(std::function<void(const std::string &)> fn);

private:
    void camera_info_callback(const sensor_msgs::msg::CameraInfo &msg);
    void compressed_image_callback(
        const sensor_msgs::msg::CompressedImage::ConstSharedPtr &msg);
    void broadcast_state();
    static double now_sec();

    int port_;
    std::string doc_root_;
    std::function<void(const std::string &)> broadcast_fn_;
    std::function<void(const std::string &)> binary_broadcast_fn_;

    // imu subscriptions
    rclcpp::Subscription<tcp_msg::msg::MPU6500Sample>::SharedPtr
        iml_left_foot_sub_;
    rclcpp::Subscription<tcp_msg::msg::MPU6500Sample>::SharedPtr
        iml_left_leg_sub_;
    rclcpp::Subscription<tcp_msg::msg::MPU6500Sample>::SharedPtr
        iml_right_foot_sub_;
    rclcpp::Subscription<tcp_msg::msg::MPU6500Sample>::SharedPtr
        iml_right_leg_sub_;
    rclcpp::Subscription<tcp_msg::msg::MPU6500Sample>::SharedPtr iml_body_sub_;
    // camera subscriptions
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr
        camera_info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr
        compressed_image_sub_;

    sensor_msgs::msg::CompressedImage::ConstSharedPtr image_;
    std::mutex image_mutex_;

    SensorState sensor_state_;
    rclcpp::TimerBase::SharedPtr state_timer_;
};

} // namespace ui_bridge_cpp
