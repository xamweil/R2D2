#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <tcp_msg/msg/mpu6500_sample.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include <rclcpp_action/rclcpp_action.hpp>
#include <serial_msg/msg/motor_command.hpp>
#include <serial_msg/srv/device_command.hpp>
#include <behavior_msgs/action/follow_track.hpp>

#include <deque>
#include <functional>
#include <mutex>
#include <string>

namespace ui_bridge_cpp {

struct ImuEntry {
    tcp_msg::msg::MPU6500Sample msg;
    double recv_time = 0.0;
    bool received = false;
};

struct DetectionEntry {
    vision_msgs::msg::Detection2DArray msg;
    double recv_time = 0.0;
    bool received = false;
};

struct PendingCommand {
    std::string kind;  // "publish", "call_service", "action_start", "action_cancel"
    std::string body;  // raw JSON body
    std::function<void(const std::string &)> respond;
};

struct SensorState {
    static constexpr size_t NUM_IMU = 5;
    static constexpr double STALE_SEC = 2.0;
    ImuEntry imu[NUM_IMU];
    DetectionEntry scene_detections;
    DetectionEntry tracking_tracks;
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
    int get_mjpeg_fps() const {
        return mjpeg_fps_;
    }

    void set_broadcast(std::function<void(const std::string &)> broadcast_fn);

    void push_command(PendingCommand cmd);
    void process_commands();

private:
    int port_;
    int mjpeg_fps_;
    std::string doc_root_;
    std::function<void(const std::string &)> broadcast_fn_;

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
    // detection subscriptions
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr
        scene_detections_sub_;
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr
        tracking_tracks_sub_;
    // camera subscriptions
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr
        camera_info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr
        compressed_image_sub_;

    sensor_msgs::msg::CompressedImage::ConstSharedPtr image_;
    std::mutex image_mutex_;

    SensorState sensor_state_;
    rclcpp::TimerBase::SharedPtr state_timer_;

    void camera_info_callback(const sensor_msgs::msg::CameraInfo &msg);
    void compressed_image_callback(
        const sensor_msgs::msg::CompressedImage::ConstSharedPtr &msg);
    void broadcast_state();
    static double now_sec();

    // Command queue
    std::deque<PendingCommand> cmd_queue_;
    std::mutex cmd_mutex_;

    // ROS clients (created lazily)
    rclcpp::Publisher<serial_msg::msg::MotorCommand>::SharedPtr motor_cmd_pub_;
    rclcpp::Client<serial_msg::srv::DeviceCommand>::SharedPtr device_cmd_client_;

    using FollowTrack = behavior_msgs::action::FollowTrack;
    rclcpp_action::Client<FollowTrack>::SharedPtr follow_track_client_;
    rclcpp_action::ClientGoalHandle<FollowTrack>::SharedPtr active_follow_goal_;

    void handle_publish(const std::string &body,
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

    static std::string json_ok();
    static std::string json_error(const std::string &msg);
};

} // namespace ui_bridge_cpp
