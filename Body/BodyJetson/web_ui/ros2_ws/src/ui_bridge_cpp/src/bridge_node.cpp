#include "bridge_node.hpp"

#include <rclcpp/logging.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/qos.hpp>

#include <chrono>
#include <cstdint>

namespace ui_bridge_cpp {

static constexpr uint64_t BROADCAST_FREQ = 100;
static constexpr int PORT = 9090;

BridgeNode::BridgeNode(const rclcpp::NodeOptions &options)
    : Node("ui_bridge_cpp", options) {
    declare_parameter("port", PORT);
    declare_parameter("doc_root", std::string("/home/ros/frontend"));

    port_ = get_parameter("port").as_int();
    doc_root_ = get_parameter("doc_root").as_string();

    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        "/relay/camera/camera_info", rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::CameraInfo &msg) {
            camera_info_callback(msg);
        });

    compressed_image_sub_ =
        create_subscription<sensor_msgs::msg::CompressedImage>(
            "/relay/camera/image_raw/compressed", rclcpp::SensorDataQoS(),
            [this](
                const sensor_msgs::msg::CompressedImage::ConstSharedPtr &msg) {
                compressed_image_callback(msg);
            });

    auto make_imu_cb = [this](size_t idx) {
        return [this, idx](const tcp_msg::msg::MPU6500Sample &msg) {
            auto &e = sensor_state_.imu[idx];
            e.msg = msg;
            e.recv_time = now_sec();
            e.received = true;
        };
    };

    iml_left_foot_sub_ = create_subscription<tcp_msg::msg::MPU6500Sample>(
        "/leg_l/imu/foot", rclcpp::SensorDataQoS(), make_imu_cb(0));

    iml_left_leg_sub_ = create_subscription<tcp_msg::msg::MPU6500Sample>(
        "/leg_l/imu/leg", rclcpp::SensorDataQoS(), make_imu_cb(1));

    iml_right_foot_sub_ = create_subscription<tcp_msg::msg::MPU6500Sample>(
        "/leg_r/imu/foot", rclcpp::SensorDataQoS(), make_imu_cb(2));

    iml_right_leg_sub_ = create_subscription<tcp_msg::msg::MPU6500Sample>(
        "/leg_r/imu/leg", rclcpp::SensorDataQoS(), make_imu_cb(3));

    iml_body_sub_ = create_subscription<tcp_msg::msg::MPU6500Sample>(
        "/Body/mpu", rclcpp::SensorDataQoS(), make_imu_cb(4));

    state_timer_ = create_wall_timer(std::chrono::milliseconds(BROADCAST_FREQ),
                                     [this]() { broadcast_state(); });
}

void BridgeNode::set_broadcast(
    std::function<void(const std::string &)> broadcast_fn) {
    broadcast_fn_ = std::move(broadcast_fn);
}

sensor_msgs::msg::CompressedImage::ConstSharedPtr
BridgeNode::get_latest_image() {
    std::lock_guard<std::mutex> lock(image_mutex_);
    return image_;
}

void BridgeNode::camera_info_callback(
    const sensor_msgs::msg::CameraInfo & /*msg*/) {
}

void BridgeNode::compressed_image_callback(
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr &msg) {
    {
        std::lock_guard<std::mutex> lock(image_mutex_);
        image_ = msg;
    }
    // if (binary_broadcast_fn_) {
    //     binary_broadcast_fn_(
    //         std::string(reinterpret_cast<const char *>(msg->data.data()),
    //                     msg->data.size()));
    // }
}

double BridgeNode::now_sec() {
    using clock = std::chrono::steady_clock;
    return std::chrono::duration<double>(clock::now().time_since_epoch())
        .count();
}

void BridgeNode::broadcast_state() {
    if (!broadcast_fn_)
        return;
    // sensor_state_.to_json(state_json_, now_sec());
    // broadcast_fn_(state_json_.dump());
}

} // namespace ui_bridge_cpp
