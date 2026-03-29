#include "bridge_node.hpp"

#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>
#include <rclcpp/logging.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/qos.hpp>

#include <chrono>
#include <cstdint>

namespace ui_bridge_cpp {

static constexpr uint64_t BROADCAST_FREQ = 100;
static constexpr const char *IMU_ALIASES[SensorState::NUM_IMU] = {
    "imu_left_foot", "imu_left_leg", "imu_right_foot", "imu_right_leg",
    "imu_body"};
static constexpr int PORT = 9090;

BridgeNode::BridgeNode(const rclcpp::NodeOptions &options)
    : Node("ui_bridge_cpp", options) {
    declare_parameter("port", PORT);
    declare_parameter("doc_root", std::string("/home/ros/frontend"));
    declare_parameter("mjpeg_fps", 3);

    port_ = static_cast<int>(get_parameter("port").as_int());
    doc_root_ = get_parameter("doc_root").as_string();
    mjpeg_fps_ = static_cast<int>(get_parameter("mjpeg_fps").as_int());

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

    imu_left_foot_sub_ = create_subscription<tcp_msg::msg::MPU6500Sample>(
        "/leg_l/imu/foot", rclcpp::SensorDataQoS(), make_imu_cb(0));

    imu_left_leg_sub_ = create_subscription<tcp_msg::msg::MPU6500Sample>(
        "/leg_l/imu/leg", rclcpp::SensorDataQoS(), make_imu_cb(1));

    imu_right_foot_sub_ = create_subscription<tcp_msg::msg::MPU6500Sample>(
        "/leg_r/imu/foot", rclcpp::SensorDataQoS(), make_imu_cb(2));

    imu_right_leg_sub_ = create_subscription<tcp_msg::msg::MPU6500Sample>(
        "/leg_r/imu/leg", rclcpp::SensorDataQoS(), make_imu_cb(3));

    imu_body_sub_ = create_subscription<tcp_msg::msg::MPU6500Sample>(
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
}

double BridgeNode::now_sec() {
    using clock = std::chrono::steady_clock;
    return std::chrono::duration<double>(clock::now().time_since_epoch())
        .count();
}

void BridgeNode::broadcast_state() {
    if (!broadcast_fn_)
        return;

    double now = now_sec();
    rapidjson::StringBuffer buf;
    rapidjson::Writer<rapidjson::StringBuffer> w(buf);

    w.StartObject();
    w.Key("type");
    w.String("robot_state");
    w.Key("payload");
    w.StartObject();
    w.Key("t");
    w.Double(now);
    w.Key("stale_sec");
    w.Double(SensorState::STALE_SEC);
    w.Key("entries");
    w.StartObject();

    for (size_t i = 0; i < SensorState::NUM_IMU; ++i) {
        const auto &e = sensor_state_.imu[i];
        w.Key(IMU_ALIASES[i]);
        w.StartObject();
        w.Key("present");
        w.Bool(e.received);
        w.Key("stale");
        w.Bool(!e.received || (now - e.recv_time) > SensorState::STALE_SEC);
        w.Key("age");
        w.Double(e.received ? now - e.recv_time : -1.0);
        w.Key("data");
        if (e.received) {
            w.StartObject();
            w.Key("accel");
            w.StartArray();
            for (int j = 0; j < 3; ++j)
                w.Int(e.msg.accel[j]);
            w.EndArray();
            w.Key("gyro");
            w.StartArray();
            for (int j = 0; j < 3; ++j)
                w.Int(e.msg.gyro[j]);
            w.EndArray();
            w.Key("ts_ms");
            w.Uint(e.msg.ts_ms);
            w.EndObject();
        } else {
            w.Null();
        }
        w.EndObject();
    }

    w.EndObject(); // entries
    w.EndObject(); // payload
    w.EndObject();

    broadcast_fn_(std::string(buf.GetString(), buf.GetSize()));
}

} // namespace ui_bridge_cpp
