#include "bridge_node.hpp"

#include <rclcpp/logging.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/qos.hpp>
#include <tcp_msg/msg/detail/mpu6500_sample__struct.hpp>

namespace ui_bridge {

BridgeNode::BridgeNode(TelemetryStore &store,
                       const rclcpp::NodeOptions &options)
    : Node("ui_bridge_cpp", options),
      store_(store) {

    // imu subscriptions
    imu_left_foot_sub_ = create_subscription<tcp_msg::msg::MPU6500Sample>(
        "/leg_l/imu/foot", rclcpp::SensorDataQoS(),
        [this](const tcp_msg::msg::MPU6500Sample::ConstSharedPtr &msg) {
            store_.imu_left_foot.store(msg);
        });
    imu_left_leg_sub_ = create_subscription<tcp_msg::msg::MPU6500Sample>(
        "/leg_l/imu/leg", rclcpp::SensorDataQoS(),
        [this](const tcp_msg::msg::MPU6500Sample::ConstSharedPtr &msg) {
            store_.imu_left_leg.store(msg);
        });
    imu_right_foot_sub_ = create_subscription<tcp_msg::msg::MPU6500Sample>(
        "/leg_r/imu/foot", rclcpp::SensorDataQoS(),
        [this](const tcp_msg::msg::MPU6500Sample::ConstSharedPtr &msg) {
            store_.imu_right_foot.store(msg);
        });
    imu_right_leg_sub_ = create_subscription<tcp_msg::msg::MPU6500Sample>(
        "/leg_r/imu/leg", rclcpp::SensorDataQoS(),
        [this](const tcp_msg::msg::MPU6500Sample::ConstSharedPtr &msg) {
            store_.imu_right_leg.store(msg);
        });
    imu_body_sub_ = create_subscription<tcp_msg::msg::MPU6500Sample>(
        "/Body/mpu", rclcpp::SensorDataQoS(),
        [this](const tcp_msg::msg::MPU6500Sample::ConstSharedPtr &msg) {
            store_.imu_body.store(msg);
        });

    // camera subscriptions
    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        "/relay/camera/camera_info", rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::CameraInfo::ConstSharedPtr &msg) {
            store_.camera_info.store(msg);
        });
    compressed_image_sub_ =
        create_subscription<sensor_msgs::msg::CompressedImage>(
            "/relay/camera/image_raw/compressed", rclcpp::SensorDataQoS(),
            [this](
                const sensor_msgs::msg::CompressedImage::ConstSharedPtr &msg) {
                store_.compressed_image.store(msg);
            });

    // tracking subscriptions
    scene_detections_sub_ =
        create_subscription<vision_msgs::msg::Detection2DArray>(
            "/scene_understanding/detections", rclcpp::SensorDataQoS(),
            [this](
                const vision_msgs::msg::Detection2DArray::ConstSharedPtr &msg) {
                store_.scene_detections.store(msg);
            });
    tracking_tracks_sub_ =
        create_subscription<vision_msgs::msg::Detection2DArray>(
            "/tracking/tracks", rclcpp::SensorDataQoS(),
            [this](
                const vision_msgs::msg::Detection2DArray::ConstSharedPtr &msg) {
                store_.tracking_tracks.store(msg);
            });
}

} // namespace ui_bridge
