#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

class CameraInfoRelayNode : public rclcpp::Node
{
public:
  CameraInfoRelayNode()
  : Node("camera_info_relay")
  {
    input_topic_ = this->declare_parameter<std::string>(
      "input_topic", "/camera/camera_info");
    output_topic_ = this->declare_parameter<std::string>(
      "output_topic", "/relay/camera/camera_info");

    auto subscription_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    subscription_qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    subscription_qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    auto publisher_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    publisher_qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    publisher_qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    publisher_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
        output_topic_, publisher_qos);

    subscription_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        input_topic_,
        subscription_qos,
        std::bind(&CameraInfoRelayNode::on_message, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "camera_info_relay started | input_topic='%s' output_topic='%s'",
      input_topic_.c_str(), output_topic_.c_str());
  }

private:
  void on_message(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    publisher_->publish(*msg);
  }

  std::string input_topic_;
  std::string output_topic_;

  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CameraInfoRelayNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}