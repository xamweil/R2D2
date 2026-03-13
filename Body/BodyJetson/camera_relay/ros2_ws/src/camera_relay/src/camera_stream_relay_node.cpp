#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "camera_relay/gst_jpeg_decoder.hpp"

class CameraStreamRelayNode : public rclcpp::Node
{
public:
  CameraStreamRelayNode()
  : Node("camera_stream_relay")
  {
    input_topic_ = this->declare_parameter<std::string>(
      "input_topic", "/camera/image_raw/compressed");
    output_compressed_topic_ = this->declare_parameter<std::string>(
      "output_compressed_topic", "/relay/camera/image_raw/compressed");
    output_raw_topic_ = this->declare_parameter<std::string>(
      "output_raw_topic", "/relay/camera/image_raw");

    auto subscription_qos = rclcpp::SensorDataQoS();

    auto compressed_publisher_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    compressed_publisher_qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    compressed_publisher_qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    auto raw_publisher_qos = rclcpp::QoS(rclcpp::KeepLast(5));
    raw_publisher_qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    raw_publisher_qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    compressed_publisher_ =
      this->create_publisher<sensor_msgs::msg::CompressedImage>(
      output_compressed_topic_, compressed_publisher_qos);

    raw_publisher_ =
      this->create_publisher<sensor_msgs::msg::Image>(
      output_raw_topic_, raw_publisher_qos);

    subscription_ =
      this->create_subscription<sensor_msgs::msg::CompressedImage>(
      input_topic_,
      subscription_qos,
      std::bind(&CameraStreamRelayNode::on_message, this, std::placeholders::_1));

    std::string decoder_error;
    if (!decoder_.initialize(decoder_error)) {
      throw std::runtime_error("Failed to initialize GstJpegDecoder: " + decoder_error);
    }

    RCLCPP_INFO(
      this->get_logger(),
      "camera_stream_relay started | input_topic='%s' output_compressed_topic='%s' output_raw_topic='%s'",
      input_topic_.c_str(),
      output_compressed_topic_.c_str(),
      output_raw_topic_.c_str());
  }

private:
  void on_message(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
  {
    compressed_publisher_->publish(*msg);

    DecodedFrame frame;
    std::string decode_error;
    if (!decoder_.decode(msg->data, frame, decode_error)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        5000,
        "JPEG decode failed: %s",
        decode_error.c_str());
      return;
    }

    sensor_msgs::msg::Image raw_msg;
    raw_msg.header = msg->header;
    raw_msg.height = frame.height;
    raw_msg.width = frame.width;
    raw_msg.encoding = frame.encoding;
    raw_msg.is_bigendian = 0;
    raw_msg.step = frame.step;
    raw_msg.data = std::move(frame.data);

    raw_publisher_->publish(raw_msg);
  }

  std::string input_topic_;
  std::string output_compressed_topic_;
  std::string output_raw_topic_;

  GstJpegDecoder decoder_;

  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr raw_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CameraStreamRelayNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}