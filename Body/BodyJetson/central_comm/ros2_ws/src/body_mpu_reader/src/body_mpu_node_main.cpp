#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "body_mpu_reader/body_mpu_node.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<body_mpu_reader::BodyMPUNode>();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("body_mpu_node"), 
                 "Exception in node: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}