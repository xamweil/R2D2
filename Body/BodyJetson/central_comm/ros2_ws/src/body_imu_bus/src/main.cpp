#include "rclcpp/rclcpp.hpp"
#include "body_imu_bus/body_imu_bus_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BodyImuBusNode>());
  rclcpp::shutdown();
  return 0;
}