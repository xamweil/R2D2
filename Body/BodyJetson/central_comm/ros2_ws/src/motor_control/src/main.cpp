#include "MotorControl.h"

#include <rclcpp/executors.hpp>
#include <rclcpp/utilities.hpp>

#include <memory>

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorControl>());
    rclcpp::shutdown();
    return 0;
}
