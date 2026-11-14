#include "rclcpp/rclcpp.hpp"
#include "teleop_control/TeleopControlNode.hpp"

#include <memory>

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<TeleopControl::TeleopControlNode>();
        rclcpp::spin(node);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("TeleopControlNode"),
                     "Exception in node %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
