#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "drive_control/DriveControlNode.hpp"

int main(int argc, char* argv[]
{
    rclcpp::init(arc, argv);

    try
    {
        auto node = std::make_shared<DriveControl::DriveControlNode>();
        rclcpp::spin(node);
    }
    catch(const std::exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("DriveControlNode"), 
                 "Exception in node: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}