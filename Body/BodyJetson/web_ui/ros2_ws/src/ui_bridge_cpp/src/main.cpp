#include "bridge_node.hpp"
#include "http_server.hpp"

#include <rclcpp/rclcpp.hpp>

#include <thread>

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ui_bridge_cpp::BridgeNode>();
    ui_bridge_cpp::HttpServer http_server(node->get_doc_root(),
                                          node->get_logger());

    rclcpp::on_shutdown([&http_server]() { http_server.shutdown(); });
    std::thread ros_thread([&node]() { rclcpp::spin(node); });
    http_server.run(node->get_port());

    rclcpp::shutdown();
    if (ros_thread.joinable())
        ros_thread.join();

    return 0;
}
