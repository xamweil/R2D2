#include "bridge_node.hpp"
#include "http_server.hpp"

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>

#include <algorithm>
#include <thread>

static constexpr int PORT = 9090;

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    try {
        RCLCPP_INFO(rclcpp::get_logger("main"), "hello world!");
        ui_bridge::TelemetryStore store;
        auto node = std::make_shared<ui_bridge::BridgeNode>(store);
        node->declare_parameter("port", PORT);
        node->declare_parameter("doc_root", std::string("/home/ros/frontend"));
        node->declare_parameter("mjpeg_fps", 3);
        node->declare_parameter("state_hz", 10.0);
        node->declare_parameter("stale_sec", 2.0);
        auto port = static_cast<int>(node->get_parameter("port").as_int());
        auto doc_root = node->get_parameter("doc_root").as_string();
        auto mjpeg_fps =
            static_cast<int>(node->get_parameter("mjpeg_fps").as_int());
        auto state_hz = std::max(0.1, node->get_parameter("state_hz").as_double());
        auto stale_sec = node->get_parameter("stale_sec").as_double();

        ui_bridge::HttpServer http_server(store, doc_root, node->get_logger(),
                                          mjpeg_fps, state_hz, stale_sec);

        std::thread ros_thread([&node]() { rclcpp::spin(node); });

        rclcpp::on_shutdown([&http_server]() { http_server.shutdown(); });

        http_server.run(port);

        ros_thread.join();
    } catch (const std::exception &e) {
        RCLCPP_FATAL(rclcpp::get_logger("main"),
                     "Fatal error during startup: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    return 0;
}
