#include "bridge_node.hpp"
#include "http_server.hpp"

#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>

#include <thread>

static constexpr int PORT = 9090;

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    try {
        ui_bridge::TelemetryStore store;
        auto node = std::make_shared<ui_bridge::BridgeNode>(store);
        node->declare_parameter("port", PORT);
        node->declare_parameter("doc_root", std::string("/home/ros/frontend"));
        node->declare_parameter("mjpeg_fps", 3);

        auto port = static_cast<int>(node->get_parameter("port").as_int());
        auto doc_root = node->get_parameter("doc_root").as_string();
        auto mjpeg_fps =
            static_cast<int>(node->get_parameter("mjpeg_fps").as_int());

        ui_bridge::HttpServer http_server(store, doc_root, node->get_logger(),
                                          mjpeg_fps);
        std::thread http_thread(
            [&http_server, &port]() { http_server.run(port); });

        rclcpp::on_shutdown([&http_server]() { http_server.shutdown(); });
        rclcpp::spin(node);
        rclcpp::shutdown();
        http_thread.join();
    } catch (const std::exception &e) {
        RCLCPP_FATAL(rclcpp::get_logger("main"),
                     "Fatal error during startup: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    return 0;
}
