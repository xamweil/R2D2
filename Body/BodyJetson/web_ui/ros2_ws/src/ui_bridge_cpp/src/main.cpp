#include "bridge_node.hpp"
#include "http_server.hpp"

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <thread>

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ui_bridge_cpp::BridgeNode>();
    ui_bridge_cpp::HttpServer http_server(node->get_doc_root(),
                                          node->get_logger(),
                                          node->get_mjpeg_fps());
    http_server.set_image_source(
        [&node]() { return node->get_latest_image(); });
    node->set_broadcast(
        [&http_server](const std::string &msg) { http_server.broadcast(msg); });

    http_server.set_command_handler(
        [&node](const std::string &kind, const std::string &body,
                std::function<void(const std::string &)> respond) {
            node->push_command({kind, body, std::move(respond)});
        });

    rclcpp::on_shutdown([&http_server]() { http_server.shutdown(); });

    std::thread ros_thread([&node]() {
        while (rclcpp::ok()) {
            rclcpp::spin_some(node);
            node->process_commands();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });
    http_server.run(node->get_port());

    rclcpp::shutdown();
    if (ros_thread.joinable())
        ros_thread.join();

    return 0;
}
