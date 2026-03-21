#include "bridge_node.hpp"
#include "http_server.hpp"

#include <boost/asio/io_context.hpp>
#include <boost/asio/signal_set.hpp>
#include <rclcpp/rclcpp.hpp>

#include <thread>

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ui_bridge_cpp::BridgeNode>();
    auto logger = node->get_logger();

    const auto port = static_cast<unsigned short>(node->get_port());
    const auto &doc_root = node->get_doc_root();

    boost::asio::io_context ioc{1}; // single-threaded hint

    auto server =
        std::make_shared<ui_bridge_ng::HttpServer>(ioc, port, doc_root, logger);
    server->set_image_getter([node]() { return node->get_latest_image(); });
    server->start();

    boost::asio::signal_set signals(ioc, SIGINT, SIGTERM);
    signals.async_wait([&](boost::beast::error_code, int) {
        ioc.stop();
        rclcpp::shutdown();
    });

    std::thread ros_thread([&node]() { rclcpp::spin(node); });

    RCLCPP_INFO(logger, "[http] listening on port %d", port);
    ioc.run();

    rclcpp::shutdown();
    if (ros_thread.joinable()) {
        ros_thread.join();
    }

    return 0;
}
