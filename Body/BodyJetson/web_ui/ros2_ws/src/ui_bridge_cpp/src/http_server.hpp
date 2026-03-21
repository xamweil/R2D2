#pragma once

#include "mjpeg_session.hpp"

#include <boost/asio.hpp>
#include <boost/beast.hpp>
#include <rclcpp/logger.hpp>

#include <memory>
#include <string>
#include <unordered_set>

namespace ui_bridge_ng {

class WsSession;

class HttpServer : public std::enable_shared_from_this<HttpServer> {
public:
    HttpServer(boost::asio::io_context &ioc, unsigned short port,
               std::string doc_root, rclcpp::Logger &logger);

    const std::string &doc_root() const {
        return doc_root_;
    }

    void start();

    void set_image_getter(ImageGetter fn);
    const ImageGetter &image_getter() const {
        return get_image_;
    }

    void broadcast(std::shared_ptr<const std::string> msg);

    void join(std::shared_ptr<WsSession> session);
    void leave(std::shared_ptr<WsSession> session);

    boost::asio::strand<boost::asio::io_context::executor_type> &strand() {
        return strand_;
    }

private:
    void do_accept();
    void on_accept(boost::beast::error_code ec,
                   boost::asio::ip::tcp::socket socket);

    boost::asio::strand<boost::asio::io_context::executor_type> strand_;
    boost::asio::ip::tcp::acceptor acceptor_;
    std::unordered_set<std::shared_ptr<WsSession>> sessions_;
    std::string doc_root_;
    ImageGetter get_image_;
    rclcpp::Logger logger_;
};

} // namespace ui_bridge_ng
