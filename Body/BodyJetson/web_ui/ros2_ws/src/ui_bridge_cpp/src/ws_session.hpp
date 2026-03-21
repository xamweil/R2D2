#pragma once

#include <boost/asio.hpp>
#include <boost/beast.hpp>
#include <rclcpp/logger.hpp>

#include <memory>
#include <queue>
#include <string>

namespace ui_bridge_ng {

class HttpServer;

class WsSession : public std::enable_shared_from_this<WsSession> {
public:
    WsSession(boost::beast::tcp_stream &&stream, HttpServer *server,
              rclcpp::Logger logger);

    void run(boost::beast::http::request<boost::beast::http::string_body> req);
    void send(std::shared_ptr<const std::string> msg);

private:
    void on_accept(boost::beast::error_code ec);
    void do_read();
    void on_read(boost::beast::error_code ec, std::size_t bytes_transferred);
    void on_write(boost::beast::error_code ec, std::size_t bytes_transferred);

    boost::beast::websocket::stream<boost::beast::tcp_stream> ws_;
    HttpServer *server_;
    rclcpp::Logger logger_;
    boost::beast::flat_buffer buf_;
    std::queue<std::shared_ptr<const std::string>> queue_;
};

} // namespace ui_bridge_ng
