#pragma once

#include <boost/asio.hpp>
#include <boost/beast.hpp>
#include <rclcpp/logger.hpp>

#include <memory>
#include <string>

namespace ui_bridge_ng {

class HttpServer;

class HttpSession : public std::enable_shared_from_this<HttpSession> {
public:
    HttpSession(boost::asio::ip::tcp::socket &&socket, HttpServer *server,
                std::string doc_root, rclcpp::Logger logger);

    void run();

private:
    void do_read();
    void on_read(boost::beast::error_code ec, std::size_t bytes_transferred);
    void handle_request(
        boost::beast::http::request<boost::beast::http::string_body> &&req);

    boost::beast::tcp_stream stream_;
    boost::beast::flat_buffer buf_;
    boost::beast::http::request<boost::beast::http::string_body> req_;
    HttpServer *server_;
    std::string doc_root_;
    rclcpp::Logger logger_;
};

} // namespace ui_bridge_ng
