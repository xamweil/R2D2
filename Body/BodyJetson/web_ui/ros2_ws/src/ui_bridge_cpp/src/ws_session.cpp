#include "ws_session.hpp"

#include "http_server.hpp"

#include <rclcpp/logging.hpp>

namespace ui_bridge_ng {

namespace beast = boost::beast;
namespace websocket = beast::websocket;
namespace net = boost::asio;

WsSession::WsSession(beast::tcp_stream &&stream, HttpServer *server,
                     rclcpp::Logger logger)
    : ws_(std::move(stream)),
      server_(server),
      logger_(logger) {
}

void WsSession::run(beast::http::request<beast::http::string_body> req) {
    ws_.set_option(
        websocket::stream_base::timeout::suggested(beast::role_type::server));

    ws_.async_accept(req, beast::bind_front_handler(&WsSession::on_accept,
                                                    shared_from_this()));
}

void WsSession::on_accept(beast::error_code ec) {
    if (ec) {
        RCLCPP_WARN(logger_, "ws accept: %s", ec.message().c_str());
        return;
    }
    server_->join(shared_from_this());
    do_read();
}

void WsSession::do_read() {
    ws_.async_read(buf_, beast::bind_front_handler(&WsSession::on_read,
                                                   shared_from_this()));
}

void WsSession::on_read(beast::error_code ec,
                        std::size_t /*bytes_transferred*/) {
    if (ec) {
        server_->leave(shared_from_this());
        return;
    }
    buf_.consume(buf_.size());
    do_read();
}

void WsSession::send(std::shared_ptr<const std::string> msg) {
    queue_.push(std::move(msg));
    if (queue_.size() > 1) {
        return;
    }
    ws_.text(true);
    ws_.async_write(
        net::buffer(*queue_.front()),
        beast::bind_front_handler(&WsSession::on_write, shared_from_this()));
}

void WsSession::on_write(beast::error_code ec,
                         std::size_t /*bytes_transferred*/) {
    if (ec) {
        server_->leave(shared_from_this());
        return;
    }
    queue_.pop();
    if (!queue_.empty()) {
        ws_.text(true);
        ws_.async_write(net::buffer(*queue_.front()),
                        beast::bind_front_handler(&WsSession::on_write,
                                                  shared_from_this()));
    }
}

} // namespace ui_bridge_ng
