#include "http_server.hpp"

#include "http_session.hpp"
#include "ws_session.hpp"

#include <rclcpp/logging.hpp>

namespace ui_bridge_ng {

namespace net = boost::asio;
namespace beast = boost::beast;
using tcp = net::ip::tcp;

HttpServer::HttpServer(net::io_context &ioc, unsigned short port,
                       std::string doc_root, rclcpp::Logger &logger)
    : strand_(net::make_strand(ioc)),
      acceptor_(ioc, tcp::endpoint(tcp::v4(), port)),
      doc_root_(std::move(doc_root)),
      logger_(logger) {
    acceptor_.set_option(net::socket_base::reuse_address(true));
}

void HttpServer::start() {
    do_accept();
}

void HttpServer::do_accept() {
    acceptor_.async_accept(
        net::make_strand(strand_.get_inner_executor()),
        beast::bind_front_handler(&HttpServer::on_accept, shared_from_this()));
}

void HttpServer::on_accept(beast::error_code ec, tcp::socket socket) {
    if (ec) {
        RCLCPP_ERROR(logger_, "accept: %s", ec.message().c_str());
    } else {
        std::make_shared<HttpSession>(std::move(socket), this, doc_root_,
                                      logger_)
            ->run();
    }
    do_accept();
}

void HttpServer::set_image_getter(ImageGetter fn) {
    get_image_ = std::move(fn);
}

void HttpServer::broadcast(std::shared_ptr<const std::string> msg) {
    net::post(strand_, [self = shared_from_this(), msg = std::move(msg)]() {
        for (const auto &session : self->sessions_) {
            session->send(msg);
        }
    });
}

void HttpServer::join(std::shared_ptr<WsSession> session) {
    net::post(strand_,
              [self = shared_from_this(), session = std::move(session)]() {
                  self->sessions_.insert(session);
              });
}

void HttpServer::leave(std::shared_ptr<WsSession> session) {
    net::post(strand_,
              [self = shared_from_this(), session = std::move(session)]() {
                  self->sessions_.erase(session);
              });
}

} // namespace ui_bridge_ng
