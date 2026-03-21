#include "http_session.hpp"

#include "http_server.hpp"
#include "mjpeg_session.hpp"
#include "ws_session.hpp"

#include <boost/beast/http/file_body.hpp>
#include <rclcpp/logging.hpp>

namespace ui_bridge_ng {

namespace beast = boost::beast;
namespace http = beast::http;
namespace net = boost::asio;
using tcp = net::ip::tcp;

namespace {
beast::string_view content_type(beast::string_view path) {
    auto pos = path.rfind('.');
    if (pos == beast::string_view::npos)
        return "application/octet-stream";
    auto ext = path.substr(pos);
    if (ext == ".html")
        return "text/html";
    if (ext == ".css")
        return "text/css";
    if (ext == ".js")
        return "application/javascript";
    if (ext == ".json")
        return "application/json";
    if (ext == ".png")
        return "image/png";
    if (ext == ".jpg")
        return "image/jpeg";
    if (ext == ".svg")
        return "image/svg+xml";
    if (ext == ".ico")
        return "image/x-icon";
    if (ext == ".woff")
        return "font/woff";
    if (ext == ".woff2")
        return "font/woff2";
    if (ext == ".ttf")
        return "font/ttf";
    return "application/octet-stream";
}
} // namespace

HttpSession::HttpSession(tcp::socket &&socket, HttpServer *server,
                         std::string doc_root, rclcpp::Logger logger)
    : stream_(std::move(socket)),
      server_(server),
      doc_root_(std::move(doc_root)),
      logger_(logger) {
}

void HttpSession::run() {
    do_read();
}

void HttpSession::do_read() {
    req_ = {};
    stream_.expires_after(std::chrono::seconds(30));

    http::async_read(
        stream_, buf_, req_,
        beast::bind_front_handler(&HttpSession::on_read, shared_from_this()));
}

void HttpSession::on_read(beast::error_code ec,
                          std::size_t /*bytes_transferred*/) {
    if (ec) {
        return;
    }
    handle_request(std::move(req_));
}

void HttpSession::handle_request(http::request<http::string_body> &&req) {
    // WebSocket upgrade
    if (beast::websocket::is_upgrade(req)) {
        auto ws =
            std::make_shared<WsSession>(std::move(stream_), server_, logger_);
        ws->run(std::move(req));
        return;
    }

    auto const version = req.version();
    auto const keep = req.keep_alive();

    // Helper: send any response and manage connection lifecycle.
    auto self = shared_from_this();
    auto send_string = [self, version, keep](http::status status,
                                             beast::string_view ctype,
                                             std::string body) {
        auto res = std::make_shared<http::response<http::string_body>>(status,
                                                                       version);
        res->set(http::field::content_type, ctype);
        res->body() = std::move(body);
        res->prepare_payload();
        res->keep_alive(keep);
        http::async_write(self->stream_, *res,
                          [self, res](beast::error_code ec, std::size_t) {
                              if (ec)
                                  return;
                              if (res->need_eof()) {
                                  beast::error_code shutdown_ec;
                                  self->stream_.socket().shutdown(
                                      tcp::socket::shutdown_send, shutdown_ec);
                              }
                          });
    };

    // MJPEG stream (target may include query string, e.g. /mjpeg?ts=...)
    if (req.method() == http::verb::get && req.target().starts_with("/mjpeg")) {
        auto const &getter = server_->image_getter();
        if (!getter) {
            send_string(http::status::service_unavailable, "text/plain",
                        "Camera not available");
            return;
        }
        std::make_shared<MjpegSession>(std::move(stream_), getter, logger_)
            ->run(req.version());
        return;
    }

    // GET /health
    if (req.method() == http::verb::get && req.target() == "/health") {
        send_string(http::status::ok, "application/json", R"({"status":"ok"})");
        return;
    }

    // Static file serving
    if (req.method() == http::verb::get && !doc_root_.empty()) {
        std::string rel_path;
        auto target = req.target();

        if (target == "/") {
            rel_path = "index.html";
        } else if (target.starts_with("/static/")) {
            rel_path = std::string(target.substr(8));
        }

        if (!rel_path.empty()) {
            if (rel_path.find("..") != std::string::npos) {
                send_string(http::status::not_found, "text/plain", "Not Found");
                return;
            }

            std::string full_path = doc_root_ + "/" + rel_path;

            beast::error_code ec;
            http::file_body::value_type body;
            body.open(full_path.c_str(), beast::file_mode::scan, ec);

            if (!ec) {
                auto const size = body.size();
                auto res = std::make_shared<http::response<http::file_body>>(
                    std::piecewise_construct, std::make_tuple(std::move(body)),
                    std::make_tuple(http::status::ok, version));
                res->set(http::field::content_type, content_type(full_path));
                res->content_length(size);
                res->keep_alive(keep);
                http::async_write(
                    self->stream_, *res,
                    [self, res](beast::error_code ec, std::size_t) {
                        if (ec)
                            return;
                        if (res->need_eof()) {
                            beast::error_code shutdown_ec;
                            self->stream_.socket().shutdown(
                                tcp::socket::shutdown_send, shutdown_ec);
                        }
                    });
                return;
            }
        }
    }

    // 404
    send_string(http::status::not_found, "text/plain", "Not Found");
}

} // namespace ui_bridge_ng
