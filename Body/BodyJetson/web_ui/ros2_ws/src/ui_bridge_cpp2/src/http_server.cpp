#include "http_server.hpp"

#include "bridge_node.hpp"

#include <rclcpp/logging.hpp>

#include <cstring>
#include <fstream>
#include <string_view>

namespace ui_bridge {

HttpServer::HttpServer(TelemetryStore &store, std::string doc_root,
                       const rclcpp::Logger &logger, int mjpeg_fps)
    : store_(&store),
      doc_root_(std::move(doc_root)),
      logger_(logger),
      mjpeg_fps_(mjpeg_fps) {

    app_.get("/", [this, logger](auto *res, auto * /*req*/) {
        RCLCPP_INFO(logger, "serving %s/index.html", doc_root_.c_str());
        serve_file(res, doc_root_ + "/index.html");
    });

    app_.get("/static/*",
             [this](auto *res, auto *req) { serve_static_file(res, req); });

    uWS::App::WebSocketBehavior<int> behavior;
    behavior.open = [this](auto *ws) {
        ws->subscribe(WS_TOPIC);
        ws->send(R"({"type":"hello","payload":{}})", uWS::OpCode::TEXT);
        RCLCPP_INFO(logger_, "[ws] client connected");
    };
    behavior.message = [](auto * /*ws*/, std::string_view /*msg*/,
                          uWS::OpCode /*op*/) {};
    behavior.close = [this](auto * /*ws*/, int /*code*/,
                            std::string_view /*msg*/) {
        RCLCPP_INFO(logger_, "[ws] client disconnected");
    };

    app_.ws<int>("/ws", std::move(behavior));

    app_.get("/*", [](auto *res, auto * /*req*/) {
        res->writeStatus("404 Not Found");
        res->writeHeader("Content-Type", "text/plain");
        res->end("Not Found");
    });
}

void HttpServer::run(int port) {
    app_.listen(port, [this, port](auto *socket) {
        if (socket) {
            RCLCPP_INFO(logger_, "[http] listening on port %d", port);
        } else {
            RCLCPP_FATAL(logger_, "[http] failed to listen on port %d", port);
        }
    });

    app_.run();
}

void HttpServer::shutdown() {
    app_.getLoop()->defer([this]() {
        // us_timer_close(mjpeg_timer_);
        app_.close();
    });
}

void HttpServer::serve_static_file(uWS::HttpResponse<false> *res,
                                   uWS::HttpRequest *req) {
    auto url = req->getUrl();
    static constexpr size_t STATIC_PREFIX_LEN = 8;
    if (url.size() <= STATIC_PREFIX_LEN) {
        res->writeStatus("404 Not Found");
        res->writeHeader("Content-Type", "text/plain");
        res->end("Not Found");
        return;
    }
    auto rel = url.substr(STATIC_PREFIX_LEN);
    if (rel.empty() || !is_safe_path(rel)) {
        res->writeStatus("404 Not Found");
        res->writeHeader("Content-Type", "text/plain");
        res->end("Not Found");
        return;
    }

    serve_file(res, doc_root_ + "/" + std::string(rel));
}

void HttpServer::serve_file(uWS::HttpResponse<false> *res,
                            const std::string &path) {
    std::string body;
    if (!read_file(path, body)) {
        res->writeStatus("404 Not Found");
        res->writeHeader("Content-Type", "text/plain");
        res->end("Not Found");
        return;
    }
    res->writeHeader("Content-Type", content_type(path));
    res->end(body);
}

bool HttpServer::read_file(const std::string &path, std::string &out) {
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f)
        return false;
    auto size = f.tellg();
    if (size < 0)
        return false;
    out.resize(static_cast<size_t>(size));
    f.seekg(0);
    f.read(out.data(), size);
    return f.good();
}

bool HttpServer::is_safe_path(std::string_view path) {
    return path.find("..") == std::string_view::npos;
}

std::string_view HttpServer::content_type(std::string_view path) {
    auto dot = path.rfind('.');
    if (dot == std::string_view::npos)
        return "application/octet-stream";
    auto ext = path.substr(dot);
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
    if (ext == ".jpg" || ext == ".jpeg")
        return "image/jpeg";
    if (ext == ".svg")
        return "image/svg+xml";
    if (ext == ".ico")
        return "image/x-icon";
    if (ext == ".ttf")
        return "font/ttf";
    if (ext == ".woff")
        return "font/woff";
    if (ext == ".woff2")
        return "font/woff2";
    return "application/octet-stream";
}

} // namespace ui_bridge
