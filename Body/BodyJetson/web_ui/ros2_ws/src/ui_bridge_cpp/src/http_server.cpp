#include "http_server.hpp"

#include <rclcpp/logging.hpp>

#include <cstring>
#include <fstream>
#include <string_view>

namespace ui_bridge_cpp {

static constexpr std::string_view BOUNDARY = "mjpeg_frame_boundary";

HttpServer::HttpServer(std::string doc_root, const rclcpp::Logger &logger)
    : doc_root_(std::move(doc_root)),
      logger_(logger)
     {

    app_.get("/", [this](auto *res, auto * /*req*/) {
        serve_file(res, doc_root_ + "/index.html");
    });

    app_.get("/static/*",
             [this](auto *res, auto *req) { serve_static_file(res, req); });

    app_.get("/mjpeg",
             [this](auto *res, auto *req) { serve_mjpeg_stream(res, req); });

    app_.get("/*", [](auto *res, auto * /*req*/) {
        res->writeStatus("404 Not Found");
        res->writeHeader("Content-Type", "text/plain");
        res->end("Not Found");
    });

    setup_mjpeg_timer();
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
        us_timer_close(mjpeg_timer_);
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

void HttpServer::serve_mjpeg_stream(uWS::HttpResponse<false> *res,
                                    uWS::HttpRequest * /*req*/) {
    std::string content_type = "multipart/x-mixed-replace; boundary=";
    content_type += BOUNDARY;

    res->writeHeader("Content-Type", content_type);
    res->writeHeader("Cache-Control", "no-cache, no-store");
    res->writeHeader("Connection", "keep-alive");
    res->writeHeader("Access-Control-Allow-Origin", "*");

    mjpeg_clients_.insert(res);
    RCLCPP_INFO(logger_, "[+] Client connected (%zu total)\n", mjpeg_clients_.size());

    res->onAborted([this, res]() {
        mjpeg_clients_.erase(res);
        RCLCPP_INFO(logger_, "[-] Client disconnected (%zu total)\n", mjpeg_clients_.size());
    });
}

// NOLINTBEGIN
const int TARGET_FPS = 15;

std::string make_mjpeg_frame(const std::string &jpeg_data) {
    std::string frame;
    frame.reserve(jpeg_data.size() + 128);
    frame += "--";
    frame += BOUNDARY;
    frame += "\r\n";
    frame += "Content-Type: image/jpeg\r\n";
    frame += "Content-Length: ";
    frame += std::to_string(jpeg_data.size());
    frame += "\r\n\r\n";
    frame += jpeg_data;
    frame += "\r\n";
    return frame;
}

void HttpServer::setup_mjpeg_timer() {
    mjpeg_timer_ = us_create_timer((struct us_loop_t *)uWS::Loop::get(), 0,
                                   sizeof(void *));

    HttpServer *self_ptr = this;
    memcpy(us_timer_ext(mjpeg_timer_), &self_ptr, sizeof(HttpServer *));

    us_timer_set(
        mjpeg_timer_,
        [](struct us_timer_t *t) {
            HttpServer *self;
            memcpy(&self, us_timer_ext(t), sizeof(HttpServer *));

            if (self->mjpeg_clients_.empty())
                return;

            std::string jpeg = self->jpeg_generator_.next_frame();
            std::string frame = make_mjpeg_frame(jpeg);
            std::string_view frame_view(frame.data(), frame.size());

            for (auto *res : self->mjpeg_clients_) {
                bool ok = res->write(frame_view);
                (void)ok;
            }
        },
        1000 / TARGET_FPS, 1000 / TARGET_FPS);
}
// NOLINTEND

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

} // namespace ui_bridge_cpp
