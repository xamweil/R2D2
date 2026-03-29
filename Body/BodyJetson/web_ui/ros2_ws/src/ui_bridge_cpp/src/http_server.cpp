#include "http_server.hpp"

#include <rclcpp/logging.hpp>

#include <cstring>
#include <fstream>
#include <string_view>

namespace ui_bridge_cpp {

static constexpr std::string_view BOUNDARY = "mjpeg_frame_boundary";

HttpServer::HttpServer(std::string doc_root, const rclcpp::Logger &logger,
                       int mjpeg_fps)
    : doc_root_(std::move(doc_root)),
      logger_(logger),
      mjpeg_fps_(mjpeg_fps) {

    app_.get("/", [this](auto *res, auto * /*req*/) {
        serve_file(res, doc_root_ + "/index.html");
    });

    app_.get("/static/*",
             [this](auto *res, auto *req) { serve_static_file(res, req); });

    app_.get("/mjpeg",
             [this](auto *res, auto *req) { serve_mjpeg_stream(res, req); });

    app_.ws<int>(
        "/ws",
        {.open =
             [this](auto *ws) {
                 ws->subscribe(WS_TOPIC);
                 ws->send(R"({"type":"hello","payload":{}})",
                          uWS::OpCode::TEXT);
                 RCLCPP_INFO(logger_, "[ws] client connected");
             },
         .message = [](auto * /*ws*/, std::string_view /*msg*/,
                       uWS::OpCode /*op*/) {},
         .close =
             [this](auto * /*ws*/, int /*code*/, std::string_view /*msg*/) {
                 RCLCPP_INFO(logger_, "[ws] client disconnected");
             }});

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

void HttpServer::broadcast(const std::string &message) {
    app_.getLoop()->defer([this, message]() {
        app_.publish(WS_TOPIC, message, uWS::OpCode::TEXT);
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
    RCLCPP_INFO(logger_, "[+] Client connected (%zu total)\n",
                mjpeg_clients_.size());

    res->onAborted([this, res]() {
        mjpeg_clients_.erase(res);
        RCLCPP_INFO(logger_, "[-] Client disconnected (%zu total)\n",
                    mjpeg_clients_.size());
    });
}

// NOLINTBEGIN

std::string make_mjpeg_header(size_t content_length) {
    std::string header;
    header.reserve(128);
    header += "--";
    header += BOUNDARY;
    header += "\r\n";
    header += "Content-Type: image/jpeg\r\n";
    header += "Content-Length: ";
    header += std::to_string(content_length);
    header += "\r\n\r\n";
    return header;
}

void HttpServer::set_image_source(
    std::function<sensor_msgs::msg::CompressedImage::ConstSharedPtr()> fn) {
    image_source_ = std::move(fn);
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

#ifdef MJPEG_TEST_PATTERN
            std::string test_jpeg = self->jpeg_generator_.next_frame();
            const char *data = test_jpeg.data();
            size_t size = test_jpeg.size();
#else
            if (!self->image_source_)
                return;
            auto img = self->image_source_();
            if (!img || img->data.empty())
                return;
            const char *data = reinterpret_cast<const char *>(img->data.data());
            size_t size = img->data.size();
#endif

            std::string header = make_mjpeg_header(size);
            std::string_view img_view(data, size);

            for (auto *res : self->mjpeg_clients_) {
                res->write(header);
                res->write(img_view);
                res->write("\r\n");
            }
        },
        1000 / self_ptr->mjpeg_fps_, 1000 / self_ptr->mjpeg_fps_);
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
