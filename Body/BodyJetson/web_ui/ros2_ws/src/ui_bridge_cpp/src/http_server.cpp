#include "http_server.hpp"

#include "Loop.h"
#include "bridge_node.hpp"
#include "libusockets.h"
#include "state_json.hpp"

#include <rclcpp/logging.hpp>

#include <algorithm>
#include <cstring>
#include <fstream>
#include <string_view>

namespace ui_bridge {

static constexpr const char *MJPEG_BOUNDARY = "mjpegframe";

HttpServer::HttpServer(TelemetryStore &store, std::string doc_root,
                       const rclcpp::Logger &logger, int mjpeg_fps,
                       double state_hz, double stale_sec)
    : store_(&store),
      doc_root_(std::move(doc_root)),
      logger_(logger),
      mjpeg_fps_(mjpeg_fps),
      state_hz_(state_hz),
      stale_sec_(stale_sec) {

    app_.get("/", [this, logger](auto *res, auto * /*req*/) {
        RCLCPP_INFO(logger, "serving %s/index.html", doc_root_.c_str());
        serve_file(res, doc_root_ + "/index.html");
    });

    app_.get("/static/*",
             [this](auto *res, auto *req) { serve_static_file(res, req); });

    uWS::App::WebSocketBehavior<int> behavior;
    behavior.open = [this](auto *ws) {
        ws->subscribe(WS_TOPIC);
        ws->send(build_hello_json(state_hz_, stale_sec_), uWS::OpCode::TEXT);
        RCLCPP_INFO(logger_, "[ws] client connected");
    };
    behavior.message = [](auto * /*ws*/, std::string_view /*msg*/,
                          uWS::OpCode /*op*/) {};
    behavior.close = [this](auto * /*ws*/, int /*code*/,
                            std::string_view /*msg*/) {
        RCLCPP_INFO(logger_, "[ws] client disconnected");
    };

    app_.ws<int>("/ws", std::move(behavior));

    app_.get("/mjpeg",
             [this](auto *res, auto * /*req*/) { serve_mjpeg_stream(res); });

    app_.get("/*", [](auto *res, auto * /*req*/) {
        res->writeStatus("404 Not Found");
        res->writeHeader("Content-Type", "text/plain");
        res->end("Not Found");
    });
}

void HttpServer::run(int port) {
    setup_state_timer();
    setup_mjpeg_timer();

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
        if (state_timer_) {
            us_timer_close(state_timer_);
            state_timer_ = nullptr;
        }
        if (mjpeg_timer_) {
            us_timer_close(mjpeg_timer_);
            mjpeg_timer_ = nullptr;
        }
        app_.close();
    });
}

void HttpServer::setup_state_timer() {
    // Runs on the uWS loop thread, so publishing to websockets is safe here.

    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    auto *loop = reinterpret_cast<us_loop_t *>(uWS::Loop::get());
    state_timer_ = us_create_timer(loop, 0, sizeof(HttpServer *));
    *static_cast<HttpServer **>(us_timer_ext(state_timer_)) = this;

    constexpr double MS_PER_SECOND = 1000.0;
    int period_ms = std::max(1, static_cast<int>(MS_PER_SECOND / state_hz_));
    us_timer_set(
        state_timer_,
        [](struct us_timer_t *t) {
            auto *self = *static_cast<HttpServer **>(us_timer_ext(t));
            self->broadcast_state();
        },
        period_ms, period_ms);
}

void HttpServer::broadcast_state() {
    if (app_.numSubscribers(WS_TOPIC) == 0)
        return;
    app_.publish(WS_TOPIC, build_robot_state_json(*store_, stale_sec_),
                 uWS::OpCode::TEXT);
}

void HttpServer::setup_mjpeg_timer() {
    // Runs on the uWS loop thread, so writing to responses is safe here.

    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    auto *loop = reinterpret_cast<us_loop_t *>(uWS::Loop::get());
    mjpeg_timer_ = us_create_timer(loop, 0, sizeof(HttpServer *));
    *static_cast<HttpServer **>(us_timer_ext(mjpeg_timer_)) = this;

    constexpr int MS_PER_SECOND = 1000;
    int period_ms = std::max(1, MS_PER_SECOND / std::max(1, mjpeg_fps_));
    us_timer_set(
        mjpeg_timer_,
        [](struct us_timer_t *t) {
            auto *self = *static_cast<HttpServer **>(us_timer_ext(t));
            self->broadcast_mjpeg_frame();
        },
        period_ms, period_ms);
}

void HttpServer::broadcast_mjpeg_frame() {
    if (mjpeg_clients_.empty())
        return;

#ifdef MJPEG_TEST_PATTERN
    std::string frame = make_mjpeg_frame(jpeg_generator_.next_frame());
#else
    // Only forward frames that arrived since the last tick.
    auto img = store_->compressed_image.load_if_newer(mjpeg_seen_generation_);
    if (!img || img->data.empty())
        return;
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    std::string frame = make_mjpeg_frame(std::string_view(
        reinterpret_cast<const char *>(img->data.data()), img->data.size()));
#endif

    for (auto *res : mjpeg_clients_) {
        // Drop frames for slow clients instead of piling up backpressure.
        if (buffered_amount(res) != 0)
            continue;
        write_mjpeg_frame(res, frame);
    }
}

void HttpServer::serve_mjpeg_stream(uWS::HttpResponse<false> *res) {
    std::string content_type = "multipart/x-mixed-replace; boundary=";
    content_type += MJPEG_BOUNDARY;

    res->writeHeader("Content-Type", content_type);
    res->writeHeader("Cache-Control", "no-cache, no-store");
    res->writeHeader("Connection", "keep-alive");

    mjpeg_clients_.insert(res);
    RCLCPP_INFO(logger_, "[mjpeg] client connected (%zu total)",
                mjpeg_clients_.size());

    res->onAborted([this, res]() {
        mjpeg_clients_.erase(res);
        RCLCPP_INFO(logger_, "[mjpeg] client disconnected (%zu total)",
                    mjpeg_clients_.size());
    });

    // With an onWritable handler registered uWS neither drains backpressure
    // itself nor arms the idle timeout, so drain here by uncorking an empty
    // cork. Keeps the long-lived stream from being closed after 10 s.
    res->onWritable([res](uintmax_t /*offset*/) {
        res->cork([]() {});
        return true;
    });

#ifndef MJPEG_TEST_PATTERN
    // Send the latest frame right away so the client doesn't wait for the
    // next camera image. Otherwise headers go out with the first frame.
    auto snapshot = store_->compressed_image.load();
    if (snapshot.msg && !snapshot.msg->data.empty()) {
        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
        write_mjpeg_frame(
            res, make_mjpeg_frame(std::string_view(
                     reinterpret_cast<const char *>(snapshot.msg->data.data()),
                     snapshot.msg->data.size())));
    }
#endif
}

void HttpServer::write_mjpeg_frame(uWS::HttpResponse<false> *res,
                                   const std::string &frame) {
    // Cork so the chunk header and frame go out in one send.
    res->cork([res, &frame]() { res->write(frame); });
}

size_t HttpServer::buffered_amount(uWS::HttpResponse<false> *res) {
    // HttpResponse keeps its backpressure buffer private; it lives in the
    // socket extension (same as HttpResponse::getHttpResponseData()).
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    auto *socket = reinterpret_cast<us_socket_t *>(res);
    auto *data =
        static_cast<uWS::HttpResponseData<false> *>(us_socket_ext(0, socket));
    return data->buffer.length();
}

std::string HttpServer::make_mjpeg_frame(std::string_view jpeg) {
    std::string frame;
    constexpr size_t PART_HEADER_RESERVE = 128;
    frame.reserve(jpeg.size() + PART_HEADER_RESERVE);
    frame += "--";
    frame += MJPEG_BOUNDARY;
    frame += "\r\nContent-Type: image/jpeg\r\nContent-Length: ";
    frame += std::to_string(jpeg.size());
    frame += "\r\n\r\n";
    frame += jpeg;
    frame += "\r\n";
    return frame;
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
