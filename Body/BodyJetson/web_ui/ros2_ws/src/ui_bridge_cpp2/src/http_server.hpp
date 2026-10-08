#pragma once

#include "bridge_node.hpp"
#ifdef MJPEG_TEST_PATTERN
#include "jpeg_generator.hpp"
#endif

#include <App.h>
#include <rclcpp/logger.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include <set>
#include <string>

namespace ui_bridge {

class HttpServer {
public:
    static constexpr const char *WS_TOPIC = "broadcast";

    HttpServer(ui_bridge::TelemetryStore &store, std::string doc_root,
               const rclcpp::Logger &logger, int mjpeg_fps, double state_hz,
               double stale_sec);

    void run(int port);
    void shutdown();

private:
    TelemetryStore *store_;
    std::string doc_root_;
    rclcpp::Logger logger_;
    int mjpeg_fps_;
    double state_hz_;
    double stale_sec_;
    uWS::App app_;
    struct us_timer_t *state_timer_ = nullptr;
#ifdef MJPEG_TEST_PATTERN
    ui_bridge_cpp::JpegGenerator jpeg_generator_;
#else
    uint64_t mjpeg_seen_generation_ = 0;
#endif
    void setup_state_timer();
    void broadcast_state();
    void serve_static_file(uWS::HttpResponse<false> *res,
                           uWS::HttpRequest *req);

    std::set<uWS::HttpResponse<false> *> mjpeg_clients_;
    struct us_timer_t *mjpeg_timer_ = nullptr;
    void setup_mjpeg_timer();
    void broadcast_mjpeg_frame();
    void serve_mjpeg_stream(uWS::HttpResponse<false> *res);

    static void write_mjpeg_frame(uWS::HttpResponse<false> *res,
                                  const std::string &frame);
    static size_t buffered_amount(uWS::HttpResponse<false> *res);
    static std::string make_mjpeg_frame(std::string_view jpeg);

    static void serve_file(uWS::HttpResponse<false> *res,
                           const std::string &path);
    static bool read_file(const std::string &path, std::string &out);
    static bool is_safe_path(std::string_view path);
    static std::string_view content_type(std::string_view path);
};

} // namespace ui_bridge
