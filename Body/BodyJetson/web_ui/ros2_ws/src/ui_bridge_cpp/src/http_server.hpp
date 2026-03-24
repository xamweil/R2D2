#pragma once

#ifdef MJPEG_TEST_PATTERN
#include "jpeg_generator.hpp"
#endif

#include <App.h>
#include <rclcpp/logger.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include <functional>
#include <string>

namespace ui_bridge_cpp {

class HttpServer {
public:
    HttpServer(std::string doc_root, const rclcpp::Logger &logger,
               int mjpeg_fps);

    void run(int port);
    void shutdown();

    void set_image_source(
        std::function<sensor_msgs::msg::CompressedImage::ConstSharedPtr()> fn);

private:
    std::string doc_root_;
    rclcpp::Logger logger_;
    int mjpeg_fps_;
    uWS::App app_;
    std::function<sensor_msgs::msg::CompressedImage::ConstSharedPtr()>
        image_source_;
#ifdef MJPEG_TEST_PATTERN
    JpegGenerator jpeg_generator_;
#endif
    std::set<uWS::HttpResponse<false> *> mjpeg_clients_;
    struct us_timer_t *mjpeg_timer_ = nullptr;

    void setup_mjpeg_timer();
    void serve_static_file(uWS::HttpResponse<false> *res,
                           uWS::HttpRequest *req);
    void serve_mjpeg_stream(uWS::HttpResponse<false> *res,
                            uWS::HttpRequest *req);

    static void serve_file(uWS::HttpResponse<false> *res,
                           const std::string &path);
    static bool read_file(const std::string &path, std::string &out);
    static bool is_safe_path(std::string_view path);
    static std::string_view content_type(std::string_view path);
};

} // namespace ui_bridge_cpp
