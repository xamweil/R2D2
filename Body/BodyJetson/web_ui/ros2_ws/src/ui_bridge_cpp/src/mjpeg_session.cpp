#include "mjpeg_session.hpp"

#include <rclcpp/logging.hpp>

#include <array>
#include <chrono>

namespace ui_bridge_ng {

namespace beast = boost::beast;
namespace net = boost::asio;

static constexpr auto FRAME_INTERVAL = std::chrono::milliseconds(333); // ~3 FPS

MjpegSession::MjpegSession(beast::tcp_stream &&stream, ImageGetter get_image,
                           rclcpp::Logger logger)
    : stream_(std::move(stream)),
      get_image_(std::move(get_image)),
      timer_(stream_.get_executor()),
      logger_(logger) {
}

void MjpegSession::run(unsigned http_version) {
    stream_.expires_never();
    send_header(http_version);
}

void MjpegSession::send_header(unsigned http_version) {
    const char *version_str =
        (http_version == 11) ? "HTTP/1.1" : "HTTP/1.0";

    header_buf_ = std::string(version_str) +
        " 200 OK\r\n"
        "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
        "Cache-Control: no-cache, no-store, must-revalidate\r\n"
        "Pragma: no-cache\r\n"
        "Connection: close\r\n"
        "\r\n";

    net::async_write(
        stream_, net::buffer(header_buf_),
        beast::bind_front_handler(&MjpegSession::on_header_sent,
                                  shared_from_this()));
}

void MjpegSession::on_header_sent(beast::error_code ec, std::size_t /*bytes*/) {
    if (ec) {
        RCLCPP_DEBUG(logger_, "mjpeg header send: %s", ec.message().c_str());
        return;
    }
    schedule_next_frame();
}

void MjpegSession::schedule_next_frame() {
    timer_.expires_after(FRAME_INTERVAL);
    timer_.async_wait(beast::bind_front_handler(&MjpegSession::on_timer,
                                                shared_from_this()));
}

void MjpegSession::on_timer(beast::error_code ec) {
    if (ec)
        return; // cancelled or shutdown

    auto img = get_image_();
    if (!img || img == last_sent_) {
        schedule_next_frame();
        return;
    }
    send_frame(std::move(img));
}

void MjpegSession::send_frame(
    sensor_msgs::msg::CompressedImage::ConstSharedPtr img) {

    frame_header_buf_ =
        "--frame\r\n"
        "Content-Type: image/jpeg\r\n"
        "Content-Length: " + std::to_string(img->data.size()) + "\r\n"
        "\r\n";

    static constexpr char kCRLF[] = "\r\n";

    std::array<net::const_buffer, 3> buffers = {
        net::buffer(frame_header_buf_),
        net::buffer(img->data.data(), img->data.size()),
        net::buffer(kCRLF, 2),
    };

    net::async_write(
        stream_, buffers,
        [self = shared_from_this(), img = std::move(img)](
            beast::error_code ec, std::size_t bytes) {
            self->on_frame_sent(ec, bytes, std::move(img));
        });
}

void MjpegSession::on_frame_sent(
    beast::error_code ec, std::size_t /*bytes*/,
    sensor_msgs::msg::CompressedImage::ConstSharedPtr img) {
    last_sent_ = std::move(img);
    if (ec) {
        RCLCPP_DEBUG(logger_, "mjpeg client disconnected: %s",
                     ec.message().c_str());
        return;
    }
    schedule_next_frame();
}

} // namespace ui_bridge_ng
