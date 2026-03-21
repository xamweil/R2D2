#pragma once

#include <boost/asio.hpp>
#include <boost/beast.hpp>
#include <rclcpp/logger.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include <functional>
#include <memory>
#include <string>

namespace ui_bridge_ng {

using ImageGetter =
    std::function<sensor_msgs::msg::CompressedImage::ConstSharedPtr()>;

class MjpegSession : public std::enable_shared_from_this<MjpegSession> {
public:
    MjpegSession(boost::beast::tcp_stream &&stream, ImageGetter get_image,
                 rclcpp::Logger logger);

    void run(unsigned http_version);

private:
    void send_header(unsigned http_version);
    void on_header_sent(boost::beast::error_code ec, std::size_t bytes);
    void schedule_next_frame();
    void on_timer(boost::beast::error_code ec);
    void send_frame(sensor_msgs::msg::CompressedImage::ConstSharedPtr img);
    void on_frame_sent(boost::beast::error_code ec, std::size_t bytes,
                       sensor_msgs::msg::CompressedImage::ConstSharedPtr img);

    boost::beast::tcp_stream stream_;
    ImageGetter get_image_;
    boost::asio::steady_timer timer_;
    rclcpp::Logger logger_;

    sensor_msgs::msg::CompressedImage::ConstSharedPtr last_sent_;
    std::string header_buf_;
    std::string frame_header_buf_;
};

} // namespace ui_bridge_ng
