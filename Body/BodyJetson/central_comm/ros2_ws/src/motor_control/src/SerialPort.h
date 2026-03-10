#pragma once

#include <rclcpp/logger.hpp>

#include <atomic>
#include <cstdint>
#include <string>
#include <thread>

class SerialPort {
public:
    explicit SerialPort();
    bool write_data(const uint8_t *data, size_t size) const;
    bool connect(const char *port_name);
    bool reconnect();
    void disconnect();
    void listen();
    std::string read_line();
    [[nodiscard]] bool is_connected() const;

private:
    const rclcpp::Logger logger_;
    std::string port_name_;
    int serial_port_fd_{};
    std::thread listener_thread_;
    std::atomic<bool> connected_ = false;
};
