#pragma once

#include <rclcpp/logger.hpp>

#include <atomic>
#include <cstdint>
#include <string>
#include <thread>

class SerialPort {
public:
    explicit SerialPort(const rclcpp::Logger &logger);
    bool writeData(const uint8_t *data, size_t size) const;
    [[nodiscard]] std::string readData() const;
    [[nodiscard]] bool is_connected() const;
    bool connect(const char *port_name);
    void disconnect();
    void listen();
    std::string read_line();

private:
    const rclcpp::Logger &logger_;
    std::thread listener_thread;
    std::atomic<bool> connected_ = false;
    int serial_port_fd_{};
};
