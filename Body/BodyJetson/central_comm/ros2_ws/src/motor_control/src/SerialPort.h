#pragma once

#include <atomic>
#include <cstdint>
#include <string>
#include <thread>

class SerialPort {
public:
    ~SerialPort();
    bool writeData(const uint8_t *data, size_t size) const;
    [[nodiscard]] std::string readData() const;
    [[nodiscard]] bool is_connected() const;
    bool connect(const char *port_name);
    void disconnect();
    void listen();

private:
    std::thread listener_thread;
    std::atomic<bool> connected_ = false;
    int serial_port_fd_{};
};
