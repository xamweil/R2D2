#pragma once

#include <cstdint>
#include <string>

class SerialPort {
public:
    bool connect(const char *port_name);
    bool writeData(const uint8_t *data, size_t size) const;
    [[nodiscard]] std::string readData() const;
    void disconnect() const;
    [[nodiscard]] bool isConnected() const;

private:
    bool connected_ = false;
    int serial_port_fd_{};
};
