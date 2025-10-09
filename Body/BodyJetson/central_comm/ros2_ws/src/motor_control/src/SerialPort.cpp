#include "SerialPort.h"

#include <fcntl.h>
#include <rmw/types.h>
#include <termios.h>
#include <unistd.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>

/*
 * References:
 * https://tldp.org/HOWTO/Serial-Programming-HOWTO/index.html
 * https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/
 */

bool SerialPort::connect(const char *port_name) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg)
    serial_port_fd_ = open(port_name, O_RDWR);

    connected_ = false;

    if (serial_port_fd_ < 0) {
        return false;
    }

    termios tty{};
    if (tcgetattr(serial_port_fd_, &tty) != 0) {
        return false;
    }

    tty.c_cflag &= ~PARENB;        // No parity bit
    tty.c_cflag &= ~CSTOPB;        // One stop bit
    tty.c_cflag &= ~CSIZE;         // Clear size bits
    tty.c_cflag |= CS8;            // 8 data bits
    tty.c_cflag &= ~CRTSCTS;       // No hardware flow control
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ, ignore ctrl lines

    tty.c_lflag &= ~ICANON; // Non-canonical mode
    tty.c_lflag &= ~ECHO;   // Disable echo
    tty.c_lflag &= ~ECHOE;  // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG;   // Disable interpretation of signals

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR |
                     ICRNL); // Disable any special handling of received bytes

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes
    tty.c_oflag &=
        ~ONLCR; // Prevent conversion of newline to carriage return/line feed

    tty.c_cc[VTIME] = 10; // Wait for up to 1s (10 deciseconds), returning as
                          // soon as any data is received.
    tty.c_cc[VMIN] = 0;

    speed_t baud_rate = B115200;
    cfsetispeed(&tty, baud_rate);
    cfsetospeed(&tty, baud_rate);

    if (tcsetattr(serial_port_fd_, TCSANOW, &tty) == 0) {
        connected_ = true;
    }

    if (connected_) {
        listener_thread = std::thread(&SerialPort::listen, this);
    }

    return connected_;
}

void SerialPort::listen() {
    while (connected_) {
    }
}

bool SerialPort::writeData(const uint8_t *data, size_t size) const {
    const ssize_t bytes_written = write(serial_port_fd_, data, size);
    return bytes_written >= 0 && static_cast<size_t>(bytes_written) == size;
}

std::string SerialPort::readData() const {
    static constexpr size_t BufferSize = 256;
    std::array<char, BufferSize> buffer = {};
    const ssize_t bytes_read =
        read(serial_port_fd_, buffer.data(), buffer.size());

    if (bytes_read > 0) {
        // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-constant-array-index)
        buffer[bytes_read] = '\0';
        return {buffer.data()};
    }
    return "";
}

void SerialPort::disconnect() {
    if (connected_) {
        connected_ = false;
        close(serial_port_fd_);
    }
}

[[nodiscard]] bool SerialPort::is_connected() const {
    return connected_;
}
