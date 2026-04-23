#include "body_imu_bus/tca9548a.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#include <iostream>
#include <string>

namespace body_imu_bus {

Tca9548a::Tca9548a(int i2c_bus, uint8_t address)
    : fd_(-1), i2c_bus_(i2c_bus), address_(address)
{}

Tca9548a::~Tca9548a()
{
    if (fd_ >= 0) {
        close(fd_);
    }
}

bool Tca9548a::open_device()
{
    if (fd_ >= 0) {
        return true;
    }

    std::string device_path = "/dev/i2c-" + std::to_string(i2c_bus_);

    fd_ = open(device_path.c_str(), O_RDWR);
    if (fd_ < 0) {
        std::cerr << "Failed to open I2C device: " << device_path << std::endl;
        return false;
    }

    if (ioctl(fd_, I2C_SLAVE, address_) < 0) {
        std::cerr << "Failed to set TCA9548A I2C slave address: 0x"
                  << std::hex << static_cast<int>(address_) << std::endl;
        close(fd_);
        fd_ = -1;
        return false;
    }

    return true;
}

bool Tca9548a::write_control(uint8_t value)
{
    if (!open_device()) {
        return false;
    }

    return (::write(fd_, &value, 1) == 1);
}

bool Tca9548a::selectChannel(uint8_t channel)
{
    if (channel > 7) {
        std::cerr << "Invalid TCA9548A channel: " << static_cast<int>(channel) << std::endl;
        return false;
    }

    // enable exactly one channel
    const uint8_t control = static_cast<uint8_t>(1u << channel);
    return write_control(control);
}

bool Tca9548a::disableAllChannels()
{
    return write_control(0x00);
}

}  // namespace body_imu_bus