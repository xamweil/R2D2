#pragma once

#include <cstdint>

namespace body_imu_bus {

class Tca9548a {
public:
    Tca9548a(int i2c_bus, uint8_t address = 0x70);
    ~Tca9548a();

    bool selectChannel(uint8_t channel);
    bool disableAllChannels();

private:
    bool open_device();
    bool write_control(uint8_t value);

    int fd_;
    int i2c_bus_;
    uint8_t address_;
};

}  // namespace body_imu_bus