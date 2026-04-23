#pragma once

#include <cstdint>
#include "rclcpp/rclcpp.hpp"

#include "body_imu_bus/mpu6500.hpp"
#include "body_imu_bus/tca9548a.hpp"

namespace body_imu_bus {

class ImuDevice {
public:
    ImuDevice(Tca9548a & mux, int i2c_bus, uint8_t mux_channel, uint8_t imu_address);

    bool initialize();
    bool read(IMUData & data);

    bool isInitialized() const;
    void markUninitialized();

    bool shouldTryInitialize(const rclcpp::Time & now, double retry_interval_sec) const;
    void noteInitializeAttempt(const rclcpp::Time & now);

    uint8_t muxChannel() const;
    uint8_t imuAddress() const;

private:
    Tca9548a & mux_;
    MPU6500 mpu_;
    uint8_t mux_channel_;
    uint8_t imu_address_;
    bool initialized_;
    rclcpp::Time last_init_attempt_;
};

}  // namespace body_imu_bus