#include "body_imu_bus/imu_device.hpp"

namespace body_imu_bus {

ImuDevice::ImuDevice(Tca9548a & mux, int i2c_bus, uint8_t mux_channel, uint8_t imu_address)
    : mux_(mux),
      mpu_(i2c_bus, imu_address),
      mux_channel_(mux_channel),
      imu_address_(imu_address),
      initialized_(false),
      last_init_attempt_(0, 0, RCL_ROS_TIME)
{}

bool ImuDevice::shouldTryInitialize(const rclcpp::Time & now, double retry_interval_sec) const
{
    if (initialized_) {
        return false;
    }

    const double dt = (now - last_init_attempt_).seconds();
    return dt >= retry_interval_sec;
}

void ImuDevice::noteInitializeAttempt(const rclcpp::Time & now) {
    last_init_attempt_ = now;
}

bool ImuDevice::initialize()
{
    if (!mux_.selectChannel(mux_channel_)) {
        initialized_ = false;
        return false;
    }

    if (!mpu_.initialize()) {
        initialized_ = false;
        return false;
    }

    initialized_ = true;
    return true;
}

bool ImuDevice::read(IMUData & data)
{
    if (!initialized_) {
        return false;
    }

    if (!mux_.selectChannel(mux_channel_)) {
        initialized_ = false;
        return false;
    }

    if (!mpu_.read(data)) {
        initialized_ = false;
        return false;
    }

    return true;
}

bool ImuDevice::isInitialized() const
{
    return initialized_;
}

void ImuDevice::markUninitialized()
{
    initialized_ = false;
}

uint8_t ImuDevice::muxChannel() const
{
    return mux_channel_;
}

uint8_t ImuDevice::imuAddress() const
{
    return imu_address_;
}

}  // namespace body_imu_bus