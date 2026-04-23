#pragma once

#include <array>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "tcp_msg/msg/mpu6500_sample.hpp"

#include "body_imu_bus/imu_device.hpp"
#include "body_imu_bus/tca9548a.hpp"

class BodyImuBusNode : public rclcpp::Node {
public:
    BodyImuBusNode();

private:
    struct AxisTransform {
        uint8_t source_axis; // 0=x, 1=y, 2=z
        int8_t sign; // +1 or -1
        int16_t accel_offset; // raw_count accel correction 
        int16_t gyro_offset; // raw_count accel correction 
    };

    struct ImuConfig {
        std::string name;
        std::string topic;
        std::string frame_id;
        uint8_t mux_channel;
        uint8_t i2c_address;
        std::array<AxisTransform, 3> transform;
    };

    struct ImuRuntime {
        ImuConfig config;
        std::unique_ptr<body_imu_bus::ImuDevice> device;
        rclcpp::Publisher<tcp_msg::msg::MPU6500Sample>::SharedPtr publisher;
    };

    void setupParameters();
    void setupSensors();
    void pollSensors();
    bool tryInitializeSensor(ImuRuntime & sensor);
    void publishSample(ImuRuntime & sensor, const body_imu_bus::IMUData & data);
    int16_t applyAxisTransform(const std::array<int16_t, 3> & raw, const AxisTransform & transform, int16_t offset) const;

    int i2c_bus_;
    int mux_address_param_;
    uint8_t mux_address_;
    int poll_rate_hz_;

    double init_retry_interval_sec_;

    std::unique_ptr<body_imu_bus::Tca9548a> mux_;
    std::array<ImuRuntime, 5> sensors_;

    rclcpp::TimerBase::SharedPtr timer_;
};