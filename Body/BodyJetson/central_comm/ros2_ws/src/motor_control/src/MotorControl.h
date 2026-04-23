#pragma once

#include "motor_protocol.h"
#include "imu_protocol.h"
#include "SerialPort.h"

#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>
#include <rclcpp/utilities.hpp>
#include <serial_msg/msg/motor_command.hpp>
#include <tcp_msg/msg/mpu6500_sample.hpp>

class MotorControl : public rclcpp::Node {
public:
    explicit MotorControl();

private:
    static constexpr std::chrono::milliseconds SEND_COOLDOWN_MS{50};
    static constexpr std::chrono::seconds RECONNECT_INTERVAL_S{3};
    static constexpr std::chrono::milliseconds IMU_SEND_PERIOD_MS{25};   // 40 Hz
    static constexpr std::chrono::milliseconds IMU_STALE_AFTER_MS{100};

    struct CachedImu {
        tcp_msg::msg::MPU6500Sample msg{};
        rclcpp::Time last_rx_time{0, 0, RCL_ROS_TIME};
        bool received = false;
    };

    std::array<uint8_t, 1 + motor_protocol::FRAME_SIZE_BYTES> motor_buf_{};
    std::array<uint8_t, 1 + imu_protocol::FRAME_SIZE_BYTES> imu_buf_{};

    motor_protocol::Frame motor_frame_{};
    imu_protocol::Frame imu_frame_{};

    std::array<CachedImu, imu_protocol::IMU_COUNT> imu_cache_{};

    bool dirty_ = false;

    rclcpp::Subscription<serial_msg::msg::MotorCommand>::SharedPtr cmd_sub_;

    rclcpp::Subscription<tcp_msg::msg::MPU6500Sample>::SharedPtr head_mpu_sub_;
    rclcpp::Subscription<tcp_msg::msg::MPU6500Sample>::SharedPtr body_mpu_sub_;
    rclcpp::Subscription<tcp_msg::msg::MPU6500Sample>::SharedPtr leg_l_foot_sub_;
    rclcpp::Subscription<tcp_msg::msg::MPU6500Sample>::SharedPtr leg_l_leg_sub_;
    rclcpp::Subscription<tcp_msg::msg::MPU6500Sample>::SharedPtr leg_r_foot_sub_;
    rclcpp::Subscription<tcp_msg::msg::MPU6500Sample>::SharedPtr leg_r_leg_sub_;

    rclcpp::TimerBase::SharedPtr cooldown_timer_;
    rclcpp::TimerBase::SharedPtr reconnect_timer_;
    rclcpp::TimerBase::SharedPtr imu_timer_;

    SerialPort serial_;

    void cmd_callback(const serial_msg::msg::MotorCommand &msg);
    void imu_callback(imu_protocol::ImuIndex idx, const tcp_msg::msg::MPU6500Sample &msg);

    void schedule_send();
    void on_cooldown_expire();

    void refresh_imu_frame();
    void send_imu_frame();
    void send_motor_frame();
    void try_serial_reconnect();
};
