#include "MotorControl.h"
#include "imu_protocol.h"

#include "SerialPort.h"

#include <rclcpp/logging.hpp>

MotorControl::MotorControl() : Node("motor_control") {
    serial_.connect("/dev/ttyACM0");

    if (!serial_.is_connected()) {
        RCLCPP_WARN(this->get_logger(),
                    "Serial: failed to connect to pico, will retry");
        reconnect_timer_ = this->create_wall_timer(
            RECONNECT_INTERVAL_S, [this]() { this->try_serial_reconnect(); });
    } else {
        RCLCPP_INFO(this->get_logger(), "Serial: connected to pico");
    }

    constexpr int queue_size = 10;

    cmd_sub_ = this->create_subscription<serial_msg::msg::MotorCommand>(
        "/motor_command", queue_size,
        [this](const serial_msg::msg::MotorCommand &msg) {
            this->cmd_callback(msg);
        });

    head_mpu_sub_ = this->create_subscription<tcp_msg::msg::MPU6500Sample>(
        "/Head/mpu", queue_size,
        [this](const tcp_msg::msg::MPU6500Sample &msg) {
            this->imu_callback(imu_protocol::IMU_head, msg);
        });

    body_mpu_sub_ = this->create_subscription<tcp_msg::msg::MPU6500Sample>(
        "/Body/mpu", queue_size,
        [this](const tcp_msg::msg::MPU6500Sample &msg) {
            this->imu_callback(imu_protocol::IMU_body, msg);
        });

    leg_l_foot_sub_ = this->create_subscription<tcp_msg::msg::MPU6500Sample>(
        "/leg_l/imu/foot", queue_size,
        [this](const tcp_msg::msg::MPU6500Sample &msg) {
            this->imu_callback(imu_protocol::IMU_leg_l_foot, msg);
        });

    leg_l_leg_sub_ = this->create_subscription<tcp_msg::msg::MPU6500Sample>(
        "/leg_l/imu/leg", queue_size,
        [this](const tcp_msg::msg::MPU6500Sample &msg) {
            this->imu_callback(imu_protocol::IMU_leg_l_leg, msg);
        });

    leg_r_foot_sub_ = this->create_subscription<tcp_msg::msg::MPU6500Sample>(
        "/leg_r/imu/foot", queue_size,
        [this](const tcp_msg::msg::MPU6500Sample &msg) {
            this->imu_callback(imu_protocol::IMU_leg_r_foot, msg);
        });

    leg_r_leg_sub_ = this->create_subscription<tcp_msg::msg::MPU6500Sample>(
        "/leg_r/imu/leg", queue_size,
        [this](const tcp_msg::msg::MPU6500Sample &msg) {
            this->imu_callback(imu_protocol::IMU_leg_r_leg, msg);
        });

    imu_timer_ = this->create_wall_timer(
        IMU_SEND_PERIOD_MS, [this]() { this->send_imu_frame(); });
}

void MotorControl::cmd_callback(const serial_msg::msg::MotorCommand &msg) {
    for (size_t i = 0; i < msg.ids.size(); ++i) {
        const uint8_t id = msg.ids[i];
        if (id >= motor_protocol::MOTOR_COUNT) {
            RCLCPP_WARN(this->get_logger(),
                        "Invalid motor id: %u (must be 0-5)", id);
            continue;
        }

        auto &m = motor_frame_.motors[id];
        m.enable = msg.enable[i];
        m.direction = msg.direction[i];
        m.angle_set = msg.angle_set[i];
        m.velocity_set = msg.velocity_set[i];
        m.angle_value = msg.angle[i];
        m.velocity_value = msg.velocity[i];
    }

    schedule_send();
}

void MotorControl::imu_callback(imu_protocol::ImuIndex idx,
                                const tcp_msg::msg::MPU6500Sample &msg) {
    auto &slot = imu_cache_[static_cast<size_t>(idx)];
    slot.msg = msg;
    slot.last_rx_time = this->now();
    slot.received = true;
}

void MotorControl::schedule_send() {
    dirty_ = true;
    if (!cooldown_timer_) {
        send_motor_frame();
        cooldown_timer_ = this->create_wall_timer(
            SEND_COOLDOWN_MS, [this]() { this->on_cooldown_expire(); });
    }
}

void MotorControl::on_cooldown_expire() {
    if (dirty_) {
        send_motor_frame();
    } else {
        cooldown_timer_.reset();
    }
}

void MotorControl::refresh_imu_frame() {
    const auto now = this->now();

    for (size_t i = 0; i < imu_cache_.size(); ++i) {
        auto &dst = imu_frame_.imus[i];
        const auto &src = imu_cache_[i];

        if (!src.received) {
            dst = {};
            continue;
        }

        const auto age = now - src.last_rx_time;
        const bool stale = age > rclcpp::Duration(IMU_STALE_AFTER_MS);

        dst.accel_x = src.msg.accel[0];
        dst.accel_y = src.msg.accel[1];
        dst.accel_z = src.msg.accel[2];
        dst.ts_ms = stale ? 0U : src.msg.ts_ms;
    }
}

void MotorControl::send_motor_frame() {
    dirty_ = false;

    if (!serial_.is_connected()) return;

    motor_buf_[0] = 0xAA;
    if (!motor_frame_.serialize(&motor_buf_[1], motor_buf_.size() - 1)) {
        RCLCPP_WARN(this->get_logger(), "Failed to serialize motor frame");
        return;
    }

    if (!serial_.write_data(motor_buf_.data(), motor_buf_.size())) {
        RCLCPP_WARN(this->get_logger(),
                    "Serial motor write failed, starting reconnect");
        serial_.disconnect();
        if (!reconnect_timer_) {
            reconnect_timer_ = this->create_wall_timer(
                RECONNECT_INTERVAL_S, [this]() { this->try_serial_reconnect(); });
        }
        return;
    }

    RCLCPP_DEBUG(this->get_logger(), "Motor frame sent:\n%s",
                 motor_frame_.to_string().c_str());
}

void MotorControl::send_imu_frame() {
    if (!serial_.is_connected()) return;

    refresh_imu_frame();

    imu_buf_[0] = imu_protocol::SOF;
    if (!imu_frame_.serialize(&imu_buf_[1], imu_buf_.size() - 1)) {
        RCLCPP_WARN(this->get_logger(), "Failed to serialize IMU frame");
        return;
    }

    if (!serial_.write_data(imu_buf_.data(), imu_buf_.size())) {
        RCLCPP_WARN(this->get_logger(),
                    "Serial IMU write failed, starting reconnect");
        serial_.disconnect();
        if (!reconnect_timer_) {
            reconnect_timer_ = this->create_wall_timer(
                RECONNECT_INTERVAL_S, [this]() { this->try_serial_reconnect(); });
        }
        return;
    }

    RCLCPP_DEBUG(this->get_logger(), "IMU frame sent:\n%s",
                 imu_frame_.to_string().c_str());
}

void MotorControl::try_serial_reconnect() {
    if (serial_.reconnect()) {
        RCLCPP_INFO(this->get_logger(), "Serial: reconnected to pico");
        reconnect_timer_.reset();
    } else {
        RCLCPP_WARN(this->get_logger(),
                    "Serial: reconnect failed, retrying...");
    }
}