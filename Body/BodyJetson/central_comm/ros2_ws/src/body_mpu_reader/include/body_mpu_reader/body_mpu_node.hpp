#ifndef BODY_MPU_READER_BODY_MPU_NODE_HPP_
#define BODY_MPU_READER_BODY_MPU_NODE_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "tcp_msg/msg/mpu6500_sample.hpp"
#include "body_mpu_reader/mpu6500.hpp"

namespace body_mpu_reader {
    class BodyMPUNode : public rclcpp::Node {
        public:
        BodyMPUNode();
        ~BodyMPUNode();

    private:
        void timer_callback();
        tcp_msg::msg::MPU6500Sample create_mpu_msg(const IMUData &data);
        std::unique_ptr<MPU6500> mpu_;
        rclcpp::Publisher<tcp_msg::msg::MPU6500Sample>::SharedPtr imu_publisher_;
        rclcpp::TimerBase::SharedPtr timer_;

        int i2c_bus_;
        int i2c_address_;
        double publish_rate_;
        std::string frame_id_;
        std::string topic_name_;
    };
}

#endif