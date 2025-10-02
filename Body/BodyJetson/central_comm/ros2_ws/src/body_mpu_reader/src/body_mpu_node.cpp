#include "body_mpu_reader/body_mpu_node.hpp"
#include <chrono>

namespace body_mpu_reader {
    BodyMPUNode::BodyMPUNode() : Node("body_mpu_node")
    {
        // Declare parameters with default values
        this->declare_parameter<int>("i2c_bus", 1);
        this->declare_parameter<int>("i2c_address", MPU6500_ADDR);
        this->declare_parameter<double>("publish_rate", 50.0); // Hz
        this->declare_parameter<std::string>("frame_id", "body_imu_link");
        this->declare_parameter<std::string>("topic_name", "Body/mpu");

        // Get parameters
        this->get_parameter("i2c_bus", i2c_bus_);
        this->get_parameter("i2c_address", i2c_address_);
        this->get_parameter("publish_rate", publish_rate_);
        this->get_parameter("frame_id", frame_id_);
        this->get_parameter("topic_name", topic_name_);

        RCLCPP_INFO(this->get_logger(), "Starting Body MPU Node");
        RCLCPP_INFO(this->get_logger(), "  I2C Bus: /dev/i2c-%d", i2c_bus_);
        RCLCPP_INFO(this->get_logger(), "  I2C Address: 0x%02X", i2c_address_);
        RCLCPP_INFO(this->get_logger(), "  Publish rate: %.1f Hz", publish_rate_);
        RCLCPP_INFO(this->get_logger(), "  Topic: %s", topic_name_.c_str());
        RCLCPP_INFO(this->get_logger(), "  Frame ID: %s", frame_id_.c_str());

        // Initialize MPU6500
        mpu_ = std::make_unique<MPU6500>(i2c_bus_, static_cast<uint8_t>(i2c_address_));

        if (!mpu_->initialize()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize MPU6500");
            rclcpp::shutdown();
            return;
        }

        // Create publisher
        imu_publisher_ = this->create_publisher<tcp_msg::msg::MPU6500Sample>(topic_name_, rclcpp::QoS(10));

        // Create timer for periodic publishing
        auto period = std::chrono::duration<double>(1.0 / publish_rate_);
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(period),
            std::bind(&BodyMPUNode::timer_callback, this)
        );

        RCLCPP_INFO(this->get_logger(), "BodyMPUNode initialized successfully");
    }

    BodyMPUNode::~BodyMPUNode()
    {
        RCLCPP_INFO(this->get_logger(), "Shutting down Body MPU Node");
    }

    void BodyMPUNode::timer_callback() {
        IMUData data;

        if(!mpu_->read(data)) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000, // Log warning at most once per second
                "Failed to read body IMU data"
            );
            return;
        }

        // Create publischer message
        auto msg = create_mpu_message(data);
        imu_publisher_->publish(msg);
    }

    tcp::msg::MPU6500Sample BodyMPUNode::create_mpu_message(const IMUData &data) {
        tcp::msg::MPU6500Sample msg;

        msg.accel[0] = data.accel_x;
        msg.accel[1] = data.accel_y;
        msg.accel[2] = data.accel_z;

        mgs.gyro[0] = data.gyro_x;
        msg.gyro[1] = data.gyro_y;
        msg.gyro[2] = data.gyro_z;

        auto now = this->now();
        msg.ts_ms = static_cast<uint32_t>(now.nanoseconds() / 1000000);

        return msg;
    }

}
