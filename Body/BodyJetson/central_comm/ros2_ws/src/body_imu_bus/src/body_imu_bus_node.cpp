#include "body_imu_bus/body_imu_bus_node.hpp"

#include <chrono>

BodyImuBusNode::BodyImuBusNode()
    : Node("body_imu_bus_node"),
      i2c_bus_(1),
      mux_address_param_(0x70),
      mux_address_(0x70),
      poll_rate_hz_(50)
{
    setupParameters();

    mux_ = std::make_unique<body_imu_bus::Tca9548a>(
        i2c_bus_,
        mux_address_);

    RCLCPP_INFO(this->get_logger(), "Starting body_imu_bus_node");
    RCLCPP_INFO(this->get_logger(), "  I2C Bus: /dev/i2c-%d", i2c_bus_);
    RCLCPP_INFO(this->get_logger(), "  Mux address: 0x%02X", mux_address_);
    RCLCPP_INFO(this->get_logger(), "  Poll rate: %d Hz", poll_rate_hz_);

    setupSensors();

    auto period = std::chrono::duration<double>(1.0 / static_cast<double>(poll_rate_hz_));
    timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        std::bind(&BodyImuBusNode::pollSensors, this));

    RCLCPP_INFO(this->get_logger(), "body_imu_bus_node initialized successfully");
}

void BodyImuBusNode::setupParameters()
{
    this->declare_parameter<int>("i2c_bus", 1);
    this->declare_parameter<int>("mux_address", 0x70);
    this->declare_parameter<int>("poll_rate_hz", 50);
    this->declare_parameter<double>("init_retry_interval_sec", 10.0);

    this->get_parameter("i2c_bus", i2c_bus_);
    this->get_parameter("mux_address", mux_address_param_);
    this->get_parameter("poll_rate_hz", poll_rate_hz_);
    this->get_parameter("init_retry_interval_sec", init_retry_interval_sec_);


    if (mux_address_param_ < 0x03 || mux_address_param_ > 0x77) {
        RCLCPP_WARN(
            this->get_logger(),
            "mux_address out of normal 7-bit I2C range, forcing to 0x70");
        mux_address_param_ = 0x70;
    }
    mux_address_ = static_cast<uint8_t>(mux_address_param_);

    if (poll_rate_hz_ <= 0) {
        RCLCPP_WARN(
            this->get_logger(),
            "poll_rate_hz <= 0, forcing to 50");
        poll_rate_hz_ = 50;
    }
}

void BodyImuBusNode::setupSensors()
{
    const std::array<ImuConfig, 5> configs = {{
        {"body",       "/Body/mpu",        "body_mpu",       5, 0x68},
        {"leg_l_leg",  "/leg_l/imu/leg",   "leg_l_imu_leg",  0, 0x69},
        {"leg_l_foot", "/leg_l/imu/foot",  "leg_l_imu_foot", 0, 0x68},
        {"leg_r_leg",  "/leg_r/imu/leg",   "leg_r_imu_leg",  3, 0x69},
        {"leg_r_foot", "/leg_r/imu/foot",  "leg_r_imu_foot", 3, 0x68},
    }};

    for (std::size_t i = 0; i < configs.size(); ++i) {
        const auto & config = configs[i];
        auto & runtime = sensors_[i];

        runtime.config = config;
        runtime.device = std::make_unique<body_imu_bus::ImuDevice>(
            *mux_,
            i2c_bus_,
            config.mux_channel,
            config.i2c_address);

        runtime.publisher =
            this->create_publisher<tcp_msg::msg::MPU6500Sample>(
                config.topic,
                rclcpp::QoS(10));

        RCLCPP_INFO(
            this->get_logger(),
            "Configured sensor %-10s -> topic=%s, mux_channel=%u, address=0x%02X",
            config.name.c_str(),
            config.topic.c_str(),
            config.mux_channel,
            config.i2c_address);
    }
}

bool BodyImuBusNode::tryInitializeSensor(ImuRuntime & sensor)
{
    if (sensor.device->isInitialized()) {
        return true;
    }

    const auto now = this->now();
    if (!sensor.device->shouldTryInitialize(now, init_retry_interval_sec_)) {
        return false;
    }

    sensor.device->noteInitializeAttempt(now);

    if (!sensor.device->initialize()) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            5000,
            "Sensor %s not responding on mux channel %u address 0x%02X",
            sensor.config.name.c_str(),
            sensor.config.mux_channel,
            sensor.config.i2c_address);
        return false;
    }

    RCLCPP_INFO(
        this->get_logger(),
        "Initialized sensor %s on mux channel %u address 0x%02X",
        sensor.config.name.c_str(),
        sensor.config.mux_channel,
        sensor.config.i2c_address);

    return true;
}

void BodyImuBusNode::publishSample(
    ImuRuntime & sensor,
    const body_imu_bus::IMUData & data)
{
    tcp_msg::msg::MPU6500Sample msg;

    msg.accel[0] = data.accel_x;
    msg.accel[1] = data.accel_y;
    msg.accel[2] = data.accel_z;

    msg.gyro[0] = data.gyro_x;
    msg.gyro[1] = data.gyro_y;
    msg.gyro[2] = data.gyro_z;

    auto now = this->now();
    msg.ts_ms = static_cast<uint32_t>(now.nanoseconds() / 1000000);

    sensor.publisher->publish(msg);
}

void BodyImuBusNode::pollSensors()
{
    for (auto & sensor : sensors_) {
        try {
            if (!tryInitializeSensor(sensor)) {
                continue;
            }

            body_imu_bus::IMUData data{};
            if (!sensor.device->read(data)) {
                sensor.device->markUninitialized();

                RCLCPP_WARN_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    3000,
                    "Read failed for sensor %s (channel %u, address 0x%02X)",
                    sensor.config.name.c_str(),
                    sensor.config.mux_channel,
                    sensor.config.i2c_address);
                continue;
            }

            publishSample(sensor, data);
        } catch (const std::exception & e) {
            sensor.device->markUninitialized();

            RCLCPP_ERROR_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                3000,
                "Exception while polling sensor %s: %s",
                sensor.config.name.c_str(),
                e.what());
        } catch (...) {
            sensor.device->markUninitialized();

            RCLCPP_ERROR_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                3000,
                "Unknown exception while polling sensor %s",
                sensor.config.name.c_str());
        }
    }
}