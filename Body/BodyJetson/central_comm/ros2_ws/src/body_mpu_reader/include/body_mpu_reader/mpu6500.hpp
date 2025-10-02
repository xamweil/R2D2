#ifndef BODY_MPU_READER_MPU6500_HPP_
#define BODY_MPU_READER_MPU6500_HPP_

#include <string>
#include <cstdint>

namespace body_mpu_reader {
// MPU-6500 Register Map
constexpr uint8_t MPU6500_ADDR = 0x68;  // Default I2C address

// Register addresses
constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;
constexpr uint8_t REG_PWR_MGMT_2 = 0x6C;
constexpr uint8_t REG_GYRO_CONFIG = 0x1B;
constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
constexpr uint8_t REG_ACCEL_CONFIG_2 = 0x1D;
constexpr uint8_t REG_SMPLRT_DIV = 0x19;
constexpr uint8_t REG_CONFIG = 0x1A;
constexpr uint8_t REG_WHO_AM_I = 0x75;

// Data registers
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
constexpr uint8_t REG_TEMP_OUT_H = 0x41;
constexpr uint8_t REG_GYRO_XOUT_H = 0x43;

// Configuration values
constexpr uint8_t GYRO_RANGE_250DPS = 0x00;
constexpr uint8_t GYRO_RANGE_500DPS = 0x08;
constexpr uint8_t GYRO_RANGE_1000DPS = 0x10;
constexpr uint8_t GYRO_RANGE_2000DPS = 0x18;

constexpr uint8_t ACCEL_RANGE_2G = 0x00;
constexpr uint8_t ACCEL_RANGE_4G = 0x08;
constexpr uint8_t ACCEL_RANGE_8G = 0x10;
constexpr uint8_t ACCEL_RANGE_16G = 0x18;

struct IMUData
{
  double accel_x;  // m/s²
  double accel_y;
  double accel_z;
  double gyro_x;   // rad/s
  double gyro_y;
  double gyro_z;
  double temp_c;   // Celsius
};

class MPU6500 {
    public:
        MPU6500(int i2c_bus, uint8_t address= MPU6500_ADDR);
        ~MPU6500();

        bool initialize();
        bool read(IMUData &data);
        bool test_connection();

    private:
        int fd_;  // File descriptor for I2C device
        int i2c_bus_;
        uint8_t address_;
        double accel_scale_;
        double gyro_scale_;

        bool open_device();
        bool write_register(uint8_t reg, uint8_t value);
        bool read_byte(uint8_t reg, uint8_t& value);
        bool read_bytes(uint8_t reg, uint8_t* buffer, size_t length);

        int16_t bytes_to_int16(uint8_t high, uint8_t low);
}
};
#endif