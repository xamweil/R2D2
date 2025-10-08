#include "body_mpu_reader/mpu6500.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <cstring>
#include <iostream>
#include <cmath>

namespace body_mpu_reader {
MPU6500::MPU6500(int i2c_bus, uint8_t address)
  : fd_(-1), i2c_bus_(i2c_bus), address_(address), accel_scale_(0.0), gyro_scale_(0.0)
{
}

MPU6500::~MPU6500()
{
  if (fd_ >= 0) {
    close(fd_);
  }
}

bool MPU6500::open_device()
{
  std::string device_path = "/dev/i2c-" + std::to_string(i2c_bus_);
  
  fd_ = open(device_path.c_str(), O_RDWR);
  if (fd_ < 0) {
    std::cerr << "Failed to open I2C device: " << device_path << std::endl;
    return false;
  }

  if (ioctl(fd_, I2C_SLAVE, address_) < 0) {
    std::cerr << "Failed to set I2C slave address: 0x" 
              << std::hex << static_cast<int>(address_) << std::endl;
    close(fd_);
    fd_ = -1;
    return false;
  }

  return true;
}

bool MPU6500::write_byte(uint8_t reg, uint8_t value)
{
  uint8_t buffer[2] = {reg, value};
  
  if (::write(fd_, buffer, 2) != 2) {
    std::cerr << "Failed to write to register 0x" 
              << std::hex << static_cast<int>(reg) << std::endl;
    return false;
  }
  
  return true;
}

bool MPU6500::read_byte(uint8_t reg, uint8_t& value)
{
  return read_bytes(reg, &value, 1);
}

bool MPU6500::read_bytes(uint8_t reg, uint8_t* buffer, size_t length)
{
  struct i2c_rdwr_ioctl_data ioctl_data{};
  struct i2c_msg msgs[2];

  // 1) write the register address (no STOP)
  msgs[0].addr  = address_;
  msgs[0].flags = 0;                 // write
  msgs[0].len   = 1;
  msgs[0].buf   = &reg;

  // 2) repeated-start + read N bytes
  msgs[1].addr  = address_;
  msgs[1].flags = I2C_M_RD;          // read
  msgs[1].len   = static_cast<__u16>(length);
  msgs[1].buf   = buffer;

  ioctl_data.msgs  = msgs;
  ioctl_data.nmsgs = 2;

  if (ioctl(fd_, I2C_RDWR, &ioctl_data) < 0) {
    std::cerr << "I2C_RDWR failed for reg 0x" << std::hex << int(reg)
              << " len " << std::dec << length << std::endl;
    return false;
  }
  return true;
}

int16_t MPU6500::bytes_to_int16(uint8_t high, uint8_t low)
{
  return static_cast<int16_t>((high << 8) | low);
}

bool MPU6500::test_connection()
{
  uint8_t who_am_i;
  if (!read_byte(REG_WHO_AM_I, who_am_i)) {
    return false;
  }
  
  // MPU-6500 should return 0x70
  if (who_am_i != 0x70) {
    std::cerr << "WHO_AM_I check failed. Expected 0x70, got 0x" 
              << std::hex << static_cast<int>(who_am_i) << std::endl;
    return false;
  }
  
  return true;
}

bool MPU6500::initialize()
{
  if (!open_device()) {
    return false;
  }
  
  // Test connection
  if (!test_connection()) {
    std::cerr << "MPU-6500 not detected on bus " << i2c_bus_ 
              << " at address 0x" << std::hex << static_cast<int>(address_) << std::endl;
    return false;
  }
  
  std::cout << "MPU-6500 detected successfully!" << std::endl;
  
  // Reset device
  if (!write_byte(REG_PWR_MGMT_1, 0x80)) {
    return false;
  }
  usleep(100000);  // Wait 100ms for reset
  
  // Wake up + select clock source = PLL with X gyro
  if (!write_byte(REG_PWR_MGMT_1, 0x01)) {
    return false;
  }
  // Ensure all axes enabled
  if (!write_byte(REG_PWR_MGMT_2, 0x00)) {
    return false;
  }
  usleep(10000);  // Wait 10ms
  
  // Configure gyroscope range: ±250 dps
  if (!write_byte(REG_GYRO_CONFIG, GYRO_RANGE_250DPS)) {
    return false;
  }

  
  // Configure accelerometer range: +/-2g
  if (!write_byte(REG_ACCEL_CONFIG, ACCEL_RANGE_2G)) {
    return false;
  }

  
  // Configure low-pass filter (DLPF): 20 Hz
  if (!write_byte(REG_CONFIG, 0x04)) {
    return false;
  }

   // Accel DLPF: ~20 Hz (ACCEL_CONFIG_2.A_DLPFCFG=4, ACCEL_FCHOICE_B=0)
  if (!write_byte(REG_ACCEL_CONFIG_2, 0x04)) {
    return false;
  }
  // Set sample rate divider (1 kHz / (1 + 4) = 200 Hz)
  if (!write_byte(REG_SMPLRT_DIV, 0x04)) {
    return false;
  }
  
  std::cout << "MPU-6500 initialized successfully!" << std::endl;
  std::cout << "  Gyro range: +/-250 dps" << std::endl;
  std::cout << "  Accel range: +/-2g" << std::endl;
  std::cout << "  Sample rate: ~200 Hz" << std::endl;
  
  return true;
}

bool MPU6500::read(IMUData& data)
{
  // Read 14 bytes starting from ACCEL_XOUT_H
  // Layout: ACCEL_X(2) ACCEL_Y(2) ACCEL_Z(2) TEMP(2) GYRO_X(2) GYRO_Y(2) GYRO_Z(2)
  uint8_t buffer[14];
  
  if (!read_bytes(REG_ACCEL_XOUT_H, buffer, 14)) {
    return false;
  }
  
  // Parse accelerometer data (raw -> g)
  int16_t accel_x_raw = bytes_to_int16(buffer[0], buffer[1]);
  int16_t accel_y_raw = bytes_to_int16(buffer[2], buffer[3]);
  int16_t accel_z_raw = bytes_to_int16(buffer[4], buffer[5]);
  
  data.accel_x = accel_x_raw;
  data.accel_y = accel_y_raw;
  data.accel_z = accel_z_raw;
  
  // Parse temperature data (raw -> Celsius)
  int16_t temp_raw = bytes_to_int16(buffer[6], buffer[7]);
  data.temp_c = (temp_raw / 333.87) + 21.0;
  
  // Parse gyroscope data (raw -> dps)
  int16_t gyro_x_raw = bytes_to_int16(buffer[8], buffer[9]);
  int16_t gyro_y_raw = bytes_to_int16(buffer[10], buffer[11]);
  int16_t gyro_z_raw = bytes_to_int16(buffer[12], buffer[13]);
  
  data.gyro_x = gyro_x_raw;
  data.gyro_y = gyro_y_raw;
  data.gyro_z = gyro_z_raw;
  
  return true;
}
}