cmake_minimum_required(VERSION 3.8)
project(body_imu_bus)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tcp_msg REQUIRED)

include_directories(include)

add_executable(body_imu_bus_node
  src/main.cpp
  src/body_imu_bus_node.cpp
  src/mpu6500.cpp
  src/tca9548a.cpp
  src/imu_device.cpp
)

ament_target_dependencies(body_imu_bus_node
  rclcpp
  tcp_msg
)

install(TARGETS body_imu_bus_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()