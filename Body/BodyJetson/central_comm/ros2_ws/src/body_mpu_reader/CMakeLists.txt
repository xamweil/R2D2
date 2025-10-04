cmake_minimum_required(VERSION 3.8)
project(body_mpu_reader)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tcp_msg REQUIRED)

# Include public headers
include_directories(include)

# Build the node executable
add_executable(body_mpu_node
  src/body_mpu_node_main.cpp
  src/body_mpu_node.cpp
  src/mpu6500.cpp
)

install(TARGETS body_mpu_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_target_dependencies(body_mpu_node rclcpp tcp_msg)

ament_package()
