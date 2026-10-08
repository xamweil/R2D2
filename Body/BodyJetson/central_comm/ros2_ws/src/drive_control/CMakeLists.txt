cmake_minimum_required(VERSION 3.8)
project(drive_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(serial_msg REQUIRED)

# Include public headers
include_directories(include)

add_executable(${PROJECT_NAME}
  src/DriveControlNodeMain.cpp
  src/DriveControlNode.cpp
  src/DriveMotor.cpp
  src/DDSM115.cpp
  src/RS485Com.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  serial_msg
)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()