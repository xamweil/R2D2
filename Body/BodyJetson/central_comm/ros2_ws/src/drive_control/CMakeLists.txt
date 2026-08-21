cmake_minimum_required(VERSION 3.8)
project(drive_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(drive_msg REQUIRED)

# Include public headers
include_directories(include)

add_executable(${PROJECT_NAME}
    src/main.cpp
    src/DriveControl.cpp
    src/SerialPort.cpp
)

ament_target_dependencies(${PROJECT_NAME} rclcpp serial_msg)


install(TARGETS ${PROJECT_NAME} 
  DESTINATION lib/${PROJECT_NAME})

ament_target_dependencies(${PROJECT_NAME} rclcpp drive_msg)

ament_package()