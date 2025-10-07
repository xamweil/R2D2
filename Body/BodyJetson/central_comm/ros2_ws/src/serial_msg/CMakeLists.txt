cmake_minimum_required(VERSION 3.5)
project(serial_msg)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate code for both msg and srv
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/DeviceCommand.srv"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
