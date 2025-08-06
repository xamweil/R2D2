cmake_minimum_required(VERSION 3.5)
project(tcp_msg)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate code for msgs + srvs
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MPU6500Sample.msg"
  "srv/XiaoCmd.srv"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
