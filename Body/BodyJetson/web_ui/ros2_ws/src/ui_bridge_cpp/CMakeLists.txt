cmake_minimum_required(VERSION 3.8)
project(ui_bridge_cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(tcp_msg REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)

# RapidJSON: try config mode first, fall back to header search
find_package(RapidJSON QUIET)
if(NOT RapidJSON_FOUND)
  find_path(RAPIDJSON_INCLUDE_DIRS
    NAMES rapidjson/document.h
    PATHS /usr/include /usr/local/include
  )
  if(NOT RAPIDJSON_INCLUDE_DIRS)
    message(FATAL_ERROR "RapidJSON not found")
  endif()
endif()

add_executable(ui_bridge_cpp_node
  src/main.cpp
  src/http_server.cpp
  src/http_session.cpp
  src/ws_session.cpp
  src/bridge_node.cpp
  src/mjpeg_session.cpp
)

target_include_directories(ui_bridge_cpp_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${RAPIDJSON_INCLUDE_DIRS}
)

ament_target_dependencies(ui_bridge_cpp_node rclcpp std_msgs sensor_msgs tcp_msg)
target_link_libraries(ui_bridge_cpp_node Boost::system pthread)

install(TARGETS ui_bridge_cpp_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
