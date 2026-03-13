cmake_minimum_required(VERSION 3.8)
project(camera_relay)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)
pkg_check_modules(GSTREAMER_APP REQUIRED gstreamer-app-1.0)
pkg_check_modules(GSTREAMER_VIDEO REQUIRED gstreamer-video-1.0)

include_directories(
  include
  ${GSTREAMER_INCLUDE_DIRS}
  ${GSTREAMER_APP_INCLUDE_DIRS}
  ${GSTREAMER_VIDEO_INCLUDE_DIRS}
)

link_directories(
  ${GSTREAMER_LIBRARY_DIRS}
  ${GSTREAMER_APP_LIBRARY_DIRS}
  ${GSTREAMER_VIDEO_LIBRARY_DIRS}
)

add_executable(camera_stream_relay_node
  src/camera_stream_relay_node.cpp
  src/gst_jpeg_decoder.cpp
)

add_executable(camera_info_relay_node
  src/camera_info_relay_node.cpp
)

ament_target_dependencies(camera_stream_relay_node
  rclcpp
  sensor_msgs
)

ament_target_dependencies(camera_info_relay_node
  rclcpp
  sensor_msgs
)

target_link_libraries(camera_stream_relay_node
  ${GSTREAMER_LIBRARIES}
  ${GSTREAMER_APP_LIBRARIES}
  ${GSTREAMER_VIDEO_LIBRARIES}
)

target_compile_options(camera_stream_relay_node PRIVATE
  ${GSTREAMER_CFLAGS_OTHER}
  ${GSTREAMER_APP_CFLAGS_OTHER}
  ${GSTREAMER_VIDEO_CFLAGS_OTHER}
)

install(TARGETS
  camera_stream_relay_node
  camera_info_relay_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()