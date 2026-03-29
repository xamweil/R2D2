cmake_minimum_required(VERSION 3.8)
project(ui_bridge_cpp LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(tcp_msg REQUIRED)
find_package(vision_msgs REQUIRED)

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

include(FetchContent)

FetchContent_Declare(
    usockets
    GIT_REPOSITORY https://github.com/uNetworking/uSockets.git
    GIT_TAG        ae52d1edea0182de655bde4d445858b3b02d6219
    GIT_SHALLOW    TRUE
    GIT_SUBMODULES ""
)
FetchContent_MakeAvailable(usockets)

file(GLOB USOCKETS_C_SOURCES
    ${usockets_SOURCE_DIR}/src/*.c
    ${usockets_SOURCE_DIR}/src/eventing/*.c
    ${usockets_SOURCE_DIR}/src/crypto/*.c
)

add_library(uSockets STATIC ${USOCKETS_C_SOURCES})
target_include_directories(uSockets PUBLIC ${usockets_SOURCE_DIR}/src)
target_compile_definitions(uSockets PUBLIC LIBUS_NO_SSL)

FetchContent_Declare(
    uwebsockets
    GIT_REPOSITORY https://github.com/uNetworking/uWebSockets.git
    GIT_TAG        e42d69ff338269a5b74d014046e3699e2f27734e
    GIT_SHALLOW    TRUE
    GIT_SUBMODULES ""
)
FetchContent_MakeAvailable(uwebsockets)

add_library(uWebSockets INTERFACE)
target_include_directories(uWebSockets INTERFACE ${uwebsockets_SOURCE_DIR}/src)
target_link_libraries(uWebSockets INTERFACE uSockets)

# Stream an animated test pattern instead of real camera frames on /mjpeg.
# Enable with: colcon build --cmake-args -DMJPEG_TEST_PATTERN=ON
option(MJPEG_TEST_PATTERN "Use generated test pattern for MJPEG stream" OFF)

if(MJPEG_TEST_PATTERN)
  FetchContent_Declare(
      stb
      GIT_REPOSITORY https://github.com/nothings/stb.git
      GIT_TAG        28d546d5eb77d4585506a20480f4de2e706dff4c
      GIT_SHALLOW    TRUE
  )
  FetchContent_MakeAvailable(stb)

  add_library(stb INTERFACE)
  target_include_directories(stb INTERFACE ${stb_SOURCE_DIR})
endif()

add_executable(ui_bridge_cpp_node
  src/main.cpp
  src/bridge_node.cpp
  src/http_server.cpp
)

if(MJPEG_TEST_PATTERN)
  target_sources(ui_bridge_cpp_node PRIVATE src/jpeg_generator.cpp)
endif()

target_include_directories(ui_bridge_cpp_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${RAPIDJSON_INCLUDE_DIRS}
)

target_compile_definitions(ui_bridge_cpp_node PRIVATE
    LIBUS_NO_SSL
    UWS_NO_ZLIB
    $<$<BOOL:${MJPEG_TEST_PATTERN}>:MJPEG_TEST_PATTERN>
)

ament_target_dependencies(ui_bridge_cpp_node rclcpp std_msgs sensor_msgs tcp_msg vision_msgs)
target_link_libraries(ui_bridge_cpp_node uWebSockets
    $<$<BOOL:${MJPEG_TEST_PATTERN}>:stb>
)

install(TARGETS ui_bridge_cpp_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
