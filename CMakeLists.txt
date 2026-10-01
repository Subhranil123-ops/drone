cmake_minimum_required(VERSION 3.16)

project(drone_navigation LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(ARDUPILOT_MAVLINK_INCLUDE
    "/home/sub/ardupilot/build/sitl/libraries/GCS_MAVLink/include/mavlink/v2.0"
    CACHE PATH "ArduPilot MAVLink include directory")

set(GZ_TRANSPORT_INCLUDE "/usr/include/gz/transport15")
set(GZ_MSGS_INCLUDE "/usr/include/gz/msgs12")
set(GZ_UTILS_INCLUDE "/usr/include/gz/utils4")
set(GZ_MATH_INCLUDE "/usr/include/gz/math9")
set(GZ_COMMON_INCLUDE "/usr/include/gz/common7")
set(GZ_PLUGIN_INCLUDE "/usr/include/gz/plugin4")
set(GZ_INCLUDE_ROOT "/usr/include")

if(NOT EXISTS "${GZ_TRANSPORT_INCLUDE}/gz/transport/Node.hh")
  message(FATAL_ERROR "Gazebo Transport headers not found")
endif()
if(NOT EXISTS "${GZ_MSGS_INCLUDE}/gz/msgs/laserscan.pb.h")
  message(FATAL_ERROR "Gazebo LaserScan headers not found")
endif()
if(NOT EXISTS "${ARDUPILOT_MAVLINK_INCLUDE}/mavlink_types.h")
  message(FATAL_ERROR "ArduPilot MAVLink headers not found: ${ARDUPILOT_MAVLINK_INCLUDE}")
endif()

find_library(GZ_TRANSPORT_LIBRARY NAMES gz-transport PATHS /usr/lib/x86_64-linux-gnu /usr/lib REQUIRED)
find_library(GZ_TRANSPORT_LOG_LIBRARY NAMES gz-transport-log PATHS /usr/lib/x86_64-linux-gnu /usr/lib REQUIRED)
find_library(GZ_TRANSPORT_PARAMETERS_LIBRARY NAMES gz-transport-parameters PATHS /usr/lib/x86_64-linux-gnu /usr/lib REQUIRED)
find_library(GZ_MSGS_LIBRARY NAMES gz-msgs PATHS /usr/lib/x86_64-linux-gnu /usr/lib REQUIRED)
find_library(PROTOBUF_LIBRARY NAMES protobuf PATHS /usr/lib/x86_64-linux-gnu /usr/lib REQUIRED)

add_executable(drone-navigation
  src/main.cpp
  src/gazebo_lidar.cpp
  src/lidar_processor.cpp
  src/navigator.cpp
  src/mavlink_client.cpp
)

target_include_directories(drone-navigation PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
  "${ARDUPILOT_MAVLINK_INCLUDE}"
  "${GZ_INCLUDE_ROOT}"
  "${GZ_TRANSPORT_INCLUDE}"
  "${GZ_MSGS_INCLUDE}"
  "${GZ_UTILS_INCLUDE}"
  "${GZ_MATH_INCLUDE}"
  "${GZ_COMMON_INCLUDE}"
  "${GZ_PLUGIN_INCLUDE}"
)

target_link_libraries(drone-navigation PRIVATE
  "${GZ_TRANSPORT_LIBRARY}"
  "${GZ_TRANSPORT_LOG_LIBRARY}"
  "${GZ_TRANSPORT_PARAMETERS_LIBRARY}"
  "${GZ_MSGS_LIBRARY}"
  "${PROTOBUF_LIBRARY}"
  pthread
)

target_compile_options(drone-navigation PRIVATE -Wall -Wextra -Wpedantic)

set_target_properties(drone-navigation PROPERTIES
  BUILD_RPATH "/usr/lib/x86_64-linux-gnu"
  INSTALL_RPATH "/usr/lib/x86_64-linux-gnu")

# Optional offline logic tests (use the real include dirs; no Gazebo needed at runtime)
option(BUILD_OFFLINE_TESTS "Build planner unit/closed-loop kinematic tests" OFF)
if(BUILD_OFFLINE_TESTS)
  foreach(t planner_test closed_loop_sim)
    add_executable(${t} tests/${t}.cpp src/lidar_processor.cpp src/navigator.cpp)
    target_include_directories(${t} PRIVATE include ${ARDUPILOT_MAVLINK_INCLUDE}
      ${GZ_INCLUDE_ROOT} ${GZ_TRANSPORT_INCLUDE} ${GZ_MSGS_INCLUDE} ${GZ_UTILS_INCLUDE} ${GZ_MATH_INCLUDE})
    target_link_libraries(${t} PRIVATE ${GZ_TRANSPORT_LIBRARY} ${GZ_MSGS_LIBRARY} ${PROTOBUF_LIBRARY} pthread)
  endforeach()
endif()
