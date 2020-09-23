cmake_minimum_required(VERSION 3.5)
project(raptor_dbw_can)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nodelet REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(raptor_dbw_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(can_msgs REQUIRED)
find_package(can_dbc_parser REQUIRED)
find_package(pdu_msgs REQUIRED)

ament_auto_add_library(
  raptor_dbw_can src/DbwNode.cpp
  src/node.cpp
)

ament_auto_add_executable(
  raptor_dbw_can_node src/node.cpp
  src/DbwNode.cpp
  
)

ament_package()