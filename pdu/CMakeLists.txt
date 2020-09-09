cmake_minimum_required(VERSION 3.5)
project(pdu)

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

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(can_msgs REQUIRED)
find_package(pdu_msgs)
find_package(can_dbc_parser REQUIRED)
#find_package(rosidl_default_generators REQUIRED)

add_executable(
  pdu_node src/node.cpp
  src/pdu.cpp
)
target_link_libraries(pdu_node 
PUBLIC ${can_dbc_parser_LIBRARIES}
)

ament_target_dependencies(
  pdu_node
  rclcpp
  can_dbc_parser
  can_msgs
  pdu_msgs
)

include_directories(
  include
  ${can_dbc_parser_INCLUDE_DIRS}
)

install(
  TARGETS pdu_node
  EXPORT export_pdu_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()