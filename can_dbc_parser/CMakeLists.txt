cmake_minimum_required(VERSION 3.5)
project(can_dbc_parser)

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
find_package(can_msgs REQUIRED)

include_directories(include)

ament_export_include_directories(include)

add_library(
  can_dbc_parser SHARED
  src/DbcMessage.cpp
  src/DbcSignal.cpp
  src/Dbc.cpp
  src/LineParser.cpp
  src/DbcBuilder.cpp
)

ament_target_dependencies(can_dbc_parser rclcpp can_msgs std_msgs)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hpp"
  PATTERN ".git" EXCLUDE
)

install(TARGETS
  can_dbc_parser
  DESTINATION lib/${PROJECT_NAME}
  PUBLIC_HEADER DESTINATION include/${PROJECT_NAME}/
)

ament_package()