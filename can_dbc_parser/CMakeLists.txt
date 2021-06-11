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
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(
  can_dbc_parser SHARED
  src/DbcMessage.cpp
  src/DbcSignal.cpp
  src/Dbc.cpp
  src/LineParser.cpp
  src/DbcBuilder.cpp
)

target_compile_options(can_dbc_parser PRIVATE -Wno-unused-function)

#run colcon test to run linters against code
if(BUILD_TESTING)
  find_package(ament_lint_auto)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
