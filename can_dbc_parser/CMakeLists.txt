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

add_library(
  can_dbc_parser SHARED
  src/DbcMessage.cpp
  src/DbcSignal.cpp
  src/Dbc.cpp
  src/LineParser.cpp
  src/DbcBuilder.cpp
)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(
  ${PROJECT_NAME} PUBLIC
  ${can_msgs_TARGETS}
  ${rclcpp_TARGETS}
)
target_compile_options(${PROJECT_NAME} PUBLIC
  $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-Wno-unused-function>
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

#run colcon test to run linters against code
if(BUILD_TESTING)
  find_package(ament_lint_auto)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
