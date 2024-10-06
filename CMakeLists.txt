cmake_minimum_required(VERSION 3.8)
project(dextrous_hand)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(basic_node src/basic_node.cpp)
ament_target_dependencies(basic_node rclcpp)

# C++ Nodes
include_directories(include)
install(TARGETS
  basic_node
  DESTINATION lib/${PROJECT_NAME})

# Python Nodes
install(PROGRAMS
  scripts/basic_node.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
