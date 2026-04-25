cmake_minimum_required(VERSION 3.8)
project(pid_control_ego_pkg)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(angles REQUIRED)
find_package(quadrotor_msgs REQUIRED)

add_executable(position_pid_controller src/pid_controller.cpp)
ament_target_dependencies(position_pid_controller
  rclcpp
  nav_msgs
  std_msgs
  geometry_msgs
  tf2
  angles
  quadrotor_msgs
)
target_include_directories(position_pid_controller PRIVATE include)

if(MSVC)
  target_compile_options(position_pid_controller PRIVATE /W4)
else()
  target_compile_options(position_pid_controller PRIVATE -Wall -Wextra -Wpedantic)
endif()

install(TARGETS position_pid_controller
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.launch.py")

ament_package()
