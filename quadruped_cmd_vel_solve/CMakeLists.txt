cmake_minimum_required(VERSION 3.8)
project(quadruped_cmd_vel_solve)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_executable(quadruped_cmd_vel_solve_node
  src/quadruped_cmd_vel_solve.cpp
)

ament_target_dependencies(
  quadruped_cmd_vel_solve_node
  rclcpp
  geometry_msgs
  std_msgs
)

install(
  TARGETS quadruped_cmd_vel_solve_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
