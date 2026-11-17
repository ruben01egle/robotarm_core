cmake_minimum_required(VERSION 3.8)
project(robotarm_trajectory_smoothing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(robotarm_interface REQUIRED)

add_executable(trajectory_smoother src/trajectory_smoother.cpp)
target_compile_features(trajectory_smoother PUBLIC cxx_std_17)
ament_target_dependencies(trajectory_smoother
  rclcpp moveit_core moveit_ros_planning trajectory_msgs robotarm_interface)

install(TARGETS trajectory_smoother DESTINATION lib/${PROJECT_NAME})

ament_package()
