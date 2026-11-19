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

# the algorithm, without ROS node or robot model (unit tested)
add_library(smoothing STATIC src/smoothing.cpp)
target_compile_features(smoothing PUBLIC cxx_std_17)
target_include_directories(smoothing PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_target_dependencies(smoothing trajectory_msgs)
set_target_properties(smoothing PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(trajectory_smoother src/trajectory_smoother.cpp)
target_link_libraries(trajectory_smoother smoothing)
ament_target_dependencies(trajectory_smoother
  rclcpp moveit_core moveit_ros_planning trajectory_msgs robotarm_interface)

install(TARGETS trajectory_smoother DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_smoothing test/test_smoothing.cpp)
  target_link_libraries(test_smoothing smoothing)
endif()

ament_package()
