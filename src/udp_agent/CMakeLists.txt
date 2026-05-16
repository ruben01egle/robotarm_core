cmake_minimum_required(VERSION 3.8)
project(udp_agent)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(interface REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(udp_agent_node 
  src/CUDPAgentNode.cpp
  src/CUDPClient.cpp
  src/main.cpp
)

target_include_directories(udp_agent_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(udp_agent_node
  rclcpp::rclcpp
  interface::protocol_headers
  interface::interface__rosidl_typesupport_cpp 
)

install(TARGETS
  udp_agent_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()