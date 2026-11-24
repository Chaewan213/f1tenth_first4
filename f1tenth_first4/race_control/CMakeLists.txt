cmake_minimum_required(VERSION 3.8)
project(race_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(race_msgs REQUIRED)

set(NODES mode_manager_node tracker_node safety_filter_node)
foreach(node ${NODES})
  add_executable(${node} src/${node}.cpp)
  target_include_directories(${node} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  ament_target_dependencies(${node}
    rclcpp std_msgs geometry_msgs nav_msgs sensor_msgs ackermann_msgs race_msgs)
  install(TARGETS ${node} DESTINATION lib/${PROJECT_NAME})
endforeach()
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  add_executable(test_control_core test/test_control_core.cpp)
  target_include_directories(test_control_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  add_test(NAME test_control_core COMMAND test_control_core)
endif()

ament_package()
