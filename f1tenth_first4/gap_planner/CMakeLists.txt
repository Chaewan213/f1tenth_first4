cmake_minimum_required(VERSION 3.8)
project(gap_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(race_msgs REQUIRED)

add_executable(gap_planner_node src/gap_planner_node.cpp)
target_include_directories(gap_planner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_target_dependencies(gap_planner_node rclcpp sensor_msgs geometry_msgs race_msgs)

install(TARGETS gap_planner_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

# ROS 없이 도는 코어 단위 테스트: tools/run_core_tests.sh 로도 실행 가능
if(BUILD_TESTING)
  add_executable(test_gap_core test/test_gap_core.cpp)
  target_include_directories(test_gap_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  add_test(NAME test_gap_core COMMAND test_gap_core)
endif()

ament_package()
