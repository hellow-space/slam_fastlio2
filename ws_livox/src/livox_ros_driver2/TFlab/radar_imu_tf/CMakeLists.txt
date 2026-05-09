cmake_minimum_required(VERSION 3.8)
project(radar_imu_tf)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(livox_body_publisher src/livox_body_publisher.cpp)
ament_target_dependencies(livox_body_publisher
  rclcpp
  geometry_msgs
)
target_link_libraries(livox_body_publisher yaml-cpp)

install(TARGETS
  livox_body_publisher
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

