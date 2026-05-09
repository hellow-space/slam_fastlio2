cmake_minimum_required(VERSION 3.8)
project(livox_pointcloud_transformer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# 节点1：静态TF发布
add_executable(livox_static_tf_publisher src/livox_static_tf_publisher.cpp)
ament_target_dependencies(livox_static_tf_publisher
  rclcpp
  tf2_ros
  geometry_msgs
)
target_link_libraries(livox_static_tf_publisher yaml-cpp)

# 节点2：点云变换
add_executable(pointcloud_transformer src/pointcloud_transformer.cpp)
ament_target_dependencies(pointcloud_transformer
  rclcpp
  sensor_msgs
  tf2_ros
  tf2_sensor_msgs
)

install(TARGETS
  livox_static_tf_publisher
  pointcloud_transformer
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

