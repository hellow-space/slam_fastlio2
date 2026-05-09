cmake_minimum_required(VERSION 3.8)
project(livox_3d_detector)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_executable(obstacle_avoidance_node
  src/obstacle_avoidance_node.cpp
)

ament_target_dependencies(obstacle_avoidance_node
  rclcpp
  sensor_msgs
  visualization_msgs
  geometry_msgs
  std_msgs
  pcl_conversions
  pcl_ros
)

target_link_libraries(obstacle_avoidance_node
  ${PCL_LIBRARIES}
)

install(TARGETS
  obstacle_avoidance_node
  DESTINATION lib/${PROJECT_NAME}
)

# 👇👇👇 这一行是关键！必须有！
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()