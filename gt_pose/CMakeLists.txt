cmake_minimum_required(VERSION 3.8)
project(gt_pose)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(gt_pose_node src/gt_pose.cpp)
ament_target_dependencies(gt_pose_node
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS
  gt_pose_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()