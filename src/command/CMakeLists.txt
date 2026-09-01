cmake_minimum_required(VERSION 3.8)
project(rebotarm_command)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rebotarm_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_executable(move_tcp_up src/move_tcp_up.cpp)
ament_target_dependencies(
  move_tcp_up
  rclcpp
  moveit_ros_planning_interface
)

install(TARGETS move_tcp_up DESTINATION lib/${PROJECT_NAME})

add_executable(vr_to_servo src/vr_to_servo.cpp)
ament_target_dependencies(
  vr_to_servo
  geometry_msgs
  rclcpp
  rebotarm_msgs
  std_msgs
  trajectory_msgs
)

install(TARGETS vr_to_servo DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  scripts/arm_only_safe_return.py
  scripts/table_collision.py
  scripts/vr_pose_target_servo.py
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(nearest_self_distance src/nearest_self_distance.cpp)
ament_target_dependencies(
  nearest_self_distance
  moveit_ros_planning_interface
  rclcpp
  sensor_msgs
)

install(TARGETS nearest_self_distance DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
