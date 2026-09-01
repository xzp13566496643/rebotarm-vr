cmake_minimum_required(VERSION 3.8)
project(rebotarm_command)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/table_collision.py
  scripts/vr_pose_target_servo.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
