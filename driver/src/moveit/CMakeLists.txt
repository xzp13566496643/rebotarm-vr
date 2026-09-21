cmake_minimum_required(VERSION 3.8)
project(rebotarm_moveit)

find_package(ament_cmake REQUIRED)

# 安装MoveIt参数、机器人模型和启动文件，供ros2 launch从install空间读取。
install(
  DIRECTORY config description launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/sim_gripper_bridge.py
    scripts/table_collision_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
