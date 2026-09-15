cmake_minimum_required(VERSION 3.8)
project(rebotarm_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArmStatus.msg"
  "msg/JointMitCmd.msg"
  "msg/JointMotorState.msg"
  "msg/JointPosVelCmd.msg"

  "srv/GripperCommand.srv"
  "srv/MoveToPoseIK.srv"
  "srv/SetGripper.srv"
  "srv/SetZero.srv"
  "action/MoveToPose.action"

  DEPENDENCIES
  std_msgs
  builtin_interfaces
  geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
