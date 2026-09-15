import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """加载真机模型，根据关节反馈发布机械臂TF，并可选启动RViz2。"""
    use_rviz = LaunchConfiguration("use_rviz")
    joint_states_topic = LaunchConfiguration("joint_states_topic")

    # 这里只加载DM机械臂URDF；MoveIt Servo所需的SRDF等由另一个launch加载。
    package_share = get_package_share_directory("rebotarm_moveit")
    xacro_path = os.path.join(
        package_share, "description", "urdf", "rebotarm.urdf.xacro"
    )
    robot_description = {
        "robot_description": xacro.process_file(xacro_path).toxml()
    }

    # 给RViz等组件提供固定的世界坐标系入口。
    world_to_base = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_base_publisher",
        output="screen",
        arguments=[
            "0", "0", "0",
            "0", "0", "0",
            "world", "base_link",
        ],
    )

    # 使用真机驱动发布的六关节状态计算base_link到gripper_tcp的整棵TF。
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
        remappings=[("/joint_states", joint_states_topic)],
    )

    # 这里只显示RobotModel，不启动move_group和MotionPlanning规划界面。
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        condition=IfCondition(use_rviz),
        parameters=[robot_description],
        remappings=[("/joint_states", joint_states_topic)],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "joint_states_topic",
            default_value="/rebotarm/joint_states",
            description="真机驱动发布的关节状态话题",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="是否同时启动普通RViz2模型显示",
        ),
        world_to_base,
        robot_state_publisher,
        rviz,
    ])
